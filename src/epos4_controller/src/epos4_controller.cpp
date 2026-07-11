// claude_opt: 全面リファクタ。外部インターフェース(ノード名/トピック/サービス/パラメータ)は
// 旧実装と完全互換。変更点:
//   - motor1/motor2 で丸ごと二重だったクライアント群を MotorInterface に集約
//   - 未使用だった halt/shutdown/velocity_mode/target クライアントと Float64 publisher、
//     コメントアウトされたままの /robot_encoder_states 系の死んだコードを削除
//   - claude_swap (motor1=右輪, motor2=左輪) 後も "motor1(left)" のままだった
//     ログラベルを物理配線に一致させた
//   - 終了時の disable が rclcpp::shutdown() 後のデストラクタから呼ばれていて
//     実際には送信されなかった問題を、pre_shutdown コールバック(context 無効化前に
//     走る)へ移して修正
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>  // claude_robust: 車体パラメータ検証の fail-fast 用
#include <string>
#include <thread>

#include "canopen_interfaces/msg/co_data.hpp"
#include "canopen_interfaces/srv/co_read.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/contexts/default_context.hpp"  // claude_opt: pre_shutdown callback 用
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using Trigger = std_srvs::srv::Trigger;
using CORead = canopen_interfaces::srv::CORead;

// claude_opt: 1 モータ分の CANopen 入出力(ライフサイクルサービス + SDO read + TPDO)。
// 物理配線は motor1 = RIGHT wheel, motor2 = LEFT wheel (claude_swap)。
struct MotorInterface
{
  std::string label;  // ログ表示用 (例: "motor1(right)")
  rclcpp::Client<Trigger>::SharedPtr init;
  rclcpp::Client<Trigger>::SharedPtr enable;
  rclcpp::Client<Trigger>::SharedPtr disable;
  rclcpp::Client<Trigger>::SharedPtr recover;
  rclcpp::Client<Trigger>::SharedPtr csv_mode;
  rclcpp::Client<CORead>::SharedPtr sdo_read;
  rclcpp::Publisher<canopen_interfaces::msg::COData>::SharedPtr tpdo;
  // timer_callback が 0x60FF に書く目標速度 [rpm]。cmdSpeedCallback と同一
  // executor スレッドで読み書きされるため排他は不要。
  double target_rpm = 0.0;
  // claude_robust: statusword 監視の前回状態 ("FAULT"/"OP_ENABLED"/"NOT_ENABLED")。
  // 変化時のみログするための記憶。executor スレッドからのみ触る。
  std::string last_status_state;
};

class Epos4ControllerNode : public rclcpp::Node
{
public:
  Epos4ControllerNode() : Node("epos4_controller_node")
  {
    motor1_ = make_motor("/motor1/cia402_device_1", "motor1(right)");
    motor2_ = make_motor("/motor2/cia402_device_2", "motor2(left)");

    cmd_speed_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/robot_speed_cmd", 10,
      std::bind(&Epos4ControllerNode::cmdSpeedCallback, this, std::placeholders::_1));

    // claude: 脱力(フリー)モードのトグル受信。data=true で両モータを disable し
    // 非励磁(手で車輪が回せる状態)に、false で enable+CSV を再投入して復帰する。
    free_mode_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/robot_free_mode", 10,
      std::bind(&Epos4ControllerNode::freeModeCallback, this, std::placeholders::_1));

    declare_parameter("tread_width", 0.41);
    declare_parameter("tire_diam", 0.15);
    declare_parameter("gear_ratio", 1.0);
    declare_parameter("invert_left", false);
    declare_parameter("invert_right", false);
    tread_width_ = get_parameter("tread_width").as_double();
    tire_diam_ = get_parameter("tire_diam").as_double();
    gear_ratio_ = get_parameter("gear_ratio").as_double();
    invert_left_ = get_parameter("invert_left").as_bool();
    invert_right_ = get_parameter("invert_right").as_bool();

    // claude_robust: cmd_timeout_sec = /robot_speed_cmd のデッドマン監視 (0 で無効)。
    // motor_status_period_sec = statusword 監視周期 (0 で無効)。
    declare_parameter("cmd_timeout_sec", 0.5);
    declare_parameter("motor_status_period_sec", 2.0);
    cmd_timeout_sec_ = get_parameter("cmd_timeout_sec").as_double();
    const double status_period = get_parameter("motor_status_period_sec").as_double();

    // claude_robust: 車体パラメータの妥当性チェック。ゼロ/負値は IK の除算・符号を
    // 静かに壊す(症状は「変な速度で走る」)ので、起動時に落として原因を明示する。
    if (tread_width_ <= 0.0 || tire_diam_ <= 0.0 || gear_ratio_ <= 0.0) {
      RCLCPP_FATAL(
        get_logger(), "invalid chassis params: tread_width=%.3f tire_diam=%.3f gear_ratio=%.3f",
        tread_width_, tire_diam_, gear_ratio_);
      throw std::invalid_argument("epos4_controller: chassis parameters must be > 0");
    }
    // claude_robust: どの yaml が効いたかを起動ログで確認できるようにする
    RCLCPP_INFO(
      get_logger(),
      "chassis: tread=%.3f m tire_diam=%.3f m gear=%.2f invert L/R=%d/%d cmd_timeout=%.2fs",
      tread_width_, tire_diam_, gear_ratio_, invert_left_, invert_right_, cmd_timeout_sec_);

    topic_timer_ =
      create_wall_timer(10ms, std::bind(&Epos4ControllerNode::timer_callback, this));

    // claude_robust: statusword 監視。init 完了後、SDO で 0x6041 を定期読みして
    // FAULT / Operation-Enabled 喪失を「状態変化時に」ログする。過去に多発した
    // 「片輪が静かに死んでいて走り出すまで気付かない」を走行前に検出するのが目的。
    // 脱力中と復帰シーケンス中は disabled が正常なのでスキップ。
    if (status_period > 0.0) {
      status_timer_ = create_wall_timer(
        std::chrono::duration<double>(status_period),
        std::bind(&Epos4ControllerNode::statusMonitorCallback, this));
    }

    // claude: init は spin 開始後に動くワーカースレッドで実行する。コンストラクタは
    // rclcpp::spin() より前に走るため、ここで async_send_request しても応答
    // (future) を処理できず、init→enable→csv を「待たずに連射」する旧実装は
    // ドライバ側で遷移が取りこぼされるレースになっていた(失敗モータが Homing
    // モード/Switch-On-Disabled に取り残され、片輪が動かない)。スレッドなら
    // 各サービス応答を future で待って逐次化でき、main() の spin が応答を捌く。
    init_thread_ = std::thread(&Epos4ControllerNode::run_init_sequence, this);

    // claude_opt: 終了時のモータ disable。旧実装はデストラクタ(= rclcpp::shutdown()
    // 完了後)から呼んでいたため wait_for_service が即失敗し、実際には disable が
    // 届いていなかった。pre_shutdown コールバックは context が無効化される「前」に
    // 呼ばれるので、ここならリクエストが実際に CAN まで届く。
    pre_shutdown_handle_ =
      rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
        [this] { disable_on_exit(); });

    RCLCPP_INFO(get_logger(), "********************************************");
    RCLCPP_INFO(get_logger(), "maxon EPOS4 Control (cyclic sync velocity)");
    RCLCPP_INFO(get_logger(), "motors: /motor1(right) /motor2(left) on can0");
    RCLCPP_INFO(get_logger(), "run first bus_config_cia402_epos4_vel.launch.py");
    RCLCPP_INFO(get_logger(), "********************************************");
  }

  ~Epos4ControllerNode() override
  {
    // claude_opt: コールバックがダングリングしないよう先に登録解除
    rclcpp::contexts::get_global_default_context()->remove_pre_shutdown_callback(
      pre_shutdown_handle_);
    stop_init_.store(true);
    if (init_thread_.joinable()) {
      init_thread_.join();
    }
    if (reenable_thread_.joinable()) {
      reenable_thread_.join();  // claude: 進行中の復帰シーケンスを回収してから終了
    }
  }

private:
  // claude_opt: モータ 1 台分のクライアント/パブリッシャ一式を生成する。
  // 名前空間パターン /motorN/cia402_device_N は external/.../bus.yml の定義に従う。
  MotorInterface make_motor(const std::string & ns, const std::string & label)
  {
    MotorInterface m;
    m.label = label;
    m.init = create_client<Trigger>(ns + "/init");
    m.enable = create_client<Trigger>(ns + "/enable");
    m.disable = create_client<Trigger>(ns + "/disable");
    m.recover = create_client<Trigger>(ns + "/recover");
    m.csv_mode = create_client<Trigger>(ns + "/cyclic_velocity_mode");
    m.sdo_read = create_client<CORead>(ns + "/sdo_read");
    m.tpdo = create_publisher<canopen_interfaces::msg::COData>(ns + "/tpdo", 10);
    return m;
  }

  void timer_callback()
  {
    // claude_robust: デッドマン監視。/robot_speed_cmd が cmd_timeout_sec 途絶えたら
    // ターゲットを 0 に落とす。teleop / Nav2 が落ちたとき「最後に受けた速度で
    // 走り続ける」暴走モードを防ぐ。teleop は停止中も 20 Hz で 0 を出し続ける
    // 設計なので、正常運転中にこれが発火することはない。
    if (
      cmd_timeout_sec_ > 0.0 && have_cmd_ &&
      (now() - last_cmd_time_).seconds() > cmd_timeout_sec_) {
      if (motor1_.target_rpm != 0.0 || motor2_.target_rpm != 0.0) {
        RCLCPP_WARN(
          get_logger(), "/robot_speed_cmd silent for %.2fs — zeroing targets (dead-man stop)",
          cmd_timeout_sec_);
        motor1_.target_rpm = 0.0;
        motor2_.target_rpm = 0.0;
      }
    }

    for (MotorInterface * m : {&motor1_, &motor2_}) {  // claude_opt
      auto msg = canopen_interfaces::msg::COData();
      msg.index = 0x60ff;   // target velocity (CSV)
      msg.subindex = 0x00;
      msg.data = static_cast<int>(m->target_rpm);
      m->tpdo->publish(msg);
    }
  }

  void cmdSpeedCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = now();  // claude_robust: デッドマン監視の生存シグナル
    have_cmd_ = true;        // claude_robust
    // claude: 脱力中は速度指令を一切反映しない(ターゲットは 0 のまま)。
    if (free_mode_) {
      return;
    }
    double x = msg->linear.x;
    double yaw = msg->angular.z;
    // differential-drive inverse kinematics: body twist -> per-wheel linear speed [m/s]
    double v_left = x - yaw * tread_width_ * 0.5;
    double v_right = x + yaw * tread_width_ * 0.5;
    // wheel linear speed [m/s] -> wheel rotational speed [rpm] (EPOS4 target velocity unit)
    double wheel_circumference = M_PI * tire_diam_;
    // wheel rpm -> motor rpm via gear ratio (motor rpm = wheel rpm * gear_ratio)
    double rpm_left = (v_left / wheel_circumference) * 60.0 * gear_ratio_;
    double rpm_right = (v_right / wheel_circumference) * 60.0 * gear_ratio_;
    // claude_swap: physical wiring is motor1 = RIGHT wheel, motor2 = LEFT wheel
    // (turning was reversed before this swap; forward was unaffected because both
    // wheels share the same rpm). invert_* stay per-motor polarity flags.
    motor1_.target_rpm = invert_right_ ? -rpm_right : rpm_right;
    motor2_.target_rpm = invert_left_ ? -rpm_left : rpm_left;
  }

  // claude_opt: pre_shutdown コールバック本体。context がまだ有効なうちに両モータへ
  // disable を投げる(応答は待たない: executor は止まりかけだがリクエスト送信自体は
  // rmw 層で即時に行われる)。100ms は CAN へフレームが掃けるまでの猶予。
  void disable_on_exit()
  {
    motor1_.target_rpm = 0.0;
    motor2_.target_rpm = 0.0;
    RCLCPP_INFO(get_logger(), "Sending disable to both motors before exit...");
    for (MotorInterface * m : {&motor1_, &motor2_}) {
      if (m->disable->service_is_ready()) {
        m->disable->async_send_request(std::make_shared<Trigger::Request>());
      }
    }
    std::this_thread::sleep_for(100ms);
  }

  void trigger_callback(rclcpp::Client<Trigger>::SharedFuture future)
  {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(get_logger(), "Service call successful: %s", response->message.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Service call failed: %s", response->message.c_str());
    }
  }

  void call_trigger_service(
    rclcpp::Client<Trigger>::SharedPtr client, const std::string & service_name)
  {
    if (!client->wait_for_service(1s)) {
      RCLCPP_ERROR(get_logger(), "Service %s not available", service_name.c_str());
      return;
    }
    auto request = std::make_shared<Trigger::Request>();
    client->async_send_request(
      request, std::bind(&Epos4ControllerNode::trigger_callback, this, std::placeholders::_1));
  }

  // claude: synchronous Trigger call. Sends the request and blocks (in the init
  // worker thread, NOT the executor thread) until the response future is ready,
  // so init→enable→csv can be issued strictly one-after-another instead of being
  // fired all at once. Returns the driver's success flag.
  bool call_trigger_sync(
    rclcpp::Client<Trigger>::SharedPtr client, const std::string & name,
    std::chrono::seconds timeout = 5s)
  {
    if (!client->wait_for_service(2s)) {
      RCLCPP_ERROR(get_logger(), "Service %s not available", name.c_str());
      return false;
    }
    auto request = std::make_shared<Trigger::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      RCLCPP_WARN(
        get_logger(), "%s: no response within %lds", name.c_str(),
        static_cast<long>(timeout.count()));
      return false;
    }
    return future.get()->success;
  }

  // claude: blocking SDO upload (read) of a 16-bit object, returns nullopt on failure.
  std::optional<uint32_t> read_sdo(
    rclcpp::Client<CORead>::SharedPtr client, uint16_t index)
  {
    if (!client->wait_for_service(2s)) {
      return std::nullopt;
    }
    auto request = std::make_shared<CORead::Request>();
    request->index = index;
    request->subindex = 0;
    auto future = client->async_send_request(request);
    if (future.wait_for(2s) != std::future_status::ready) {
      return std::nullopt;
    }
    auto response = future.get();
    if (!response->success) {
      return std::nullopt;
    }
    return response->data;
  }

  // claude: verify a drive actually reached "Operation Enabled" in cyclic sync
  // velocity mode. statusword (0x6041) low byte masked with 0x6F == 0x27 means
  // Operation Enabled; mode-of-operation display (0x6061) == 9 means CSV. A drive
  // stuck after a lost transition reads e.g. statusword 0x0240 (Switch On Disabled)
  // / mode 6 (Homing) — exactly the silent "wheel doesn't move" failure.
  bool motor_ready(MotorInterface & m)  // claude_opt: 引数を MotorInterface に集約
  {
    auto statusword = read_sdo(m.sdo_read, 0x6041);
    auto mode = read_sdo(m.sdo_read, 0x6061);
    if (!statusword.has_value() || !mode.has_value()) {
      RCLCPP_WARN(get_logger(), "%s: could not read statusword/mode via SDO", m.label.c_str());
      return false;
    }
    const bool op_enabled = ((statusword.value() & 0x6F) == 0x27);
    const bool csv_mode = (static_cast<int8_t>(mode.value()) == 9);
    RCLCPP_INFO(
      get_logger(), "%s: statusword=0x%04X mode=%d (op_enabled=%d csv=%d)", m.label.c_str(),
      statusword.value(), static_cast<int8_t>(mode.value()), op_enabled, csv_mode);
    return op_enabled && csv_mode;
  }

  // claude_robust: statusword(0x6041) を可読状態へ分類する。
  //   FAULT       — bit3 (fault) が立っている
  //   OP_ENABLED  — (sw & 0x6F) == 0x27 (CiA402 Operation Enabled)
  //   NOT_ENABLED — それ以外 (Switch-On-Disabled 等。車輪は動かない)
  static const char * classify_status(uint16_t sw)
  {
    if (sw & 0x0008) {
      return "FAULT";
    }
    if ((sw & 0x6F) == 0x27) {
      return "OP_ENABLED";
    }
    return "NOT_ENABLED";
  }

  // claude_robust: 定期 statusword 監視 (executor スレッド、非ブロッキング)。
  // init 前・脱力中・復帰中は期待状態が Operation Enabled ではないためスキップ。
  // 応答はコールバックで受け、状態が「変化したときだけ」ログする。
  void statusMonitorCallback()
  {
    if (!init_done_.load() || free_mode_ || reenable_active_.load()) {
      return;
    }
    for (MotorInterface * m : {&motor1_, &motor2_}) {
      if (!m->sdo_read->service_is_ready()) {
        continue;
      }
      auto req = std::make_shared<CORead::Request>();
      req->index = 0x6041;
      req->subindex = 0;
      m->sdo_read->async_send_request(
        req, [this, m](rclcpp::Client<CORead>::SharedFuture future) {
          auto response = future.get();
          if (!response->success) {
            return;
          }
          const uint16_t sw = static_cast<uint16_t>(response->data);
          const std::string state = classify_status(sw);
          if (state == m->last_status_state) {
            return;  // 変化時のみログ
          }
          if (state == "FAULT") {
            RCLCPP_ERROR(
              get_logger(), "%s: DRIVE FAULT (statusword=0x%04X) — check EPOS4 / CAN wiring",
              m->label.c_str(), sw);
          } else if (state == "OP_ENABLED") {
            RCLCPP_INFO(
              get_logger(), "%s: Operation Enabled (statusword=0x%04X)", m->label.c_str(), sw);
          } else {
            RCLCPP_WARN(
              get_logger(), "%s: NOT enabled (statusword=0x%04X) — wheel will not move",
              m->label.c_str(), sw);
          }
          m->last_status_state = state;
        });
    }
  }

  // claude: bring a single drive to CSV / Operation-Enabled, verifying via SDO and
  // retrying (with a recover in between) on failure. This is the actual fix for the
  // intermittent dead wheel.
  void init_motor(MotorInterface & m)  // claude_opt: 引数 6 個 → 1 個
  {
    constexpr int max_attempts = 5;
    for (int attempt = 1; attempt <= max_attempts && !stop_init_.load() && rclcpp::ok();
         ++attempt) {
      // init triggers homing, which reliably "fails" for CSV (homing not required) — expected.
      call_trigger_sync(m.init, m.label + " init");
      call_trigger_sync(m.enable, m.label + " enable");
      call_trigger_sync(m.csv_mode, m.label + " cyclic_velocity_mode");
      std::this_thread::sleep_for(200ms);  // let the drive settle before reading back

      if (motor_ready(m)) {
        RCLCPP_INFO(
          get_logger(), "%s ready (Operation Enabled, CSV) on attempt %d", m.label.c_str(),
          attempt);
        return;
      }
      RCLCPP_WARN(
        get_logger(), "%s not ready on attempt %d/%d; recovering and retrying", m.label.c_str(),
        attempt, max_attempts);
      call_trigger_sync(m.recover, m.label + " recover");
      std::this_thread::sleep_for(300ms);
    }
    RCLCPP_ERROR(
      get_logger(),
      "%s FAILED to reach CSV Operation-Enabled after %d attempts — wheel will not move",
      m.label.c_str(), max_attempts);
  }

  // claude: runs in init_thread_ (after main() starts spinning, so service/SDO
  // response futures get processed). Brings up motor1 then motor2 sequentially.
  // claude_opt: この 20s の wait_for_service が bus_config 起動待ちを担うため、
  // launch 側の TimerAction による遅延は不要(撤去済み)。
  void run_init_sequence()
  {
    RCLCPP_INFO(get_logger(), "Auto-initializing EPOS4 (sequential, verified)...");
    // device_manager advertises the lifecycle services only once both drivers
    // have booted; wait for that instead of racing it.
    if (!motor1_.init->wait_for_service(20s)) {
      RCLCPP_ERROR(get_logger(), "motor1 init service never appeared; aborting auto-init");
      return;
    }
    init_motor(motor1_);
    init_motor(motor2_);
    RCLCPP_INFO(get_logger(), "EPOS4 auto-init complete.");
    init_done_.store(true);  // claude_robust: ここから statusword 監視が有効になる
  }

  // claude: /robot_free_mode の受信ハンドラ。data=true で脱力 ON、false で復帰。
  // 脱力 ON: disable を非同期で投げるだけ(励磁を切るだけなので順序不問)。
  // 復帰   : enable→cyclic_velocity_mode は逐次に効かせないと mode 設定がレースして
  //          片輪が CSV に入りきらず遅れる。短命スレッドで sync 呼び出しを順番に行う
  //          (init=homing は呼ばない。再 init は復帰失敗の原因になるため)。
  void freeModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    free_mode_ = msg->data;
    motor1_.target_rpm = 0.0;  // 脱力中も復帰直後も指令を 0 から始める
    motor2_.target_rpm = 0.0;
    if (free_mode_) {
      RCLCPP_INFO(get_logger(), "脱力モード ON: disabling both motors (free wheel)");
      call_trigger_service(motor1_.disable, "disable");
      call_trigger_service(motor2_.disable, "disable");
    } else {
      RCLCPP_INFO(get_logger(), "脱力モード OFF: re-enabling both motors (enable + CSV)");
      if (reenable_thread_.joinable()) {
        reenable_thread_.join();  // 直前の復帰は完了済み、即座に返る
      }
      reenable_active_.store(true);  // claude_robust: 復帰中は statusword 監視を止める
      reenable_thread_ = std::thread([this] {
        call_trigger_sync(motor1_.enable, "motor1 enable");
        call_trigger_sync(motor1_.csv_mode, "motor1 cyclic_velocity_mode");
        call_trigger_sync(motor2_.enable, "motor2 enable");
        call_trigger_sync(motor2_.csv_mode, "motor2 cyclic_velocity_mode");
        // 成否はサービス戻り値ではなく実ドライブ状態で判定する。enable/csv が
        // no-op 遷移のときドライバが success=false を返すことがあり(偽陰性)、
        // 戻り値だけ見ると正常復帰でも失敗扱いになるため。motor_ready は
        // SDO で statusword(0x6041)/mode(0x6061)を読み、Operation Enabled & CSV を確認する。
        std::this_thread::sleep_for(200ms);  // 読み戻し前にドライブを落ち着かせる
        const bool m1 = motor_ready(motor1_);
        const bool m2 = motor_ready(motor2_);
        if (m1 && m2) {
          RCLCPP_INFO(get_logger(), "脱力モード OFF: both motors re-enabled (CSV)");
        } else {
          RCLCPP_WARN(
            get_logger(),
            "脱力モード OFF: re-enable incomplete (m1_ready=%d m2_ready=%d); toggle f to retry",
            m1, m2);
        }
        reenable_active_.store(false);  // claude_robust: 監視再開
      });
    }
  }

  MotorInterface motor1_;  // claude_opt: 物理 RIGHT wheel
  MotorInterface motor2_;  // claude_opt: 物理 LEFT wheel

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_speed_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr free_mode_subscription_;  // claude
  rclcpp::TimerBase::SharedPtr topic_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;  // claude_robust: statusword 監視

  // claude_robust: デッドマン監視。last_cmd_time_/have_cmd_ は executor スレッド
  // (cmdSpeedCallback と timer_callback) からのみ触るため排他不要。
  double cmd_timeout_sec_ = 0.5;
  rclcpp::Time last_cmd_time_;
  bool have_cmd_ = false;

  // claude_robust: statusword 監視のゲート。init_done_ は init スレッドが、
  // reenable_active_ は復帰スレッドが書くため atomic。
  std::atomic<bool> init_done_{false};
  std::atomic<bool> reenable_active_{false};

  // claude: background driver-init sequence (sequential + verified + retried)
  std::thread init_thread_;
  std::atomic<bool> stop_init_{false};

  // claude: 脱力(フリー)モード。true の間 cmdSpeedCallback が速度指令を無視する。
  // 読み書きとも executor スレッド(コールバック)上なので plain bool で十分。
  bool free_mode_ = false;
  // claude: 復帰(enable→csv)を逐次化するための短命スレッド。並行に投げると
  // mode 設定がレースして片輪が CSV に入りきらず遅れるため、別スレッドで
  // call_trigger_sync を順番に効かせる(executor を塞がない/init_thread_ と同じ理由)。
  std::thread reenable_thread_;

  // claude_opt: 終了時 disable 用 pre_shutdown コールバックのハンドル
  rclcpp::PreShutdownCallbackHandle pre_shutdown_handle_;

  double tread_width_;
  double tire_diam_;
  double gear_ratio_;
  bool invert_left_;
  bool invert_right_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Epos4ControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
