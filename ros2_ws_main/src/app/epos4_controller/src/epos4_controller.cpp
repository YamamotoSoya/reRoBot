#include <algorithm>  // claude_watchdog: std::min
#include <atomic>     // claude
#include <chrono>
#include <cmath>
#include <cstdio>   // claude_watchdog: std::snprintf
#include <fstream>  // claude_watchdog: /sys/class/net/<can>/ifindex 読み取り
#include <future>   // claude
#include <iostream>
#include <map>  // claude_watchdog: EPOS4 エラーコード名テーブル
#include <memory>
#include <optional>  // claude
#include <string>    // claude_watchdog
#include <thread>
#include <vector>  // claude_watchdog

#include "canopen_interfaces/msg/co_data.hpp"
#include "canopen_interfaces/srv/co_read.hpp"  // claude: SDO read for statusword/mode verification
#include "canopen_interfaces/srv/co_target_double.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"  // claude_watchdog: フォルト/リンク状態を /diagnostics へ
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"  // claude: 脱力(フリー)モードのトグル受信用
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class Epos4_Control2_Node : public rclcpp::Node
{
public:
  Epos4_Control2_Node() : Node("epos4_controller_node")
  {
    // motor1
    m1_client_driver_init_ =
      this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/init");
    m1_client_driver_halt_ =
      this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/halt");
    m1_client_driver_recover_ =
      this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/recover");
    m1_client_driver_shutdown_ =
      this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/shutdown");
    m1_client_driver_enable_ =
      this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/enable");
    m1_client_driver_disable_ =
      this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/disable");
    m1_client_driver_vel_mode_ =
      this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/velocity_mode");
    m1_client_driver_csv_mode_ =
      this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/cyclic_velocity_mode");
    m1_client_target_ = this->create_client<canopen_interfaces::srv::COTargetDouble>(
      "/motor1/cia402_device_1/target");
    m1_client_sdo_read_ = this->create_client<canopen_interfaces::srv::CORead>(
      "/motor1/cia402_device_1/sdo_read");  // claude
    m1_tpdo_publisher_ =
      this->create_publisher<canopen_interfaces::msg::COData>("/motor1/cia402_device_1/tpdo", 10);

    // m1_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
    //     "/motor1/cia402_device_1/joint_states", 10,
    //     std::bind(&Epos4_Control2_Node::jointStateCallback_m1, this, std::placeholders::_1));

    // motor2
    m2_client_driver_init_ =
      this->create_client<std_srvs::srv::Trigger>("/motor2/cia402_device_2/init");
    m2_client_driver_halt_ =
      this->create_client<std_srvs::srv::Trigger>("/motor2/cia402_device_2/halt");
    m2_client_driver_recover_ =
      this->create_client<std_srvs::srv::Trigger>("/motor2/cia402_device_2/recover");
    m2_client_driver_shutdown_ =
      this->create_client<std_srvs::srv::Trigger>("/motor2/cia402_device_2/shutdown");
    m2_client_driver_enable_ =
      this->create_client<std_srvs::srv::Trigger>("/motor2/cia402_device_2/enable");
    m2_client_driver_disable_ =
      this->create_client<std_srvs::srv::Trigger>("/motor2/cia402_device_2/disable");
    m2_client_driver_vel_mode_ =
      this->create_client<std_srvs::srv::Trigger>("/motor2/cia402_device_2/velocity_mode");
    m2_client_driver_csv_mode_ =
      this->create_client<std_srvs::srv::Trigger>("/motor2/cia402_device_2/cyclic_velocity_mode");
    m2_client_target_ = this->create_client<canopen_interfaces::srv::COTargetDouble>(
      "/motor2/cia402_device_2/target");
    m2_client_sdo_read_ = this->create_client<canopen_interfaces::srv::CORead>(
      "/motor2/cia402_device_2/sdo_read");  // claude
    m2_tpdo_publisher_ =
      this->create_publisher<canopen_interfaces::msg::COData>("/motor2/cia402_device_2/tpdo", 10);

    // m2_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
    //     "/motor2/cia402_device_2/joint_states", 10,
    //     std::bind(&Epos4_Control2_Node::jointStateCallback_m2, this, std::placeholders::_1));

    cmd_speed_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/robot_speed_cmd", 10,
      std::bind(&Epos4_Control2_Node::cmdSpeedCallback, this, std::placeholders::_1));

    // claude: 脱力(フリー)モードのトグル受信。data=true で両モータを disable し
    // 非励磁(手で車輪が回せる状態)に、false で enable+CSV を再投入して復帰する。
    free_mode_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/robot_free_mode", 10,
      std::bind(&Epos4_Control2_Node::freeModeCallback, this, std::placeholders::_1));

    // encoder_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/robot_encoder_states", 10);

    // claude_watchdog: フォルトコード・リンク状態を /diagnostics に出す (bag に残せる)。
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    // claude_watchdog: ros2_canopen の ProxyDriver は受信 PDO の各オブジェクトを ~/rpdo (COData)
    // に流す。TPDO1 に載っている statusword (0x6041) をここから取れば SDO ポーリングが不要になり
    // (sdo_read ごとに driver が INFO を出してユーザが「接続待ち」と誤認した 2026-09-19)、
    // 受信時刻そのものが「PDO が流れているか」= リンク生存の指標になる。
    m1_rpdo_sub_ = this->create_subscription<canopen_interfaces::msg::COData>(
      "/motor1/cia402_device_1/rpdo", 50,
      [this](const canopen_interfaces::msg::COData::SharedPtr msg) { on_rpdo(m1_rpdo_, *msg); });
    m2_rpdo_sub_ = this->create_subscription<canopen_interfaces::msg::COData>(
      "/motor2/cia402_device_2/rpdo", 50,
      [this](const canopen_interfaces::msg::COData::SharedPtr msg) { on_rpdo(m2_rpdo_, *msg); });

    //YAMLparams
    declare_parameter("tread_width", 0.41);
    declare_parameter("tire_diam", 0.15);
    declare_parameter("gear_ratio", 1.0);
    declare_parameter("invert_left", false);
    declare_parameter("invert_right", false);
    // claude: 加減速ランプ上限 [モータ軸 rpm/s]。CSV モードはドライブ側の加減速
    // プロファイルを使わないため、指令ステップ(例: joy 離し = 最高速→0)がそのまま
    // 最大制動電流+回生スパイクになる(2026-07-31 の全モータ同時停止の原因)。
    // ここで 0x60FF へ渡す値の変化率を制限する。EPOS4 の Max acceleration(0x60C5)と
    // 同じ単位にしてあるので、EPOS 側の安全網はこれより大きい値に設定する。
    declare_parameter("max_motor_accel_rpm_per_s", 2000.0);
    declare_parameter("max_motor_decel_rpm_per_s", 2000.0);
    tread_width_ = get_parameter("tread_width").as_double();
    tire_diam_ = get_parameter("tire_diam").as_double();
    gear_ratio_ = get_parameter("gear_ratio").as_double();
    invert_left_ = get_parameter("invert_left").as_bool();
    invert_right_ = get_parameter("invert_right").as_bool();
    // claude: 100 Hz タイマ 1 tick あたりの最大変化量 [rpm] に前計算しておく
    accel_step_ = get_parameter("max_motor_accel_rpm_per_s").as_double() * 0.01;
    decel_step_ = get_parameter("max_motor_decel_rpm_per_s").as_double() * 0.01;

    // claude_watchdog (2026-09-19): 「通信は生きているのに指令が更新されない」型の暴走対策。
    //   EPOS4 側は RPDO timeout (0x8250, 補間周期 10 ms) で PDO 途絶を自衛するが、
    //   PDO が届き続ける形 (joy のデバイス喪失で teleop が publish 停止、master の受信側のみ
    //   死亡) では最後の非ゼロ指令が無期限に送られ続ける (2026-09-19 14:57 の暴走)。
    //   - cmd_timeout_s: /robot_speed_cmd がこの秒数途絶し目標が非ゼロなら 0 へランプ (0 で無効)
    //   - monitor_period_s: 監視ループの評価周期 (statusword は ~/rpdo 購読で得るのでバス負荷なし)
    //   - monitor_sdo_timeout_s: Fault 検知時にコードを読む SDO の待ち時間
    //   - link_loss_timeout_s: 両ノードの PDO (statusword) がこの秒数途絶したらリンク喪失と判定して目標 0
    //   - can_interface: /sys/class/net/<if>/ifindex を監視。値が変わる = can0 が作り直された
    //     (USB 再列挙) が ros2_canopen master は旧 can0 を掴んだまま → スタック再起動が必要
    declare_parameter("cmd_timeout_s", 0.5);
    declare_parameter("monitor_period_s", 0.5);
    declare_parameter("monitor_sdo_timeout_s", 0.5);
    declare_parameter("link_loss_timeout_s", 1.0);
    declare_parameter("can_interface", std::string("can0"));
    cmd_timeout_s_ = get_parameter("cmd_timeout_s").as_double();
    monitor_period_s_ = get_parameter("monitor_period_s").as_double();
    monitor_sdo_timeout_s_ = get_parameter("monitor_sdo_timeout_s").as_double();
    link_loss_timeout_s_ = get_parameter("link_loss_timeout_s").as_double();
    can_interface_ = get_parameter("can_interface").as_string();

    // initializing and conection
    topic_timer_ =
      this->create_wall_timer(10ms, std::bind(&Epos4_Control2_Node::timer_callback, this));

    // claude: init は spin 開始後に動くワーカースレッドへ移動。コンストラクタは
    // rclcpp::spin() より前に走るため、ここで async_send_request しても応答
    // (future) を処理できず、init→enable→csv を「待たずに連射」する旧実装は
    // ドライバ側で遷移が取りこぼされるレースになっていた(失敗モータが Homing
    // モード/Switch-On-Disabled に取り残され、片輪が動かない)。スレッドなら
    // 各サービス応答を future で待って逐次化でき、main() の spin が応答を捌く。
    init_thread_ = std::thread(&Epos4_Control2_Node::run_init_sequence, this);

    RCLCPP_INFO(get_logger(), "********************************************");
    RCLCPP_INFO(get_logger(), "maxon EPOS4 Control (velocity)");
    RCLCPP_INFO(get_logger(), "To be used with 1 EPOS4: /cia402_device_1");
    RCLCPP_INFO(get_logger(), "run first bus_config_cia402_epos4_vel.launch.py");
    RCLCPP_INFO(get_logger(), "********************************************");
  }

  ~Epos4_Control2_Node()
  {
    // claude: stop the init retry loop and join its thread before tearing down.
    stop_init_.store(true);
    if (init_thread_.joinable()) {
      init_thread_.join();
    }
    if (reenable_thread_.joinable()) {
      reenable_thread_.join();  // claude: 進行中の復帰シーケンスを回収してから終了
    }
    shutdown_node();
  }

private:
  // motor1
  double m1_value_ = 0.0;
  // bool js_arrived_m1_ = false;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m1_client_driver_init_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m1_client_driver_halt_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m1_client_driver_recover_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m1_client_driver_shutdown_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m1_client_driver_enable_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m1_client_driver_disable_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m1_client_driver_vel_mode_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m1_client_driver_csv_mode_;

  rclcpp::Client<canopen_interfaces::srv::COTargetDouble>::SharedPtr m1_client_target_;
  rclcpp::Client<canopen_interfaces::srv::CORead>::SharedPtr m1_client_sdo_read_;  // claude

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m1_publisher_;
  rclcpp::Publisher<canopen_interfaces::msg::COData>::SharedPtr m1_tpdo_publisher_;
  // rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m1_subscription_;

  // sensor_msgs::msg::JointState m1_joint_state_;

  // motor2
  double m2_value_ = 0.0;
  // bool js_arrived_m2_ = false;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m2_client_driver_init_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m2_client_driver_halt_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m2_client_driver_recover_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m2_client_driver_shutdown_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m2_client_driver_enable_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m2_client_driver_disable_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m2_client_driver_vel_mode_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m2_client_driver_csv_mode_;

  rclcpp::Client<canopen_interfaces::srv::COTargetDouble>::SharedPtr m2_client_target_;
  rclcpp::Client<canopen_interfaces::srv::CORead>::SharedPtr m2_client_sdo_read_;  // claude

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m2_publisher_;
  rclcpp::Publisher<canopen_interfaces::msg::COData>::SharedPtr m2_tpdo_publisher_;
  // rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m2_subscription_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_speed_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr free_mode_subscription_;  // claude
  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoder_publisher_;

  // sensor_msgs::msg::JointState m2_joint_state_;

  //
  rclcpp::TimerBase::SharedPtr topic_timer_;

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

  //YAMLparameter
  double tread_width_;
  double tire_diam_;
  double gear_ratio_;
  bool invert_left_;
  bool invert_right_;

  // claude: 加減速ランプ。m*_value_ は「要求ターゲット」(cmdSpeedCallback が即時更新)、
  // m*_cmd_ は「実際に tpdo へ出す値」で、timer_callback が毎 tick ステップ制限付きで
  // 要求へ近づける。両者とも executor スレッド上でのみ触るので排他は不要。
  double m1_cmd_ = 0.0;
  double m2_cmd_ = 0.0;
  double accel_step_;  // [rpm/tick] 加速側(|rpm| が増える向き)の上限
  double decel_step_;  // [rpm/tick] 減速側(0 へ向かう・符号反転を跨ぐ向き)の上限

  // claude_watchdog: 指令ウォッチドッグ (executor スレッド専用)
  double cmd_timeout_s_ = 0.5;
  bool cmd_received_ = false;
  std::atomic<bool> cmd_timed_out_{false};  // 途絶で 0 にした状態 (再受信で解除。監視スレッドが diagnostics に載せるため atomic)
  std::chrono::steady_clock::time_point last_cmd_time_{};

  // claude_watchdog: リンク喪失時に監視スレッドが立て、timer_callback が目標を 0 に落とす。
  // 監視スレッドは m*_value_ を直接触らない (executor スレッドとのデータ競合を避ける)。
  std::atomic<bool> force_zero_{false};

  // claude_watchdog: 監視ループの設定と状態 (init_thread_ 上でのみ触る)
  double monitor_period_s_ = 0.5;
  double monitor_sdo_timeout_s_ = 0.5;
  double link_loss_timeout_s_ = 1.0;
  std::string can_interface_ = "can0";
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // claude_watchdog: ~/rpdo から拾った statusword と受信時刻。executor が書き、監視スレッドが読む。
  struct RpdoState
  {
    std::atomic<uint32_t> statusword{0};
    std::atomic<int64_t> last_ns{0};  // steady_clock, 0 = 未受信
  };
  RpdoState m1_rpdo_;
  RpdoState m2_rpdo_;
  rclcpp::Subscription<canopen_interfaces::msg::COData>::SharedPtr m1_rpdo_sub_;
  rclcpp::Subscription<canopen_interfaces::msg::COData>::SharedPtr m2_rpdo_sub_;

  static void on_rpdo(RpdoState & st, const canopen_interfaces::msg::COData & msg)
  {
    if (msg.index != 0x6041) {
      return;
    }
    st.statusword.store(msg.data);
    st.last_ns.store(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch())
        .count());
  }

  struct MotorMonitor
  {
    MotorMonitor(
      std::string n, std::string h, rclcpp::Client<canopen_interfaces::srv::CORead>::SharedPtr s,
      RpdoState * r)
    : name(std::move(n)), hardware_id(std::move(h)), sdo(std::move(s)), rpdo(r)
    {
    }
    std::string name;
    std::string hardware_id;
    rclcpp::Client<canopen_interfaces::srv::CORead>::SharedPtr sdo;
    RpdoState * rpdo;
    double stale_s = 0.0;  // 最終 PDO からの経過 [s] (未受信なら大きな値)
    bool alive = false;
    bool fault_latched = false;
    std::optional<uint32_t> statusword;
    uint32_t error_code = 0;
    std::vector<uint32_t> history;
    double supply_v = 0.0;
  };

  void shutdown_node()
  {
    m1_value_ = 0.0;
    m2_value_ = 0.0;
    RCLCPP_INFO(get_logger(), "Sending disable command before exit...");
    call_trigger_service(m1_client_driver_disable_, "disable");
    call_trigger_service(m2_client_driver_disable_, "disable");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    RCLCPP_INFO(get_logger(), "Exit node ");
    rclcpp::shutdown();
  }

  // claude: current(現在出力)を target(要求)へ 1 tick 分だけ近づける slew rate limiter。
  // |rpm| が増える向き(同符号での増速)は accel_step_、それ以外(減速・符号反転)は
  // decel_step_ で制限する。符号反転は「decel で 0 を跨ぎ、その後 accel で加速」になる。
  double ramp_toward(double current, double target) const
  {
    const bool speeding_up = (target * current >= 0.0) && (std::abs(target) > std::abs(current));
    const double step = speeding_up ? accel_step_ : decel_step_;
    const double delta = target - current;
    if (std::abs(delta) <= step) {
      return target;
    }
    return current + std::copysign(step, delta);
  }

  void timer_callback()
  {
    // claude_watchdog: (a) 監視スレッドがリンク喪失を検知していれば目標を 0 に固定。
    //   PDO が届く経路が生きていれば EPOS はこの 0 を受けて止まる。届かなければ
    //   EPOS 側の 0x8250 が止める (二重防御)。
    if (force_zero_.load()) {
      m1_value_ = 0.0;
      m2_value_ = 0.0;
    } else if (cmd_timeout_s_ > 0.0 && cmd_received_) {
      // (b) 指令途絶: 最後の Twist から cmd_timeout_s 経過し、目標が非ゼロなら 0 へ。
      //   teleop が LB 離しでゼロを出して沈黙する通常運用では目標が既に 0 なので何もしない。
      const double since_cmd =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_cmd_time_).count();
      if (since_cmd > cmd_timeout_s_ && (m1_value_ != 0.0 || m2_value_ != 0.0)) {
        RCLCPP_WARN(
          get_logger(),
          "指令ウォッチドッグ: /robot_speed_cmd が %.2f s 途絶 (最後の目標 m1=%.0f m2=%.0f rpm) → 0 へランプ",
          since_cmd, m1_value_, m2_value_);
        m1_value_ = 0.0;
        m2_value_ = 0.0;
        cmd_timed_out_ = true;
      }
    }

    // claude: 要求ターゲットへランプしながら追従(急変指令をここで滑らかにする)
    m1_cmd_ = ramp_toward(m1_cmd_, m1_value_);
    m2_cmd_ = ramp_toward(m2_cmd_, m2_value_);

    auto m1_msg = canopen_interfaces::msg::COData();
    m1_msg.index = 0x60ff;
    m1_msg.subindex = 0x00;
    m1_msg.data = static_cast<int>(m1_cmd_);
    m1_tpdo_publisher_->publish(m1_msg);

    auto m2_msg = canopen_interfaces::msg::COData();
    m2_msg.index = 0x60ff;
    m2_msg.subindex = 0x00;
    m2_msg.data = static_cast<int>(m2_cmd_);
    m2_tpdo_publisher_->publish(m2_msg);

    // if (js_arrived_m1_ && js_arrived_m2_ &&
    //     !m1_joint_state_.position.empty() && !m2_joint_state_.position.empty())
    // {
    //     int count_enc_m1 = m1_joint_state_.position[0];
    //     int count_enc_m2 = m1_joint_state_.position[0];
    //     auto encoder_msg = sensor_msgs::msg::JointState();
    //     encoder_msg.header.stamp = this->now();
    //     encoder_msg.name = {"m1_wheel", "m2_wheel"};
    //     encoder_msg.position = {m1_joint_state_.position[0], m2_joint_state_.position[0]};
    //
    //     if (!m1_joint_state_.velocity.empty() && !m2_joint_state_.velocity.empty())
    //     {//velocity topic
    //         encoder_msg.velocity = {m1_joint_state_.velocity[0], m2_joint_state_.velocity[0]};
    //     }
    //
    //     encoder_publisher_->publish(encoder_msg);
    // }
  }

  void cmdSpeedCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // claude_watchdog: 受信時刻を記録 (途絶判定の基準)。途絶から復帰したら 1 行だけ知らせる。
    last_cmd_time_ = std::chrono::steady_clock::now();
    cmd_received_ = true;
    if (cmd_timed_out_) {
      RCLCPP_INFO(get_logger(), "指令ウォッチドッグ: /robot_speed_cmd の受信が再開");
      cmd_timed_out_ = false;
    }
    // claude: 脱力中は速度指令を一切反映しない(ターゲットは 0 のまま)。
    if (free_mode_) {
      return;
    }
    double x = msg->linear.x;
    double yaw = msg->angular.z;
    //Write IK here!!
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
    m1_value_ = invert_right_ ? -rpm_right : rpm_right;  // claude_swap: motor1 -> RIGHT wheel
    m2_value_ = invert_left_ ? -rpm_left : rpm_left;     // claude_swap: motor2 -> LEFT wheel
  }

  // void jointStateCallback_m1(const sensor_msgs::msg::JointState::SharedPtr msg)
  // {
  //     m1_joint_state_ = *msg;
  //     js_arrived_m1_ = true;
  // }

  // void jointStateCallback_m2(const sensor_msgs::msg::JointState::SharedPtr msg)
  // {
  //     m2_joint_state_ = *msg;
  //     js_arrived_m2_ = true;
  // }

  void trigger_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
  {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(get_logger(), "Service call successful: %s", response->message.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Service  call failed: %s", response->message.c_str());
    }
  }

  void call_trigger_service(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client, const std::string & service_name)
  {
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(get_logger(), "Service %s not available", service_name.c_str());
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(
      request, std::bind(&Epos4_Control2_Node::trigger_callback, this, std::placeholders::_1));
  }

  // claude: synchronous Trigger call. Sends the request and blocks (in the init
  // worker thread, NOT the executor thread) until the response future is ready,
  // so init→enable→csv can be issued strictly one-after-another instead of being
  // fired all at once. Returns the driver's success flag.
  bool call_trigger_sync(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client, const std::string & name,
    std::chrono::seconds timeout = 5s)
  {
    if (!client->wait_for_service(2s)) {
      RCLCPP_ERROR(get_logger(), "Service %s not available", name.c_str());
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      RCLCPP_WARN(
        get_logger(), "%s: no response within %lds", name.c_str(),
        static_cast<long>(timeout.count()));
      return false;
    }
    return future.get()->success;
  }

  // claude: blocking SDO upload (read) of an object, returns nullopt on failure.
  // claude_watchdog: subindex と待ち時間を引数化 (0x1003:01〜05 の履歴読み、監視用の短い待ち)。
  std::optional<uint32_t> read_sdo(
    rclcpp::Client<canopen_interfaces::srv::CORead>::SharedPtr client, uint16_t index,
    uint8_t subindex = 0, std::chrono::milliseconds timeout = 2000ms)
  {
    if (!client->wait_for_service(timeout)) {
      return std::nullopt;
    }
    auto request = std::make_shared<canopen_interfaces::srv::CORead::Request>();
    request->index = index;
    request->subindex = subindex;
    auto future = client->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
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
  bool motor_ready(
    rclcpp::Client<canopen_interfaces::srv::CORead>::SharedPtr sdo_client, const std::string & name)
  {
    auto statusword = read_sdo(sdo_client, 0x6041);
    auto mode = read_sdo(sdo_client, 0x6061);
    if (!statusword.has_value() || !mode.has_value()) {
      RCLCPP_WARN(get_logger(), "%s: could not read statusword/mode via SDO", name.c_str());
      return false;
    }
    const bool op_enabled = ((statusword.value() & 0x6F) == 0x27);
    const bool csv_mode = (static_cast<int8_t>(mode.value()) == 9);
    RCLCPP_INFO(
      get_logger(), "%s: statusword=0x%04X mode=%d (op_enabled=%d csv=%d)", name.c_str(),
      statusword.value(), static_cast<int8_t>(mode.value()), op_enabled, csv_mode);
    return op_enabled && csv_mode;
  }

  // claude: bring a single drive to CSV / Operation-Enabled, verifying via SDO and
  // retrying (with a recover in between) on failure. This is the actual fix for the
  // intermittent dead wheel.
  void init_motor(
    const std::string & name, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr init_client,
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr enable_client,
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr csv_client,
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr recover_client,
    rclcpp::Client<canopen_interfaces::srv::CORead>::SharedPtr sdo_client)
  {
    constexpr int max_attempts = 5;
    for (int attempt = 1; attempt <= max_attempts && !stop_init_.load() && rclcpp::ok();
         ++attempt) {
      // init triggers homing, which reliably "fails" for CSV (homing not required) — expected.
      call_trigger_sync(init_client, name + " init");
      call_trigger_sync(enable_client, name + " enable");
      call_trigger_sync(csv_client, name + " cyclic_velocity_mode");
      std::this_thread::sleep_for(200ms);  // let the drive settle before reading back

      if (motor_ready(sdo_client, name)) {
        RCLCPP_INFO(
          get_logger(), "%s ready (Operation Enabled, CSV) on attempt %d", name.c_str(), attempt);
        return;
      }
      RCLCPP_WARN(
        get_logger(), "%s not ready on attempt %d/%d; recovering and retrying", name.c_str(),
        attempt, max_attempts);
      call_trigger_sync(recover_client, name + " recover");
      std::this_thread::sleep_for(300ms);
    }
    RCLCPP_ERROR(
      get_logger(),
      "%s FAILED to reach CSV Operation-Enabled after %d attempts — wheel will not move",
      name.c_str(), max_attempts);
  }

  // claude: runs in init_thread_ (after main() starts spinning, so service/SDO
  // response futures get processed). Brings up motor1 then motor2 sequentially.
  void run_init_sequence()
  {
    RCLCPP_INFO(get_logger(), "Auto-initializing EPOS4 (sequential, verified)...");
    // device_manager advertises the lifecycle services only once both drivers
    // have booted; wait for that instead of racing it.
    if (!m1_client_driver_init_->wait_for_service(20s)) {
      // claude_watchdog: init は諦めるが、監視ループは回す (EPOS 電源 OFF・boot 失敗でも
      // リンク状態と後からの復帰を /diagnostics に出せるように)。
      RCLCPP_ERROR(
        get_logger(), "motor1 init service never appeared; aborting auto-init (監視のみ継続)");
    } else {
      init_motor(
        "motor1(left)", m1_client_driver_init_, m1_client_driver_enable_,
        m1_client_driver_csv_mode_, m1_client_driver_recover_, m1_client_sdo_read_);
      init_motor(
        "motor2(right)", m2_client_driver_init_, m2_client_driver_enable_,
        m2_client_driver_csv_mode_, m2_client_driver_recover_, m2_client_sdo_read_);
      RCLCPP_INFO(get_logger(), "EPOS4 auto-init complete.");
    }

    // claude_watchdog: 初期化が終わったら同じスレッドで監視ループに入る
    run_monitor_loop();
  }

  // claude_watchdog: EPOS4 Firmware Specification §7.2 のエラーコード名 (主要なもの)。
  static const char * epos4_error_name(uint32_t code)
  {
    static const std::map<uint32_t, const char *> names = {
      {0x0000, "No error"},
      {0x1000, "Generic error"},
      {0x2310, "Overcurrent"},
      {0x2320, "Power stage protection"},
      {0x3210, "Overvoltage (power supply)"},
      {0x3220, "Undervoltage (power supply cannot supply acceleration current / +Vcc lost)"},
      {0x4210, "Thermal overload (power stage)"},
      {0x4380, "Thermal motor overload"},
      {0x5113, "Logic supply voltage too low"},
      {0x5280, "Hardware defect"},
      {0x6320, "Software parameter error"},
      {0x7320, "Position sensor error"},
      {0x8110, "CAN overrun (objects lost)"},
      {0x8120, "CAN passive mode"},
      {0x8130, "Heartbeat error"},
      {0x81FD, "CAN bus off"},
      {0x81FE, "CAN Rx queue overflow"},
      {0x81FF, "CAN Tx queue overflow"},
      {0x8250, "RPDO timeout (no target PDO within interpolation period = master/link lost)"},
      {0x8611, "Following error"},
    };
    auto it = names.find(code & 0xFFFF);
    return it == names.end() ? "(unknown, see EPOS4 Firmware Spec §7.2)" : it->second;
  }

  // claude_watchdog: /sys/class/net/<if>/ifindex を読む。無ければ -1 (インタフェース消滅)。
  int read_can_ifindex() const
  {
    std::ifstream f("/sys/class/net/" + can_interface_ + "/ifindex");
    int idx = -1;
    if (!(f >> idx)) {
      return -1;
    }
    return idx;
  }

  // claude_watchdog: 1 ノード分の状態評価とフォルト時のコード即読み。
  // statusword は ~/rpdo 購読 (TPDO1) から取る (SDO ポーリングなし)。Fault ビット (bit 3) の
  // 立ち上がりでだけ SDO で 0x603F / 0x1003:00〜05 / 0x2200:01 を読み、電源を切る前にコードを
  // ログへ残す (0x1003 は揮発、ros2_canopen は EMCY をログに出さない)。
  void poll_motor(MotorMonitor & m)
  {
    const auto to = std::chrono::milliseconds(static_cast<int>(monitor_sdo_timeout_s_ * 1000.0));
    const int64_t last = m.rpdo->last_ns.load();
    if (last == 0) {
      m.stale_s = 1e9;  // 一度も PDO を受けていない
      m.alive = false;
      m.statusword.reset();
      return;
    }
    const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::steady_clock::now().time_since_epoch())
                             .count();
    m.stale_s = static_cast<double>(now_ns - last) * 1e-9;
    m.alive = (m.stale_s <= link_loss_timeout_s_);
    m.statusword = m.rpdo->statusword.load();
    if (!m.alive) {
      return;  // 古い statusword で Fault 判定はしない
    }
    const bool fault = (m.statusword.value() & 0x0008) != 0;
    if (fault && !m.fault_latched) {
      m.fault_latched = true;
      m.error_code = read_sdo(m.sdo, 0x603F, 0, to).value_or(0);
      m.history.clear();
      const uint32_t n = read_sdo(m.sdo, 0x1003, 0, to).value_or(0);
      for (uint32_t i = 1; i <= std::min<uint32_t>(n, 5); ++i) {
        m.history.push_back(read_sdo(m.sdo, 0x1003, static_cast<uint8_t>(i), to).value_or(0) & 0xFFFF);
      }
      m.supply_v = read_sdo(m.sdo, 0x2200, 1, to).value_or(0) / 10.0;
      std::string hist;
      for (auto h : m.history) {
        char buf[16];
        std::snprintf(buf, sizeof(buf), " 0x%04X", h);
        hist += buf;
      }
      RCLCPP_ERROR(
        get_logger(),
        "%s FAULT: statusword=0x%04X error=0x%04X (%s) history[%u]=[%s ] Vcc=%.1f V "
        "— コードは記録済み。復帰は recover/再起動、原因は docs/issue 参照",
        m.name.c_str(), m.statusword.value(), m.error_code, epos4_error_name(m.error_code), n,
        hist.c_str(), m.supply_v);
    } else if (!fault && m.fault_latched) {
      m.fault_latched = false;
      RCLCPP_INFO(get_logger(), "%s: Fault cleared (statusword=0x%04X)", m.name.c_str(),
                  m.statusword.value());
    }
  }

  // claude_watchdog: 監視ループ (init_thread_ 上)。
  //   1. can0 の ifindex 監視: 変化 = USB 再列挙で can0 が作り直された。ros2_canopen master は
  //      旧 socket を掴んだままなので通信は二度と戻らない → ERROR で再起動を促し目標を 0 に。
  //   2. 両ノードの PDO (statusword) が link_loss_timeout_s 途絶 = PC↔CAN リンク喪失 → 目標を 0 に
  //      (送信経路だけ生きていれば EPOS はこの 0 を受けて止まる)。
  //   3. 各ノードのフォルト検知とコード記録 (poll_motor)。
  //   4. 以上を /diagnostics に流す (bag に残る)。
  void run_monitor_loop()
  {
    MotorMonitor m1{"motor1(right)", "EPOS4 node 1", m1_client_sdo_read_, &m1_rpdo_};
    MotorMonitor m2{"motor2(left)", "EPOS4 node 2", m2_client_sdo_read_, &m2_rpdo_};
    const int ifindex_at_start = read_can_ifindex();
    bool link_lost_prev = false;
    auto last_link_log = std::chrono::steady_clock::now() - 10s;
    RCLCPP_INFO(
      get_logger(),
      "監視ループ開始: %s ifindex=%d, period=%.2f s, cmd_timeout=%.2f s, link_loss_timeout=%.2f s "
      "(statusword は ~/rpdo 購読、SDO は Fault 時のみ)",
      can_interface_.c_str(), ifindex_at_start, monitor_period_s_, cmd_timeout_s_,
      link_loss_timeout_s_);

    while (!stop_init_.load() && rclcpp::ok()) {
      std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(monitor_period_s_ * 1000.0)));
      if (stop_init_.load() || !rclcpp::ok()) {
        break;
      }

      const int ifindex_now = read_can_ifindex();
      const bool can_gone = (ifindex_now < 0);
      const bool can_recreated =
        (ifindex_at_start > 0 && ifindex_now > 0 && ifindex_now != ifindex_at_start);

      poll_motor(m1);
      poll_motor(m2);
      const bool pdo_dead = (!m1.alive && !m2.alive);

      const bool link_lost = can_gone || can_recreated || pdo_dead;
      force_zero_.store(link_lost);

      std::string reason;
      if (can_gone) {
        reason = can_interface_ + " が消滅 (USB アダプタ切断?)";
      } else if (can_recreated) {
        reason = can_interface_ + " が作り直された (ifindex " + std::to_string(ifindex_at_start) +
                 " → " + std::to_string(ifindex_now) +
                 "): master は旧 socket のまま → scripts/stop.sh → 再 launch が必要";
      } else if (pdo_dead) {
        char buf[96];
        std::snprintf(
          buf, sizeof(buf), "両ノードの PDO が途絶 (m1 %.1f s / m2 %.1f s > %.1f s)",
          std::min(m1.stale_s, 1e6), std::min(m2.stale_s, 1e6), link_loss_timeout_s_);
        reason = std::string(buf) + " (CANUSB ストール / CAN 配線 / EPOS 電源断 / ドライバ未 activate)";
      }

      const auto now = std::chrono::steady_clock::now();
      if (link_lost && (!link_lost_prev || now - last_link_log > 2s)) {
        RCLCPP_ERROR(
          get_logger(), "CAN リンク喪失: %s → 目標速度を 0 に固定 (EPOS 側は 0x8250 で自衛)",
          reason.c_str());
        last_link_log = now;
      } else if (!link_lost && link_lost_prev) {
        RCLCPP_WARN(get_logger(), "CAN リンク復帰 (SDO 応答あり)。目標 0 から再開");
      }
      link_lost_prev = link_lost;

      publish_diagnostics(m1, m2, link_lost, reason, ifindex_now);
    }
  }

  void publish_diagnostics(
    const MotorMonitor & m1, const MotorMonitor & m2, bool link_lost, const std::string & reason,
    int ifindex_now)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    using diagnostic_msgs::msg::KeyValue;
    auto kv = [](const std::string & k, const std::string & v) {
      KeyValue x;
      x.key = k;
      x.value = v;
      return x;
    };
    auto hex = [](uint32_t v) {
      char buf[16];
      std::snprintf(buf, sizeof(buf), "0x%04X", v & 0xFFFF);
      return std::string(buf);
    };

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = this->now();

    DiagnosticStatus link;
    link.name = "epos4_controller/can_link";
    link.hardware_id = can_interface_;
    link.level = link_lost ? DiagnosticStatus::ERROR : DiagnosticStatus::OK;
    link.message = link_lost ? reason : "OK";
    link.values.push_back(kv("ifindex", std::to_string(ifindex_now)));
    link.values.push_back(kv("force_zero", force_zero_.load() ? "true" : "false"));
    link.values.push_back(kv("cmd_timed_out", cmd_timed_out_ ? "true" : "false"));
    arr.status.push_back(link);

    for (const MotorMonitor * m : {&m1, &m2}) {
      DiagnosticStatus s;
      s.name = "epos4_controller/" + m->name;
      s.hardware_id = m->hardware_id;
      if (!m->alive) {
        s.level = DiagnosticStatus::STALE;
        s.message = (m->rpdo->last_ns.load() == 0)
                      ? "PDO 未受信 (ドライバ未 activate / EPOS 無応答)"
                      : "PDO 途絶 " + std::to_string(m->stale_s).substr(0, 5) + " s";
      } else if (m->fault_latched) {
        s.level = DiagnosticStatus::ERROR;
        s.message = "FAULT " + hex(m->error_code) + " " + epos4_error_name(m->error_code);
      } else {
        s.level = DiagnosticStatus::OK;
        s.message = "statusword " + hex(m->statusword.value());
      }
      if (m->statusword.has_value()) {
        s.values.push_back(kv("statusword", hex(m->statusword.value())));
      }
      if (m->fault_latched) {
        s.values.push_back(kv("error_code", hex(m->error_code)));
        std::string hist;
        for (auto h : m->history) {
          hist += hex(h) + " ";
        }
        s.values.push_back(kv("error_history", hist));
        s.values.push_back(kv("supply_voltage_V", std::to_string(m->supply_v)));
      }
      arr.status.push_back(s);
    }
    diag_pub_->publish(arr);
  }

  // claude: /robot_free_mode の受信ハンドラ。data=true で脱力 ON、false で復帰。
  // 脱力 ON: disable を非同期で投げるだけ(励磁を切るだけなので順序不問)。
  // 復帰   : enable→cyclic_velocity_mode は逐次に効かせないと mode 設定がレースして
  //          片輪が CSV に入りきらず遅れる。短命スレッドで sync 呼び出しを順番に行う
  //          (init=homing は呼ばない。再 init は復帰失敗の原因になるため)。
  void freeModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    free_mode_ = msg->data;
    m1_value_ = 0.0;  // 脱力中も復帰直後も指令を 0 から始める
    m2_value_ = 0.0;
    m1_cmd_ = 0.0;  // claude: ランプ出力も 0 リセット(非励磁中に旧値から滑らかに
    m2_cmd_ = 0.0;  //         下げる意味はなく、復帰時に旧速度が残ると危険)
    if (free_mode_) {
      RCLCPP_INFO(get_logger(), "脱力モード ON: disabling both motors (free wheel)");
      call_trigger_service(m1_client_driver_disable_, "disable");
      call_trigger_service(m2_client_driver_disable_, "disable");
    } else {
      RCLCPP_INFO(get_logger(), "脱力モード OFF: re-enabling both motors (enable + CSV)");
      if (reenable_thread_.joinable()) {
        reenable_thread_.join();  // 直前の復帰は完了済み、即座に返る
      }
      reenable_thread_ = std::thread([this] {
        call_trigger_sync(m1_client_driver_enable_, "motor1 enable");
        call_trigger_sync(m1_client_driver_csv_mode_, "motor1 cyclic_velocity_mode");
        call_trigger_sync(m2_client_driver_enable_, "motor2 enable");
        call_trigger_sync(m2_client_driver_csv_mode_, "motor2 cyclic_velocity_mode");
        // 成否はサービス戻り値ではなく実ドライブ状態で判定する。enable/csv が
        // no-op 遷移のときドライバが success=false を返すことがあり(偽陰性)、
        // 戻り値だけ見ると正常復帰でも失敗扱いになるため。motor_ready は
        // SDO で statusword(0x6041)/mode(0x6061)を読み、Operation Enabled & CSV を確認する。
        std::this_thread::sleep_for(200ms);  // 読み戻し前にドライブを落ち着かせる
        const bool m1 = motor_ready(m1_client_sdo_read_, "motor1(left)");
        const bool m2 = motor_ready(m2_client_sdo_read_, "motor2(right)");
        if (m1 && m2) {
          RCLCPP_INFO(get_logger(), "脱力モード OFF: both motors re-enabled (CSV)");
        } else {
          RCLCPP_WARN(
            get_logger(),
            "脱力モード OFF: re-enable incomplete (m1_ready=%d m2_ready=%d); toggle f to retry", m1,
            m2);
        }
      });
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Epos4_Control2_Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}