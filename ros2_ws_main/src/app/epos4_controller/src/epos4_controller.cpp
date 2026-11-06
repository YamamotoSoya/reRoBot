#include <linux/can.h>      // claude_recover: EMCY 受動受信 (raw SocketCAN)
#include <linux/can/raw.h>  // claude_recover
#include <net/if.h>         // claude_recover
#include <sys/ioctl.h>      // claude_recover
#include <sys/socket.h>     // claude_recover
#include <sys/time.h>       // claude_recover
#include <unistd.h>         // claude_recover

#include <algorithm>  // claude_watchdog: std::min
#include <atomic>     // claude
#include <cerrno>     // claude_recover
#include <chrono>
#include <cmath>
#include <cstdio>   // claude_watchdog: std::snprintf
#include <cstring>  // claude_recover
#include <deque>    // claude_recover
#include <fstream>  // claude_watchdog: /sys/class/net/<can>/ifindex 読み取り
#include <future>   // claude
#include <iostream>
#include <map>  // claude_watchdog: EPOS4 エラーコード名テーブル
#include <memory>
#include <mutex>     // claude_recover
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
#include "sensor_msgs/msg/joy.hpp"  // claude_recover: Start (走行許可) / LB (DISARMED 中の joy)
#include "std_msgs/msg/bool.hpp"    // claude: 脱力(フリー)モードのトグル受信用
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"  // claude_recover: /epos4_controller/state
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

    // claude_recover (2026-11-06): 旧監視は ~/rpdo の statusword を使っていたが、ros2_canopen の
    // rpdo キューは 50 ms ごとに最大 10 件しか取り出されず (TPDO は 300 件/s)、起動時間とともに
    // 遅れが伸びる (11-04 bag で 10 分遅れ、issue 2026-10-22 §11.2)。Fault 検出は EMCY の受動受信
    // (emcy_loop) と SDO に置き換え、~/rpdo は購読しない。
    // 設計: docs/features/2026-11-05_epos_auto_recovery_plan.md

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
    //   PDO 途絶そのものは EPOS4 側の heartbeat 監視 (0x1016, 500 ms → 0x8130 quick stop) が止めるが、
    //   (claude 2026-11-06: 以前は「0x8250 で自衛」としていたが、台上で PDO 途絶 10 s・USB 抜去でも
    //    0x8250 は出ず回転継続を確認 → bus.yml に heartbeat を追加して担わせた)
    //   PDO が届き続ける形 (joy のデバイス喪失で teleop が publish 停止、master の受信側のみ
    //   死亡) では最後の非ゼロ指令が無期限に送られ続ける (2026-09-19 14:57 の暴走)。
    //   - cmd_timeout_s: /robot_speed_cmd がこの秒数途絶し目標が非ゼロなら 0 へランプ (0 で無効)
    //   - monitor_period_s: 監視ループ (状態機械) の評価周期
    //   - monitor_sdo_timeout_s: Fault 検知時に Vcc / コードを読む SDO 1 回の待ち時間
    //   - can_interface: /sys/class/net/<if>/ifindex を監視。値が変わる = can0 が作り直された
    //     (USB 再列挙) が ros2_canopen master は旧 can0 を掴んだまま → スタック再起動が必要
    declare_parameter("cmd_timeout_s", 0.5);
    declare_parameter("monitor_period_s", 0.05);
    declare_parameter("monitor_sdo_timeout_s", 0.3);
    declare_parameter("can_interface", std::string("can0"));
    cmd_timeout_s_ = get_parameter("cmd_timeout_s").as_double();
    monitor_period_s_ = get_parameter("monitor_period_s").as_double();
    monitor_sdo_timeout_s_ = get_parameter("monitor_sdo_timeout_s").as_double();
    can_interface_ = get_parameter("can_interface").as_string();

    // claude_recover (2026-11-06): 自動復帰と走行許可 (DISARMED)。値の意味は params.yaml と設計文書 §3。
    declare_parameter("vcc_ok_v", 20.0);
    declare_parameter("vcc_read_deadline_s", 1.0);
    declare_parameter("recover_window_s", 120.0);
    declare_parameter("recover_per_window", 3);
    declare_parameter("recover_per_run", 10);
    declare_parameter("auto_recover_8250", true);
    declare_parameter("recover_8250_per_run", 3);
    declare_parameter("refault_8250_s", 30.0);
    declare_parameter("long_fault_s", 5.0);
    declare_parameter("executor_stall_s", 0.3);
    declare_parameter("stall_ratio", 0.3);
    declare_parameter("stall_window_s", 0.3);
    declare_parameter("stall_min_rpm", 300.0);
    declare_parameter("allow_joy_when_disarmed", true);
    declare_parameter("joy_fresh_s", 0.2);
    declare_parameter("arm_button", 7);
    declare_parameter("enable_button", 4);
    declare_parameter("enable_turbo_button", 5);  // RB: teleop_twist_joy では RB 単独でも turbo の deadman
    declare_parameter("joy_buttons", 11);
    declare_parameter("joy_axes", 8);
    declare_parameter("joy_topic", std::string("/joy"));
    declare_parameter("joy_vel_topic", std::string("/joy_vel"));
    vcc_ok_v_ = get_parameter("vcc_ok_v").as_double();
    vcc_read_deadline_s_ = get_parameter("vcc_read_deadline_s").as_double();
    recover_window_s_ = get_parameter("recover_window_s").as_double();
    recover_per_window_ = get_parameter("recover_per_window").as_int();
    recover_per_run_ = get_parameter("recover_per_run").as_int();
    auto_recover_8250_ = get_parameter("auto_recover_8250").as_bool();
    recover_8250_per_run_ = get_parameter("recover_8250_per_run").as_int();
    refault_8250_s_ = get_parameter("refault_8250_s").as_double();
    long_fault_s_ = get_parameter("long_fault_s").as_double();
    executor_stall_s_ = get_parameter("executor_stall_s").as_double();
    stall_ratio_ = get_parameter("stall_ratio").as_double();
    stall_window_s_ = get_parameter("stall_window_s").as_double();
    stall_min_rpm_ = get_parameter("stall_min_rpm").as_double();
    allow_joy_when_disarmed_ = get_parameter("allow_joy_when_disarmed").as_bool();
    joy_fresh_s_ = get_parameter("joy_fresh_s").as_double();
    arm_button_ = get_parameter("arm_button").as_int();
    enable_button_ = get_parameter("enable_button").as_int();
    enable_turbo_button_ = get_parameter("enable_turbo_button").as_int();
    joy_buttons_ = get_parameter("joy_buttons").as_int();
    joy_axes_ = get_parameter("joy_axes").as_int();

    // claude_recover: DISARMED 中の例外 (LB を握った joy) と走行許可 (Start) のための入力。
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      get_parameter("joy_topic").as_string(), 10,
      std::bind(&Epos4_Control2_Node::joyCallback, this, std::placeholders::_1));
    joy_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      get_parameter("joy_vel_topic").as_string(), 10,
      std::bind(&Epos4_Control2_Node::joyVelCallback, this, std::placeholders::_1));
    // claude_recover: 物理トリガ (指令に対して車輪が回っていない) 用。位置は driver が辞書を直接読むので遅れない。
    m1_js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/motor1/cia402_device_1/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_state(0, *msg); });
    m2_js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/motor2/cia402_device_2/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_state(1, *msg); });
    // claude_recover: 状態は transient_local (後から購読した runner / RViz にも最新が届く)
    state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/epos4_controller/state", rclcpp::QoS(1).reliable().transient_local());
    arm_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/epos4_controller/arm",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        arm_request_.store(true);
        res->success = true;
        res->message = "arm requested (current: " + state_text() + ")";
      });
    emcy_thread_ = std::thread(&Epos4_Control2_Node::emcy_loop, this);

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
    if (emcy_thread_.joinable()) {
      emcy_thread_.join();  // claude_recover: stop_init_ で抜ける (受信は 200 ms タイムアウト)
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

  // claude_watchdog: 監視ループの設定 (init_thread_ 上でのみ触る)
  double monitor_period_s_ = 0.05;
  double monitor_sdo_timeout_s_ = 0.3;
  std::string can_interface_ = "can0";
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // ===== claude_recover (2026-11-06): 自動復帰と走行許可 =====
  // 状態機械。監視スレッドが書き、timer_callback (executor) が読んで出力を決める。
  //   DISARMED   : 指令 0 固定 (例外: LB を握った新しい joy)。起動直後・非常停止の後・自動復帰しない Fault の後
  //   RUNNING    : /robot_speed_cmd をそのまま通す
  //   RECOVERING : Fault を検知して分類・recover 中。指令 0
  //   WAIT_POWER : 非常停止 (0x3220 で Vcc が低い)。電源復帰を待ってから enable だけして DISARMED へ
  enum State : int { DISARMED = 0, RUNNING = 1, RECOVERING = 2, WAIT_POWER = 3 };
  std::atomic<int> state_{DISARMED};
  std::mutex state_mtx_;  // state_reason_ 用
  std::string state_reason_ = "起動中";

  // パラメータ (コンストラクタで確定、以後読むだけ)
  double vcc_ok_v_ = 20.0;
  double vcc_read_deadline_s_ = 1.0;
  double recover_window_s_ = 120.0;
  int64_t recover_per_window_ = 3;
  int64_t recover_per_run_ = 10;
  bool auto_recover_8250_ = true;
  int64_t recover_8250_per_run_ = 3;
  double refault_8250_s_ = 30.0;
  double long_fault_s_ = 5.0;
  double executor_stall_s_ = 0.3;
  double stall_ratio_ = 0.3;
  double stall_window_s_ = 0.3;
  double stall_min_rpm_ = 300.0;
  bool allow_joy_when_disarmed_ = true;
  double joy_fresh_s_ = 0.2;
  int64_t arm_button_ = 7;
  int64_t enable_button_ = 4;
  int64_t enable_turbo_button_ = 5;
  int64_t joy_buttons_ = 11;
  int64_t joy_axes_ = 8;

  // 監視スレッド ← 他スレッドの受け渡し
  std::atomic<bool> arm_request_{false};  // Start (DISARMED 中の押し下げ) / ~/arm
  std::atomic<bool> reset_ramp_{false};   // 立てると timer_callback がランプ出力を 0 に戻す
  std::atomic<int64_t> last_tick_ns_{0};  // timer_callback の最終実行 (executor 停止監視)
  std::atomic<double> cmd_out_[2] = {0.0, 0.0};     // ランプ後の出力 [rpm] (物理トリガ用)
  std::atomic<double> actual_rpm_[2] = {0.0, 0.0};  // joint_states の位置差分から出した実速度 [rpm]
  std::atomic<int64_t> js_ns_[2] = {0, 0};          // joint_states の受信時刻

  // joint_states の前回値 (executor 専用)
  double js_prev_pos_[2] = {0.0, 0.0};
  int64_t js_prev_ns_[2] = {0, 0};

  // joy (executor 専用)。DISARMED 中は /joy_vel を直接読み、鮮度 joy_fresh_s 以内のものだけ通す
  // (/robot_speed_cmd だと、LB を握ったままパッドが瞬断したとき twist_mux が Nav2 に落ちた指令が通る)
  double joy_m1_ = 0.0;
  double joy_m2_ = 0.0;
  std::chrono::steady_clock::time_point joy_vel_time_{};
  std::chrono::steady_clock::time_point joy_time_{};
  bool joy_lb_ = false;
  bool joy_prev_start_ = true;  // 「押されている」から始める = 一度離れを見るまで Start を数えない
  bool joy_shape_warned_ = false;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m1_js_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m2_js_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_srv_;

  // EMCY 受動受信 (emcy_thread_ が積み、監視スレッドが取り出す)
  struct EmcyEvent
  {
    int motor;      // 0 = motor1 (node 1, COB 0x081) / 1 = motor2 (node 2, COB 0x082)
    uint16_t code;  // 0x0000 = エラー解除の通知
    int64_t ns;     // 受信時刻 (steady)
  };
  std::thread emcy_thread_;
  std::mutex emcy_mtx_;
  std::deque<EmcyEvent> emcy_events_;
  std::atomic<bool> emcy_socket_ok_{false};

  // 監視スレッド専用の状態
  struct MotorMonitor
  {
    MotorMonitor(
      std::string n, std::string h, rclcpp::Client<canopen_interfaces::srv::CORead>::SharedPtr s,
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr r)
    : name(std::move(n)), hardware_id(std::move(h)), sdo(std::move(s)), recover(std::move(r))
    {
    }
    std::string name;
    std::string hardware_id;
    rclcpp::Client<canopen_interfaces::srv::CORead>::SharedPtr sdo;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr recover;
    std::optional<uint32_t> statusword;  // 最後に SDO で読んだ値
    bool fault = false;                  // 今回の Fault エピソードで Fault していた
    uint32_t error_code = 0;
    std::vector<uint32_t> history;
    double supply_v = -1.0;  // 最後に読んだ Vcc [V] (-1 = 未読)
    int64_t emcy_ns = 0;     // 今回の Fault の EMCY 受信時刻 (0 = EMCY なし = 時刻不明)
    uint16_t emcy_code = 0;
    int64_t stall_since_ns = 0;  // 物理トリガ: 条件が成立し始めた時刻
    int64_t phys_checked_ns = 0;  // 物理トリガ: 最後に SDO で確かめた時刻
  };

  // 予算 (監視スレッド専用)
  std::deque<int64_t> recover_times_ns_;  // 0x8130 / 0x3220 の自動復帰時刻 (窓の数え上げ)
  int64_t recover_count_run_ = 0;
  int64_t recover_8250_count_run_ = 0;
  int64_t last_8250_recover_ns_ = 0;

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
    // claude_recover: executor 停止監視の心拍と、監視スレッドからのランプ 0 リセット要求
    last_tick_ns_.store(now_ns());
    if (reset_ramp_.exchange(false)) {
      m1_cmd_ = 0.0;
      m2_cmd_ = 0.0;
    }

    // claude_watchdog: (a) 監視スレッドがリンク喪失を検知していれば目標を 0 に固定。
    //   PDO が届く経路が生きていれば EPOS はこの 0 を受けて止まる。届かなければ
    //   EPOS 側の heartbeat 監視 (0x8130、bus.yml 2026-11-06) が止める (二重防御)。
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

    // claude_recover: 走行許可で出力の目標を選ぶ。
    //   RUNNING    → /robot_speed_cmd 由来 (m*_value_)
    //   DISARMED / WAIT_POWER → 0。ただし LB を握った新しい joy (/joy_vel) だけ通す
    //   RECOVERING → 0
    double target1 = 0.0;
    double target2 = 0.0;
    const int st = state_.load();
    if (force_zero_.load() || free_mode_) {
      // 0 のまま
    } else if (st == RUNNING) {
      target1 = m1_value_;
      target2 = m2_value_;
    } else if ((st == DISARMED || st == WAIT_POWER) && joy_live()) {
      target1 = joy_m1_;
      target2 = joy_m2_;
    }

    // claude: 要求ターゲットへランプしながら追従(急変指令をここで滑らかにする)
    m1_cmd_ = ramp_toward(m1_cmd_, target1);
    m2_cmd_ = ramp_toward(m2_cmd_, target2);
    cmd_out_[0].store(m1_cmd_);  // claude_recover: 物理トリガ用
    cmd_out_[1].store(m2_cmd_);

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
    twist_to_motor_rpm(msg->linear.x, msg->angular.z, m1_value_, m2_value_);
  }

  // claude_recover: cmdSpeedCallback の逆運動学を joy 用にも使うため関数に切り出した (中身は従来どおり)
  void twist_to_motor_rpm(double x, double yaw, double & m1_rpm, double & m2_rpm) const
  {
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
    m1_rpm = invert_right_ ? -rpm_right : rpm_right;  // claude_swap: motor1 -> RIGHT wheel
    m2_rpm = invert_left_ ? -rpm_left : rpm_left;     // claude_swap: motor2 -> LEFT wheel
  }

  static int64_t now_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
  }

  // ===== claude_recover: joy 入力 (executor スレッド) =====
  void joyVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    twist_to_motor_rpm(msg->linear.x, msg->angular.z, joy_m1_, joy_m2_);
    joy_vel_time_ = std::chrono::steady_clock::now();
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const auto now = std::chrono::steady_clock::now();
    // 0.5 s 以上途切れていた = パッドの再接続・joy_node の再起動。押しっぱなしの Start を数えない
    if (std::chrono::duration<double>(now - joy_time_).count() > 0.5) {
      joy_prev_start_ = true;
    }
    joy_time_ = now;
    // Xbox (XInput) 以外の構成 (DirectInput で再列挙等) はボタン番号が変わるので使わない
    if (
      static_cast<int64_t>(msg->buttons.size()) != joy_buttons_ ||
      static_cast<int64_t>(msg->axes.size()) != joy_axes_) {
      joy_lb_ = false;
      joy_prev_start_ = true;
      if (!joy_shape_warned_) {
        RCLCPP_WARN(
          get_logger(), "/joy の構成が想定 (buttons %ld, axes %ld) と違う (%zu, %zu) → 無視する",
          joy_buttons_, joy_axes_, msg->buttons.size(), msg->axes.size());
        joy_shape_warned_ = true;
      }
      return;
    }
    joy_shape_warned_ = false;
    // claude 2026-11-06: teleop_twist_joy は RB (turbo) 単独でも turbo 速度で指令を出す (RB も deadman)。
    //   LB だけを見ていたら DISARMED 中に RB 単独の操作が 0 に潰れた (11-06 20:02 bag: RB のみ 79 回中 4 回しか動かず)。
    //   人が握っているボタンなので LB / RB のどちらでも通す。
    joy_lb_ = msg->buttons[enable_button_] == 1 ||
              (enable_turbo_button_ >= 0 && enable_turbo_button_ < static_cast<int64_t>(msg->buttons.size()) &&
               msg->buttons[enable_turbo_button_] == 1);
    const bool start = msg->buttons[arm_button_] == 1;
    // 走行許可は DISARMED 中の押し下げだけ。WAIT_POWER / RECOVERING 中の押下は捨てる (溜めない)
    if (start && !joy_prev_start_ && state_.load() == DISARMED) {
      RCLCPP_INFO(get_logger(), "Start 押下 → 走行許可を要求");
      arm_request_.store(true);
    }
    joy_prev_start_ = start;
  }

  // DISARMED 中に joy を通してよいか: LB を握っていて、/joy と /joy_vel の両方が新しい
  bool joy_live() const
  {
    if (!allow_joy_when_disarmed_ || !joy_lb_) {
      return false;
    }
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - joy_time_).count() <= joy_fresh_s_ &&
           std::chrono::duration<double>(now - joy_vel_time_).count() <= joy_fresh_s_;
  }

  // claude_recover: joint_states (モータ軸 rad) の位置差分から実速度 [rpm] を出す (executor スレッド)
  void on_joint_state(int i, const sensor_msgs::msg::JointState & msg)
  {
    if (msg.position.empty()) {
      return;
    }
    const int64_t t = now_ns();
    const double pos = msg.position[0];
    if (js_prev_ns_[i] != 0) {
      const double dt = static_cast<double>(t - js_prev_ns_[i]) * 1e-9;
      if (dt > 1e-3) {
        actual_rpm_[i].store((pos - js_prev_pos_[i]) / dt * 60.0 / (2.0 * M_PI));
        js_ns_[i].store(t);
      }
    }
    js_prev_pos_[i] = pos;
    js_prev_ns_[i] = t;
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

  // ===== claude_recover (2026-11-06): 状態機械・EMCY 受動受信・自動復帰 =====
  // 設計 docs/features/2026-11-05_epos_auto_recovery_plan.md (§2.3 分類、§3.1 状態機械、§3.2 検出)。
  // 旧監視 (~/rpdo の statusword を 0.5 s ごとに見る poll_motor) は rpdo キューの遅延で使えないため置き換えた。

  static const char * state_name(int s)
  {
    switch (s) {
      case RUNNING:
        return "RUNNING";
      case RECOVERING:
        return "RECOVERING";
      case WAIT_POWER:
        return "WAIT_POWER";
      default:
        return "DISARMED";
    }
  }

  std::string state_reason()
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return state_reason_;
  }

  std::string state_text()
  {
    const std::string r = state_reason();
    return std::string(state_name(state_.load())) + (r.empty() ? "" : " " + r);
  }

  void publish_state()
  {
    std_msgs::msg::String m;
    m.data = state_text();
    state_pub_->publish(m);
  }

  // 状態を変えてログと /epos4_controller/state に出す
  void set_state(int s, const std::string & reason)
  {
    const int prev = state_.exchange(s);
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      state_reason_ = reason;
    }
    publish_state();
    if (s == RUNNING) {
      RCLCPP_INFO(get_logger(), "[状態] %s → %s", state_name(prev), state_text().c_str());
    } else {
      RCLCPP_WARN(get_logger(), "[状態] %s → %s", state_name(prev), state_text().c_str());
    }
  }

  // 状態は変えず理由だけ更新 (WAIT_POWER 中の Vcc 表示など。ログは出さない)
  void set_reason(const std::string & reason)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    state_reason_ = reason;
  }

  std::chrono::milliseconds sdo_to() const
  {
    return std::chrono::milliseconds(static_cast<int>(monitor_sdo_timeout_s_ * 1000.0));
  }

  static std::string hex4(uint32_t v)
  {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "0x%04X", v & 0xFFFF);
    return buf;
  }

  // ---- EMCY 受動受信 (emcy_thread_) ----
  // can0 に読み取り専用の raw ソケットを開き、EPOS の EMCY (COB-ID 0x080 + node = 0x081 / 0x082) だけ受ける。
  // lely を経由しないので rpdo キューの遅延やマスタの停止に影響されない。送信はしない。
  int open_emcy_socket()
  {
    const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
      return -1;
    }
    struct can_filter flt[2];
    flt[0].can_id = 0x081;
    flt[0].can_mask = CAN_SFF_MASK;
    flt[1].can_id = 0x082;
    flt[1].can_mask = CAN_SFF_MASK;
    ::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &flt, sizeof(flt));
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 200000;  // stop_init_ を見るため 200 ms で read から戻る
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
      ::close(fd);
      return -1;
    }
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      ::close(fd);
      return -1;
    }
    return fd;
  }

  void emcy_loop()
  {
    int fd = -1;
    bool warned = false;
    while (!stop_init_.load() && rclcpp::ok()) {
      if (fd < 0) {
        fd = open_emcy_socket();
        if (fd < 0) {
          emcy_socket_ok_.store(false);
          if (!warned) {
            RCLCPP_WARN(
              get_logger(),
              "EMCY 受信: %s を開けない (再試行を続ける)。この間の Fault 検出は物理トリガだけ",
              can_interface_.c_str());
            warned = true;
          }
          std::this_thread::sleep_for(500ms);
          continue;
        }
        emcy_socket_ok_.store(true);
        warned = false;
        RCLCPP_INFO(get_logger(), "EMCY 受信: %s の 0x081 / 0x082 を受動受信中", can_interface_.c_str());
      }
      struct can_frame f;
      const ssize_t n = ::read(fd, &f, sizeof(f));
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
          continue;
        }
        RCLCPP_WARN(get_logger(), "EMCY 受信: read 失敗 (%s) → 開き直す", std::strerror(errno));
        ::close(fd);
        fd = -1;
        emcy_socket_ok_.store(false);
        std::this_thread::sleep_for(500ms);
        continue;
      }
      if (n != static_cast<ssize_t>(sizeof(f)) || (f.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG))) {
        continue;
      }
      const uint32_t id = f.can_id & CAN_SFF_MASK;
      const int motor = (id == 0x081) ? 0 : ((id == 0x082) ? 1 : -1);
      if (motor < 0) {
        continue;
      }
      const uint16_t code = (f.len >= 2) ? static_cast<uint16_t>(f.data[0] | (f.data[1] << 8)) : 0;
      std::lock_guard<std::mutex> lk(emcy_mtx_);
      emcy_events_.push_back({motor, code, now_ns()});
      if (emcy_events_.size() > 64) {
        emcy_events_.pop_front();
      }
    }
    if (fd >= 0) {
      ::close(fd);
    }
  }

  // 溜まった EMCY を取り出して各輪に反映。Fault を示す EMCY (code != 0) があれば true
  bool drain_emcy(MotorMonitor * ms[2])
  {
    std::deque<EmcyEvent> evs;
    {
      std::lock_guard<std::mutex> lk(emcy_mtx_);
      evs.swap(emcy_events_);
    }
    bool fault = false;
    for (const auto & e : evs) {
      MotorMonitor & m = *ms[e.motor];
      if (e.code == 0) {
        RCLCPP_INFO(get_logger(), "EMCY %s: 0x0000 (エラー解除の通知)", m.name.c_str());
        continue;
      }
      RCLCPP_WARN(
        get_logger(), "EMCY %s: %s (%s)", m.name.c_str(), hex4(e.code).c_str(), epos4_error_name(e.code));
      if (m.emcy_ns == 0) {
        m.emcy_ns = e.ns;  // そのエピソードの最初の EMCY 時刻
      }
      m.emcy_code = e.code;
      fault = true;
    }
    return fault;
  }

  void clear_episode(MotorMonitor * ms[2])
  {
    {
      std::lock_guard<std::mutex> lk(emcy_mtx_);
      emcy_events_.clear();  // 処理中に届いた同じ事象の EMCY (もう片輪・解除通知) は捨てる
    }
    for (int i = 0; i < 2; ++i) {
      ms[i]->emcy_ns = 0;
      ms[i]->emcy_code = 0;
      ms[i]->stall_since_ns = 0;
    }
  }

  // ---- 物理トリガ: 出力指令に対して車輪が回っていない (EMCY を取りこぼしたとき用) ----
  bool physical_trigger(MotorMonitor & m, int i, int64_t now)
  {
    const double cmd = std::abs(cmd_out_[i].load());
    const double act = std::abs(actual_rpm_[i].load());
    const double js_age = static_cast<double>(now - js_ns_[i].load()) * 1e-9;
    const bool cond = cmd >= stall_min_rpm_ && js_age < 0.3 && act < stall_ratio_ * cmd;
    if (!cond) {
      m.stall_since_ns = 0;
      return false;
    }
    if (m.stall_since_ns == 0) {
      m.stall_since_ns = now;
      return false;
    }
    if (
      static_cast<double>(now - m.stall_since_ns) * 1e-9 < stall_window_s_ ||
      static_cast<double>(now - m.phys_checked_ns) * 1e-9 < 2.0) {
      return false;
    }
    m.phys_checked_ns = now;
    const auto sw = read_sdo(m.sdo, 0x6041, 0, sdo_to());
    const bool fault = sw.has_value() && (sw.value() & 0x0008);
    RCLCPP_WARN(
      get_logger(), "物理トリガ: %s 出力 %.0f rpm に対し実速度 %.0f rpm → statusword %s%s", m.name.c_str(),
      cmd, act, sw ? hex4(sw.value()).c_str() : "(読めない)", fault ? " = Fault" : "");
    return fault;
  }

  // ---- Fault の記録 (従来どおり 0x1003 の履歴まで) ----
  void log_fault_record(MotorMonitor & m, double elapsed_s)
  {
    const auto to = sdo_to();
    m.history.clear();
    const uint32_t n = read_sdo(m.sdo, 0x1003, 0, to).value_or(0);
    std::string hist;
    for (uint32_t i = 1; i <= std::min<uint32_t>(n, 5); ++i) {
      const uint32_t h = read_sdo(m.sdo, 0x1003, static_cast<uint8_t>(i), to).value_or(0) & 0xFFFF;
      m.history.push_back(h);
      hist += " " + hex4(h);
    }
    char when[48];
    if (elapsed_s < 1e8) {
      std::snprintf(when, sizeof(when), "Fault から %.2f s 後に読んだ値", elapsed_s);
    } else {
      std::snprintf(when, sizeof(when), "Fault 時刻不明 = EMCY なし");
    }
    RCLCPP_ERROR(
      get_logger(), "%s FAULT: statusword=%s error=%s (%s) history[%u]=[%s ] Vcc=%.1f V (%s)",
      m.name.c_str(), m.statusword ? hex4(m.statusword.value()).c_str() : "?",
      hex4(m.error_code).c_str(), epos4_error_name(m.error_code), n, hist.c_str(), m.supply_v, when);
  }

  // ---- Fault の分類と自動復帰 (§2.3) ----
  void handle_fault(MotorMonitor * ms[2])
  {
    const auto to = sdo_to();
    const int prev = state_.load();
    const std::string prev_reason = state_reason();
    const int return_state = (prev == RUNNING) ? RUNNING : DISARMED;
    const int64_t t_detect = now_ns();
    set_state(RECOVERING, "Fault 検知 — 分類中");
    reset_ramp_.store(true);  // 片輪 Fault でもう片輪が回り続けて旋回しないよう、両輪とも即 0

    std::this_thread::sleep_for(30ms);  // もう片輪の EMCY も拾う
    drain_emcy(ms);

    // (1) Vcc を最初に読む (非常停止はラッチ式で解除まで ≥3 s。Fault から 1 s 以内なら必ず低い)
    for (int i = 0; i < 2; ++i) {
      const auto v = read_sdo(ms[i]->sdo, 0x2200, 1, to);
      ms[i]->supply_v = v ? v.value() / 10.0 : -1.0;
    }
    const int64_t t_vcc = now_ns();
    // (2) statusword とエラーコード
    for (int i = 0; i < 2; ++i) {
      MotorMonitor & m = *ms[i];
      m.statusword = read_sdo(m.sdo, 0x6041, 0, to);
      const auto e = read_sdo(m.sdo, 0x603F, 0, to);
      m.error_code = e ? (e.value() & 0xFFFF) : m.emcy_code;  // SDO が読めない (マスタ停止中) なら EMCY のコード
      m.fault = m.statusword ? ((m.statusword.value() & 0x0008) != 0) : (m.emcy_code != 0);
    }
    if (!ms[0]->fault && !ms[1]->fault) {
      RCLCPP_INFO(get_logger(), "Fault の兆候があったが両輪とも Fault していない → 元の状態に戻す");
      set_state(prev, prev_reason);
      return;
    }

    // (3) 分類
    int64_t t_fault = 0;
    bool has3220 = false;
    bool has8250 = false;
    bool all_comm = true;
    bool vcc_ok = true;
    std::string codes;
    for (int i = 0; i < 2; ++i) {
      MotorMonitor & m = *ms[i];
      if (!m.fault) {
        continue;
      }
      if (m.emcy_ns != 0 && (t_fault == 0 || m.emcy_ns < t_fault)) {
        t_fault = m.emcy_ns;  // 最も早い EMCY を起点にする (経過が長く出る = 安全側)
      }
      has3220 |= (m.error_code == 0x3220);
      has8250 |= (m.error_code == 0x8250);
      all_comm &= (m.error_code == 0x8130 || m.error_code == 0x8250);
      vcc_ok &= (m.supply_v >= vcc_ok_v_);
      codes += (codes.empty() ? "" : "/") + hex4(m.error_code);
    }
    const double elapsed = (t_fault != 0) ? static_cast<double>(t_vcc - t_fault) * 1e-9 : 1e9;
    for (int i = 0; i < 2; ++i) {
      if (ms[i]->fault) {
        log_fault_record(*ms[i], elapsed);
      }
    }
    // 自動復帰は 0.3 s 程度で終わり 2 Hz の定期送信に載らないので、Fault の中身をここで 1 回出す (epos_state_log が拾う)
    publish_diagnostics(*ms[0], *ms[1], false, "", read_can_ifindex());
    char buf[192];
    if (has3220) {
      if (!(elapsed <= vcc_read_deadline_s_ && vcc_ok)) {
        std::snprintf(
          buf, sizeof(buf), "非常停止 (%s, Vcc %.1f / %.1f V, Fault から %s) — 電源復帰待ち",
          codes.c_str(), ms[0]->supply_v, ms[1]->supply_v,
          t_fault ? (std::to_string(elapsed).substr(0, 4) + " s").c_str() : "時刻不明");
        set_state(WAIT_POWER, buf);
        return;
      }
      // 段差ブラウンアウト: Fault 後 1 s 以内に Vcc ≥ 20 V に戻っていた → 一時的
    } else if (!all_comm) {
      const uint32_t c = ms[0]->fault ? ms[0]->error_code : ms[1]->error_code;
      std::snprintf(
        buf, sizeof(buf), "Fault %s (%s) — 自動復帰しない。原因を確認して Start", codes.c_str(),
        epos4_error_name(c));
      set_state(DISARMED, buf);
      return;
    }

    // (4) 予算
    std::string label;
    if (has8250 && !has3220) {
      if (!auto_recover_8250_) {
        set_state(DISARMED, "Fault " + codes + " — 0x8250 の自動復帰は無効 (auto_recover_8250)。Start で再開");
        return;
      }
      if (recover_8250_count_run_ >= recover_8250_per_run_) {
        set_state(
          DISARMED, "Fault " + codes + " — 0x8250 の自動復帰は 1 起動 " +
                      std::to_string(recover_8250_per_run_) + " 回まで (予算超過)。Start で再開");
        return;
      }
      if (
        last_8250_recover_ns_ != 0 &&
        static_cast<double>(t_detect - last_8250_recover_ns_) * 1e-9 < refault_8250_s_) {
        set_state(DISARMED, "Fault " + codes + " — 0x8250 が復帰後すぐ再発。Start で再開");
        return;
      }
      ++recover_8250_count_run_;
      label = codes + " (0x8250 " + std::to_string(recover_8250_count_run_) + "/" +
              std::to_string(recover_8250_per_run_) + ")";
    } else {
      while (!recover_times_ns_.empty() &&
             static_cast<double>(t_detect - recover_times_ns_.front()) * 1e-9 > recover_window_s_) {
        recover_times_ns_.pop_front();
      }
      if (static_cast<int64_t>(recover_times_ns_.size()) >= recover_per_window_) {
        std::snprintf(
          buf, sizeof(buf), "Fault %s — 自動復帰の予算超過 (%.0f s に %ld 回)。Start で再開", codes.c_str(),
          recover_window_s_, recover_per_window_);
        set_state(DISARMED, buf);
        return;
      }
      if (recover_count_run_ >= recover_per_run_) {
        set_state(
          DISARMED, "Fault " + codes + " — 自動復帰の予算超過 (1 起動 " + std::to_string(recover_per_run_) +
                      " 回)。Start で再開");
        return;
      }
      recover_times_ns_.push_back(t_detect);
      ++recover_count_run_;
      label = codes + " (" + std::to_string(recover_times_ns_.size()) + "/" +
              std::to_string(recover_per_window_) + ")";
    }
    set_state(RECOVERING, "自動復帰中 " + label);
    do_recover(ms, return_state, t_detect, label, has8250 && !has3220);
  }

  bool both_ready(MotorMonitor * ms[2])
  {
    const bool r1 = motor_ready(ms[0]->sdo, ms[0]->name);
    const bool r2 = motor_ready(ms[1]->sdo, ms[1]->name);
    if (r1 && r2) {
      ms[0]->statusword.reset();  // Fault 中に読んだ値を /diagnostics に残さない
      ms[1]->statusword.reset();
    }
    return r1 && r2;
  }

  void do_recover(
    MotorMonitor * ms[2], int return_state, int64_t t_detect, const std::string & label, bool is8250)
  {
    const auto to = sdo_to();
    while (!stop_init_.load() && rclcpp::ok() &&
           static_cast<double>(now_ns() - t_detect) * 1e-9 < long_fault_s_) {
      // recover の前に読み直す (途中で非常停止が押された等)
      bool estop = false;
      for (int i = 0; i < 2; ++i) {
        MotorMonitor & m = *ms[i];
        if (!m.fault) {
          continue;
        }
        const auto v = read_sdo(m.sdo, 0x2200, 1, to);
        const auto e = read_sdo(m.sdo, 0x603F, 0, to);
        if (v) {
          m.supply_v = v.value() / 10.0;
        }
        if (e && (e.value() & 0xFFFF) == 0x3220 && (!v || m.supply_v < vcc_ok_v_)) {
          estop = true;
        }
      }
      if (estop) {
        set_state(WAIT_POWER, "復帰中に電圧低下 (非常停止?) — 電源復帰待ち");
        return;
      }
      for (int i = 0; i < 2; ++i) {
        if (ms[i]->fault) {
          call_trigger_sync(ms[i]->recover, ms[i]->name + " recover", 2s);
        }
      }
      std::this_thread::sleep_for(200ms);
      if (both_ready(ms)) {
        // RUNNING に戻す直前にもう一度 Vcc
        for (int i = 0; i < 2; ++i) {
          const auto v = read_sdo(ms[i]->sdo, 0x2200, 1, to);
          if (v && v.value() / 10.0 < vcc_ok_v_) {
            set_state(WAIT_POWER, "復帰直後に電圧低下 — 電源復帰待ち");
            return;
          }
        }
        if (is8250) {
          last_8250_recover_ns_ = now_ns();
        }
        ms[0]->fault = false;
        ms[1]->fault = false;
        reset_ramp_.store(true);
        const double dt = static_cast<double>(now_ns() - t_detect) * 1e-9;
        RCLCPP_WARN(get_logger(), "自動復帰: %s を %.1f s で復帰", label.c_str(), dt);
        set_state(
          return_state, return_state == RUNNING ? "自動復帰済み " + label
                                                : "自動復帰済み " + label + " — Start で走行許可");
        return;
      }
      std::this_thread::sleep_for(300ms);
    }
    set_state(DISARMED, "自動復帰できず (" + label + "、" + std::to_string(long_fault_s_).substr(0, 3) +
                          " s 超過) — 周囲を確認して Start");
  }

  // ---- 非常停止の後: 電源 (Vcc) が戻るのを待ち、enable だけして DISARMED へ ----
  void wait_power_step(MotorMonitor * ms[2], int64_t & ok_since, int64_t & last_read)
  {
    const int64_t now = now_ns();
    if (static_cast<double>(now - last_read) * 1e-9 < 1.0) {
      return;
    }
    last_read = now;
    bool all_ok = true;
    for (int i = 0; i < 2; ++i) {
      const auto v = read_sdo(ms[i]->sdo, 0x2200, 1, sdo_to());
      ms[i]->supply_v = v ? v.value() / 10.0 : -1.0;
      all_ok &= (ms[i]->supply_v >= vcc_ok_v_);
    }
    char buf[128];
    std::snprintf(
      buf, sizeof(buf), "非常停止 — 電源復帰待ち (Vcc %.1f / %.1f V)", ms[0]->supply_v, ms[1]->supply_v);
    if (!all_ok) {
      ok_since = 0;
      set_reason(buf);
      return;
    }
    if (ok_since == 0) {
      ok_since = now;  // 20 V 以上が 1 s 続くのを待つ
      return;
    }
    if (static_cast<double>(now - ok_since) * 1e-9 < 1.0) {
      return;
    }
    RCLCPP_WARN(get_logger(), "非常停止: 電源が戻った (Vcc %.1f / %.1f V) → enable だけする", ms[0]->supply_v, ms[1]->supply_v);
    for (int i = 0; i < 2; ++i) {
      const auto sw = read_sdo(ms[i]->sdo, 0x6041, 0, sdo_to());
      if (sw && (sw.value() & 0x0008)) {
        call_trigger_sync(ms[i]->recover, ms[i]->name + " recover", 2s);
      }
    }
    std::this_thread::sleep_for(200ms);
    if (both_ready(ms)) {
      ms[0]->fault = false;
      ms[1]->fault = false;
      ok_since = 0;
      reset_ramp_.store(true);
      set_state(DISARMED, "非常停止解除 — 周囲を確認して Start で走行許可");
    } else {
      RCLCPP_WARN(get_logger(), "非常停止: 電源は戻ったが enable できない → 1 s 後に再試行");
    }
  }

  // ---- 走行許可 (Start / ~/arm)。DISARMED のときだけ ----
  void handle_arm(MotorMonitor * ms[2])
  {
    const int st = state_.load();
    if (st != DISARMED) {
      RCLCPP_INFO(get_logger(), "走行許可の要求を無視 (状態 %s)", state_name(st));
      return;
    }
    // Fault している輪があれば recover する (人の判断なので予算は使わない)
    for (int i = 0; i < 2; ++i) {
      const auto sw = read_sdo(ms[i]->sdo, 0x6041, 0, sdo_to());
      if (sw && (sw.value() & 0x0008)) {
        RCLCPP_WARN(get_logger(), "Start: %s が Fault (%s) → recover", ms[i]->name.c_str(), hex4(sw.value()).c_str());
        call_trigger_sync(ms[i]->recover, ms[i]->name + " recover", 2s);
      }
    }
    std::this_thread::sleep_for(200ms);
    if (both_ready(ms)) {
      ms[0]->fault = false;
      ms[1]->fault = false;
      recover_times_ns_.clear();  // 人が見て再開した = 窓の予算は数え直す (1 起動あたりの上限は残す)
      reset_ramp_.store(true);
      set_state(RUNNING, "走行許可 (Start)");
    } else {
      set_state(DISARMED, "Start: モータが準備できない (EPOS の状態・電源を確認) — もう一度 Start");
    }
  }

  // executor が止まったとき、監視スレッドから目標 0 を直接送る (timer_callback が回らないため)
  void publish_zero_direct()
  {
    canopen_interfaces::msg::COData m;
    m.index = 0x60ff;
    m.subindex = 0x00;
    m.data = 0;
    m1_tpdo_publisher_->publish(m);
    m2_tpdo_publisher_->publish(m);
  }

  // claude_watchdog → claude_recover: 監視ループ (init_thread_ 上)。
  //   1. executor 停止監視 (executor_stall_s 以上 timer_callback が回らなければ 0 を直送)
  //   2. can0 の消滅・ifindex 変化 = USB 再列挙。master は旧 socket を掴んだままなので再起動が要る
  //   3. EMCY (主) と物理トリガ (予備) で Fault を検出 → 分類・自動復帰 (handle_fault)
  //   4. 走行許可 (Start / ~/arm)、非常停止後の電源待ち
  //   5. 状態と /diagnostics を 2 Hz で出す (bag に残る)
  void run_monitor_loop()
  {
    MotorMonitor m1{"motor1(right)", "EPOS4 node 1", m1_client_sdo_read_, m1_client_driver_recover_};
    MotorMonitor m2{"motor2(left)", "EPOS4 node 2", m2_client_sdo_read_, m2_client_driver_recover_};
    MotorMonitor * ms[2] = {&m1, &m2};
    const int ifindex_at_start = read_can_ifindex();
    bool link_lost_prev = false;
    bool stall_logged = false;
    int64_t last_pub_ns = 0;
    int64_t wp_ok_since = 0;
    int64_t wp_last_read = 0;
    clear_episode(ms);  // init 中の EMCY (起動時の fault reset 等) は捨てる
    RCLCPP_INFO(
      get_logger(),
      "監視ループ開始: %s ifindex=%d, period=%.2f s, cmd_timeout=%.2f s。Fault 検出 = EMCY 受動受信 + 物理トリガ、"
      "自動復帰 = 0x8130 / 0x8250 / 0x3220 (Fault 後 %.1f s 以内の Vcc ≥ %.0f V)",
      can_interface_.c_str(), ifindex_at_start, monitor_period_s_, cmd_timeout_s_,
      vcc_read_deadline_s_, vcc_ok_v_);
    set_state(DISARMED, "起動完了 — Start で走行許可");

    while (!stop_init_.load() && rclcpp::ok()) {
      std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(monitor_period_s_ * 1000.0)));
      if (stop_init_.load() || !rclcpp::ok()) {
        break;
      }
      const int64_t now = now_ns();

      // 1. executor 停止監視
      const int64_t tick = last_tick_ns_.load();
      if (tick > 0 && static_cast<double>(now - tick) * 1e-9 > executor_stall_s_) {
        publish_zero_direct();
        if (!stall_logged) {
          RCLCPP_ERROR(
            get_logger(), "executor が %.2f s 止まっている → 監視スレッドから目標 0 を直接送信",
            static_cast<double>(now - tick) * 1e-9);
          stall_logged = true;
        }
      } else if (stall_logged) {
        RCLCPP_WARN(get_logger(), "executor 再開");
        stall_logged = false;
      }

      // 2. can0
      const int ifindex_now = read_can_ifindex();
      const bool can_gone = (ifindex_now < 0);
      const bool can_recreated =
        (ifindex_at_start > 0 && ifindex_now > 0 && ifindex_now != ifindex_at_start);
      const bool link_lost = can_gone || can_recreated;
      force_zero_.store(link_lost);
      std::string reason;
      if (can_gone) {
        reason = can_interface_ + " が消滅 (USB アダプタ切断?)";
      } else if (can_recreated) {
        reason = can_interface_ + " が作り直された (ifindex " + std::to_string(ifindex_at_start) + " → " +
                 std::to_string(ifindex_now) + ")";
      }
      if (link_lost && !link_lost_prev) {
        RCLCPP_ERROR(
          get_logger(),
          "CAN リンク喪失: %s → 目標 0。EPOS は heartbeat (0x8130) で止まる。master は旧 socket のままなので "
          "bringup の再起動が必要",
          reason.c_str());
        set_state(DISARMED, "CAN リンク喪失 — bringup 再起動が必要");
      }
      link_lost_prev = link_lost;

      if (!link_lost) {
        // 3. Fault 検出: EMCY (主) → 物理トリガ (予備、RUNNING 中だけ)
        bool detected = drain_emcy(ms);
        if (!detected && state_.load() == RUNNING) {
          for (int i = 0; i < 2 && !detected; ++i) {
            detected = physical_trigger(*ms[i], i, now);
          }
        } else if (state_.load() != RUNNING) {
          m1.stall_since_ns = 0;
          m2.stall_since_ns = 0;
        }

        // 4. 走行許可
        if (arm_request_.exchange(false)) {
          handle_arm(ms);
        }

        // 5. Fault の分類と自動復帰 (非常停止の電源待ち中に届く EMCY は記録だけ)
        if (detected) {
          if (state_.load() != WAIT_POWER) {
            handle_fault(ms);
          }
          clear_episode(ms);
        }

        // 6. 非常停止の後
        if (state_.load() == WAIT_POWER) {
          wait_power_step(ms, wp_ok_since, wp_last_read);
        } else {
          wp_ok_since = 0;
        }
      }

      // 7. 状態と /diagnostics (2 Hz)
      if (static_cast<double>(now - last_pub_ns) * 1e-9 >= 0.5) {
        publish_state();
        publish_diagnostics(m1, m2, link_lost, reason, ifindex_now);
        last_pub_ns = now;
      }
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
    link.values.push_back(kv("emcy_socket", emcy_socket_ok_.load() ? "ok" : "down"));
    arr.status.push_back(link);
    DiagnosticStatus st;
    st.name = "epos4_controller/state";
    st.hardware_id = "epos4_controller";
    const int s = state_.load();
    st.level = (s == RUNNING) ? DiagnosticStatus::OK
                              : (s == WAIT_POWER ? DiagnosticStatus::ERROR : DiagnosticStatus::WARN);
    st.message = state_text();
    st.values.push_back(kv("recover_count_run", std::to_string(recover_count_run_)));
    st.values.push_back(kv("recover_8250_count_run", std::to_string(recover_8250_count_run_)));
    arr.status.push_back(st);
    for (const MotorMonitor * m : {&m1, &m2}) {
      DiagnosticStatus d;
      d.name = "epos4_controller/" + m->name;
      d.hardware_id = m->hardware_id;
      if (m->fault) {
        d.level = DiagnosticStatus::ERROR;
        d.message = "FAULT " + hex4(m->error_code) + " " + epos4_error_name(m->error_code);
      } else {
        d.level = DiagnosticStatus::OK;
        d.message = m->statusword ? "statusword " + hex4(m->statusword.value()) + " (最後に SDO で読んだ値)"
                                  : "OK (Fault 検出は EMCY 受信)";
      }
      if (m->statusword) {
        d.values.push_back(kv("statusword", hex4(m->statusword.value())));
      }
      if (m->fault) {
        d.values.push_back(kv("error_code", hex4(m->error_code)));
        std::string hist;
        for (auto h : m->history) {
          hist += hex4(h) + " ";
        }
        d.values.push_back(kv("error_history", hist));
      }
      if (m->supply_v >= 0.0) {
        char v[16];
        std::snprintf(v, sizeof(v), "%.1f", m->supply_v);
        d.values.push_back(kv("supply_voltage_V", v));
      }
      arr.status.push_back(d);
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