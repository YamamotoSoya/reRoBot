#include <atomic>  // claude
#include <chrono>
#include <cmath>
#include <future>  // claude
#include <iostream>
#include <memory>
#include <optional>  // claude
#include <thread>

#include "canopen_interfaces/msg/co_data.hpp"
#include "canopen_interfaces/srv/co_read.hpp"  // claude: SDO read for statusword/mode verification
#include "canopen_interfaces/srv/co_target_double.hpp"
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

    //YAMLparams
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

  void timer_callback()
  {
    auto m1_msg = canopen_interfaces::msg::COData();
    m1_msg.index = 0x60ff;
    m1_msg.subindex = 0x00;
    m1_msg.data = static_cast<int>(m1_value_);
    m1_tpdo_publisher_->publish(m1_msg);

    auto m2_msg = canopen_interfaces::msg::COData();
    m2_msg.index = 0x60ff;
    m2_msg.subindex = 0x00;
    m2_msg.data = static_cast<int>(m2_value_);
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

  // claude: blocking SDO upload (read) of a 16-bit object, returns nullopt on failure.
  std::optional<uint32_t> read_sdo(
    rclcpp::Client<canopen_interfaces::srv::CORead>::SharedPtr client, uint16_t index)
  {
    if (!client->wait_for_service(2s)) {
      return std::nullopt;
    }
    auto request = std::make_shared<canopen_interfaces::srv::CORead::Request>();
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
      RCLCPP_ERROR(get_logger(), "motor1 init service never appeared; aborting auto-init");
      return;
    }
    init_motor(
      "motor1(left)", m1_client_driver_init_, m1_client_driver_enable_, m1_client_driver_csv_mode_,
      m1_client_driver_recover_, m1_client_sdo_read_);
    init_motor(
      "motor2(right)", m2_client_driver_init_, m2_client_driver_enable_, m2_client_driver_csv_mode_,
      m2_client_driver_recover_, m2_client_sdo_read_);
    RCLCPP_INFO(get_logger(), "EPOS4 auto-init complete.");
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