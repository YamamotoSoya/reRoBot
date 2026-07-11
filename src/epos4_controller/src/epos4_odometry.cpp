#include <cmath>
#include <memory>
#include <stdexcept>  // claude_robust
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.h"                      // claude_tire claude_odom claude_sync
#include "message_filters/sync_policies/approximate_time.h"  // claude_tire claude_odom claude_sync
#include "message_filters/synchronizer.h"                    // claude_tire claude_odom claude_sync
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class Epos4OdometryNode : public rclcpp::Node
{
public:
  Epos4OdometryNode() : Node("epos4_odometry_node")
  {
    declare_parameter("tread_width", 0.41);
    declare_parameter("tire_diam", 0.15);
    declare_parameter("gear_ratio", 1.0);
    declare_parameter("odom_frame_id", std::string("odom"));
    declare_parameter("base_frame_id", std::string("base_link"));
    declare_parameter("publish_tf", true);
    declare_parameter("invert_left", false);
    declare_parameter("invert_right", false);
    // claude_robust: 1 更新 (PDO sync 50ms) あたりの車輪移動量 [m] がこれを超えたら
    // 「エンコーダ値の飛び」とみなして積分しない (0 で無効)。既定 0.5 m/50 ms = 10 m/s
    // 相当で、実車では絶対に出ない値。典型原因は canopen ドライバの再起動で
    // position が 0 に戻るケースで、旧実装では /odom がその瞬間ワープした。
    declare_parameter("pose_jump_threshold", 0.5);

    tread_width_ = get_parameter("tread_width").as_double();
    wheel_radius_ = get_parameter("tire_diam").as_double() * 0.5;
    gear_ratio_ = get_parameter("gear_ratio").as_double();
    odom_frame_id_ = get_parameter("odom_frame_id").as_string();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    invert_left_ = get_parameter("invert_left").as_bool();
    invert_right_ = get_parameter("invert_right").as_bool();
    pose_jump_threshold_ = get_parameter("pose_jump_threshold").as_double();  // claude_robust

    // claude_robust: 車体パラメータの fail-fast (controller と同じ理由)
    if (tread_width_ <= 0.0 || wheel_radius_ <= 0.0 || gear_ratio_ <= 0.0) {
      RCLCPP_FATAL(
        get_logger(), "invalid chassis params: tread_width=%.3f tire_diam=%.3f gear_ratio=%.3f",
        tread_width_, wheel_radius_ * 2.0, gear_ratio_);
      throw std::invalid_argument("epos4_odometry: chassis parameters must be > 0");
    }

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    // claude_tire: /joint_states for robot_state_publisher (wheel-side angles)
    joint_state_publisher_ =                                                // claude_tire
      create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);  // claude_tire
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // claude_tire claude_odom claude_sync: subscribe both EPOS4 joint_states directly
    // and pair them by stamp. PDO sync runs at 50 ms on both sides, so ApproximateTime
    // collapses each pair into a single callback. Replaces the old single-topic
    // /robot_encoder_states subscription.
    m1_sub_.subscribe(
      this, "/motor1/cia402_device_1/joint_states");  // claude_tire claude_odom claude_sync
    m2_sub_.subscribe(
      this, "/motor2/cia402_device_2/joint_states");  // claude_tire claude_odom claude_sync
    // claude_swap: physical wiring is motor1 = RIGHT wheel, motor2 = LEFT wheel.
    // Feed motor2 (physical LEFT) into the callback's left_msg slot and motor1
    // (physical RIGHT) into right_msg, so pos_left/pos_right, d_theta and the
    // m1_wheel(+y)/m2_wheel(-y) URDF links all match the real robot.
    sync_ = std::make_shared<Synchronizer>(
      SyncPolicy(10), m2_sub_, m1_sub_);  // claude_tire claude_odom claude_sync claude_swap
    sync_->setMaxIntervalDuration(
      rclcpp::Duration(0, 50 * 1000 * 1000));  // claude_tire claude_odom claude_sync
    sync_->registerCallback(                   // claude_tire claude_odom claude_sync
      std::bind(
        &Epos4OdometryNode::onJointStates, this,         // claude_tire claude_odom claude_sync
        std::placeholders::_1, std::placeholders::_2));  // claude_tire claude_odom claude_sync

    // claude_robust: 入力監視用のトピック別カウンタ。ApproximateTime は「片方でも
    // 止まると何も出さなくなる」ため、無音時にどちらが原因か(あるいは両方流れて
    // いるのに stamp が噛み合っていないのか)を切り分けられるようにする。
    m1_sub_.registerCallback(
      [this](const JointStateMsg::ConstSharedPtr &) { ++m1_msg_count_; });  // claude_robust
    m2_sub_.registerCallback(
      [this](const JointStateMsg::ConstSharedPtr &) { ++m2_msg_count_; });  // claude_robust

    // claude_robust: 5 秒ごとの無音監視。同期ペアが 1 つも来ていなければ、
    // トピック別カウンタから原因を推定して警告する。
    watchdog_timer_ = create_wall_timer(5s, [this] { checkInputHealth(); });

    RCLCPP_INFO(
      get_logger(),
      "EPOS4 odometry started (tread=%.3f m, wheel_radius=%.3f m, gear=%.2f, invert L/R=%d/%d)",
      tread_width_, wheel_radius_, gear_ratio_, invert_left_, invert_right_);
  }

private:
  // claude_tire claude_odom claude_sync: type aliases for message_filters synchronizer
  using JointStateMsg = sensor_msgs::msg::JointState;  // claude_tire claude_odom claude_sync
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    JointStateMsg, JointStateMsg>;  // claude_tire claude_odom claude_sync
  using Synchronizer =
    message_filters::Synchronizer<SyncPolicy>;  // claude_tire claude_odom claude_sync

  // claude_tire claude_odom claude_sync: synchronizer callback. Receives a (left, right)
  // pair already aligned by header.stamp from the two per-motor /motor*/cia402_device_*/joint_states.
  void onJointStates(
    const JointStateMsg::ConstSharedPtr & left_msg,   // claude_tire claude_odom claude_sync
    const JointStateMsg::ConstSharedPtr & right_msg)  // claude_tire claude_odom claude_sync
  {
    if (left_msg->position.empty() || right_msg->position.empty()) {  // claude_tire claude_odom
      return;
    }
    ++pair_count_;  // claude_robust: 無音監視用

    // motor-side position -> wheel-side position via gear ratio
    const double inv_gear = (gear_ratio_ != 0.0) ? (1.0 / gear_ratio_) : 1.0;
    double pos_left = left_msg->position[0] * inv_gear;    // claude_tire claude_odom
    double pos_right = right_msg->position[0] * inv_gear;  // claude_tire claude_odom
    if (invert_left_) pos_left = -pos_left;
    if (invert_right_) pos_right = -pos_right;

    rclcpp::Time stamp = left_msg->header.stamp;  // claude_tire claude_odom
    if (stamp.nanoseconds() == 0) {
      stamp = this->now();
    }

    // claude_tire: republish wheel-side /joint_states so robot_state_publisher can emit dynamic TF
    publishWheelJointStates(
      stamp, pos_left, pos_right, left_msg, right_msg, inv_gear);  // claude_tire

    if (!initialized_) {
      prev_pos_left_ = pos_left;
      prev_pos_right_ = pos_right;
      last_stamp_ = stamp;
      initialized_ = true;
      return;
    }

    double dt = (stamp - last_stamp_).seconds();
    if (dt <= 0.0) {
      return;
    }

    // wheel travel since last update [m]
    double d_left = (pos_left - prev_pos_left_) * wheel_radius_;
    double d_right = (pos_right - prev_pos_right_) * wheel_radius_;

    // claude_robust: エンコーダ値の飛び (典型: canopen ドライバ再起動で position が
    // 0 に戻る) を検出したら、その 1 回は積分せず基準を取り直す。pose は保持される
    // (ワープしない) が、飛びの間の実移動は失われるため WARN を残す。
    if (
      pose_jump_threshold_ > 0.0 &&
      (std::abs(d_left) > pose_jump_threshold_ || std::abs(d_right) > pose_jump_threshold_)) {
      RCLCPP_WARN(
        get_logger(),
        "wheel position jump detected (L=%.3f m, R=%.3f m in one update) — re-seeding, pose held",
        d_left, d_right);
      prev_pos_left_ = pos_left;
      prev_pos_right_ = pos_right;
      last_stamp_ = stamp;
      return;
    }

    double d_s = 0.5 * (d_left + d_right);
    double d_theta = (d_right - d_left) / tread_width_;

    // 2nd-order pose integration (use mid-step heading)
    double mid_theta = theta_ + 0.5 * d_theta;
    x_ += d_s * std::cos(mid_theta);
    y_ += d_s * std::sin(mid_theta);
    theta_ += d_theta;
    // wrap to [-pi, pi]
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    // body-frame velocities
    double v_lin = 0.0;
    double v_ang = 0.0;
    if (!left_msg->velocity.empty() && !right_msg->velocity.empty()) {  // claude_odom
      double w_left = left_msg->velocity[0] * inv_gear;                 // claude_odom
      double w_right = right_msg->velocity[0] * inv_gear;               // claude_odom
      if (invert_left_) w_left = -w_left;
      if (invert_right_) w_right = -w_right;
      double v_l = w_left * wheel_radius_;
      double v_r = w_right * wheel_radius_;
      v_lin = 0.5 * (v_l + v_r);
      v_ang = (v_r - v_l) / tread_width_;
    } else {
      v_lin = d_s / dt;
      v_ang = d_theta / dt;
    }

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = v_lin;
    odom.twist.twist.angular.z = v_ang;
    odom_publisher_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = odom_frame_id_;
      tf_msg.child_frame_id = base_frame_id_;
      tf_msg.transform.translation.x = x_;
      tf_msg.transform.translation.y = y_;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(tf_msg);
    }

    prev_pos_left_ = pos_left;
    prev_pos_right_ = pos_right;
    last_stamp_ = stamp;
  }

  // claude_tire: emit JointState{m1_wheel, m2_wheel} on /joint_states so robot_state_publisher
  // can produce dynamic TFs for the wheel links. Position is wheel-side angle (rad).
  void publishWheelJointStates(
    const rclcpp::Time & stamp,                                         // claude_tire
    double pos_left, double pos_right,                                  // claude_tire
    const JointStateMsg::ConstSharedPtr & left_msg,                     // claude_tire
    const JointStateMsg::ConstSharedPtr & right_msg,                    // claude_tire
    double inv_gear)                                                    // claude_tire
  {                                                                     // claude_tire
    sensor_msgs::msg::JointState js;                                    // claude_tire
    js.header.stamp = stamp;                                            // claude_tire
    js.name = {"m1_wheel", "m2_wheel"};                                 // claude_tire
    js.position = {pos_left, pos_right};                                // claude_tire
    if (!left_msg->velocity.empty() && !right_msg->velocity.empty()) {  // claude_tire
      double w_left = left_msg->velocity[0] * inv_gear;                 // claude_tire
      double w_right = right_msg->velocity[0] * inv_gear;               // claude_tire
      if (invert_left_) w_left = -w_left;                               // claude_tire
      if (invert_right_) w_right = -w_right;                            // claude_tire
      js.velocity = {w_left, w_right};                                  // claude_tire
    }                                                                   // claude_tire
    joint_state_publisher_->publish(js);                                // claude_tire
  }                                                                     // claude_tire

  // claude_robust: 5 秒ごとの入力健全性チェック。前回チェックからの増分で判定し、
  // 無音の原因 (どちらのドライバが止まったか / stamp 同期の失敗か) を切り分ける。
  void checkInputHealth()
  {
    const int d_m1 = m1_msg_count_ - last_m1_count_;
    const int d_m2 = m2_msg_count_ - last_m2_count_;
    const int d_pair = pair_count_ - last_pair_count_;
    last_m1_count_ = m1_msg_count_;
    last_m2_count_ = m2_msg_count_;
    last_pair_count_ = pair_count_;

    if (d_pair > 0) {
      if (input_was_silent_) {
        RCLCPP_INFO(get_logger(), "joint_states input recovered (%d pairs in last 5s)", d_pair);
        input_was_silent_ = false;
      }
      return;
    }
    input_was_silent_ = true;
    if (d_m1 == 0 && d_m2 == 0) {
      RCLCPP_WARN(
        get_logger(),
        "no joint_states from either motor in last 5s — is bus_config / device_manager up?");
    } else if (d_m1 == 0 || d_m2 == 0) {
      RCLCPP_WARN(
        get_logger(),
        "%s joint_states silent (m1=%d, m2=%d msgs in 5s) — that cia402 driver is down; /odom "
        "is frozen",
        (d_m1 == 0) ? "motor1" : "motor2", d_m1, d_m2);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "both joint_states flowing (m1=%d, m2=%d in 5s) but no stamp-synced pairs — check "
        "header.stamp / PDO sync",
        d_m1, d_m2);
    }
  }

  // claude_tire claude_odom claude_sync: replaces encoder_subscription_ with two
  // message_filters subscribers + synchronizer (feeds both wheel-TF and odometry paths).
  message_filters::Subscriber<JointStateMsg> m1_sub_;  // claude_tire claude_odom claude_sync
  message_filters::Subscriber<JointStateMsg> m2_sub_;  // claude_tire claude_odom claude_sync
  std::shared_ptr<Synchronizer> sync_;                 // claude_tire claude_odom claude_sync
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;  // claude_tire
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double tread_width_;
  double wheel_radius_;
  double gear_ratio_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  bool publish_tf_;
  bool invert_left_;
  bool invert_right_;

  bool initialized_ = false;
  double prev_pos_left_ = 0.0;
  double prev_pos_right_ = 0.0;
  rclcpp::Time last_stamp_;

  // claude_robust: 入力監視。カウンタはすべて executor スレッドからのみ触る。
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  double pose_jump_threshold_ = 0.5;
  int m1_msg_count_ = 0;
  int m2_msg_count_ = 0;
  int pair_count_ = 0;
  int last_m1_count_ = 0;
  int last_m2_count_ = 0;
  int last_pair_count_ = 0;
  bool input_was_silent_ = false;

  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Epos4OdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
