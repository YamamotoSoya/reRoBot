#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <cmath>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class Epos4OdometryNode : public rclcpp::Node
{
public:
    Epos4OdometryNode() : Node("epos4_odometry_node")
    {
        declare_parameter("tread_width", 0.41);
        declare_parameter("tire_diam", 0.15);
        declare_parameter("odom_frame_id", std::string("odom"));
        declare_parameter("base_frame_id", std::string("base_link"));
        declare_parameter("publish_tf", true);
        declare_parameter("invert_left", false);
        declare_parameter("invert_right", false);

        tread_width_ = get_parameter("tread_width").as_double();
        wheel_radius_ = get_parameter("tire_diam").as_double() * 0.5;
        odom_frame_id_ = get_parameter("odom_frame_id").as_string();
        base_frame_id_ = get_parameter("base_frame_id").as_string();
        publish_tf_ = get_parameter("publish_tf").as_bool();
        invert_left_ = get_parameter("invert_left").as_bool();
        invert_right_ = get_parameter("invert_right").as_bool();

        encoder_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
            "/robot_encoder_states", 10,
            std::bind(&Epos4OdometryNode::encoderCallback, this, std::placeholders::_1));

        odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(get_logger(),
            "EPOS4 odometry started (tread=%.3f m, wheel_radius=%.3f m)",
            tread_width_, wheel_radius_);
    }

private:
    void encoderCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < 2) {
            return;
        }

        double pos_left = msg->position[0];
        double pos_right = msg->position[1];
        if (invert_left_)  pos_left  = -pos_left;
        if (invert_right_) pos_right = -pos_right;

        rclcpp::Time stamp = msg->header.stamp;
        if (stamp.nanoseconds() == 0) {
            stamp = this->now();
        }

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
        if (msg->velocity.size() >= 2) {
            double w_left = msg->velocity[0];
            double w_right = msg->velocity[1];
            if (invert_left_)  w_left  = -w_left;
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

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double tread_width_;
    double wheel_radius_;
    std::string odom_frame_id_;
    std::string base_frame_id_;
    bool publish_tf_;
    bool invert_left_;
    bool invert_right_;

    bool initialized_ = false;
    double prev_pos_left_ = 0.0;
    double prev_pos_right_ = 0.0;
    rclcpp::Time last_stamp_;

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Epos4OdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
