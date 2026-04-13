#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "canopen_interfaces/srv/co_target_double.hpp"
#include "canopen_interfaces/msg/co_data.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class Epos4_Control2_Node : public rclcpp::Node
{
public:
    Epos4_Control2_Node() : Node("epos4_cmd_vel_node")
    {

        // motor1
        m1_client_driver_init_ = this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/init");
        m1_client_driver_halt_ = this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/halt");
        m1_client_driver_recover_ = this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/recover");
        m1_client_driver_shutdown_ = this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/shutdown");
        m1_client_driver_enable_ = this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/enable");
        m1_client_driver_disable_ = this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/disable");
        m1_client_driver_vel_mode_ = this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/velocity_mode");
        m1_client_driver_csv_mode_ = this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/cyclic_velocity_mode");
        m1_client_target_ = this->create_client<canopen_interfaces::srv::COTargetDouble>("/motor1/cia402_device_1/target");
        m1_tpdo_publisher_ = this->create_publisher<canopen_interfaces::msg::COData>("/motor1/cia402_device_1/tpdo", 10);

        m1_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
            "/motor1/cia402_device_1/joint_states", 10,
            std::bind(&Epos4_Control2_Node::jointStateCallback_m1, this, std::placeholders::_1));

        // motor2
        m2_client_driver_init_ = this->create_client<std_srvs::srv::Trigger>("/motor2/cia402_device_2/init");
        m2_client_driver_halt_ = this->create_client<std_srvs::srv::Trigger>("/motor2/cia402_device_2/halt");
        m2_client_driver_recover_ = this->create_client<std_srvs::srv::Trigger>("/motor2/cia402_device_2/recover");
        m2_client_driver_shutdown_ = this->create_client<std_srvs::srv::Trigger>("/motor2/cia402_device_2/shutdown");
        m2_client_driver_enable_ = this->create_client<std_srvs::srv::Trigger>("/motor2/cia402_device_2/enable");
        m2_client_driver_disable_ = this->create_client<std_srvs::srv::Trigger>("/motor2/cia402_device_2/disable");
        m2_client_driver_vel_mode_ = this->create_client<std_srvs::srv::Trigger>("/motor2/cia402_device_2/velocity_mode");
        m2_client_driver_csv_mode_ = this->create_client<std_srvs::srv::Trigger>("/motor2/cia402_device_2/cyclic_velocity_mode");
        m2_client_target_ = this->create_client<canopen_interfaces::srv::COTargetDouble>("/motor2/cia402_device_2/target");
        m2_tpdo_publisher_ = this->create_publisher<canopen_interfaces::msg::COData>("/motor2/cia402_device_2/tpdo", 10);

        m2_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
            "/motor2/cia402_device_2/joint_states", 10,
            std::bind(&Epos4_Control2_Node::jointStateCallback_m2, this, std::placeholders::_1));

        // initializing and conection
        topic_timer_ = this->create_wall_timer(10ms, std::bind(&Epos4_Control2_Node::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Auto-initializing EPOS4...");
        call_trigger_service(m1_client_driver_init_, "init");
        call_trigger_service(m1_client_driver_enable_, "enable");
        call_trigger_service(m1_client_driver_csv_mode_, "cyclic_velocity_mode");
        call_trigger_service(m2_client_driver_init_, "init");
        call_trigger_service(m2_client_driver_enable_, "enable");
        call_trigger_service(m2_client_driver_csv_mode_, "cyclic_velocity_mode");

        input_thread_ = std::thread(&Epos4_Control2_Node::input_loop, this);

        RCLCPP_INFO(get_logger(), "********************************************");
        RCLCPP_INFO(get_logger(), "maxon EPOS4 Control (velocity)");
        RCLCPP_INFO(get_logger(), "To be used with 1 EPOS4: /cia402_device_1");
        RCLCPP_INFO(get_logger(), "run first bus_config_cia402_epos4_vel.launch.py");
        RCLCPP_INFO(get_logger(), "********************************************");

        display_menu();
    }

    ~Epos4_Control2_Node()
    {
        if (input_thread_.joinable())
        {
            input_thread_.join();
        }
    }

private:
    // motor1
    double m1_value_ = 0.0;
    bool js_arrived_m1_ = false;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m1_client_driver_init_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m1_client_driver_halt_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m1_client_driver_recover_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m1_client_driver_shutdown_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m1_client_driver_enable_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m1_client_driver_disable_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m1_client_driver_vel_mode_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m1_client_driver_csv_mode_;

    rclcpp::Client<canopen_interfaces::srv::COTargetDouble>::SharedPtr m1_client_target_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m1_publisher_;
    rclcpp::Publisher<canopen_interfaces::msg::COData>::SharedPtr m1_tpdo_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m1_subscription_;

    sensor_msgs::msg::JointState m1_joint_state_;

    // motor2
    double m2_value_ = 0.0;
    bool js_arrived_m2_ = false;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m2_client_driver_init_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m2_client_driver_halt_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m2_client_driver_recover_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m2_client_driver_shutdown_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m2_client_driver_enable_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m2_client_driver_disable_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m2_client_driver_vel_mode_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m2_client_driver_csv_mode_;

    rclcpp::Client<canopen_interfaces::srv::COTargetDouble>::SharedPtr m2_client_target_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m2_publisher_;
    rclcpp::Publisher<canopen_interfaces::msg::COData>::SharedPtr m2_tpdo_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m2_subscription_;

    sensor_msgs::msg::JointState m2_joint_state_;

    //
    rclcpp::TimerBase::SharedPtr topic_timer_;
    std::thread input_thread_;

    void shutdown_node()
    {
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
    }

    void jointStateCallback_m1(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        m1_joint_state_ = *msg;
        js_arrived_m1_ = true;
    }

    void jointStateCallback_m2(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        m2_joint_state_ = *msg;
        js_arrived_m2_ = true;
    }

    void trigger_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success)
        {
            RCLCPP_INFO(get_logger(), "Service call successful: %s", response->message.c_str());
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Service  call failed: %s", response->message.c_str());
        }
    }

    void call_trigger_service(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
        const std::string &service_name)
    {
        if (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(get_logger(), "Service %s not available", service_name.c_str());
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        client->async_send_request(
            request,
            std::bind(&Epos4_Control2_Node::trigger_callback, this, std::placeholders::_1));
    }

    void target_callback(rclcpp::Client<canopen_interfaces::srv::COTargetDouble>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success)
        {
            RCLCPP_INFO(get_logger(), "Target Service call successful");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Target Service call failed");
        }
    }

    void display_menu()
    {
        RCLCPP_INFO(get_logger(), "MENU");
        RCLCPP_INFO(get_logger(), "w - UP");
        RCLCPP_INFO(get_logger(), "s - DOWN");
        RCLCPP_INFO(get_logger(), "q - Exit");
    }

    void input_loop()
    {
        char key;
        std::cin >> key;
        float max_speed = 99999.9;

        while (key != 'q')
        {
            std::cin >> key;

            if (key == 'w')
            {
                m1_value_ += 10;
                m2_value_ += 10;
                if (m1_value_ > max_speed)
                    m1_value_ = max_speed;
                if (m2_value_ > max_speed)
                    m2_value_ = max_speed;
            }
            else if (key == 's')
            {
                m1_value_ -= 10;
                m2_value_ -= 10;
                if (m1_value_ < max_speed)
                    m1_value_ = -1 * max_speed;
                if (m2_value_ < max_speed)
                    m2_value_ = -1 * max_speed;
            }

            if (js_arrived_m1_ && js_arrived_m2_)
            {
                RCLCPP_INFO(get_logger(), "Input Value = m1:%f m2:%f", m1_value_, m2_value_);
                RCLCPP_INFO(get_logger(), "Position Actual Value = m1:%f m2:%f", m1_joint_state_.position[0], m2_joint_state_.position[0]);
                js_arrived_m1_ = false;
                js_arrived_m2_ = false;
            }
        }
        RCLCPP_INFO(get_logger(), "Exiting...");
        shutdown_node();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Epos4_Control2_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}