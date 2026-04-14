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

class Epos4_Vel_Node : public rclcpp::Node
{
public:
    Epos4_Vel_Node()
        : Node("epos4_cmd_vel_node")
    {
        client_driver_init_ = this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/init");
        client_driver_halt_ = this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/halt");
        client_driver_recover_ = this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/recover");
        client_driver_shutdown_ = this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/shutdown");
        client_driver_enable_ = this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/enable");
        client_driver_disable_ = this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/disable");
        client_driver_vel_mode_ = this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/velocity_mode");
        client_driver_csv_mode_ = this->create_client<std_srvs::srv::Trigger>("/motor1/cia402_device_1/cyclic_velocity_mode");

        client_target_ = this->create_client<canopen_interfaces::srv::COTargetDouble>("/motor1/cia402_device_1/target");

        tpdo_publisher_ = this->create_publisher<canopen_interfaces::msg::COData>("/motor1/cia402_device_1/tpdo", 10);

        sine_wave_timer_ = this->create_wall_timer(10ms, std::bind(&Epos4_Vel_Node::timer_callback, this));

        subscription_ = create_subscription<sensor_msgs::msg::JointState>(
            "/motor1/cia402_device_1/joint_states", 10,
            std::bind(&Epos4_Vel_Node::jointStateCallback, this, std::placeholders::_1));

        input_thread_ = std::thread(&Epos4_Vel_Node::input_loop, this);

        RCLCPP_INFO(get_logger(), "********************************************");
        RCLCPP_INFO(get_logger(), "maxon EPOS4 Control (velocity)");
        RCLCPP_INFO(get_logger(), "To be used with 1 EPOS4: /cia402_device_1");
        RCLCPP_INFO(get_logger(), "run first bus_config_cia402_epos4_vel.launch.py");
        RCLCPP_INFO(get_logger(), "********************************************");

        display_menu();
    }

    ~Epos4_Vel_Node()
    {
        if (input_thread_.joinable())
        {
            input_thread_.join();
        }
    }

private:
    int counter = 0;
    bool js_arrived_ = false;
    double sine_wave_time_ = 3.0;
    bool csv_active = false;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_driver_init_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_driver_halt_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_driver_recover_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_driver_shutdown_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_driver_enable_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_driver_disable_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_driver_vel_mode_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_driver_csv_mode_;

    rclcpp::Client<canopen_interfaces::srv::COTargetDouble>::SharedPtr client_target_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Publisher<canopen_interfaces::msg::COData>::SharedPtr tpdo_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

    rclcpp::TimerBase::SharedPtr sine_wave_timer_;

    sensor_msgs::msg::JointState joint_state_;
    std::thread input_thread_;

    void shutdown_node()
    {
        RCLCPP_INFO(get_logger(), "Exit node ");
        rclcpp::shutdown();
    }

    void timer_callback()
    {
        double frequency = 0.25;
        double amplitude = 2000.0;
        double sine_value = amplitude * (std::sin(2 * M_PI * frequency * sine_wave_time_)) / 2.0;
        auto tpdo_msg = canopen_interfaces::msg::COData();

        sine_wave_time_ += 0.01;
        if (sine_wave_time_ >= 4.0)
        {
            sine_wave_time_ = 0.0;
        }

        if (csv_active == true)
        {
            tpdo_msg.index = 0x60FF;
            tpdo_msg.subindex = 0x00;
            tpdo_msg.data = static_cast<int>(sine_value);
            tpdo_publisher_->publish(tpdo_msg);
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        joint_state_ = *msg;
        js_arrived_ = true;
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
            std::bind(&Epos4_Vel_Node::trigger_callback, this, std::placeholders::_1));
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

    void call_target_vel_service(double target_vel_value)
    {
        //-----
        if (!std::isfinite(target_vel_value))
        {
            RCLCPP_ERROR(get_logger(), "Target velocity must be a finite number");
            return;
        }

        if (target_vel_value != 0.0 && std::fabs(target_vel_value) < 1.0)
        {
            RCLCPP_ERROR(get_logger(), "Target velocity is too small. Enter an integer rpm value, for example 100 or -100.");
            return;
        }

        const double target_vel_rpm = static_cast<double>(std::lround(target_vel_value));
        if (target_vel_rpm != target_vel_value)
        {
            RCLCPP_WARN(get_logger(), "Target velocity rounded from %f to %.0f rpm", target_vel_value, target_vel_rpm);
        }
        //-----

        if (!client_target_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(get_logger(), "Service 'target' not available");
            return;
        }

        auto request = std::make_shared<canopen_interfaces::srv::COTargetDouble::Request>();
        request->target = target_vel_rpm;   //-----

        client_target_->async_send_request(
            request,
            std::bind(&Epos4_Vel_Node::target_callback, this, std::placeholders::_1));
    }

    void display_menu()
    {
        RCLCPP_INFO(get_logger(), "MENU");
        RCLCPP_INFO(get_logger(), "1 - driver init");
        RCLCPP_INFO(get_logger(), "2 - driver halt");
        RCLCPP_INFO(get_logger(), "3 - driver enable");
        RCLCPP_INFO(get_logger(), "4 - driver disable");
        RCLCPP_INFO(get_logger(), "5 - velocity mode");
        RCLCPP_INFO(get_logger(), "6 - set Target velocity ");
        RCLCPP_INFO(get_logger(), "7 - get Position Actual Value");
        RCLCPP_INFO(get_logger(), "8 - cyclic velocity mode");
        RCLCPP_INFO(get_logger(), "9 - Exit");
    }

    void input_loop()
    {
        int menu_i = 0;
        bool exit = false;

        while (rclcpp::ok() && !exit)
        {
            RCLCPP_INFO(get_logger(), "Please enter a menu choice:");
            std::cin >> menu_i;

            switch (menu_i)
            {
            case 1:
                call_trigger_service(client_driver_init_, "init");
                break;
            case 2:
                call_trigger_service(client_driver_halt_, "halt");
                break;
            case 3:
                call_trigger_service(client_driver_enable_, "enable");
                break;
            case 4:
                call_trigger_service(client_driver_disable_, "disable");
                break;
            case 5:
                csv_active = false;
                call_trigger_service(client_driver_vel_mode_, "velocity_mode");
                break;
            case 6:
            {
                double target_vel_value_input;
                RCLCPP_INFO(get_logger(), "Enter a target velocity value:");
                std::cin >> target_vel_value_input;
                call_target_vel_service(target_vel_value_input);
                break;
            }
            case 7:
                if (js_arrived_)
                {
                    RCLCPP_INFO(get_logger(), "Position Actual Value = %f", joint_state_.position[0]);
                    js_arrived_ = false;
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "No joint state data received yet");
                }
                break;
            case 8:
                csv_active = true;
                sine_wave_time_ = 3.0;
                call_trigger_service(client_driver_csv_mode_, "cyclic_velocity_mode");
                break;
            case 9:
                exit = true;
                RCLCPP_INFO(get_logger(), "Exiting...");
                shutdown_node();
                break;
            default:
                RCLCPP_ERROR(get_logger(), "Please enter a correct menu value!");
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Epos4_Vel_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
