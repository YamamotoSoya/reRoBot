#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace
{
struct TerminalGuard
{
    TerminalGuard()
    {
        if (tcgetattr(STDIN_FILENO, &saved_) == 0) {
            saved_valid_ = true;
            termios raw = saved_;
            raw.c_lflag &= ~(ICANON | ECHO);
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        }
    }
    ~TerminalGuard() { restore(); }
    void restore()
    {
        if (saved_valid_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
            saved_valid_ = false;
        }
    }
    termios saved_{};
    bool saved_valid_ = false;
};
}  // namespace

class TeleopKeyboardNode : public rclcpp::Node
{
public:
    TeleopKeyboardNode() : Node("teleop_keyboard_node")
    {
        tire_diam_       = declare_parameter("tire_diam", 0.15);
        gear_ratio_      = declare_parameter("gear_ratio", 1.0);
        invert_left_     = declare_parameter("invert_left", false);
        invert_right_    = declare_parameter("invert_right", false);
        linear_step_     = declare_parameter("linear_step", 0.1);
        angular_step_    = declare_parameter("angular_step", 0.3);
        max_linear_      = declare_parameter("max_linear", 1.0);
        max_angular_     = declare_parameter("max_angular", 2.0);
        double rate_hz   = declare_parameter("publish_rate_hz", 20.0);
        auto cmd_topic   = declare_parameter("cmd_topic", std::string("/robot_speed_cmd"));
        auto m1_topic    = declare_parameter("m1_joint_topic",
                                             std::string("/motor1/cia402_device_1/joint_states"));
        auto m2_topic    = declare_parameter("m2_joint_topic",
                                             std::string("/motor2/cia402_device_2/joint_states"));

        cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

        m1_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
            m1_topic, 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                updateDistance(msg, /*is_left=*/true);
            });
        m2_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
            m2_topic, 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                updateDistance(msg, /*is_left=*/false);
            });

        const auto period = std::chrono::milliseconds(
            static_cast<int>(1000.0 / std::max(1.0, rate_hz)));
        timer_ = create_wall_timer(period, std::bind(&TeleopKeyboardNode::onTick, this));

        printHelp();
    }

    ~TeleopKeyboardNode() override
    {
        // final zero command so the controller stops the motors
        auto stop = geometry_msgs::msg::Twist();
        cmd_publisher_->publish(stop);
    }

private:
    void printHelp()
    {
        std::printf("\n");
        std::printf("================ reRoBot Keyboard Teleop ================\n");
        std::printf("  w / s : linear  +/- (step=%.2f m/s, max=%.2f)\n", linear_step_, max_linear_);
        std::printf("  a / d : angular +/- (step=%.2f rad/s, max=%.2f)\n", angular_step_, max_angular_);
        std::printf("  space : stop\n");
        std::printf("  + / - : scale step size x1.1 / /1.1\n");
        std::printf("  r     : reset wheel distance counters\n");
        std::printf("  q     : quit\n");
        std::printf("=========================================================\n\n");
        std::fflush(stdout);
    }

    void onTick()
    {
        handleKey();

        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = linear_;
        cmd.angular.z = angular_;
        cmd_publisher_->publish(cmd);

        render();
    }

    void handleKey()
    {
        char ch = 0;
        ssize_t n = ::read(STDIN_FILENO, &ch, 1);
        if (n <= 0) return;

        switch (ch) {
            case 'w': linear_  = std::min(max_linear_,  linear_  + linear_step_);  break;
            case 's': linear_  = std::max(-max_linear_, linear_  - linear_step_);  break;
            case 'a': angular_ = std::min(max_angular_, angular_ + angular_step_); break;
            case 'd': angular_ = std::max(-max_angular_, angular_ - angular_step_); break;
            case ' ':
            case 'x':
                linear_ = 0.0;
                angular_ = 0.0;
                break;
            case '+':
            case '=':
                linear_step_  *= 1.1;
                angular_step_ *= 1.1;
                break;
            case '-':
            case '_':
                linear_step_  /= 1.1;
                angular_step_ /= 1.1;
                break;
            case 'r':
                {
                    std::lock_guard<std::mutex> lk(dist_mutex_);
                    dist_left_ = 0.0;
                    dist_right_ = 0.0;
                }
                break;
            case 'q':
            case 3: // Ctrl-C: also caught by rclcpp's SIGINT handler
                {
                    auto stop = geometry_msgs::msg::Twist();
                    cmd_publisher_->publish(stop);
                }
                rclcpp::shutdown();
                break;
            default:
                break;
        }
    }

    void updateDistance(const sensor_msgs::msg::JointState::SharedPtr & msg, bool is_left)
    {
        if (msg->position.empty()) return;

        // joint_states.position is motor-shaft angle [rad] (bus.yml applies
        // scale_pos_from_dev_ = 2*pi/counts_per_rev before publishing).
        const double pos_rad = msg->position.front();
        const double inv_gear = (gear_ratio_ > 0.0) ? (1.0 / gear_ratio_) : 1.0;
        const double wheel_radius = tire_diam_ * 0.5;

        std::lock_guard<std::mutex> lk(dist_mutex_);
        double & prev      = is_left ? prev_pos_left_  : prev_pos_right_;
        bool   & have_prev = is_left ? have_prev_left_ : have_prev_right_;
        double & dist      = is_left ? dist_left_      : dist_right_;
        const bool invert  = is_left ? invert_left_    : invert_right_;

        if (!have_prev) {
            prev = pos_rad;
            have_prev = true;
            return;
        }
        double d_motor_rad = pos_rad - prev;
        prev = pos_rad;
        if (invert) d_motor_rad = -d_motor_rad;

        // motor rad -> wheel rad -> arc length
        const double d_wheel_rad = d_motor_rad * inv_gear;
        dist += d_wheel_rad * wheel_radius;
    }

    void render()
    {
        double l_m, r_m;
        {
            std::lock_guard<std::mutex> lk(dist_mutex_);
            l_m = dist_left_;
            r_m = dist_right_;
        }
        // \r overwrites current line; don't use \n so the status stays pinned.
        std::printf(
            "\r[cmd v=%+6.2f m/s w=%+6.2f rad/s | step v=%.3f w=%.3f] "
            "left=%+8.3f m  right=%+8.3f m   ",
            linear_, angular_, linear_step_, angular_step_, l_m, r_m);
        std::fflush(stdout);
    }

    // parameters
    double tire_diam_;
    double gear_ratio_;
    bool   invert_left_;
    bool   invert_right_;
    double linear_step_;
    double angular_step_;
    double max_linear_;
    double max_angular_;

    // command state
    double linear_  = 0.0;
    double angular_ = 0.0;

    // distance state
    std::mutex dist_mutex_;
    bool   have_prev_left_  = false;
    bool   have_prev_right_ = false;
    double prev_pos_left_  = 0.0;
    double prev_pos_right_ = 0.0;
    double dist_left_  = 0.0;
    double dist_right_ = 0.0;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m1_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m2_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    TerminalGuard term_guard;  // raw stdin for the lifetime of the program
    auto node = std::make_shared<TeleopKeyboardNode>();
    rclcpp::spin(node);
    term_guard.restore();
    std::printf("\n");
    rclcpp::shutdown();
    return 0;
}
