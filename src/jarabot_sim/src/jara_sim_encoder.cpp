#include <chrono>
#include <cmath>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "jarabot_sim_interfaces/msg/ecd.hpp"

using namespace std::chrono_literals;

class JaraSimEncoder : public rclcpp::Node
{
public:
    JaraSimEncoder()
        : Node("jara_sim_encoder"),
          wheel_radius_(0.035),
          wheel_base_(0.220),
          ticks_per_rev_(1000),
          loop_rate_hz_(50.0),
          max_linear_speed_(0.4),
          max_angular_speed_(1.5),
          encoder_noise_std_(0.5),
          left_ticks_(0),
          right_ticks_(0),
          rng_(std::random_device{}()),
          noise_dist_(0.0, 1.0)
    {
        declare_parameter("wheel_radius", wheel_radius_);
        declare_parameter("wheel_base", wheel_base_);
        declare_parameter("ticks_per_rev", ticks_per_rev_);
        declare_parameter("loop_rate_hz", loop_rate_hz_);
        declare_parameter("max_linear_speed", max_linear_speed_);
        declare_parameter("max_angular_speed", max_angular_speed_);
        declare_parameter("encoder_noise_std", encoder_noise_std_);

        wheel_radius_      = get_parameter("wheel_radius").as_double();
        wheel_base_        = get_parameter("wheel_base").as_double();
        ticks_per_rev_     = get_parameter("ticks_per_rev").as_int();
        loop_rate_hz_      = get_parameter("loop_rate_hz").as_double();
        max_linear_speed_  = get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = get_parameter("max_angular_speed").as_double();
        encoder_noise_std_ = get_parameter("encoder_noise_std").as_double();

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&JaraSimEncoder::cmdVelCallback, this, std::placeholders::_1));

        ecd_pub_ = create_publisher<jarabot_sim_interfaces::msg::Ecd>("/ecd", 10);

        auto period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&JaraSimEncoder::update, this));

        last_time_ = now();

        RCLCPP_INFO(get_logger(),
                    "JaraSimEncoder started (wheel_radius=%.3f, wheel_base=%.3f, ticks_per_rev=%d)",
                    wheel_radius_, wheel_base_, ticks_per_rev_);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        current_linear_ = msg->linear.x;
        current_angular_ = msg->angular.z;
    }

    void update()
    {
        rclcpp::Time now_time = now();
        double dt = (now_time - last_time_).seconds();
        if (dt <= 0.0) return;
        last_time_ = now_time;

        double v = std::clamp(current_linear_,  -max_linear_speed_,  max_linear_speed_);
        double w = std::clamp(current_angular_, -max_angular_speed_, max_angular_speed_);

        double v_left  = v - (w * wheel_base_ / 2.0);
        double v_right = v + (w * wheel_base_ / 2.0);

        double omega_left  = v_left  / wheel_radius_;
        double omega_right = v_right / wheel_radius_;

        double dtheta_left  = omega_left  * dt;
        double dtheta_right = omega_right * dt;

        double rev_left  = dtheta_left  / (2.0 * M_PI);
        double rev_right = dtheta_right / (2.0 * M_PI);

        double raw_ticks_left  = rev_left  * ticks_per_rev_;
        double raw_ticks_right = rev_right * ticks_per_rev_;

        raw_ticks_left  += noise_dist_(rng_) * encoder_noise_std_;
        raw_ticks_right += noise_dist_(rng_) * encoder_noise_std_;

        int32_t delta_ticks_left  = static_cast<int32_t>(std::round(raw_ticks_left));
        int32_t delta_ticks_right = static_cast<int32_t>(std::round(raw_ticks_right));

        left_ticks_  += delta_ticks_left;
        right_ticks_ += delta_ticks_right;

        jarabot_sim_interfaces::msg::Ecd msg;
        msg.header.stamp = now_time;
        msg.header.frame_id = "base_link";

        msg.left_ticks  = left_ticks_;
        msg.right_ticks = right_ticks_;

        ecd_pub_->publish(msg);
    }

    double wheel_radius_;
    double wheel_base_;
    int    ticks_per_rev_;
    double loop_rate_hz_;
    double max_linear_speed_;
    double max_angular_speed_;
    double encoder_noise_std_;

    double current_linear_  = 0.0;
    double current_angular_ = 0.0;
    int32_t left_ticks_;
    int32_t right_ticks_;
    rclcpp::Time last_time_;

    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<jarabot_sim_interfaces::msg::Ecd>::SharedPtr ecd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JaraSimEncoder>());
    rclcpp::shutdown();
    return 0;
}
