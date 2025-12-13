#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "jarabot_sim_interfaces/msg/ecd.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class JaraOdometry : public rclcpp::Node
{
public:
    JaraOdometry()
        : Node("jara_sim_odometry"),
          wheel_radius_(0.035),
          wheel_base_(0.220),
          ticks_per_rev_(1000),
          x_(0.0),
          y_(0.0),
          theta_(0.0),
          initialized_(false)
    {
        declare_parameter("wheel_radius", wheel_radius_);
        declare_parameter("wheel_base", wheel_base_);
        declare_parameter("ticks_per_rev", ticks_per_rev_);
        declare_parameter("frame_id", std::string("odom"));
        declare_parameter("child_frame_id", std::string("base_link"));

        wheel_radius_   = get_parameter("wheel_radius").as_double();
        wheel_base_     = get_parameter("wheel_base").as_double();
        ticks_per_rev_  = get_parameter("ticks_per_rev").as_int();
        frame_id_       = get_parameter("frame_id").as_string();
        child_frame_id_ = get_parameter("child_frame_id").as_string();

        ecd_sub_ = create_subscription<jarabot_sim_interfaces::msg::Ecd>(
            "/ecd", 10,
            std::bind(&JaraOdometry::ecdCallback, this, std::placeholders::_1));

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(get_logger(),
                    "JaraSimOdometry started (wheel_radius=%.3f, wheel_base=%.3f, ticks_per_rev=%d)",
                    wheel_radius_, wheel_base_, ticks_per_rev_);
    }

private:
    void ecdCallback(const jarabot_sim_interfaces::msg::Ecd::SharedPtr msg)
    {
        // 첫 메시지는 기준값만 저장
        if (!initialized_) {
            prev_left_ticks_  = msg->left_ticks;
            prev_right_ticks_ = msg->right_ticks;
            last_time_        = rclcpp::Time(msg->header.stamp);  // builtin Time → rclcpp::Time
            initialized_      = true;
            return;
        }

        // builtin_interfaces::msg::Time 를 rclcpp::Time으로 감싸서 사용
        rclcpp::Time current_time(msg->header.stamp);
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) {
            last_time_ = current_time;
            return;
        }
        last_time_ = current_time;

        int32_t curr_left_ticks  = msg->left_ticks;
        int32_t curr_right_ticks = msg->right_ticks;

        int32_t delta_left_ticks  = curr_left_ticks  - prev_left_ticks_;
        int32_t delta_right_ticks = curr_right_ticks - prev_right_ticks_;

        prev_left_ticks_  = curr_left_ticks;
        prev_right_ticks_ = curr_right_ticks;

        double dist_per_tick = 2.0 * M_PI * wheel_radius_ / static_cast<double>(ticks_per_rev_);
        double d_left  = static_cast<double>(delta_left_ticks)  * dist_per_tick;
        double d_right = static_cast<double>(delta_right_ticks) * dist_per_tick;

        double d_center = (d_left + d_right) / 2.0;
        double d_theta  = (d_right - d_left) / wheel_base_;

        double theta_mid = theta_ + d_theta / 2.0;
        x_     += d_center * std::cos(theta_mid);
        y_     += d_center * std::sin(theta_mid);
        theta_ += d_theta;

        theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

        publishOdometry(current_time, d_center / dt, d_theta / dt);
    }

    void publishOdometry(const rclcpp::Time & stamp, double v, double w)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = frame_id_;
        odom.child_frame_id  = child_frame_id_;

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x  = v;
        odom.twist.twist.linear.y  = 0.0;
        odom.twist.twist.angular.z = w;

        for (int i = 0; i < 36; ++i) {
            odom.pose.covariance[i]  = 0.0;
            odom.twist.covariance[i] = 0.0;
        }
        odom.pose.covariance[0]  = 0.001;
        odom.pose.covariance[7]  = 0.001;
        odom.pose.covariance[35] = 0.01;

        odom.twist.covariance[0]  = 0.001;
        odom.twist.covariance[7]  = 0.001;
        odom.twist.covariance[35] = 0.01;

        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = frame_id_;
        tf_msg.child_frame_id  = child_frame_id_;

        tf_msg.transform.translation.x = x_;
        tf_msg.transform.translation.y = y_;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_msg);
    }

    // 파라미터
    double wheel_radius_;
    double wheel_base_;
    int    ticks_per_rev_;
    std::string frame_id_;
    std::string child_frame_id_;

    // 상태
    double x_;
    double y_;
    double theta_;
    bool   initialized_;
    int32_t prev_left_ticks_;
    int32_t prev_right_ticks_;
    rclcpp::Time last_time_;

    // ROS 인터페이스
    rclcpp::Subscription<jarabot_sim_interfaces::msg::Ecd>::SharedPtr ecd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JaraOdometry>());
    rclcpp::shutdown();
    return 0;
}
