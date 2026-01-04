#include <memory>
#include <cmath>
#include <vector>
#include <algorithm>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "jarabot_sim_interfaces/msg/ecd.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

// =========================
// Collision helper
// =========================
struct Segment
{
    double x1, y1, x2, y2;
};

static double clampd(double v, double lo, double hi)
{
    return std::max(lo, std::min(v, hi));
}

static double pointToSegmentDist(double px, double py,
                                 double x1, double y1,
                                 double x2, double y2)
{
    const double vx = x2 - x1;
    const double vy = y2 - y1;

    const double wx = px - x1;
    const double wy = py - y1;

    const double vv = vx * vx + vy * vy;
    if (vv < 1e-12) {
        const double dx = px - x1;
        const double dy = py - y1;
        return std::sqrt(dx * dx + dy * dy);
    }

    const double t = clampd((wx * vx + wy * vy) / vv, 0.0, 1.0);
    const double cx = x1 + t * vx;
    const double cy = y1 + t * vy;

    const double dx = px - cx;
    const double dy = py - cy;
    return std::sqrt(dx * dx + dy * dy);
}

static bool isCollision(double x, double y,
                        const std::vector<Segment> & segs,
                        double robot_radius,
                        double wall_margin)
{
    const double th = robot_radius + wall_margin;
    for (const auto & s : segs) {
        const double d = pointToSegmentDist(x, y, s.x1, s.y1, s.x2, s.y2);
        if (d <= th) {
            return true;
        }
    }
    return false;
}

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
        // ===== base params =====
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

        // ===== collision params =====
        // enable_collision: true면 벽 통과 금지
        // robot_radius: 로봇을 원으로 봤을 때 반지름(m)
        // wall_margin: 벽에서 추가로 멈추는 여유(m)
        // allow_rotate_when_blocked: 벽에 막혔을 때 회전은 허용할지
        declare_parameter("enable_collision", true);
        declare_parameter("robot_radius", 0.18);
        declare_parameter("wall_margin", 0.03);
        declare_parameter("allow_rotate_when_blocked", true);

        enable_collision_ = get_parameter("enable_collision").as_bool();
        robot_radius_ = get_parameter("robot_radius").as_double();
        wall_margin_ = get_parameter("wall_margin").as_double();
        allow_rotate_when_blocked_ = get_parameter("allow_rotate_when_blocked").as_bool();

        // ===== ROS I/O =====
        ecd_sub_ = create_subscription<jarabot_sim_interfaces::msg::Ecd>(
            "/ecd", 10,
            std::bind(&JaraOdometry::ecdCallback, this, std::placeholders::_1));

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // ===== Map segments (PNG 기반 맵) =====
        // NOTE:
        // - 이 segments_ 는 jara_sim_lidar.cpp에 넣은 맵과 동일해야 함.
        // - 좌표 범위: x,y = -6 ~ +6
        segments_.clear();

        const double SCALE = 0.75;
        auto S = [&](double x1,double y1,double x2,double y2){
            segments_.push_back({x1*SCALE, y1*SCALE,
                                x2*SCALE, y2*SCALE});
        };

        // ⬇️ 기존 push_back들을 S(...) 로 교체
        S(-4.802,  5.828,  5.380,  5.828);
        S(-4.118,  4.315,  3.497,  4.315);
        S( 0.588,  3.982,  1.893,  3.982);
        S(-4.075,  3.914,  3.626,  3.914);
        S(-0.717,  2.186,  3.968,  2.186);
        S(-3.861,  0.848, -2.663,  0.848);
        S(-3.882,  0.140, -2.620,  0.140);
        S(-3.797, -1.707,  0.995, -1.707);
        S( 0.845, -3.381,  3.968, -3.381);
        S(-6.000, -5.882,  6.000, -5.882);

        S(-5.818,  5.807, -5.818, -5.764);
        S(-3.704, -1.621, -3.704, -4.025);
        S(-1.337,  4.154, -1.337,  2.758);
        S( 0.888, -1.621,  0.888, -3.467);
        S( 3.882,  2.243,  3.882, -3.338);
        S( 5.925,  6.000,  5.925, -5.957);

        S(-1.422,  2.694, -0.738,  2.072);


        RCLCPP_INFO(get_logger(),
                    "JaraSimOdometry started | wheel_radius=%.3f, wheel_base=%.3f, ticks_per_rev=%d | collision=%s (r=%.2f, margin=%.2f, rotate_ok=%s)",
                    wheel_radius_, wheel_base_, ticks_per_rev_,
                    enable_collision_ ? "ON" : "OFF",
                    robot_radius_, wall_margin_,
                    allow_rotate_when_blocked_ ? "true" : "false");
    }

private:
    void ecdCallback(const jarabot_sim_interfaces::msg::Ecd::SharedPtr msg)
    {
        // 첫 메시지는 기준값만 저장
        if (!initialized_) {
            prev_left_ticks_  = msg->left_ticks;
            prev_right_ticks_ = msg->right_ticks;
            last_time_        = rclcpp::Time(msg->header.stamp);
            initialized_      = true;
            return;
        }

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

        const double dist_per_tick =
            2.0 * M_PI * wheel_radius_ / static_cast<double>(ticks_per_rev_);

        double d_left  = static_cast<double>(delta_left_ticks)  * dist_per_tick;
        double d_right = static_cast<double>(delta_right_ticks) * dist_per_tick;

        double d_center = (d_left + d_right) / 2.0;
        double d_theta  = (d_right - d_left) / wheel_base_;

        // ===== Predict next pose =====
        const double theta_mid = theta_ + d_theta / 2.0;

        double x_new = x_ + d_center * std::cos(theta_mid);
        double y_new = y_ + d_center * std::sin(theta_mid);
        double theta_new = theta_ + d_theta;

        // ===== Collision (block translation) =====
        if (enable_collision_) {
            if (isCollision(x_new, y_new, segments_, robot_radius_, wall_margin_)) {
                // 이동 차단
                x_new = x_;
                y_new = y_;
                d_center = 0.0; // publish v=0

                // 회전도 막고 싶으면 옵션 false로
                if (!allow_rotate_when_blocked_) {
                    theta_new = theta_;
                    d_theta = 0.0; // publish w=0
                }
            }
        }

        // ===== Apply =====
        x_ = x_new;
        y_ = y_new;
        theta_ = theta_new;

        // normalize yaw
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

        // covariance (간단)
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

    // ===== params =====
    double wheel_radius_;
    double wheel_base_;
    int    ticks_per_rev_;
    std::string frame_id_;
    std::string child_frame_id_;

    // collision params
    bool enable_collision_{true};
    bool allow_rotate_when_blocked_{true};
    double robot_radius_{0.18};
    double wall_margin_{0.03};

    // ===== state =====
    double x_;
    double y_;
    double theta_;
    bool   initialized_;
    int32_t prev_left_ticks_{0};
    int32_t prev_right_ticks_{0};
    rclcpp::Time last_time_;

    // ===== map =====
    std::vector<Segment> segments_;

    // ===== ROS =====
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
