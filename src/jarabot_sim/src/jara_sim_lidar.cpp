#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <limits>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

struct Segment
{
    double x1, y1, x2, y2;
};

class JaraSimLidar : public rclcpp::Node
{
public:
    JaraSimLidar()
        : Node("jara_sim_lidar"),
          angle_min_(-3.0 * M_PI / 4.0),
          angle_max_(3.0 * M_PI / 4.0),
          angle_increment_(0.0058),
          range_min_(0.05),
          range_max_(8.0),
          publish_rate_hz_(10.0),
          range_noise_std_(0.01),
          has_odom_(false),
          rng_(std::random_device{}()),
          noise_dist_(0.0, 1.0)
    {
        declare_parameter("angle_min", angle_min_);
        declare_parameter("angle_max", angle_max_);
        declare_parameter("angle_increment", angle_increment_);
        declare_parameter("range_min", range_min_);
        declare_parameter("range_max", range_max_);
        declare_parameter("publish_rate_hz", publish_rate_hz_);
        declare_parameter("range_noise_std", range_noise_std_);

        angle_min_        = get_parameter("angle_min").as_double();
        angle_max_        = get_parameter("angle_max").as_double();
        angle_increment_  = get_parameter("angle_increment").as_double();
        range_min_        = get_parameter("range_min").as_double();
        range_max_        = get_parameter("range_max").as_double();
        publish_rate_hz_  = get_parameter("publish_rate_hz").as_double();
        range_noise_std_  = get_parameter("range_noise_std").as_double();

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&JaraSimLidar::odomCallback, this, std::placeholders::_1));

        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&JaraSimLidar::publishScan, this));

        segments_.push_back({-3.0, -3.0, -3.0,  3.0});
        segments_.push_back({ 3.0, -3.0,  3.0,  3.0});
        segments_.push_back({-3.0, -3.0,  3.0, -3.0});
        segments_.push_back({-3.0,  3.0,  3.0,  3.0});

        RCLCPP_INFO(get_logger(), "JaraSimLidar started (virtual room [-3,3]x[-3,3])");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;

        const auto & q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        theta_ = std::atan2(siny_cosp, cosy_cosp);

        has_odom_ = true;
    }

    double raycastSegment(double dx, double dy, const Segment & seg)
    {
        double sx = seg.x2 - seg.x1;
        double sy = seg.y2 - seg.y1;

        double det = (-dx * sy + dy * sx);
        if (std::fabs(det) < 1e-9) {
            return std::numeric_limits<double>::infinity();
        }

        double s = (-sy * (seg.x1) + sx * (seg.y1)) / det;
        double t = (dx * (seg.y1) - dy * (seg.x1)) / det;

        if (s >= 0.0 && t >= 0.0 && t <= 1.0) {
            return s;
        }
        return std::numeric_limits<double>::infinity();
    }

    void publishScan()
    {
        if (!has_odom_) return;

        auto now_time = now();

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = now_time;
        scan.header.frame_id = "base_link";

        scan.angle_min = angle_min_;
        scan.angle_max = angle_max_;
        scan.angle_increment = angle_increment_;
        scan.range_min = range_min_;
        scan.range_max = range_max_;
        scan.scan_time = 1.0 / publish_rate_hz_;
        scan.time_increment = 0.0;

        int beam_count = static_cast<int>((angle_max_ - angle_min_) / angle_increment_) + 1;
        scan.ranges.resize(beam_count);

        for (int i = 0; i < beam_count; ++i) {
            double angle = angle_min_ + i * angle_increment_;
            double global_angle = theta_ + angle;

            double dx = std::cos(global_angle);
            double dy = std::sin(global_angle);

            double min_t = std::numeric_limits<double>::infinity();

            for (const auto & seg_world : segments_) {
                Segment seg_robot;
                seg_robot.x1 = seg_world.x1 - x_;
                seg_robot.y1 = seg_world.y1 - y_;
                seg_robot.x2 = seg_world.x2 - x_;
                seg_robot.y2 = seg_world.y2 - y_;

                double t = raycastSegment(dx, dy, seg_robot);
                if (t < min_t) {
                    min_t = t;
                }
            }

            double range = range_max_;
            if (std::isfinite(min_t)) {
                range = min_t;
            }

            if (range < range_min_) range = range_min_;
            if (range > range_max_) range = range_max_;

            range += noise_dist_(rng_) * range_noise_std_;
            if (range < range_min_) range = range_min_;
            if (range > range_max_) range = range_max_;

            scan.ranges[i] = static_cast<float>(range);
        }

        scan_pub_->publish(scan);
    }

    double angle_min_;
    double angle_max_;
    double angle_increment_;
    double range_min_;
    double range_max_;
    double publish_rate_hz_;
    double range_noise_std_;

    double x_{0.0};
    double y_{0.0};
    double theta_{0.0};
    bool has_odom_;

    std::vector<Segment> segments_;

    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JaraSimLidar>());
    rclcpp::shutdown();
    return 0;
}
