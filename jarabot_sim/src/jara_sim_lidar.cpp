#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <limits>
#include <random>
#include <string>

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
          angle_max_( 3.0 * M_PI / 4.0),
          angle_increment_(0.0058),
          range_min_(0.05),
          range_max_(8.0),
          publish_rate_hz_(10.0),
          range_noise_std_(0.01),
          frame_id_("base_link"),
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
        declare_parameter("frame_id", frame_id_);

        angle_min_       = get_parameter("angle_min").as_double();
        angle_max_       = get_parameter("angle_max").as_double();
        angle_increment_ = get_parameter("angle_increment").as_double();
        range_min_       = get_parameter("range_min").as_double();
        range_max_       = get_parameter("range_max").as_double();
        publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
        range_noise_std_ = get_parameter("range_noise_std").as_double();
        frame_id_        = get_parameter("frame_id").as_string();

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&JaraSimLidar::odomCallback, this, std::placeholders::_1));

        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&JaraSimLidar::publishScan, this));

        // =========================================================
        // Map from sim_map_simple.png  (원본 12m: x,y -6~+6)
        // ✅ 9m x 9m 로 비례 축소: SCALE=0.75 적용
        // =========================================================
        segments_.clear();

        // ✅ 여기 추가: 스케일 적용 헬퍼
        const double SCALE = 0.75;   // 9.0 / 12.0
        auto S = [&](double x1, double y1, double x2, double y2) {
            segments_.push_back({x1 * SCALE, y1 * SCALE, x2 * SCALE, y2 * SCALE});
        };

        // ---- horizontals ----
        S(-4.802,  5.828,  5.380,  5.828);  // top long
        S(-4.118,  4.315,  3.497,  4.315);  // top corridor
        S( 0.588,  3.982,  1.893,  3.982);  // top small block (part)
        S(-4.075,  3.914,  3.626,  3.914);  // top inside long
        S(-0.717,  2.186,  3.968,  2.186);  // mid horizontal
        S(-3.861,  0.848, -2.663,  0.848);  // left obstacle top
        S(-3.882,  0.140, -2.620,  0.140);  // left obstacle bottom
        S(-3.797, -1.707,  0.995, -1.707);  // lower long
        S( 0.845, -3.381,  3.968, -3.381);  // lower right
        S(-6.000, -5.882,  6.000, -5.882);  // bottom border

        // ---- verticals ----
        S(-5.818,  5.807, -5.818, -5.764);  // left border
        S(-3.704, -1.621, -3.704, -4.025);  // left lower vertical
        S(-1.337,  4.154, -1.337,  2.758);  // inner vertical (left-top)
        S( 0.888, -1.621,  0.888, -3.467);  // inner vertical (center-bottom)
        S( 3.882,  2.243,  3.882, -3.338);  // right inner long vertical
        S( 5.925,  6.000,  5.925, -5.957);  // right border

        // ---- diagonal ---- (top-left chamfer)
        S(-1.422,  2.694, -0.738,  2.072);

        RCLCPP_INFO(get_logger(),
                    "JaraSimLidar started (map scaled to 9m x 9m, SCALE=0.75, frame_id=%s)",
                    frame_id_.c_str());
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

    double raycastSegment(double ox, double oy, double dx, double dy, const Segment & seg)
    {
        const double x1 = seg.x1;
        const double y1 = seg.y1;
        const double x2 = seg.x2;
        const double y2 = seg.y2;

        const double sx = x2 - x1;
        const double sy = y2 - y1;

        const double det = (-dx * sy + dy * sx);
        if (std::fabs(det) < 1e-12) {
            return std::numeric_limits<double>::infinity();
        }

        const double rx = x1 - ox;
        const double ry = y1 - oy;

        const double t = (-sy * rx + sx * ry) / det;
        const double u = ( dx * ry - dy * rx) / det;

        if (t >= 0.0 && u >= 0.0 && u <= 1.0) {
            return t;
        }
        return std::numeric_limits<double>::infinity();
    }

    void publishScan()
    {
        if (!has_odom_) return;

        auto stamp = now();

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = stamp;
        scan.header.frame_id = frame_id_;

        scan.angle_min = angle_min_;
        scan.angle_max = angle_max_;
        scan.angle_increment = angle_increment_;
        scan.range_min = range_min_;
        scan.range_max = range_max_;
        scan.scan_time = 1.0 / publish_rate_hz_;
        scan.time_increment = 0.0;

        const int beam_count =
            static_cast<int>((angle_max_ - angle_min_) / angle_increment_) + 1;
        scan.ranges.resize(beam_count);

        for (int i = 0; i < beam_count; ++i) {
            const double angle = angle_min_ + i * angle_increment_;
            const double global_angle = theta_ + angle;

            const double dx = std::cos(global_angle);
            const double dy = std::sin(global_angle);

            double min_t = std::numeric_limits<double>::infinity();

            for (const auto & seg : segments_) {
                const double t = raycastSegment(x_, y_, dx, dy, seg);
                if (t < min_t) min_t = t;
            }

            double range = range_max_;
            if (std::isfinite(min_t)) {
                range = min_t;
            }

            range = std::max(range_min_, std::min(range, range_max_));
            range += noise_dist_(rng_) * range_noise_std_;
            range = std::max(range_min_, std::min(range, range_max_));

            scan.ranges[i] = static_cast<float>(range);
        }

        scan_pub_->publish(scan);
    }

    // params
    double angle_min_;
    double angle_max_;
    double angle_increment_;
    double range_min_;
    double range_max_;
    double publish_rate_hz_;
    double range_noise_std_;
    std::string frame_id_;

    // state
    double x_{0.0};
    double y_{0.0};
    double theta_{0.0};
    bool has_odom_;

    // world walls
    std::vector<Segment> segments_;

    // noise
    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_;

    // ros
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
