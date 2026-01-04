#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"

struct Segment { double x1,y1,x2,y2; };

class JaraSimImage : public rclcpp::Node
{
public:
    JaraSimImage() : Node("jara_sim_image")
    {
        declare_parameter("map_size_m", 9.0);          // 전체 가로/세로 (m)
        declare_parameter("image_size_px", 600);        // 정사각 이미지
        declare_parameter("publish_hz", 10.0);
        declare_parameter("odom_topic", std::string("/odom"));
        declare_parameter("image_topic", std::string("/jara_sim/map_image"));
        declare_parameter("max_trail_points", 5000);

        map_size_m_ = get_parameter("map_size_m").as_double();
        img_size_   = get_parameter("image_size_px").as_int();
        publish_hz_ = get_parameter("publish_hz").as_double();
        odom_topic_ = get_parameter("odom_topic").as_string();
        image_topic_= get_parameter("image_topic").as_string();
        max_trail_  = get_parameter("max_trail_points").as_int();

        // ✅ PNG(sim_map_simple.png)와 동일한 맵 선분으로 교체
        buildSegmentsFromSimMapSimple();

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 20,
            std::bind(&JaraSimImage::odomCb, this, std::placeholders::_1));

        img_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, 10);

        // 베이스 맵(벽) 미리 렌더
        base_img_.assign(img_size_ * img_size_ * 3, 255); // 흰 배경
        drawWalls(base_img_);

        auto period = std::chrono::duration<double>(1.0 / publish_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&JaraSimImage::publish, this));

        RCLCPP_INFO(get_logger(), "JaraSimImage started: %s -> %s",
                    odom_topic_.c_str(), image_topic_.c_str());
    }

private:
    // =========================
    // ✅ sim_map_simple.png 기반 segments
    // 좌표계: 12m x 12m, x,y = -6 ~ +6
    // =========================
    void buildSegmentsFromSimMapSimple()
    {
        segments_.clear();

        const double SCALE = 0.75; // 9m / 12m

        auto S = [&](double x1,double y1,double x2,double y2){
            segments_.push_back({x1*SCALE, y1*SCALE, x2*SCALE, y2*SCALE});
        };

        // ---- horizontals ----
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

        // ---- verticals ----
        S(-5.818,  5.807, -5.818, -5.764);
        S(-3.704, -1.621, -3.704, -4.025);
        S(-1.337,  4.154, -1.337,  2.758);
        S( 0.888, -1.621,  0.888, -3.467);
        S( 3.882,  2.243,  3.882, -3.338);
        S( 5.925,  6.000,  5.925, -5.957);

        // ---- diagonal ----
        S(-1.422,  2.694, -0.738,  2.072);

    }

    void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        trail_.push_back({x, y});
        if ((int)trail_.size() > max_trail_) {
            trail_.erase(trail_.begin(), trail_.begin() + (trail_.size() - max_trail_));
        }
        has_odom_ = true;
    }

    // world (x,y) -> pixel (u,v)
    bool worldToPixel(double x, double y, int &u, int &v) const
    {
        const double half = map_size_m_ * 0.5;
        if (x < -half || x > half || y < -half || y > half) return false;

        double nx = (x + half) / map_size_m_; // 0..1
        double ny = (y + half) / map_size_m_; // 0..1

        u = (int)std::round(nx * (img_size_ - 1));
        v = (int)std::round((1.0 - ny) * (img_size_ - 1)); // y축 반전
        return true;
    }

    void setPixel(std::vector<uint8_t> &img, int u, int v, uint8_t r, uint8_t g, uint8_t b)
    {
        if (u < 0 || v < 0 || u >= img_size_ || v >= img_size_) return;
        size_t idx = (size_t)(v * img_size_ + u) * 3;
        img[idx+0] = r;
        img[idx+1] = g;
        img[idx+2] = b;
    }

    // Bresenham line
    void drawLine(std::vector<uint8_t> &img, int x0, int y0, int x1, int y1,
                  uint8_t r, uint8_t g, uint8_t b)
    {
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;

        while (true) {
            setPixel(img, x0, y0, r, g, b);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    void drawWalls(std::vector<uint8_t> &img)
    {
        for (const auto &s : segments_) {
            int u0,v0,u1,v1;
            if (!worldToPixel(s.x1, s.y1, u0, v0)) continue;
            if (!worldToPixel(s.x2, s.y2, u1, v1)) continue;
            drawLine(img, u0, v0, u1, v1, 0, 0, 0); // black wall
        }
    }

    void publish()
    {
        if (!has_odom_) return;

        std::vector<uint8_t> img = base_img_;

        // trail: blue
        for (size_t i = 1; i < trail_.size(); ++i) {
            int u0,v0,u1,v1;
            if (!worldToPixel(trail_[i-1].first, trail_[i-1].second, u0, v0)) continue;
            if (!worldToPixel(trail_[i].first,   trail_[i].second,   u1, v1)) continue;
            drawLine(img, u0, v0, u1, v1, 0, 80, 255);
        }

        // robot: red dot
        int ur, vr;
        if (worldToPixel(trail_.back().first, trail_.back().second, ur, vr)) {
            for (int du=-2; du<=2; ++du)
                for (int dv=-2; dv<=2; ++dv)
                    setPixel(img, ur+du, vr+dv, 255, 0, 0);
        }

        sensor_msgs::msg::Image msg;
        msg.header.stamp = now();
        msg.header.frame_id = "map"; // 표기용
        msg.height = img_size_;
        msg.width  = img_size_;
        msg.encoding = "rgb8";
        msg.is_bigendian = false;
        msg.step = img_size_ * 3;
        msg.data = std::move(img);

        img_pub_->publish(msg);
    }

private:
    double map_size_m_{9.0};
    int img_size_{600};
    double publish_hz_{10.0};
    std::string odom_topic_{"/odom"};
    std::string image_topic_{"/jara_sim/map_image"};
    int max_trail_{5000};

    bool has_odom_{false};
    std::vector<Segment> segments_;
    std::vector<uint8_t> base_img_;
    std::vector<std::pair<double,double>> trail_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JaraSimImage>());
    rclcpp::shutdown();
    return 0;
}
