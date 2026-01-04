#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class JaraSimMarker : public rclcpp::Node
{
public:
    JaraSimMarker()
        : Node("jara_sim_marker")
    {
        declare_parameter("frame_id", std::string("base_link"));
        frame_id_ = get_parameter("frame_id").as_string();

        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "/jara_robot_marker", 10);

        timer_ = create_wall_timer(
            100ms, std::bind(&JaraSimMarker::publishMarkers, this));

        RCLCPP_INFO(get_logger(), "JaraSimMarker started (frame_id=%s)",
                    frame_id_.c_str());
    }

private:
    void publishMarkers()
    {
        const rclcpp::Time now = get_clock()->now();

        // =========================
        // 1) 차체(CUBE)
        // =========================
        visualization_msgs::msg::Marker body;
        body.header.stamp = now;
        body.header.frame_id = frame_id_;
        body.ns = "jarabot";
        body.id = 0;
        body.type = visualization_msgs::msg::Marker::CUBE;
        body.action = visualization_msgs::msg::Marker::ADD;

        // 크기 확정 (m)
        body.scale.x = 0.30;  // 길이 30cm
        body.scale.y = 0.15;  // 폭   15cm
        body.scale.z = 0.09;  // 높이  9cm

        // 로봇 중심(바닥이 z=0이라면, 중심은 z=높이/2)
        body.pose.position.x = 0.0;
        body.pose.position.y = 0.0;
        body.pose.position.z = body.scale.z * 0.5;

        body.pose.orientation.x = 0.0;
        body.pose.orientation.y = 0.0;
        body.pose.orientation.z = 0.0;
        body.pose.orientation.w = 1.0;

        // 색(파랑, 불투명)
        body.color.r = 0.0f;
        body.color.g = 0.3f;
        body.color.b = 1.0f;
        body.color.a = 1.0f;

        body.lifetime = rclcpp::Duration(0, 0);
        marker_pub_->publish(body);

        // =========================
        // 2) 진행 방향 화살표(ARROW)
        // =========================
        visualization_msgs::msg::Marker arrow;
        arrow.header.stamp = now;
        arrow.header.frame_id = frame_id_;
        arrow.ns = "jarabot";
        arrow.id = 1;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        const double L = body.scale.x;
        const double H = body.scale.z;

        // 화살표 시작점을 차체 앞쪽으로
        arrow.pose.position.x = L * 0.20;     // 앞쪽
        arrow.pose.position.y = 0.0;
        arrow.pose.position.z = H * 0.85;     // 차체 위

        arrow.pose.orientation.x = 0.0;
        arrow.pose.orientation.y = 0.0;
        arrow.pose.orientation.z = 0.0;
        arrow.pose.orientation.w = 1.0;       // +x 방향

        // 화살표 스케일
        arrow.scale.x = L * 0.90; // 길이
        arrow.scale.y = 0.03;     // 두께
        arrow.scale.z = 0.03;

        // 색(빨강)
        arrow.color.r = 1.0f;
        arrow.color.g = 0.0f;
        arrow.color.b = 0.0f;
        arrow.color.a = 1.0f;

        arrow.lifetime = rclcpp::Duration(0, 0);
        marker_pub_->publish(arrow);
    }

    std::string frame_id_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JaraSimMarker>());
    rclcpp::shutdown();
    return 0;
}
