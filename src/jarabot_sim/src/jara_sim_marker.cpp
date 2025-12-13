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
        // 기본 좌표계: base_link (로봇 몸체 기준)
        declare_parameter("frame_id", std::string("base_link"));
        frame_id_ = get_parameter("frame_id").as_string();

        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "/jara_robot_marker", 10);

        // 10Hz 로 마커 갱신
        timer_ = create_wall_timer(
            100ms, std::bind(&JaraSimMarker::publishMarkers, this));

        RCLCPP_INFO(get_logger(), "JaraSimMarker started (frame_id=%s)",
                    frame_id_.c_str());
    }

private:
    void publishMarkers()
    {
        rclcpp::Time now = get_clock()->now();

        // 1) 자동차 몸체: 직육면체 (2:1 비율)
        visualization_msgs::msg::Marker body;
        body.header.stamp = now;
        body.header.frame_id = frame_id_;
        body.ns = "jarabot";
        body.id = 0;
        body.type = visualization_msgs::msg::Marker::CUBE;
        body.action = visualization_msgs::msg::Marker::ADD;

        // 로봇 중심에 위치
        body.pose.position.x = 0.0;
        body.pose.position.y = 0.0;
        body.pose.position.z = 0.05;  // 바닥 위로 조금 띄우기
        body.pose.orientation.x = 0.0;
        body.pose.orientation.y = 0.0;
        body.pose.orientation.z = 0.0;
        body.pose.orientation.w = 1.0;

        // ★ 2:1 비율 (길이:폭)
        body.scale.x = 0.40;  // 길이 40cm
        body.scale.y = 0.20;  // 폭   20cm
        body.scale.z = 0.10;  // 높이 10cm

        // 파란색 반투명 박스
        body.color.r = 0.0f;
        body.color.g = 0.0f;
        body.color.b = 1.0f;
        body.color.a = 1.0f;

        body.lifetime = rclcpp::Duration(0, 0);  // 0이면 계속 유지

        marker_pub_->publish(body);

        // 2) 진행 방향 화살표 (앞쪽에 하나만)
        visualization_msgs::msg::Marker arrow;
        arrow.header.stamp = now;
        arrow.header.frame_id = frame_id_;
        arrow.ns = "jarabot";
        arrow.id = 1;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        double body_length = body.scale.x;

        // 몸체 중앙보다 약간 앞 + 위에서 시작
        arrow.pose.position.x = body_length * 0.25;  // 앞쪽으로 1/4 길이
        arrow.pose.position.y = 0.0;
        arrow.pose.position.z = body.scale.z * 0.6;  // 몸체 위쪽

        arrow.pose.orientation.x = 0.0;
        arrow.pose.orientation.y = 0.0;
        arrow.pose.orientation.z = 0.0;
        arrow.pose.orientation.w = 1.0;  // x축(+앞) 방향

        // 화살표 길이/두께
        arrow.scale.x = body_length * 0.8;  // 전체 길이
        arrow.scale.y = 0.03;               // 굵기
        arrow.scale.z = 0.03;

        // 빨간색 화살표
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
