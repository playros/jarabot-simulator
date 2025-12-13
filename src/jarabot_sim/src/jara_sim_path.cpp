#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class JaraSimPath : public rclcpp::Node
{
public:
    JaraSimPath()
        : Node("jara_sim_path"),
          max_points_(5000)
    {
        declare_parameter("max_points", max_points_);
        max_points_ = get_parameter("max_points").as_int();

        path_pub_ = create_publisher<nav_msgs::msg::Path>("/odom_path", 10);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&JaraSimPath::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "JaraSimPath started (max_points=%d)", max_points_);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose   = msg->pose.pose;

        path_.header = msg->header;
        path_.poses.push_back(pose);

        // 너무 길어지면 앞부분부터 버리기
        //if (static_cast<int>(path_.poses.size()) > max_points_) {
        //    path_.poses.erase(path_.poses.begin());
        //}

        path_pub_->publish(path_);
    }

    int max_points_;
    nav_msgs::msg::Path path_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JaraSimPath>());
    rclcpp::shutdown();
    return 0;
}
