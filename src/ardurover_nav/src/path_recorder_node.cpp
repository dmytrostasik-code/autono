#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "ardurover_nav/path_io.hpp"

namespace ardurover_nav {

class PathRecorderNode : public rclcpp::Node {
  public:
    PathRecorderNode() : Node("path_recorder_node") {
        std::string defaultFile = "paths/recorded.path";
        if (const char *root = std::getenv("ARDUROVER_NAV_ROOT")) {
            defaultFile = std::string(root) + "/paths/recorded.path";
        }
        outputFile_ = declare_parameter("output_file", defaultFile);
        const double periodS = declare_parameter("sample_period_s", 0.2);

        odomSub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth/odom", 10,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { latestOdom_ = std::move(msg); }
        );

        timer_ = create_wall_timer(std::chrono::duration<double>(periodS), std::bind(&PathRecorderNode::OnTimer, this));

        RCLCPP_INFO_STREAM(get_logger(), "Recording to " << outputFile_ << " every " << periodS << " s");
    }

    void Flush() {
        if (flushed_) {
            return;
        }
        flushed_ = true;
        save_path(outputFile_, waypoints_);
        RCLCPP_INFO_STREAM(get_logger(), "Saved " << waypoints_.size() << " waypoints to " << outputFile_);
    }

  private:
    void OnTimer() {
        if (!latestOdom_) {
            return;
        }

        const auto &pose = latestOdom_->pose.pose;
        waypoints_.push_back({pose.position.x, pose.position.y, yaw_from_quat(pose.orientation)});

        if (waypoints_.size() % 25 == 0) {
            RCLCPP_INFO_STREAM(get_logger(), "Recorded " << waypoints_.size() << " waypoints");
        }
    }

    std::string outputFile_;
    bool flushed_{false};
    nav_msgs::msg::Odometry::ConstSharedPtr latestOdom_;
    std::vector<Waypoint> waypoints_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ardurover_nav

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ardurover_nav::PathRecorderNode>();
    rclcpp::spin(node);
    node->Flush();
    rclcpp::shutdown();
    return 0;
}
