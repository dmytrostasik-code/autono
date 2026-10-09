#include <algorithm>
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ardurover_nav/ardurover_controller.hpp"
#include "ardurover_nav/path_io.hpp"

namespace ardurover_nav {

class TrajectoryControllerNode : public rclcpp::Node {
  public:
    TrajectoryControllerNode() : Node("trajectory_controller_node") {
        pathFile_ = declare_parameter("path_file", std::string("paths/recorded.path"));
        controlEnabled_ = declare_parameter("control_enabled", true);
        const double rateHz = declare_parameter("control_rate_hz", 20.0);

        auto path = load_path(pathFile_);
        if (path.empty()) {
            throw std::runtime_error("Path file is empty: " + pathFile_);
        }

        markerPub_ =
            create_publisher<visualization_msgs::msg::MarkerArray>("/path_markers", rclcpp::QoS(1).transient_local());
        odomSub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth/odom", 10,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { latestOdom_ = std::move(msg); }
        );

        for (const auto& waypoint : path) {
            geometry_msgs::msg::Point point;
            point.x = waypoint.x;
            point.y = waypoint.y;
            point.z = 0.05;
            refPoints_.push_back(point);
        }

        controller_ = std::make_unique<ArduroverController>(*this, std::move(path));

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rateHz), std::bind(&TrajectoryControllerNode::OnTimer, this)
        );

        RCLCPP_INFO_STREAM(get_logger(), "Loaded " << refPoints_.size() << " waypoints from " << pathFile_);
    }

  private:
    void OnTimer() {
        UpdateDrivenPath();
        DrawTargetAndDrivenPath();

        if (!controlEnabled_) {
            return;
        }

        if (!controller_->SetupArdurover()) {
            return;
        }

        if (!latestOdom_) {
            return;
        }

        controller_->Control(*latestOdom_);
    }

    void UpdateDrivenPath() {
        if (!latestOdom_) {
            return;
        }
        const auto& pos = latestOdom_->pose.pose.position;
        if (!drivenPoints_.empty()) {
            const auto& last = drivenPoints_.back();
            const double dx = pos.x - last.x;
            const double dy = pos.y - last.y;
            if (dx * dx + dy * dy < 0.0025) {
                return;
            }
        }
        drivenPoints_.push_back(pos);
    }

    void DrawTargetAndDrivenPath() {
        visualization_msgs::msg::MarkerArray msg;
        msg.markers.push_back(MakeLineStrip(0, "target_path", 0.1f, 0.85f, 0.15f, refPoints_));
        if (drivenPoints_.size() >= 2) {
            msg.markers.push_back(MakeLineStrip(1, "driven_path", 0.95f, 0.15f, 0.1f, drivenPoints_));
        }
        markerPub_->publish(msg);
    }

    visualization_msgs::msg::Marker MakeLineStrip(
        int id, const char* ns, float r, float g, float b, const std::vector<geometry_msgs::msg::Point>& points
    ) const {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.frame_locked = true;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.12;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        marker.points = points;
        return marker;
    }

    std::string pathFile_;
    bool controlEnabled_{true};
    std::vector<geometry_msgs::msg::Point> refPoints_;
    std::vector<geometry_msgs::msg::Point> drivenPoints_;
    nav_msgs::msg::Odometry::ConstSharedPtr latestOdom_;
    std::unique_ptr<ArduroverController> controller_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ardurover_nav

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ardurover_nav::TrajectoryControllerNode>());
    rclcpp::shutdown();
    return 0;
}
