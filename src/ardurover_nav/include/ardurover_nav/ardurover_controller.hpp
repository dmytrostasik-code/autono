#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "ardurover_nav/path_io.hpp"

namespace ardurover_nav {

class ArduroverController {
  public:
    ArduroverController(rclcpp::Node& node, std::vector<Waypoint> path);

    bool SetupArdurover();
    void Control(const nav_msgs::msg::Odometry& odom);

  private:
    enum class SetupState { WaitServices, SetFrame, Prime, SetMode, Arm, Confirm, Ready };

    struct Leg {
        size_t first;
        size_t last;
        bool reverse;
    };

    struct Point {
        double x;
        double y;
    };

    void BuildLegs();
    void BuildSpeedProfile();
    bool IsReverse(size_t segment) const;
    double RunLength(size_t segment, bool reverse) const;
    size_t SegmentAt(const Leg& leg, double s) const;
    Point PointAt(const Leg& leg, double s) const;
    double PlannedSpeed(const Leg& leg, double s) const;
    void UpdateProgress(double x, double y);
    void SendCommand(double speed, double yawRate);
    bool Booted() const;
    bool Engaged() const;
    void RestartSetup();

    rclcpp::Node& node_;
    std::vector<Waypoint> path_;
    std::vector<double> arcLength_;
    std::vector<double> speedLimit_;
    std::vector<Leg> legs_;
    size_t leg_{0};
    size_t segment_{0};
    double progress_{0.0};
    double crossTrack_{0.0};
    bool pivoting_{false};
    bool unsticking_{false};
    bool settling_{false};
    bool moved_{false};
    int stallTicks_{0};
    int stallCount_{0};
    double stallProgress_{0.0};
    int restTicks_{0};
    int settleTicks_{0};
    double commandSpeed_{0.0};
    double slowUntil_{0.0};
    bool finished_{false};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPub_;

    SetupState setupState_{SetupState::WaitServices};
    int primeTicks_{0};
    int confirmTicks_{0};
    mavros_msgs::msg::State::ConstSharedPtr state_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr stateSub_;
    std::shared_future<mavros_msgs::srv::SetMode::Response::SharedPtr> modeFuture_;
    std::shared_future<mavros_msgs::srv::CommandBool::Response::SharedPtr> armFuture_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr setMode_;
    rclcpp::AsyncParametersClient::SharedPtr paramClient_;
};

}  // namespace ardurover_nav
