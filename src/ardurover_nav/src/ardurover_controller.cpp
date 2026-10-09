#include "ardurover_nav/ardurover_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

namespace ardurover_nav {
namespace {

constexpr double kMinWaypointSpacing = 0.05;
constexpr double kMinLegLength = 0.3;
constexpr double kSearchWindow = 1.5;
constexpr double kLookaheadMin = 0.6;
constexpr double kLookaheadTime = 1.2;
constexpr double kLookaheadMax = 1.5;
constexpr double kResponseTime = 0.5;
constexpr double kCurvatureBase = 0.5;
constexpr double kMaxSpeed = 1.5;
constexpr double kMinSpeed = 0.15;
constexpr double kMaxLateralAccel = 0.45;
constexpr double kMaxDecel = 0.5;
constexpr double kMaxYawRate = 1.2;
constexpr double kPivotEnterAngle = 0.8;
constexpr double kPivotExitAngle = 0.2;
constexpr double kPivotGain = 1.5;
constexpr double kArrivalTolerance = 0.05;
constexpr double kStallSpeed = 0.05;
constexpr double kStallSpeedRatio = 0.3;
constexpr int kStallTicks = 20;
constexpr double kUnstickExitAngle = 0.05;
constexpr int kRestTicks = 10;
constexpr int kMaxSettleTicks = 80;
constexpr double kRecoveryReach = 1.0;
constexpr double kRecoverySpeed = 0.5;
constexpr double kRecoveryDistance = 1.5;
constexpr int kConfirmTicks = 100;

}  // namespace

ArduroverController::ArduroverController(rclcpp::Node &node, std::vector<Waypoint> path) : node_(node) {
    for (const auto &waypoint : path) {
        if (path_.empty() ||
            std::hypot(waypoint.x - path_.back().x, waypoint.y - path_.back().y) >= kMinWaypointSpacing) {
            path_.push_back(waypoint);
        }
    }
    arcLength_.assign(path_.size(), 0.0);
    for (size_t i = 1; i < path_.size(); ++i) {
        arcLength_[i] = arcLength_[i - 1] + std::hypot(path_[i].x - path_[i - 1].x, path_[i].y - path_[i - 1].y);
    }
    BuildLegs();
    BuildSpeedProfile();
    finished_ = legs_.empty();

    cmdPub_ = node_.create_publisher<geometry_msgs::msg::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    arming_ = node_.create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    setMode_ = node_.create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    paramClient_ = std::make_shared<rclcpp::AsyncParametersClient>(&node_, "/mavros/setpoint_velocity");
    stateSub_ = node_.create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 10, [this](mavros_msgs::msg::State::ConstSharedPtr msg) { state_ = std::move(msg); }
    );

    RCLCPP_INFO(
        node_.get_logger(), "Tracking %.1f m path: %zu waypoints, %zu legs",
        arcLength_.empty() ? 0.0 : arcLength_.back(), path_.size(), legs_.size()
    );
}

bool ArduroverController::SetupArdurover() {
    switch (setupState_) {
        case SetupState::WaitServices:
            if (arming_->service_is_ready() && setMode_->service_is_ready()) {
                setupState_ = SetupState::SetFrame;
            }
            return false;
        case SetupState::SetFrame:
            if (!paramClient_->service_is_ready()) {
                return false;
            }
            paramClient_->set_parameters({rclcpp::Parameter("mav_frame", "BODY_NED")});
            setupState_ = SetupState::Prime;
            return false;
        case SetupState::Prime:
            if (++primeTicks_ >= 20 && Booted()) {
                setupState_ = SetupState::SetMode;
            }
            return false;
        case SetupState::SetMode:
            if (!modeFuture_.valid()) {
                auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                req->custom_mode = "GUIDED";
                modeFuture_ = setMode_->async_send_request(req).future.share();
            } else if (modeFuture_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                const auto resp = modeFuture_.get();
                if (resp && resp->mode_sent) {
                    setupState_ = SetupState::Arm;
                } else {
                    RCLCPP_WARN(node_.get_logger(), "GUIDED request was not sent, retrying");
                    RestartSetup();
                }
            }
            return false;
        case SetupState::Arm:
            if (!armFuture_.valid()) {
                auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                req->value = true;
                armFuture_ = arming_->async_send_request(req).future.share();
                return false;
            }
            if (armFuture_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
                return false;
            }
            {
                const auto resp = armFuture_.get();
                if (resp && resp->success) {
                    confirmTicks_ = 0;
                    setupState_ = SetupState::Confirm;
                    return false;
                }
                armFuture_ = {};
            }
            return false;
        case SetupState::Confirm:
            if (Engaged()) {
                setupState_ = SetupState::Ready;
                RCLCPP_INFO(node_.get_logger(), "Controller running");
                return true;
            }
            if (++confirmTicks_ < kConfirmTicks) {
                return false;
            }
            break;
        case SetupState::Ready:
            if (Engaged()) {
                return true;
            }
            break;
    }
    RCLCPP_WARN(
        node_.get_logger(), "Rover is %s in mode '%s', repeating setup", state_ && state_->armed ? "armed" : "disarmed",
        state_ ? state_->mode.c_str() : ""
    );
    RestartSetup();
    return false;
}

void ArduroverController::Control(const nav_msgs::msg::Odometry &odom) {
    if (finished_) {
        SendCommand(0.0, 0.0);
        return;
    }

    const auto &pose = odom.pose.pose;
    UpdateProgress(pose.position.x, pose.position.y);

    const Leg &leg = legs_[leg_];
    if (arcLength_[leg.last] - progress_ < kArrivalTolerance) {
        SendCommand(0.0, 0.0);
        if (++leg_ < legs_.size()) {
            segment_ = legs_[leg_].first;
            progress_ = arcLength_[segment_];
            pivoting_ = false;
            unsticking_ = false;
            settling_ = false;
            stallTicks_ = 0;
            RCLCPP_INFO(
                node_.get_logger(), "Leg %zu/%zu: driving %s", leg_ + 1, legs_.size(),
                legs_[leg_].reverse ? "backward" : "forward"
            );
        } else {
            finished_ = true;
            RCLCPP_INFO(node_.get_logger(), "Path complete");
        }
        return;
    }

    const double speed = std::hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y);
    moved_ = moved_ || speed > kStallSpeed;
    if (pivoting_ || settling_ || !moved_ || speed > std::max(kStallSpeed, kStallSpeedRatio * commandSpeed_)) {
        stallTicks_ = 0;
    } else if (++stallTicks_ >= kStallTicks) {
        stallTicks_ = 0;
        stallCount_ = stallCount_ > 0 && progress_ < stallProgress_ + kRecoveryDistance ? stallCount_ + 1 : 1;
        stallProgress_ = progress_;
        pivoting_ = true;
        unsticking_ = true;
        RCLCPP_WARN(node_.get_logger(), "Stalled at s %.1f m, turning in place", progress_);
    }

    const double plannedSpeed = PlannedSpeed(leg, progress_ + speed * kResponseTime);
    const double lookahead = std::clamp(kLookaheadTime * plannedSpeed, kLookaheadMin, kLookaheadMax);
    const double reach = unsticking_ ? lookahead + kRecoveryReach * stallCount_ : lookahead;
    const Point target = PointAt(leg, progress_ + reach);
    const double heading = yaw_from_quat(pose.orientation) + (leg.reverse ? M_PI : 0.0);
    const double dx = target.x - pose.position.x;
    const double dy = target.y - pose.position.y;
    const double forward = std::cos(heading) * dx + std::sin(heading) * dy;
    const double lateral = std::cos(heading) * dy - std::sin(heading) * dx;
    const double alpha = std::atan2(lateral, forward);

    if (settling_) {
        restTicks_ = speed < kStallSpeed ? restTicks_ + 1 : 0;
        settling_ = restTicks_ < kRestTicks && ++settleTicks_ < kMaxSettleTicks;
        if (!settling_) {
            slowUntil_ = progress_ + kRecoveryDistance;
        }
    } else if (std::abs(alpha) > kPivotEnterAngle) {
        pivoting_ = true;
    } else if (std::abs(alpha) < (unsticking_ ? kUnstickExitAngle : kPivotExitAngle)) {
        settling_ = unsticking_;
        restTicks_ = 0;
        settleTicks_ = 0;
        pivoting_ = false;
        unsticking_ = false;
    }
    if (pivoting_ || settling_) {
        commandSpeed_ = 0.0;
        SendCommand(0.0, pivoting_ ? std::clamp(kPivotGain * alpha, -kMaxYawRate, kMaxYawRate) : 0.0);
        return;
    }

    const double curvature = 2.0 * lateral / std::max(dx * dx + dy * dy, 1e-6);
    const double turnSpeed = std::sqrt(kMaxLateralAccel / std::max(std::abs(curvature), 1e-6));
    double v = std::max(std::min(plannedSpeed, turnSpeed), kMinSpeed);
    if (progress_ < slowUntil_) {
        v = std::min(v, kRecoverySpeed);
    }
    commandSpeed_ = v;
    SendCommand(leg.reverse ? -v : v, std::clamp(v * curvature, -kMaxYawRate, kMaxYawRate));

    RCLCPP_INFO_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000, "s %.1f/%.1f m, cte %.2f m, v %.2f m/s", progress_,
        arcLength_.back(), crossTrack_, v
    );
}

void ArduroverController::BuildLegs() {
    for (size_t i = 0; i + 1 < path_.size(); ++i) {
        const bool reverse = IsReverse(i);
        if (legs_.empty() || (reverse != legs_.back().reverse && RunLength(i, reverse) >= kMinLegLength)) {
            legs_.push_back({i, i + 1, reverse});
        } else {
            legs_.back().last = i + 1;
        }
    }
    if (legs_.empty()) {
        return;
    }
    Leg &tail = legs_.back();
    while (tail.last > tail.first + 1 && IsReverse(tail.last - 1) != tail.reverse) {
        --tail.last;
    }
}

void ArduroverController::BuildSpeedProfile() {
    speedLimit_.assign(path_.size(), kMaxSpeed);
    for (const auto &leg : legs_) {
        for (size_t i = leg.first; i <= leg.last; ++i) {
            const Point a = PointAt(leg, arcLength_[i] - kCurvatureBase);
            const Point c = PointAt(leg, arcLength_[i] + kCurvatureBase);
            const double abx = path_[i].x - a.x;
            const double aby = path_[i].y - a.y;
            const double bcx = c.x - path_[i].x;
            const double bcy = c.y - path_[i].y;
            const double curvature = 2.0 * std::abs(abx * bcy - aby * bcx) /
                                     (std::hypot(abx, aby) * std::hypot(bcx, bcy) * std::hypot(c.x - a.x, c.y - a.y));
            speedLimit_[i] = std::min(speedLimit_[i], std::sqrt(kMaxLateralAccel / std::max(curvature, 1e-6)));
        }
        speedLimit_[leg.last] = 0.0;
        for (size_t i = leg.last; i > leg.first; --i) {
            const double brakingSpeed =
                std::sqrt(speedLimit_[i] * speedLimit_[i] + 2.0 * kMaxDecel * (arcLength_[i] - arcLength_[i - 1]));
            speedLimit_[i - 1] = std::min(speedLimit_[i - 1], brakingSpeed);
        }
    }
}

bool ArduroverController::IsReverse(size_t segment) const {
    const auto &a = path_[segment];
    const auto &b = path_[segment + 1];
    const double headingX = std::cos(a.yaw) + std::cos(b.yaw);
    const double headingY = std::sin(a.yaw) + std::sin(b.yaw);
    return (b.x - a.x) * headingX + (b.y - a.y) * headingY < 0.0;
}

double ArduroverController::RunLength(size_t segment, bool reverse) const {
    size_t end = segment;
    while (end + 1 < path_.size() && IsReverse(end) == reverse) {
        ++end;
    }
    return arcLength_[end] - arcLength_[segment];
}

size_t ArduroverController::SegmentAt(const Leg &leg, double s) const {
    const auto first = arcLength_.begin() + static_cast<std::ptrdiff_t>(leg.first) + 1;
    const auto last = arcLength_.begin() + static_cast<std::ptrdiff_t>(leg.last);
    return static_cast<size_t>(std::upper_bound(first, last, s) - arcLength_.begin()) - 1;
}

ArduroverController::Point ArduroverController::PointAt(const Leg &leg, double s) const {
    const size_t i = SegmentAt(leg, s);
    const double t = (s - arcLength_[i]) / (arcLength_[i + 1] - arcLength_[i]);
    return {path_[i].x + t * (path_[i + 1].x - path_[i].x), path_[i].y + t * (path_[i + 1].y - path_[i].y)};
}

double ArduroverController::PlannedSpeed(const Leg &leg, double s) const {
    const size_t i = SegmentAt(leg, s);
    const double t = std::clamp((s - arcLength_[i]) / (arcLength_[i + 1] - arcLength_[i]), 0.0, 1.0);
    return speedLimit_[i] + t * (speedLimit_[i + 1] - speedLimit_[i]);
}

void ArduroverController::UpdateProgress(double x, double y) {
    const Leg &leg = legs_[leg_];
    const size_t start = segment_;
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = start; i < leg.last && arcLength_[i] - arcLength_[start] <= kSearchWindow; ++i) {
        const auto &a = path_[i];
        const auto &b = path_[i + 1];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double t = std::clamp(((x - a.x) * dx + (y - a.y) * dy) / (dx * dx + dy * dy), 0.0, 1.0);
        const double distance = std::hypot(x - a.x - t * dx, y - a.y - t * dy);
        if (distance < best) {
            best = distance;
            segment_ = i;
            progress_ = arcLength_[i] + t * (arcLength_[i + 1] - arcLength_[i]);
        }
    }
    crossTrack_ = best;
}

void ArduroverController::SendCommand(double speed, double yawRate) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = speed;
    cmd.angular.z = yawRate;
    cmdPub_->publish(cmd);
}

bool ArduroverController::Booted() const {
    return state_ && state_->connected && !state_->mode.empty() && state_->mode != "INITIALISING";
}

bool ArduroverController::Engaged() const {
    return state_ && state_->connected && state_->armed && state_->mode == "GUIDED";
}

void ArduroverController::RestartSetup() {
    modeFuture_ = {};
    armFuture_ = {};
    primeTicks_ = 0;
    setupState_ = SetupState::Prime;
}

}  // namespace ardurover_nav
