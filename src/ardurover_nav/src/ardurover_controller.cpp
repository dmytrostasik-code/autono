#include "ardurover_nav/ardurover_controller.hpp"

#include <chrono>
#include <memory>

namespace ardurover_nav {

ArduroverController::ArduroverController(rclcpp::Node &node, std::vector<Waypoint> path)
    : node_(node), path_(std::move(path)) {
    arming_ = node_.create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    setMode_ = node_.create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    paramClient_ = std::make_shared<rclcpp::AsyncParametersClient>(&node_, "/mavros/setpoint_velocity");
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
            if (++primeTicks_ >= 20) {
                setupState_ = SetupState::SetMode;
            }
            return false;
        case SetupState::SetMode:
            if (!modeFuture_.valid()) {
                auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                req->custom_mode = "GUIDED";
                modeFuture_ = setMode_->async_send_request(req).future.share();
            } else if (modeFuture_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                setupState_ = SetupState::Arm;
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
                    setupState_ = SetupState::Ready;
                    RCLCPP_INFO(node_.get_logger(), "Controller running");
                    return true;
                }
                armFuture_ = {};
            }
            return false;
        case SetupState::Ready:
            return true;
    }
    return false;
}

void ArduroverController::Control(const nav_msgs::msg::Odometry & /*odometry*/) {
    // Implement the control logic here.
    // Feel free to modify the whole controller class as you see fit.
    // You will need to figure out how to send the command to the flight controller through MAVROS
}

}  // namespace ardurover_nav
