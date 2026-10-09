#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "ardurover_nav/path_io.hpp"

namespace ardurover_nav {
namespace {

double point_to_segment(double px, double py, double ax, double ay, double bx, double by) {
    const double abx = bx - ax;
    const double aby = by - ay;
    const double length2 = abx * abx + aby * aby;
    if (length2 < 1e-12) {
        return std::hypot(px - ax, py - ay);
    }
    const double t = std::clamp(((px - ax) * abx + (py - ay) * aby) / length2, 0.0, 1.0);
    return std::hypot(px - (ax + t * abx), py - (ay + t * aby));
}

double cross_track(double px, double py, const std::vector<Waypoint>& path) {
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < path.size(); ++i) {
        best = std::min(best, point_to_segment(px, py, path[i - 1].x, path[i - 1].y, path[i].x, path[i].y));
    }
    if (!std::isfinite(best) && !path.empty()) {
        best = std::hypot(px - path.front().x, py - path.front().y);
    }
    return best;
}

double path_length(const std::vector<Waypoint>& path) {
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        length += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    }
    return length;
}

double progress_along_path(double px, double py, const std::vector<Waypoint>& path) {
    if (path.size() < 2) {
        return 0.0;
    }

    double bestDist = std::numeric_limits<double>::infinity();
    double bestProgress = 0.0;
    double traveled = 0.0;

    for (size_t i = 1; i < path.size(); ++i) {
        const double ax = path[i - 1].x;
        const double ay = path[i - 1].y;
        const double bx = path[i].x;
        const double by = path[i].y;
        const double abx = bx - ax;
        const double aby = by - ay;
        const double segLen = std::hypot(abx, aby);
        const double length2 = abx * abx + aby * aby;
        const double t = (length2 < 1e-12) ? 0.0 : std::clamp(((px - ax) * abx + (py - ay) * aby) / length2, 0.0, 1.0);
        const double dist = std::hypot(px - (ax + t * abx), py - (ay + t * aby));
        if (dist < bestDist) {
            bestDist = dist;
            bestProgress = traveled + t * segLen;
        }
        traveled += segLen;
    }
    return bestProgress;
}

}  // namespace

class PathScorerNode : public rclcpp::Node {
  public:
    PathScorerNode() : Node("path_scorer_node") {
        pathFile_ = declare_parameter("path_file", std::string("paths/example.path"));
        outputFile_ = declare_parameter("output_file", std::string("paths/score.txt"));
        goalRadius_ = declare_parameter("goal_radius_m", 1.0);
        timeoutS_ = declare_parameter("timeout_s", 180.0);
        const double sampleHz = declare_parameter("sample_hz", 10.0);

        path_ = load_path(pathFile_);
        if (path_.empty()) {
            throw std::runtime_error("Path file is empty: " + pathFile_);
        }
        pathLength_ = path_length(path_);

        odomSub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth/odom", 10,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { latestOdom_ = std::move(msg); }
        );
        timer_ =
            create_wall_timer(std::chrono::duration<double>(1.0 / sampleHz), std::bind(&PathScorerNode::OnTimer, this));

        RCLCPP_INFO_STREAM(get_logger(), "Scoring " << path_.size() << " waypoints, timeout " << timeoutS_ << " s");
    }

    void Finish(const std::string& reason) {
        if (finished_) {
            return;
        }
        finished_ = true;

        double sumSq = 0.0;
        double maxCte = 0.0;
        double maxProgress = 0.0;
        for (const auto& sample : samples_) {
            const double cte = cross_track(sample.first, sample.second, path_);
            sumSq += cte * cte;
            maxCte = std::max(maxCte, cte);
            maxProgress = std::max(maxProgress, progress_along_path(sample.first, sample.second, path_));
        }

        const double rmsCte = samples_.empty() ? 0.0 : std::sqrt(sumSq / static_cast<double>(samples_.size()));
        const double completion = pathLength_ < 1e-6 ? 0.0 : std::clamp(maxProgress / pathLength_, 0.0, 1.0);
        const bool nearGoal =
            !samples_.empty() &&
            std::hypot(samples_.back().first - path_.back().x, samples_.back().second - path_.back().y) <= goalRadius_;
        const bool goalReached = nearGoal && completion >= 0.8;
        const double score = 100.0 * completion * std::exp(-rmsCte / 0.75) * std::exp(-maxCte / 4.0);

        std::ofstream out(outputFile_);
        if (!out) {
            throw std::runtime_error("Failed to write score file: " + outputFile_);
        }
        out << "reason: " << reason << "\n";
        out << "score: " << score << "\n";
        out << "completion: " << completion << "\n";
        out << "rms_cross_track_m: " << rmsCte << "\n";
        out << "max_cross_track_m: " << maxCte << "\n";
        out << "goal_reached: " << (goalReached ? "true" : "false") << "\n";
        out << "samples: " << samples_.size() << "\n";

        RCLCPP_INFO_STREAM(
            get_logger(), "Score " << score << " (completion=" << completion << ", rms_cte=" << rmsCte
                                   << " m, max_cte=" << maxCte << " m, goal=" << goalReached << ", " << reason << ")"
        );
        RCLCPP_INFO_STREAM(get_logger(), "Wrote " << outputFile_);
        rclcpp::shutdown();
    }

  private:
    void OnTimer() {
        if (!latestOdom_) {
            return;
        }
        if (!t0_) {
            t0_ = now();
        }

        const double x = latestOdom_->pose.pose.position.x;
        const double y = latestOdom_->pose.pose.position.y;
        samples_.emplace_back(x, y);

        maxProgress_ = std::max(maxProgress_, progress_along_path(x, y, path_));
        const double minProgress = std::min(2.0, 0.5 * pathLength_);
        const double goalDist = std::hypot(x - path_.back().x, y - path_.back().y);
        if (goalDist <= goalRadius_ && maxProgress_ >= minProgress) {
            ++goalTicks_;
        } else {
            goalTicks_ = 0;
        }
        if (goalTicks_ >= 10) {
            Finish("goal_reached");
            return;
        }

        const double elapsed = (now() - *t0_).seconds();
        if (elapsed >= timeoutS_) {
            Finish("timeout");
        }
    }

    std::string pathFile_;
    std::string outputFile_;
    double goalRadius_{1.0};
    double timeoutS_{180.0};
    double pathLength_{0.0};
    double maxProgress_{0.0};
    bool finished_{false};
    int goalTicks_{0};
    std::optional<rclcpp::Time> t0_;
    std::vector<Waypoint> path_;
    std::vector<std::pair<double, double>> samples_;
    nav_msgs::msg::Odometry::ConstSharedPtr latestOdom_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ardurover_nav

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ardurover_nav::PathScorerNode>();
    rclcpp::spin(node);
    node->Finish("stopped");
    return 0;
}
