#pragma once

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>

namespace ardurover_nav {

struct Waypoint {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

inline double yaw_from_quat(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline std::vector<Waypoint> load_path(const std::string& file) {
    std::ifstream in(file);
    if (!in) {
        throw std::runtime_error("Failed to open path file: " + file);
    }

    std::vector<Waypoint> waypoints;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream ss(line);
        Waypoint waypoint;
        if (!(ss >> waypoint.x >> waypoint.y >> waypoint.yaw)) {
            throw std::runtime_error("Invalid path line: " + line);
        }
        waypoints.push_back(waypoint);
    }
    return waypoints;
}

inline void save_path(const std::string& file, const std::vector<Waypoint>& waypoints) {
    std::ofstream out(file);
    if (!out) {
        throw std::runtime_error("Failed to write path file: " + file);
    }

    out << "# x y yaw\n";
    out << std::fixed << std::setprecision(6);
    for (const auto& waypoint : waypoints) {
        out << waypoint.x << " " << waypoint.y << " " << waypoint.yaw << "\n";
    }
}

}  // namespace ardurover_nav
