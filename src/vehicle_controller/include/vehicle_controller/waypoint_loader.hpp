#ifndef WAYPOINT_LOADER_HPP
#define WAYPOINT_LOADER_HPP

#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"

struct Waypoint
{
    double x, y;
    std::string name; // Optional name field (can be empty)
};

class WaypointLoader
{
public:
    WaypointLoader(const rclcpp::Logger &logger);

    std::vector<Waypoint> load_waypoints(const std::string &filename);

private:
    rclcpp::Logger logger_;
};

#endif // WAYPOINT_LOADER_HPP