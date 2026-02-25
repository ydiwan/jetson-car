#include "vehicle_controller/waypoint_loader.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>

WaypointLoader::WaypointLoader(const rclcpp::Logger &logger)
    : logger_(logger)
{
}

std::vector<Waypoint> WaypointLoader::load_waypoints(const std::string &filename)
{
    std::vector<Waypoint> waypoints;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        RCLCPP_ERROR(logger_, "Failed to open file: %s", filename.c_str());
        return waypoints;
    }

    RCLCPP_INFO(logger_, "Loading waypoints from: %s", filename.c_str());

    std::string line;
    int line_number = 0;

    while (std::getline(file, line))
    {
        line_number++;

        // Skip comments or empty lines
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        std::stringstream ss(line);
        std::string value;
        Waypoint point;

        if (std::getline(ss, value, ','))
        {
            point.x = std::stod(value);
        }
        else
        {
            continue; // Malformed line
        }

        if (std::getline(ss, value, ','))
        {
            point.y = std::stod(value);
        }
        else
        {
            continue; // Malformed line
        }

        waypoints.push_back(point);
        // RCLCPP_DEBUG(logger_, "Added waypoint: (%.2f, %.2f) %s",
        //              point.x, point.y, point.name.empty() ? "" : point.name.c_str());
    }

    RCLCPP_INFO(logger_, "Loaded %zu waypoints", waypoints.size());
    return waypoints;
}