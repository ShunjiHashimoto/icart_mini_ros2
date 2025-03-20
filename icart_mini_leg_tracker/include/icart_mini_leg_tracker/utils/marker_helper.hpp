#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <random>

class MarkerHelper {
public:
    MarkerHelper(int num_clusters = 1000);

    visualization_msgs::msg::Marker createMarker(
        const std::string& ns, int id, int type,
        const geometry_msgs::msg::Point& position,
        float scale_x, float scale_y, float scale_z,
        int cluster_id = -1, 
        float r = 1.0, float g = 1.0, float b = 1.0, float a = 1.0);
    std::vector<std_msgs::msg::ColorRGBA> color_palette_;

private:
    std::vector<std_msgs::msg::ColorRGBA> generateColors(int num_clusters);
};