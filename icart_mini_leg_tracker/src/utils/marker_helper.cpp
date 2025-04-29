#include "icart_mini_leg_tracker/utils/marker_helper.hpp"

MarkerHelper::MarkerHelper(int num_clusters) {
    color_palette_ = generateColors(num_clusters);  // 色を事前生成
}

visualization_msgs::msg::Marker MarkerHelper::createMarker(
    const std::string& ns, int id, int type,
    const geometry_msgs::msg::Point& position,
    float scale_x, float scale_y, float scale_z,
    int cluster_id, float r, float g, float b, float a) {

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "laser";
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = position;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = scale_z;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2); 

    if (cluster_id >= 0 && cluster_id < static_cast<int>(color_palette_.size())) {
        marker.color = color_palette_[cluster_id % color_palette_.size()];
    } else {
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;
    }
    return marker;
}

std::vector<std_msgs::msg::ColorRGBA> MarkerHelper::generateColors(int num_clusters) {
    std::vector<std_msgs::msg::ColorRGBA> colors(num_clusters);
    std::mt19937 rng(42);  // シード固定
    std::uniform_real_distribution<float> dist(0.0, 1.0);
    for (auto &color : colors) {
        color.r = dist(rng);
        color.g = dist(rng);
        color.b = dist(rng);
        color.a = 1.0;
    }
    return colors;
}