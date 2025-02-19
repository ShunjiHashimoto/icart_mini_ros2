#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <cmath>

#define MAX_NOISE_DISTANCE_THRESH 0.003  // ノイズ除去距離閾値
#define MAX_SAMPLING_INTERVAL 0.001      // ダウンサンプリング間隔

class PreprocessingFilterNode : public rclcpp::Node {
public:
    PreprocessingFilterNode() : Node("preprocessing_filter_node") {
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&PreprocessingFilterNode::scanCallback, this, std::placeholders::_1)
        );

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/leg_tracker/filtered_points", 10
        );

        RCLCPP_INFO(this->get_logger(), "Preprocessing & Filter Node started.");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::vector<geometry_msgs::msg::Point> points = generateXYPoints(msg);
        removeNoise(points);
        downSampling(points);
        publishMarker(points);
    }

    std::vector<geometry_msgs::msg::Point> generateXYPoints(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::vector<geometry_msgs::msg::Point> points;
        for (size_t i = 0; i < msg->ranges.size(); i++) {
            double angle = msg->angle_min + i * msg->angle_increment;
            double distance = msg->ranges[i];
            if (distance > msg->range_min && distance < msg->range_max) {
                geometry_msgs::msg::Point p;
                p.x = distance * cos(angle); // X: 前方
                p.y = distance * sin(angle); // Y: 左方向
                p.z = 0.0;
                points.push_back(p);
            }
        }
        return points;
    }

    void removeNoise(std::vector<geometry_msgs::msg::Point> &points) {
        for (size_t i = 0; i < points.size(); i++) {
            for (size_t j = i + 1; j < points.size(); j++) {
                double dist = sqrt(pow(points[i].x - points[j].x, 2) + pow(points[i].y - points[j].y, 2));
                if (dist < MAX_NOISE_DISTANCE_THRESH) {
                    points[j].x = points[j].y = 0.0;  // ノイズ除去
                }
            }
        }
    }

    void downSampling(std::vector<geometry_msgs::msg::Point> &points) {
        for (size_t i = 0; i < points.size(); i++) {
            if (points[i].x == 0.0 && points[i].y == 0.0) continue;
            for (size_t j = i + 1; j < points.size(); j++) {
                double dist = sqrt(pow(points[i].x - points[j].x, 2) + pow(points[i].y - points[j].y, 2));
                if (dist < MAX_SAMPLING_INTERVAL) {
                    points[j].x = points[j].y = 0.0;  // ダウンサンプリング
                }
            }
        }
    }

    void publishMarker(const std::vector<geometry_msgs::msg::Point> &points) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "laser";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "filtered_points";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        for (const auto &p : points) {
            if (p.x != 0.0 || p.y != 0.0) {
                marker.points.push_back(p);
            }
        }
        marker_publisher_->publish(marker);
        RCLCPP_INFO(this->get_logger(), "Filtered points published: [%zu] points", marker.points.size());
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};
 
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PreprocessingFilterNode>());
    rclcpp::shutdown();
    return 0;
}