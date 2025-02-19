#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <cmath>

class PointCloudProcessor : public rclcpp::Node {
public:
    PointCloudProcessor() : Node("pointcloud_processor_node") {
        // Lidarデータの購読
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&PointCloudProcessor::scanCallback, this, std::placeholders::_1)
        );

        // RViz表示用のマーカーをパブリッシュ
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/leg_tracker/processed_points", 10
        );

        RCLCPP_INFO(this->get_logger(), "PointCloud Processor Node started.");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        visualization_msgs::msg::Marker points;
        points.header.frame_id = "laser";
        points.header.stamp = this->get_clock()->now();
        points.ns = "processed_points";
        points.id = 0;
        points.type = visualization_msgs::msg::Marker::POINTS;
        points.action = visualization_msgs::msg::Marker::ADD;
        points.scale.x = 0.02;
        points.scale.y = 0.02;
        points.color.a = 1.0;
        points.color.r = 0.0;
        points.color.g = 1.0;
        points.color.b = 0.0;

        // XY座標に変換してマーカーに追加
        for (size_t i = 0; i < msg->ranges.size(); i++) {
            double angle = msg->angle_min + i * msg->angle_increment;
            double distance = msg->ranges[i];

            if (distance > msg->range_min && distance < msg->range_max) {
                geometry_msgs::msg::Point p;
                p.x = distance * cos(angle);
                p.y = distance * sin(angle);
                p.z = 0.0;
                points.points.push_back(p);
            }
        }

        marker_publisher_->publish(points);
        RCLCPP_INFO(this->get_logger(), "Published processed points with [%zu] points.", points.points.size());
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};
 
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcessor>());
    rclcpp::shutdown();
    return 0;
}



