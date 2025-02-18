#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <cmath>
#include <set>
#include <random>

#define MAX_NOISE_DISTANCE_THRESH 0.003  // ノイズ除去距離閾値
#define MAX_SAMPLING_INTERVAL 0.001      // ダウンサンプリング間隔
#define CLUSTER_DISTANCE_THRESH 0.1     // クラスタ間の距離閾値

class PreprocessingClusterNode : public rclcpp::Node {
public:
    PreprocessingClusterNode() : Node("preprocessing_cluster_node") {
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&PreprocessingClusterNode::scanCallback, this, std::placeholders::_1)
        );

        cluster_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/leg_tracker/cluster_markers", 10
        );

        center_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/leg_tracker/cluster_centers", 10
        );

        RCLCPP_INFO(this->get_logger(), "Preprocessing, Filter & Clustering Node started.");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto points = generateXYPoints(msg);
        removeNoise(points);
        downSampling(points);
        auto clusters = makeClusters(points);
        publishClusterMarkers(points, clusters);
        publishClusterCenters(points, clusters);
    }

    std::vector<geometry_msgs::msg::Point> generateXYPoints(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::vector<geometry_msgs::msg::Point> points;
        for (size_t i = 0; i < msg->ranges.size(); i++) {
            double angle = msg->angle_min + i * msg->angle_increment;
            double distance = msg->ranges[i];
            if (distance > msg->range_min && distance < msg->range_max) {
                geometry_msgs::msg::Point p;
                p.x = distance * cos(angle);
                p.y = distance * sin(angle);
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
                    points[j].x = points[j].y = 0.0;
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
                    points[j].x = points[j].y = 0.0;
                }
            }
        }
    }

    std::vector<int> makeClusters(const std::vector<geometry_msgs::msg::Point> &points) {
        std::vector<int> clusters(points.size(), 0);
        int cluster_id = 1;
        for (size_t i = 0; i < points.size(); i++) {
            if (points[i].x == 0.0 && points[i].y == 0.0) continue;
            if (clusters[i] == 0) {
                clusters[i] = cluster_id;
                for (size_t j = i + 1; j < points.size(); j++) {
                    if (clusters[j] == 0) {
                        double dist = sqrt(pow(points[i].x - points[j].x, 2) + pow(points[i].y - points[j].y, 2));
                        if (dist < CLUSTER_DISTANCE_THRESH) {
                            clusters[j] = cluster_id;
                        }
                    }
                }
                cluster_id++;
            }
        }
        return clusters;
    }

    void publishClusterMarkers(const std::vector<geometry_msgs::msg::Point> &points, const std::vector<int> &clusters) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "laser";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "cluster_markers";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.03;
        marker.scale.y = 0.03;
        marker.color.a = 1.0;

        std::vector<std_msgs::msg::ColorRGBA> colors = generateColors(*std::max_element(clusters.begin(), clusters.end()));

        for (size_t i = 0; i < points.size(); i++) {
            if (points[i].x != 0.0 || points[i].y != 0.0) {
                marker.points.push_back(points[i]);
                marker.colors.push_back(colors[clusters[i] % colors.size()]);
            }
        }
        cluster_marker_publisher_->publish(marker);
    }

    std::vector<std_msgs::msg::ColorRGBA> generateColors(int num_clusters) {
        std::vector<std_msgs::msg::ColorRGBA> colors(num_clusters);
        std::mt19937 rng(42);
        std::uniform_real_distribution<float> dist(0.0, 1.0);
        for (auto &color : colors) {
            color.r = dist(rng);
            color.g = dist(rng);
            color.b = dist(rng);
            color.a = 1.0;
        }
        return colors;
    }

    void publishClusterCenters(const std::vector<geometry_msgs::msg::Point> &points, const std::vector<int> &clusters) {
        visualization_msgs::msg::Marker center_marker;
        center_marker.header.frame_id = "laser";
        center_marker.header.stamp = this->get_clock()->now();
        center_marker.ns = "cluster_centers";
        center_marker.id = 1;
        center_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        center_marker.action = visualization_msgs::msg::Marker::ADD;
        center_marker.scale.x = 0.05;
        center_marker.scale.y = 0.05;
        center_marker.scale.z = 0.05;
        center_marker.color.a = 1.0;

        std::map<int, std::vector<geometry_msgs::msg::Point>> cluster_points;
        for (size_t i = 0; i < points.size(); i++) {
            if (clusters[i] > 0) {
                cluster_points[clusters[i]].push_back(points[i]);
            }
        }

        for (auto &[id, pts] : cluster_points) {
            geometry_msgs::msg::Point center;
            center.x = 0.0;
            center.y = 0.0;
            for (auto &p : pts) {
                center.x += p.x;
                center.y += p.y;
            }
            center.x /= pts.size();
            center.y /= pts.size();
            center_marker.points.push_back(center);
        }
        center_marker.color.r = 1.0;
        center_marker.color.g = 1.0;
        center_marker.color.b = 0.0;
        center_marker.color.a = 1.0;
        center_marker_publisher_->publish(center_marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cluster_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr center_marker_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PreprocessingClusterNode>());
    rclcpp::shutdown();
    return 0;
}

