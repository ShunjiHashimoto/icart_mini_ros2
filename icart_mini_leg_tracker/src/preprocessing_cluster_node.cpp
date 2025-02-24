#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <cmath>
#include <set>
#include <random>
#include <fstream>

#define MAX_NOISE_DISTANCE_THRESH 0.005  // ノイズ除去距離閾値
#define MAX_SAMPLING_INTERVAL 0.001      // ダウンサンプリング間隔
#define MIN_CLUSTER_SIZE 10               // 最小クラスタサイズの閾値 
#define MAX_CLUSTER_SIZE 100               // 最小クラスタサイズの閾値 
#define CLUSTER_MATCHED_THRESH 0.3  // クラスタマッチ距離閾値

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
        color_palette_ = generateColors(100);  // 最大100クラスタ用のカラーを事前生成

        RCLCPP_INFO(this->get_logger(), "Preprocessing, Filter & Clustering Node started.");
    }

private:
    std::map<int, int> cluster_id_mapping_;  // <現在のクラスタIndex, 前回のクラスタID>
    std::vector<geometry_msgs::msg::Point> previous_cluster_centers_;
    std::vector<std_msgs::msg::ColorRGBA> color_palette_;
    int next_cluster_id_ = 1;
    bool is_first_frame_ = true;  // 最初のフレームかどうかを判定

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto points = generateXYPoints(msg);
        removeNoise(points);
        downSampling(points);
        auto clusters = makeClusters(points);

        std::set<int> valid_cluster_ids;
        auto cluster_centers = calculateClusterCenters(points, clusters, valid_cluster_ids);

        trackClusters(cluster_centers);  // トラッキング処理を追加

        publishClusterMarkers(points, clusters, valid_cluster_ids);  
        publishMatchedClusterCenters(cluster_centers, valid_cluster_ids);  
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

    std::vector<int> makeClusters(const std::vector<geometry_msgs::msg::Point> &points, double distance_threshold = 0.05625) {
        std::vector<int> clusters(points.size(), 0);
        int cluster_id = 1;

        for (size_t i = 0; i < points.size(); i++) {
            if (points[i].x == 0.0 && points[i].y == 0.0) continue;
            if (clusters[i] == 0) {
                clusters[i] = cluster_id;
                size_t prev = i;

                for (size_t j = i + 1; j < points.size(); j++) {
                    if (clusters[j] == 0) {
                        double dist = sqrt(pow(points[prev].x - points[j].x, 2) + pow(points[prev].y - points[j].y, 2));
                        if (dist < distance_threshold) {
                            clusters[j] = cluster_id;
                            prev = j;  // 直前の点を更新
                        }
                    }
                }
                cluster_id++;
            }
        }
        return clusters;
    }

    std::vector<geometry_msgs::msg::Point> calculateClusterCenters(
        const std::vector<geometry_msgs::msg::Point> &points, 
        const std::vector<int> &clusters, 
        std::set<int> &valid_cluster_ids)
        {
        std::map<int, std::vector<geometry_msgs::msg::Point>> cluster_points;

        // クラスタごとに点を集める
        for (size_t i = 0; i < points.size(); i++) {
            if (clusters[i] > 0) {
                cluster_points[clusters[i]].push_back(points[i]);
            }
        }

        std::vector<geometry_msgs::msg::Point> cluster_centers;
        // 各クラスタの中心を計算
        for (auto &[id, pts] : cluster_points) {
            if (pts.size() >= MIN_CLUSTER_SIZE && pts.size() <= MAX_CLUSTER_SIZE) {
                geometry_msgs::msg::Point center;
                center.x = 0.0;
                center.y = 0.0;
                for (auto &p : pts) {
                    center.x += p.x;
                    center.y += p.y;
                }
                center.x /= pts.size();
                center.y /= pts.size();
                center.z = 0.0;
                cluster_centers.push_back(center);
                valid_cluster_ids.insert(id);
            }
        }
        return cluster_centers;
    }
    
    double calculateDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }

    // 前回のクラスタをもとにトラッキング
    void trackClusters(const std::vector<geometry_msgs::msg::Point> &current_centers) {
        cluster_id_mapping_.clear();
        std::vector<bool> matched_previous(previous_cluster_centers_.size(), false);

        for (size_t i = 0; i < current_centers.size(); i++) {
            double min_distance = std::numeric_limits<double>::max();
            int matched_id = -1;
            for (size_t j = 0; j < previous_cluster_centers_.size(); j++) {
                double dist = calculateDistance(current_centers[i], previous_cluster_centers_[j]);
                if (dist < CLUSTER_MATCHED_THRESH && dist < min_distance && !matched_previous[j]) {  // 0.3m以内で一致
                    min_distance = dist;
                    matched_id = j;
                }
            }
            if (matched_id != -1) {
                cluster_id_mapping_[i] = matched_id;  // 同一IDを付与
                matched_previous[matched_id] = true; // 前回のクラスタ中心と一致したかどうかのフラグ
            } else {
                cluster_id_mapping_[i] = next_cluster_id_++;  // 新規クラスタに新しいIDを付与
                std::cout << "新規クラスタ: " << i << " 新しいクラスタ番号: " << cluster_id_mapping_[i] << std::endl;
            }
        }
        for( int i=0; i<cluster_id_mapping_.size(); i++){
            std::cout << "i:  " << i << " クラスタ番号: " << cluster_id_mapping_[i] << std::endl;
        }
        previous_cluster_centers_ = current_centers;
    }

    void publishClusterMarkers(
        const std::vector<geometry_msgs::msg::Point> &points, 
        const std::vector<int> &clusters, 
        const std::set<int> &valid_cluster_ids) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "laser";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "cluster_markers";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.color.a = 1.0;

        for (size_t i = 0; i < points.size(); i++) {
            if (points[i].x != 0.0 || points[i].y != 0.0) {
                if (valid_cluster_ids.find(clusters[i]) != valid_cluster_ids.end()) {
                    marker.points.push_back(points[i]);
                    int cluster_id = clusters[i];
                    marker.colors.push_back(color_palette_[cluster_id % color_palette_.size()]);
                    std::cout << "点群 " << i << " はmapping後のクラスタ番号 " << cluster_id << " color番号: " << cluster_id%color_palette_.size() << std::endl;
                }
            }
        }
        cluster_marker_publisher_->publish(marker);
    }

    void publishMatchedClusterCenters(
        const std::vector<geometry_msgs::msg::Point> &current_centers, 
        const std::set<int> &valid_cluster_ids) {
        visualization_msgs::msg::Marker center_marker;
        center_marker.header.frame_id = "laser";
        center_marker.header.stamp = this->get_clock()->now();
        center_marker.ns = "matched_cluster_centers";
        center_marker.id = 1;
        center_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        center_marker.action = visualization_msgs::msg::Marker::ADD;
        center_marker.scale.x = 0.05;
        center_marker.scale.y = 0.05;
        center_marker.scale.z = 0.05;
        center_marker.color.a = 1.0;

        // 前回と一致したクラスタのみをパブリッシュ
        for (size_t i = 0; i < current_centers.size(); i++) {
            if (cluster_id_mapping_.find(i) != cluster_id_mapping_.end()) {  // 一致したクラスタIDが存在
                int cluster_id = cluster_id_mapping_[i];
                center_marker.points.push_back(current_centers[i]);
                center_marker.colors.push_back(color_palette_[cluster_id % color_palette_.size()]);
                // std::cout << "中心" << i << " mapping後のクラスタ番号: " << cluster_id << " color番号: " << cluster_id%color_palette_.size()<< std::endl;
            }
        }
        center_marker_publisher_->publish(center_marker);
    }

    std::vector<std_msgs::msg::ColorRGBA> generateColors(int num_clusters) {
        std::vector<std_msgs::msg::ColorRGBA> colors(num_clusters);
        std::mt19937 rng(42);  // シードを固定して毎回同じ色を生成
        std::uniform_real_distribution<float> dist(0.0, 1.0);
        for (auto &color : colors) {
            color.r = dist(rng);
            color.g = dist(rng);
            color.b = dist(rng);
            color.a = 1.0;
        }
        return colors;
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