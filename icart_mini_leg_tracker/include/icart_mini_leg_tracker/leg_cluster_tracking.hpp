#ifndef LEG_CLUSTER_TRACKING_HPP
#define LEG_CLUSTER_TRACKING_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <vector>
#include <cmath>
#include <map>
#include <set>
#include <random>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// 定数定義
#define MAX_NOISE_DISTANCE_THRESH 0.005  
#define MAX_SAMPLING_INTERVAL 0.001      
#define MIN_CLUSTER_SIZE 10              
#define MAX_CLUSTER_SIZE 100             
#define CLUSTER_MATCHED_THRESH 0.3  
#define CLUSTER_TOLERANCE 0.05
#define LOST_CLUSTER_TIMEOUT 2.0 

class LegClusterTracking : public rclcpp::Node {
public:
    LegClusterTracking();
    
private:
    // コールバック関数
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    // データ処理関連
    std::vector<geometry_msgs::msg::Point> generateXYPoints(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void removeNoise(std::vector<geometry_msgs::msg::Point> &points);
    void downSampling(std::vector<geometry_msgs::msg::Point> &points);
    std::vector<int> makeClustersPCL(const std::vector<geometry_msgs::msg::Point> &points);
    std::map<int, geometry_msgs::msg::Point> calculateClusterCenters(
        const std::vector<geometry_msgs::msg::Point> &points, 
        const std::vector<int> &clusters);
    double calculateDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2); 

    // トラッキング関連
    void trackClusters(std::map<int, geometry_msgs::msg::Point> &current_centers);
    void calculateClusterVelocities(
        const std::map<int, geometry_msgs::msg::Point> &current_centers, 
        const rclcpp::Time &current_time);
    bool filterClustersByRegion(std::map<int, geometry_msgs::msg::Point> &cluster_centers);
    void followTarget(const std::map<int, geometry_msgs::msg::Point> &cluster_centers);

    // 可視化関連
    void publishClusterMarkers(const std::vector<geometry_msgs::msg::Point> &points, const std::vector<int> &clusters);
    void publishMatchedClusterCenters(const std::map<int, geometry_msgs::msg::Point> &current_centers);
    std::vector<std_msgs::msg::ColorRGBA> generateColors(int num_clusters);

    // 移動制御
    void publishCmdVel(double target_distance, double target_angle);

    // メンバ変数
    std::map<int, int> cluster_id_mapping_;
    std::map<int, geometry_msgs::msg::Point> previous_cluster_centers_;
    std::vector<std_msgs::msg::ColorRGBA> color_palette_;
    int next_cluster_id_;
    bool is_ready_for_tracking;
    std::map<int, geometry_msgs::msg::Vector3> cluster_velocities_;
    rclcpp::Time previous_time_;
    std::map<int, std::pair<geometry_msgs::msg::Point, rclcpp::Time>> lost_clusters_;
    std::map<int, geometry_msgs::msg::Vector3> lost_cluster_velocities_;
    int previous_target_id_;
    bool is_target_initialized_;
    bool stop_by_joystick_;

    // ROS2 ノード関連
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cluster_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr center_marker_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

#endif // LEG_CLUSTER_TRACKING_HPP

