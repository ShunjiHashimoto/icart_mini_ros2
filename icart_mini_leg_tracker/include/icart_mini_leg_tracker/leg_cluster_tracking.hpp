#ifndef LEG_CLUSTER_TRACKING_HPP
#define LEG_CLUSTER_TRACKING_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <vector>
#include <cmath>
#include <map>
#include <set>
#include <random>
#include <iomanip> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <fstream>

// 定数定義
#define MAX_NOISE_DISTANCE_THRESH 0.005  
#define MAX_SAMPLING_INTERVAL 0.001      
#define MIN_CLUSTER_SIZE 10              
#define MAX_CLUSTER_SIZE 100             
#define CLUSTER_MATCHED_THRESH 0.2 // クラスタとのマッチング閾値  
#define CLUSTER_LOST_MATCHED_THRESH 0.2 // 失われたクラスタとのマッチング閾値  
#define CLUSTER_TOLERANCE 0.05
#define LOST_CLUSTER_TIMEOUT 1.0 
#define FOOT_DISTANCE_THRESHOLD 0.3
#define STOP_DISTANCE_THRESHOLD 0.3
#define MAX_CLUSTER_DISTANCE 3.0 // クラスタとする距離範囲

// 速度制限
#define MAX_SPEED 0.1
#define MIN_SPEED 0.05
#define MAX_TURN_SPEED M_PI / 4.0
#define PREDICTED_VEL_GAIN 0.1

// PID制御用のゲイン
#define KP_DIST  0.5
#define KI_DIST 0.01
#define KP_ANGLE 1.0 
#define KI_ANGLE 0.001

// ログファイル
#define FILENAME "/root/icart_ws/src/icart_mini_r.os2/icart_mini_leg_tracker/logs/cluster_tracking_log.csv"

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
    void saveClusterDataToCSV(const std::map<int, std::vector<int>>& cluster_id_history_,
        const std::map<int, geometry_msgs::msg::Vector3>& cluster_velocities_,  const std::map<int, geometry_msgs::msg::Point>& current_centers);

    // トラッキング関連
    void trackClusters(std::map<int, geometry_msgs::msg::Point> &current_centers);
    void matchLostClusters(std::map<int, geometry_msgs::msg::Point> &current_centers, std::map<int, int> &temp_cluster_mapping_);
    void matchPreviousClusters(std::map<int, geometry_msgs::msg::Point> &current_centers, std::map<int, int> &temp_cluster_mapping_, std::map<int, bool> &matched_previous);
    void storeLostClusters(std::map<int, bool> &matched_previous);
    void calculateClusterVelocities(
        const std::map<int, geometry_msgs::msg::Point> &current_centers, 
        const rclcpp::Time &current_time);
    bool filterClustersByRegion(std::map<int, geometry_msgs::msg::Point> &cluster_centers);
    void followTarget(const std::map<int, geometry_msgs::msg::Point> &cluster_centers);

    // 可視化関連
    void publishClusterMarkers(const std::vector<geometry_msgs::msg::Point> &points, const std::vector<int> &clusters);
    void publishMatchedClusterCenters(const std::map<int, geometry_msgs::msg::Point> &current_centers);
    void publishPersonMarker(const geometry_msgs::msg::Point &target_pos);
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
    std::map<int, std::vector<int>> cluster_id_history_;  // クラスタID履歴（複数フレーム分保存）
    std::map<int, std::vector<geometry_msgs::msg::Vector3>> cluster_velocity_history_;  // クラスタごとの速度履歴
    int previous_target_id_;
    int previous_second_id_;
    bool is_target_initialized_;
    bool stop_by_joystick_;
    int current_target_id_;  // 追従対象のクラスタID
    int current_second_id_;  // 追従対象のクラスタID
    
    // PID
    double prev_error_dist = 0.0, integral_dist = 0.0;
    double prev_error_angle = 0.0, integral_angle = 0.0;

    // ROS2 ノード関連
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cluster_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr person_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr center_marker_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

#endif // LEG_CLUSTER_TRACKING_HPP
