#ifndef LEG_CLUSTER_TRACKING_HPP
#define LEG_CLUSTER_TRACKING_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "std_msgs/msg/bool.hpp" 
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
#include "icart_mini_leg_tracker/utils/marker_helper.hpp"
#include "icart_mini_leg_tracker/utils/csv_logger.hpp"
#include "icart_mini_leg_tracker/msg/cluster_info.hpp"
#include "icart_mini_leg_tracker/msg/cluster_info_array.hpp"

// 定数定義
#define MAX_NOISE_DISTANCE_THRESH 0.005  
#define MAX_SAMPLING_INTERVAL 0.001      
#define MIN_CLUSTER_SIZE 10              
#define MAX_CLUSTER_SIZE 100             
#define CLUSTER_MATCHED_THRESH 0.2 // クラスタとのマッチング閾値  
#define CLUSTER_LOST_MATCHED_THRESH 0.2 // 失われたクラスタとのマッチング閾値  
#define CLUSTER_TOLERANCE 0.05 // ?m以内の点を同じクラスタにする[m]
#define LOST_CLUSTER_TIMEOUT 1.0 // 失われたクラスタのタイムアウト[s]
#define FOOT_DISTANCE_THRESHOLD 0.3 //[m]
#define STOP_DISTANCE_THRESHOLD 0.3 //[m]
#define MAX_CLUSTER_DISTANCE 2.5 // クラスタとする距離範囲
#define MOVEMENT_THRESHOLD 0.5 // 急激な移動と判定するしきい値[m]
#define STATIC_SPEED_THRESHOLD 0.1 // 静止状態と判定するしきい値[m/s]
#define STATIC_FRAME_LIMIT 30 // 静止状態と判定するフレーム数

// 速度制限
#define MAX_SPEED 0.25
#define MIN_SPEED 0.05
#define MAX_TURN_SPEED M_PI / 4.0
#define PREDICTED_VEL_GAIN 0.1
#define LOST_PREDICTED_VEL_GAIN 0.05

// PID制御用のゲイン
#define KP_DIST  0.5
#define KI_DIST 0.01
#define KP_ANGLE 1.0 
#define KI_ANGLE 0.001
#define MAX_DIST_INTEGRAL 10.0
#define MAX_ANGLE_INTEGRAL M_PI

#define EMERGENCY_BUTTON 5
#define UNLOCK_EMERGENCY_BUTTON 4
#define FOLLOWME_START_BUTTON 7
#define FOLLOWME_STOP_BUTTON 6

// ログファイル
#define FILENAME "/root/icart_ws/src/icart_mini_ros2/icart_mini_leg_tracker/logs/cluster_tracking_log.csv"

namespace icart_msg = icart_mini_leg_tracker::msg;

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
    void matchLostClusters(std::map<int, geometry_msgs::msg::Point> &current_centers, std::map<int, int> &temp_cluster_mapping_);
    void matchPreviousClusters(std::map<int, geometry_msgs::msg::Point> &current_centers, std::map<int, int> &temp_cluster_mapping_, std::map<int, bool> &matched_previous);
    void storeLostClusters(std::map<int, bool> &matched_previous);
    void calculateClusterVelocities(
        const std::map<int, geometry_msgs::msg::Point> &current_centers, 
        const rclcpp::Time &current_time);
    void smoothAndFilterVelocities(const std::map<int, geometry_msgs::msg::Point> &current_centers);
    bool filterClustersByRegion(std::map<int, geometry_msgs::msg::Point> &cluster_centers);
    int initializeTarget(const std::map<int, geometry_msgs::msg::Point> &cluster_centers, geometry_msgs::msg::Point &target_pos);
    bool verifyPreviousTarget(const std::map<int, geometry_msgs::msg::Point> &cluster_centers, int &target_id, geometry_msgs::msg::Point &target_pos, double &movement);
    std::optional<std::pair<int, geometry_msgs::msg::Point>> selectNewTarget(
        const std::map<int, geometry_msgs::msg::Point> &cluster_centers, 
        const geometry_msgs::msg::Point &previous_target_pos, 
        bool previous_target_found);
    std::optional<std::pair<int, geometry_msgs::msg::Point>> findSecondaryCluster(
        const std::map<int, geometry_msgs::msg::Point> &cluster_centers, 
        int primary_target_id, 
        const geometry_msgs::msg::Point &primary_target_pos);
    void updateTrackingState(int target_id, int second_id);
    void followTarget(const std::map<int, geometry_msgs::msg::Point> &cluster_centers);
    void resetFollowTarget();

    // 可視化関連
    void publishClusterMarkers(const std::vector<geometry_msgs::msg::Point> &points, const std::vector<int> &clusters);
    void publishMatchedClusterCenters(const std::map<int, geometry_msgs::msg::Point> &current_centers);
    void publishPersonMarker(const geometry_msgs::msg::Point &target_pos);
    void publishClusterInfoMap();
    
    // 移動制御
    void publishCmdVel(double target_distance, double target_angle);
    
    // メンバ変数
    std::vector<std_msgs::msg::ColorRGBA> color_palette_;
    int next_cluster_id_;
    bool is_ready_for_tracking;
    bool start_followme_flag;
    rclcpp::Time previous_time_;
    
    std::map<int, int> cluster_id_mapping_;
    std::map<int, geometry_msgs::msg::Vector3> cluster_velocities_;
    std::map<int, std::pair<geometry_msgs::msg::Point, rclcpp::Time>> lost_clusters_;
    std::map<int, geometry_msgs::msg::Vector3> lost_cluster_velocities_;
    std::map<int, std::vector<int>> cluster_id_history_;  // クラスタID履歴（複数フレーム分保存）
    std::map<int, std::vector<geometry_msgs::msg::Vector3>> cluster_velocity_history_;  // クラスタごとの速度履歴
    std::map<int, icart_msg::ClusterInfo> cluster_info_map_; // クラスタ情報（ID, 中心座標, 速度, 静止状態）
    std::map<int, icart_msg::ClusterInfo> previous_cluster_info_map_;  // 前回のクラスタ情報
    std::map<int, int> cluster_static_frame_count_;  // クラスタの静止フレーム数

    int target_id = -1;
    int previous_target_id_;
    int previous_second_id_;
    bool is_target_initialized_;
    bool stop_by_joystick_;
    int current_target_id_;  // 追従対象のクラスタID
    int current_second_id_;  // 追従対象のクラスタID
    
    // PID
    double prev_error_dist = 0.0, integral_dist = 0.0;
    double prev_error_angle = 0.0, integral_angle = 0.0;
    
    // utils
    std::shared_ptr<MarkerHelper> marker_helper_;
    std::shared_ptr<CSVLogger> csv_logger_;

    // ROS2 ノード関連
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr person_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr center_marker_publisher_;
    rclcpp::Publisher<icart_msg::ClusterInfoArray>::SharedPtr cluster_info_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_lost_target_publisher_;
};

#endif // LEG_CLUSTER_TRACKING_HPP
