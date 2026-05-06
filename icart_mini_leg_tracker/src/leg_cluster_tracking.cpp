#include "icart_mini_leg_tracker/leg_cluster_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstddef>
#include <limits>

LegClusterTracking::LegClusterTracking() : 
    Node("leg_cluster_tracking_node"),
    next_cluster_id_(1), 
    is_ready_for_tracking(false), 
    start_followme_flag(false),
    is_target_initialized_(false), 
    stop_by_joystick_(false),
    follow_tracking_state_(FollowTrackingState::Idle),
    debug_tracking_state_("idle"),
    target_lost_timer_active_(false),
    has_rejected_candidate_(false),
    marker_helper_(std::make_shared<MarkerHelper>(1000)), 
    csv_logger_(std::make_shared<CSVLogger>(FILENAME)),
    accumulated_loop_period_(0.0),
    loop_sample_count_(0)
    {

    lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(10).best_effort(), 
        std::bind(&LegClusterTracking::scanCallback, this, std::placeholders::_1)
    );
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", rclcpp::QoS(10).best_effort(), std::bind(&LegClusterTracking::joyCallback, this, std::placeholders::_1)
    );
    follow_control_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/follow_me/control", 10, std::bind(&LegClusterTracking::followControlCallback, this, std::placeholders::_1)
    );

    cluster_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/leg_tracker/cluster_markers", 10);
    center_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/leg_tracker/cluster_centers", 10);
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    person_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/leg_tracker/person_marker", 10);
    debug_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/leg_tracker/debug_markers", 10);
    cluster_info_publisher_ = this->create_publisher<icart_msg::ClusterInfoArray>("/leg_tracker/cluster_infos", 10);
    is_lost_target_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/leg_tracker/is_lost_target", 10);

    RCLCPP_INFO(this->get_logger(), "Leg cluster and tracking started.");

    target_id = -1;
    previous_target_id_ = -1;
    previous_second_id_ = -1;
    current_target_id_ = -1;
    current_second_id_ = -1;
    clearLastSelectionInfo();

    target_lost_start_time_ = this->now();
    last_callback_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    publishLostState(false);
}

void LegClusterTracking::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    auto is_pressed = [&msg](std::size_t index) {
        return index < msg->buttons.size() && msg->buttons[index] == 1;
    };

    if (is_pressed(EMERGENCY_BUTTON)) {
        setEmergencyStop(true, "/joy");
    }
    else if (is_pressed(UNLOCK_EMERGENCY_BUTTON)) {
        setEmergencyStop(false, "/joy");
    }
    else if (is_pressed(FOLLOWME_START_BUTTON)) {
        startFollowMe("/joy");
    }
    else if (is_pressed(FOLLOWME_STOP_BUTTON)) {
        stopFollowMe("/joy");
    }
}

void LegClusterTracking::followControlCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::string command = msg->data;
    std::transform(command.begin(), command.end(), command.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (command == "start" || command == "follow_start" || command == "f") {
        startFollowMe("/follow_me/control");
    } else if (command == "stop" || command == "follow_stop" || command == "g") {
        stopFollowMe("/follow_me/control");
    } else if (command == "emergency_stop" || command == "estop" || command == "space") {
        setEmergencyStop(true, "/follow_me/control");
    } else if (command == "clear_emergency_stop" || command == "clear_estop" || command == "clear" || command == "c") {
        setEmergencyStop(false, "/follow_me/control");
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown follow control command: '%s'", msg->data.c_str());
    }
}

void LegClusterTracking::startFollowMe(const std::string &source) {
    resetFollowTarget();
    start_followme_flag = true;
    setFollowTrackingState(FollowTrackingState::WaitingInitial);
    RCLCPP_WARN(this->get_logger(), "追従開始 (%s)", source.c_str());
    publishLostState(false);
}

void LegClusterTracking::stopFollowMe(const std::string &source) {
    start_followme_flag = false;
    resetFollowTarget();
    setFollowTrackingState(FollowTrackingState::Stopped);
    publishCmdVel(0.0, 0.0);
    RCLCPP_WARN(this->get_logger(), "追従停止 (%s)", source.c_str());
    publishLostState(true);
}

void LegClusterTracking::setEmergencyStop(bool enabled, const std::string &source) {
    stop_by_joystick_ = enabled;
    if (enabled) {
        setFollowTrackingState(FollowTrackingState::EmergencyStop);
        publishCmdVel(0.0, 0.0);
        publishLostState(true);
        RCLCPP_WARN(this->get_logger(), "非常停止 (%s)", source.c_str());
    } else {
        publishLostState(false);
        RCLCPP_WARN(this->get_logger(), "非常停止解除 (%s)", source.c_str());
    }
}

void LegClusterTracking::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (start_followme_flag == false) return;
    if (stop_by_joystick_ == true) return;
    auto current_time = this->get_clock()->now();
    if (last_callback_time_.nanoseconds() != 0) {
        double loop_period = (current_time - last_callback_time_).seconds();
        accumulated_loop_period_ += loop_period;
        loop_sample_count_++;
        if (loop_sample_count_ >= LOOP_PERIOD_SAMPLE_WINDOW) {
            double average_period = accumulated_loop_period_ / static_cast<double>(loop_sample_count_);
            double frequency = (average_period > 1e-6) ? (1.0 / average_period) : 0.0;
            RCLCPP_INFO(this->get_logger(), "scanCallback 平均周期: %.3f s (%.1f Hz)", average_period, frequency);
            accumulated_loop_period_ = 0.0;
            loop_sample_count_ = 0;
        }
    }
    last_callback_time_ = current_time;
    auto points = generateXYPoints(msg);
    removeNoise(points);
    downSampling(points);
    auto clusters = makeClustersPCL(points);
    auto cluster_centers = calculateClusterCenters(points, clusters);

    if (!is_ready_for_tracking) {
        is_ready_for_tracking = filterClustersByRegion(cluster_centers);
    } else {
        trackClusters(cluster_centers);
        followTarget(cluster_centers);
        publishDebugMarkers();
        publishClusterMarkers(points, clusters);
        publishMatchedClusterCenters(cluster_centers);
        publishClusterInfoMap();
    }
}

std::vector<geometry_msgs::msg::Point> LegClusterTracking::generateXYPoints(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
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

void LegClusterTracking::removeNoise(std::vector<geometry_msgs::msg::Point> &points) {
    std::vector<bool> to_remove(points.size(), false);
    for (size_t i = 0; i < points.size(); i++) {
        for (size_t j = i + 1; j < points.size(); j++) {
            double dist = sqrt(pow(points[i].x - points[j].x, 2) + pow(points[i].y - points[j].y, 2));
            if (dist < MAX_NOISE_DISTANCE_THRESH) {
                to_remove[j] = true;
            }
        }
    }
    points.erase(std::remove_if(points.begin(), points.end(),
                [&](const geometry_msgs::msg::Point &p) { return to_remove[&p - &points[0]]; }),
                points.end());
}

void LegClusterTracking::downSampling(std::vector<geometry_msgs::msg::Point> &points) {
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

std::vector<int> LegClusterTracking::makeClustersPCL(const std::vector<geometry_msgs::msg::Point> &points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> cloud_to_points_index;  // cloud→元のpointsのインデックス対応
    
    // PCLのPointCloudに変換
    for (size_t i = 0; i < points.size(); ++i) {
        double distance = sqrt(points[i].x * points[i].x + points[i].y * points[i].y);
        if (distance > MAX_CLUSTER_DISTANCE) continue;  // フィルタ
        
        pcl::PointXYZ pcl_point;
        pcl_point.x = points[i].x;
        pcl_point.y = points[i].y;
        pcl_point.z = 0.0;
        cloud->points.push_back(pcl_point);
        cloud_to_points_index.push_back(i);  // このcloud点は元のpoints[i]に対応
    }

    std::vector<int> clusters(points.size(), 0);
    if (cloud->points.empty()) {
        return clusters;
    }

    // KD-Treeを作成
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
        
    std::vector<pcl::PointIndices> cluster_indices; // クラスタのインデックスを格納するベクタ
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(CLUSTER_TOLERANCE);  // 5cm以内の点を同じクラスタにする
    ec.setMinClusterSize(MIN_CLUSTER_SIZE);      // 最小クラスタサイズ
    ec.setMaxClusterSize(MAX_CLUSTER_SIZE);     // 最大クラスタサイズ
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // クラスタリング実行
    int cluster_id = 1;
    for (const auto &indices : cluster_indices) {
        for (int idx : indices.indices) {
            int original_index = cloud_to_points_index[idx];  // 対応する元のpointsのインデックス
            clusters[original_index] = cluster_id;
        }
        cluster_id++;
    }
    return clusters;
}

std::map<int, geometry_msgs::msg::Point> LegClusterTracking::calculateClusterCenters(
    const std::vector<geometry_msgs::msg::Point> &points, 
    const std::vector<int> &clusters)
    {
    std::map<int, std::vector<geometry_msgs::msg::Point>> cluster_points;
    // クラスタごとに点を集める
    for (size_t i = 0; i < points.size(); i++) {
        if (clusters[i] > 0) {
            cluster_points[clusters[i]].push_back(points[i]);
        }
    }

    std::map<int, geometry_msgs::msg::Point> cluster_centers;
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
            cluster_centers[id] = center;
        }
    }

    // クラスタごとにIDと中心座標を出力
    for (const auto &pair : cluster_centers) {
        int cluster_id = pair.first;                         // クラスタID
        const geometry_msgs::msg::Point &center = pair.second; // 中心座標
        RCLCPP_DEBUG(this->get_logger(), "現在のクラスタID: %d | 中心座標: (%.2f, %.2f, %.2f)", cluster_id, center.x, center.y, center.z);
    }
    return cluster_centers;
}
    
double LegClusterTracking::calculateDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

void LegClusterTracking::smoothAndFilterVelocities(const std::map<int, geometry_msgs::msg::Point> &current_centers) {
    for (const auto &[id, raw_velocity] : cluster_velocities_) {
        geometry_msgs::msg::Vector3 velocity = raw_velocity;

        // 履歴に追加
        cluster_velocity_history_[id].push_back(velocity);
        if (cluster_velocity_history_[id].size() > 5) {
            cluster_velocity_history_[id].erase(cluster_velocity_history_[id].begin());
        }

        // 平滑化
        geometry_msgs::msg::Vector3 smoothed_velocity{};
        for (const auto &v : cluster_velocity_history_[id]) {
            smoothed_velocity.x += v.x;
            smoothed_velocity.y += v.y;
        }
        int size = cluster_velocity_history_[id].size();
        smoothed_velocity.x /= size;
        smoothed_velocity.y /= size;
        // 最大速度制限
        double speed = sqrt(smoothed_velocity.x * smoothed_velocity.x + smoothed_velocity.y * smoothed_velocity.y);
        if (speed > 1.0) {
            smoothed_velocity.x *= (1.0 / speed);
            smoothed_velocity.y *= (1.0 / speed);
        }
        // 静止クラスタのカウント
        if (speed < STATIC_SPEED_THRESHOLD) {
            cluster_static_frame_count_[id]++;
        } else {
            cluster_static_frame_count_[id] = 0;
        }
        // is_static 判定 & 保存
        bool is_static = (cluster_static_frame_count_[id] > STATIC_FRAME_LIMIT && current_target_id_ != id && current_second_id_ != id);

        icart_msg::ClusterInfo info;
        info.id = id;
        info.center = current_centers.at(id);
        info.velocity = smoothed_velocity;
        info.is_static = is_static;
        info.is_target = (current_target_id_ == id || current_second_id_ == id);
        cluster_info_map_[id] = info;
        // if (is_static) {
        //     RCLCPP_INFO(this->get_logger(), "クラスタID: %d は静止状態", id);
        // }
    }
}

void LegClusterTracking::calculateClusterVelocities(
    const std::map<int, geometry_msgs::msg::Point> &current_centers, 
    const rclcpp::Time &current_time) {
    if (previous_cluster_info_map_.empty() || previous_time_.nanoseconds() == 0) {
        previous_time_ = current_time;
        return;  // 最初のフレームは速度計算をスキップ
    }

    double delta_time = (current_time - previous_time_).seconds();  // 経過時間[s]
    if (delta_time <= 0) return;  // 経過時間が0以下の場合は計算しない

    cluster_velocities_.clear();
    for (const auto &[id, current_center] : current_centers) {
        if (previous_cluster_info_map_.count(id) > 0) {
            const auto &prev_center = previous_cluster_info_map_[id].center;
            geometry_msgs::msg::Vector3 velocity;
            velocity.x = (current_center.x - prev_center.x) / delta_time;
            velocity.y = (current_center.y - prev_center.y) / delta_time;
            velocity.z = 0.0;
            cluster_velocities_[id] = velocity;
        }
    }
    smoothAndFilterVelocities(current_centers);
    previous_time_ = current_time;  // 次回のために時間を更新
}

bool LegClusterTracking::filterClustersByRegion(std::map<int, geometry_msgs::msg::Point> &cluster_centers) {
    for (auto it = cluster_centers.begin(); it != cluster_centers.end(); ) {
        const auto &center = it->second;
        // 正面1.0m以内かつ左右0.5m以内のクラスタのみ採用
        if (!(center.x > 0 && center.x < INITIAL_TARGET_MAX_X && fabs(center.y) < INITIAL_TARGET_MAX_ABS_Y)) {
            RCLCPP_INFO(this->get_logger(), "クラスタID %d は有効領域外のため除外", it->first);
            it = cluster_centers.erase(it);  // 条件を満たさないクラスタを削除
        } else {
            ++it;
        }
    }
    if(cluster_centers.empty()) {
        RCLCPP_INFO(this->get_logger(), "有効なクラスタが見つかりませんでした");
        return false;
    }
    else{
        return true;
    }
}

void LegClusterTracking::matchLostClusters(
    std::map<int, geometry_msgs::msg::Point> &current_centers,
    std::map<int, int> &temp_cluster_mapping_,
    std::map<int, geometry_msgs::msg::Point> &recovered_lost_centers) {
    for (auto &[current_id, current_center] : current_centers) {
        double min_distance = std::numeric_limits<double>::max();
        int matched_id = -1;
        geometry_msgs::msg::Point best_predicted_center{};
        bool has_prediction = false;

        for (auto it = lost_clusters_.begin(); it != lost_clusters_.end();) {
            int lost_id = it->first;
            geometry_msgs::msg::Point lost_center = it->second.first;
            rclcpp::Time lost_time = it->second.second;

            // 一定時間経過したクラスタは破棄
            double elapsed_time = this->get_clock()->now().seconds() - lost_time.seconds();
            // RCLCPP_INFO(this->get_logger(), "失われたクラスタID: %d | 経過時間: %.2f秒", lost_id, elapsed_time);
            if (elapsed_time > LOST_CLUSTER_TIMEOUT) {
                it = lost_clusters_.erase(it);
                lost_cluster_velocities_.erase(lost_id);
                cluster_id_history_.erase(lost_id);
                continue;
            }

            // 失われたクラスタの予測位置を計算
            geometry_msgs::msg::Point predicted_center = lost_center;
            if (lost_cluster_velocities_.count(lost_id) > 0) {
                const auto &velocity = lost_cluster_velocities_[lost_id];
                predicted_center.x += velocity.x * elapsed_time * LOST_PREDICTED_VEL_GAIN;
                predicted_center.y += velocity.y * elapsed_time * LOST_PREDICTED_VEL_GAIN;
            }

            // 距離を比較してマッチング候補を決定
            double dist = calculateDistance(current_center, predicted_center);
            if (dist < CLUSTER_LOST_MATCHED_THRESH && dist < min_distance) {
                min_distance = dist;
                matched_id = lost_id;
                best_predicted_center = predicted_center;
                has_prediction = true;
            }
            ++it;
        }

        if (matched_id != -1) {
            temp_cluster_mapping_[current_id] = matched_id;
            if (has_prediction) {
                recovered_lost_centers[matched_id] = best_predicted_center;
            }
            lost_clusters_.erase(matched_id);
            lost_cluster_velocities_.erase(matched_id);
            // std::cout << "失われたクラスタID " << matched_id << " が復活" << std::endl;
        }
    }
}

// 前回のクラスタとマッチングを試みる関数
void LegClusterTracking::matchPreviousClusters(
    std::map<int, geometry_msgs::msg::Point> &current_centers,
    std::map<int, int> &temp_cluster_mapping_,
    std::map<int, bool> &matched_previous,
    const std::map<int, geometry_msgs::msg::Point> &recovered_lost_centers) {
    for (auto &[current_id, current_center] : current_centers) {
        if (cluster_id_mapping_.count(current_id) > 0) continue;  // すでにマッチ済み

        double min_distance = std::numeric_limits<double>::max();
        double nearest_distance = std::numeric_limits<double>::max();
        int matched_id = -1;

        for (const auto &[prev_id, prev_info] : previous_cluster_info_map_) {
            auto matched_it = matched_previous.find(prev_id);
            if (matched_it != matched_previous.end() && matched_it->second) {
                continue;  // この prev_id は既に別クラスタに割り当て済み
            }
            geometry_msgs::msg::Point predicted_center = prev_info.center;
            if (cluster_info_map_.count(prev_id) > 0) {
                const auto &velocity = cluster_info_map_[prev_id].velocity;
                double delta_time = (this->get_clock()->now() - previous_time_).seconds();
                predicted_center.x += velocity.x * delta_time * PREDICTED_VEL_GAIN;
                predicted_center.y += velocity.y * delta_time * PREDICTED_VEL_GAIN;
                // RCLCPP_INFO(this->get_logger(), "クラスタID: %d | 差分xy: (%.2f, %.2f), 予測位置: (%.2f, %.2f)", prev_id, velocity.x*delta_time*PREDICTED_VEL_GAIN, velocity.y*delta_time*PREDICTED_VEL_GAIN, predicted_center.x, predicted_center.y);
            }
            double dist = calculateDistance(current_center, predicted_center);
            if (dist < nearest_distance) {
                nearest_distance = dist;
            }

            if (dist < CLUSTER_MATCHED_THRESH && dist < min_distance) {
                min_distance = dist;
                matched_id = prev_id;
            }
        }

        // ここでロストクラスタと前回のクラスタの両方を比較し、最適なものを採用
        if (temp_cluster_mapping_.count(current_id) > 0) {
            int lost_matched_id = temp_cluster_mapping_[current_id];
            auto recovered_it = recovered_lost_centers.find(lost_matched_id);
            if (recovered_it != recovered_lost_centers.end()) {
                double lost_dist = calculateDistance(current_center, recovered_it->second);
                if (matched_id == -1 || lost_dist < min_distance) {
                    matched_id = lost_matched_id;  // ロストクラスタのIDを採用
                    min_distance = lost_dist;
                    RCLCPP_DEBUG(this->get_logger(), "ロストクラスタID: %d | 距離: %.2f が優先されました", matched_id, lost_dist);
                }
            }
        }

        if (matched_id == -1) {
            if (nearest_distance == std::numeric_limits<double>::max()) {
                RCLCPP_DEBUG(this->get_logger(), "現在のクラスタID: %d | 前回のクラスタID: %d | 距離: N/A", current_id, matched_id);
            } else {
                RCLCPP_DEBUG(this->get_logger(), "現在のクラスタID: %d | 前回のクラスタID: %d | 距離: %.3f (未マッチ)", current_id, matched_id, nearest_distance);
            }
        } else {
            RCLCPP_DEBUG(this->get_logger(), "現在のクラスタID: %d | 前回のクラスタID: %d | 距離: %.3f", current_id, matched_id, min_distance);
        }

        // マッチしたクラスタIDをマッピング、なければ新規付与
        if (matched_id != -1) {
            cluster_id_mapping_[current_id] = matched_id;
            matched_previous[matched_id] = true;
            cluster_id_history_[matched_id].push_back(current_id);
            if (cluster_id_history_[matched_id].size() > 5) {  // 過去5フレーム分だけ保持
                cluster_id_history_[matched_id].erase(cluster_id_history_[matched_id].begin());
            }
        } else {
            cluster_id_mapping_[current_id] = next_cluster_id_++;
        }
    }
}

// マッチしなかったクラスタを失われたクラスタとして保存する関数
void LegClusterTracking::storeLostClusters(std::map<int, bool> &matched_previous) {
    for (const auto &[prev_id, prev_info] : previous_cluster_info_map_) {
        if (!matched_previous[prev_id] && lost_clusters_.count(prev_id) == 0) {
            lost_clusters_[prev_id] = {prev_info.center, this->get_clock()->now()};
            if (cluster_info_map_.count(prev_id) > 0) {
                lost_cluster_velocities_[prev_id] = cluster_info_map_[prev_id].velocity;
            }
        }
    }
}

// 前回のクラスタをもとにトラッキング
void LegClusterTracking::trackClusters(std::map<int, geometry_msgs::msg::Point> &current_centers) {
    cluster_id_mapping_.clear();
    std::map<int, bool> matched_previous;
    std::map<int, int> temp_cluster_mapping_;  // 仮のクラスタマッピング（ロストクラスタ用）
    std::map<int, geometry_msgs::msg::Point> recovered_lost_centers;

    if (previous_cluster_info_map_.empty()) {
        // 初期化として current_centers の ID に対する ClusterInfo を作って保存
        for (const auto& [id, center] : current_centers) {
            icart_msg::ClusterInfo info;
            info.id = id;
            info.center = center;
            info.velocity = geometry_msgs::msg::Vector3();  // 初期化
            info.is_static = false;
            previous_cluster_info_map_[id] = info;
        }
        return;
    }
    // 前回のクラスタをまだマッチしていない状態に初期化
    for (const auto &[prev_id, prev_info] : previous_cluster_info_map_) {
        matched_previous[prev_id] = false;
    }

    // 【1】失われたクラスタとマッチングを試みる
    matchLostClusters(current_centers, temp_cluster_mapping_, recovered_lost_centers);

    // 【2】前回のクラスタと比較して最も近いクラスタを探す, ロストクラスタのほうが近い場合はそちらを採用
    matchPreviousClusters(current_centers, temp_cluster_mapping_, matched_previous, recovered_lost_centers);

    // 【3】マッチしなかったクラスタを"失われたクラスタ"として保存
    storeLostClusters(matched_previous);

    // 最終マッピング結果を出力
    std::map<int, geometry_msgs::msg::Point> updated_centers;
    for (const auto &[current_id, previous_id] : cluster_id_mapping_) {
        updated_centers[previous_id] = current_centers[current_id];
    }

    calculateClusterVelocities(updated_centers, this->get_clock()->now());
    // 最新のクラスタ中心を保存
    // previous_cluster_info_map_.clear();
    for (const auto& [id, center] : updated_centers) {
        previous_cluster_info_map_[id].center = center;
    }
    current_centers = updated_centers; 
}

// 追従対象の初期選択を行う関数
int LegClusterTracking::initializeTarget(const std::map<int, geometry_msgs::msg::Point> &cluster_centers, geometry_msgs::msg::Point &target_pos) {
    double min_distance = std::numeric_limits<double>::max();
    int selected_target_id = -1;
    for (const auto &[current_id, current_center] : cluster_centers) {
        // 初期取得だけはロボット正面の有効領域に限定し、横や遠方の障害物を開始対象にしない。
        if (!(current_center.x > 0 &&
              current_center.x < INITIAL_TARGET_MAX_X &&
              std::fabs(current_center.y) < INITIAL_TARGET_MAX_ABS_Y)) {
            continue;
        }

        double dist = sqrt(current_center.x * current_center.x + current_center.y * current_center.y);
        if (dist < min_distance) {
            min_distance = dist;
            selected_target_id = current_id;
            target_pos = current_center;
        }
    }
    target_id = selected_target_id;
    return target_id;
}

// 追従対象がまだ存在しているか確認する関数
bool LegClusterTracking::verifyPreviousTarget(const std::map<int, geometry_msgs::msg::Point> &cluster_centers, int &target_id, geometry_msgs::msg::Point &target_pos, double &movement) {
    if (previous_target_id_ != -1 && cluster_centers.count(previous_target_id_)) {
        geometry_msgs::msg::Point current_pos = cluster_centers.at(previous_target_id_);
        target_id = previous_target_id_;
        target_pos = current_pos;
        movement = calculateDistance(current_pos, previous_target_pos_);
        // RCLCPP_INFO(this->get_logger(), "前回の追従対象1 (ID: %d) を継続 [移動距離: %.3f]", target_id, movement);
        return true;
    } else if (previous_second_id_ != -1 && cluster_centers.count(previous_second_id_)) {
        geometry_msgs::msg::Point current_pos = cluster_centers.at(previous_second_id_);
        target_pos = current_pos;
        target_id = previous_second_id_;
        movement = calculateDistance(current_pos, previous_target_pos_);
        // RCLCPP_INFO(this->get_logger(), "前回の追従対象2 (ID: %d) を継続 [移動距離: %.3f]", target_id, movement);
        return true;
    }
    // RCLCPP_INFO(this->get_logger(), "前回の追従対象（ID: %d）をロスト", target_id);
    movement = 0.0;
    return false;
}

std::optional<std::pair<int, geometry_msgs::msg::Point>> 
LegClusterTracking::selectReacquisitionTarget(
    const std::map<int, geometry_msgs::msg::Point> &cluster_centers) {
    double best_gate_distance = std::numeric_limits<double>::max();
    double best_distance_to_robot = std::numeric_limits<double>::max();
    double best_angle_diff = -1.0;
    int new_target_id = -1;
    geometry_msgs::msg::Point new_target_pos;

    double nearest_rejected_gate_distance = std::numeric_limits<double>::max();
    double nearest_rejected_distance_to_robot = -1.0;
    double nearest_rejected_angle_diff = -1.0;
    int nearest_rejected_id = -1;
    geometry_msgs::msg::Point nearest_rejected_pos;
    const auto predicted_target_pos = predictedTargetPosition();

    last_selection_called_ = true;
    last_selection_target_id_ = -1;
    last_selection_reason_ = "processing";
    last_selection_movement_ = -1.0;
    last_selection_distance_to_robot_ = -1.0;
    last_selection_timestamp_ = this->now().seconds();

    RCLCPP_INFO(this->get_logger(),
                "selectReacquisitionTarget開始: cluster_count=%zu, previous=(%.2f, %.2f), predicted=(%.2f, %.2f)",
                cluster_centers.size(),
                previous_target_pos_.x, previous_target_pos_.y,
                predicted_target_pos.x, predicted_target_pos.y);

    for (const auto &[current_id, current_center] : cluster_centers) {
        const double distance_to_previous = calculateDistance(current_center, previous_target_pos_);
        const double distance_to_prediction = calculateDistance(current_center, predicted_target_pos);
        const double gate_distance = std::min(distance_to_previous, distance_to_prediction);
        const double distance_to_robot = std::hypot(current_center.x, current_center.y);
        const double previous_angle = std::atan2(previous_target_pos_.y, previous_target_pos_.x);
        const double current_angle = std::atan2(current_center.y, current_center.x);
        const double angle_diff = std::fabs(std::atan2(
            std::sin(current_angle - previous_angle),
            std::cos(current_angle - previous_angle)));

        RCLCPP_INFO(this->get_logger(),
                    "再捕捉候補ID: %d, center=(%.2f, %.2f), previous_dist=%.3f, predicted_dist=%.3f, gate_dist=%.3f",
                    current_id, current_center.x, current_center.y,
                    distance_to_previous, distance_to_prediction, gate_distance);

        // 追従中の再捕捉では、近い障害物へ乗り移らないようにロボット距離では選ばない。
        // 前回位置または予測位置の近傍だけを、同じ人物の候補として扱う。
        if (gate_distance <= MOVEMENT_THRESHOLD) {
            if (gate_distance < best_gate_distance) {
                best_gate_distance = gate_distance;
                best_distance_to_robot = distance_to_robot;
                best_angle_diff = angle_diff;
                new_target_id = current_id;
                new_target_pos = current_center;
            }
        } else if (gate_distance < nearest_rejected_gate_distance) {
            nearest_rejected_gate_distance = gate_distance;
            nearest_rejected_distance_to_robot = distance_to_robot;
            nearest_rejected_angle_diff = angle_diff;
            nearest_rejected_id = current_id;
            nearest_rejected_pos = current_center;
        }
    }

    if (new_target_id == -1) {
        if (nearest_rejected_id != -1) {
            last_selection_target_id_ = nearest_rejected_id;
            last_selection_reason_ = "movement_threshold";
            last_selection_movement_ = nearest_rejected_gate_distance;
            last_selection_distance_to_robot_ = nearest_rejected_distance_to_robot;
            last_selection_angle_diff_ = nearest_rejected_angle_diff;
            last_rejection_reason_ = "movement_threshold";
            has_rejected_candidate_ = true;
            rejected_candidate_pos_ = nearest_rejected_pos;
            RCLCPP_WARN(this->get_logger(),
                        "再捕捉候補ID %d は前回/予測位置から離れすぎているためreject (距離: %.3f)",
                        nearest_rejected_id, nearest_rejected_gate_distance);
        } else {
            last_selection_reason_ = "not_found";
            last_rejection_reason_ = "not_found";
            RCLCPP_INFO(this->get_logger(), "selectReacquisitionTarget結果: 候補が見つかりませんでした");
        }
        return std::nullopt;
    }

    RCLCPP_INFO(this->get_logger(),
                "selectReacquisitionTarget結果: ID=%d を前回/予測位置優先で選択 (距離: %.3f, ロボット距離: %.3f)",
                new_target_id, best_gate_distance, best_distance_to_robot);
    last_selection_target_id_ = new_target_id;
    last_selection_reason_ = "previous_target";
    last_selection_movement_ = best_gate_distance;
    last_selection_distance_to_robot_ = best_distance_to_robot;
    last_selection_angle_diff_ = best_angle_diff;
    last_rejection_reason_ = "none";

    return std::make_pair(new_target_id, new_target_pos);
}

std::optional<std::pair<int, geometry_msgs::msg::Point>> 
LegClusterTracking::findSecondaryCluster(const std::map<int, geometry_msgs::msg::Point> &cluster_centers, 
                                         int primary_target_id, 
                                         const geometry_msgs::msg::Point &primary_target_pos) {
    int second_id = -1;
    geometry_msgs::msg::Point second_center;
    for (const auto &[current_id, current_center] : cluster_centers) {
        if (current_id == primary_target_id) continue;
        double dist = sqrt(pow(current_center.x - primary_target_pos.x, 2) + 
                           pow(current_center.y - primary_target_pos.y, 2));
        if (dist < FOOT_DISTANCE_THRESHOLD) {
            second_id = current_id;
            second_center = current_center;
            return std::make_pair(second_id, second_center);
        }
    }
    return std::nullopt;  // 近くに適切なクラスタなし
}

void LegClusterTracking::updateTrackingState(int target_id, int second_id, geometry_msgs::msg::Point target_pos) {
    previous_target_id_ = target_id;
    previous_second_id_ = second_id;
    current_target_id_ = target_id;
    current_second_id_ = second_id;
    previous_target_pos_ = target_pos;
}

void LegClusterTracking::handleInitialTargetNotFound() {
    RCLCPP_WARN(this->get_logger(), "初期追従対象が見つかりませんでした。");
    setFollowTrackingState(FollowTrackingState::Lost);
    target_id = -1;
    previous_target_id_ = -1;
    previous_second_id_ = -1;
    current_target_id_ = -1;
    current_second_id_ = -1;
    previous_target_pos_ = geometry_msgs::msg::Point();
    startLostDebugTimer();
    publishCmdVel(0.0, 0.0);
    publishLostState(true);
    saveDebugCsv();
    start_followme_flag = false;
}

void LegClusterTracking::handleTargetLostTimeout() {
    RCLCPP_WARN(
        this->get_logger(),
        "追従対象を %.2f 秒以上再捕捉できないためLostに遷移します。",
        TARGET_LOST_TIMEOUT);
    setFollowTrackingState(FollowTrackingState::Lost);
    current_target_id_ = -1;
    current_second_id_ = -1;
    last_rejection_reason_ = "target_lost_timeout";
    publishCmdVel(0.0, 0.0);
    publishLostState(true);
    saveDebugCsv();
    publishDebugMarkers();
    start_followme_flag = false;
}

void LegClusterTracking::followTarget(const std::map<int, geometry_msgs::msg::Point> &cluster_centers) {
    // 【1】最初のフレームでは、ロボットの前方（x > 0）の最も近いクラスタを選ぶ
    clearLastSelectionInfo();
    if (cluster_centers.empty() && is_target_initialized_) {
        publishCmdVel(0.0, 0.0);  // 停止指令を送信

        setFollowTrackingState(FollowTrackingState::TemporarilyLost);
        startLostDebugTimer();
        // 一時ロスト中は前回ターゲット情報を保持し、予測方向への移動は行わない。
        if (targetLostTimedOut()) {
            handleTargetLostTimeout();
            return;
        }
        publishLostState(false);

        saveDebugCsv();
        return;
    }
    geometry_msgs::msg::Point target_pos;
    if (!is_target_initialized_) {
        int target_id = this->initializeTarget(cluster_centers, target_pos);
        if (target_id == -1) {
            handleInitialTargetNotFound();
            return;
        }

        is_target_initialized_ = true;
        previous_target_id_ = target_id;
        previous_second_id_ = -1;
        previous_target_pos_ = target_pos;
        current_target_id_ = target_id;
        current_second_id_ = -1;
        setFollowTrackingState(FollowTrackingState::Tracking);
        clearLostDebugTimer();
        // RCLCPP_INFO(this->get_logger(), "初期追従対象ID: %d", target_id);
        return;
    }

    // 【2】前回の追従対象がまだ存在しているか確認
    double movement = std::numeric_limits<double>::max();  // 移動距離の初期値
    bool previous_target_found = this->verifyPreviousTarget(cluster_centers, target_id, target_pos, movement);

    // 【3】前回の追従対象が見つからなかった場合 or 移動が大きすぎる場合、新しい対象を探す
    if (!previous_target_found || movement > MOVEMENT_THRESHOLD) {
        setFollowTrackingState(FollowTrackingState::TemporarilyLost);
        startLostDebugTimer();
        publishCmdVel(0.0, 0.0);
        if (targetLostTimedOut()) {
            handleTargetLostTimeout();
            return;
        }
        publishLostState(false);
        RCLCPP_INFO(this->get_logger(), "前回の追従対象をロスト, movement: %lf", movement);

        if (previous_target_found && movement > MOVEMENT_THRESHOLD) {
            const double previous_angle = std::atan2(previous_target_pos_.y, previous_target_pos_.x);
            const double target_angle = std::atan2(target_pos.y, target_pos.x);
            const double angle_diff = std::fabs(std::atan2(
                std::sin(target_angle - previous_angle),
                std::cos(target_angle - previous_angle)));

            RCLCPP_WARN(this->get_logger(),
                        "前回ターゲットID %d は移動量が大きすぎるため一時ロストとして保持 (移動距離: %.3f)",
                        target_id, movement);
            last_selection_called_ = true;
            last_selection_target_id_ = target_id;
            last_selection_reason_ = "movement_threshold";
            last_selection_movement_ = movement;
            last_selection_distance_to_robot_ = std::hypot(target_pos.x, target_pos.y);
            last_selection_angle_diff_ = angle_diff;
            last_selection_timestamp_ = this->now().seconds();
            last_rejection_reason_ = "movement_threshold";
            has_rejected_candidate_ = true;
            rejected_candidate_pos_ = target_pos;
            saveDebugCsv();
            return;
        }

        if (auto new_target = selectReacquisitionTarget(cluster_centers)) {
            target_id = new_target->first;
            target_pos = new_target->second;
            if (is_target_initialized_) {
                double previous_angle = std::atan2(previous_target_pos_.y, previous_target_pos_.x);
                double new_angle = std::atan2(target_pos.y, target_pos.x);
                double angle_diff = std::atan2(std::sin(new_angle - previous_angle), std::cos(new_angle - previous_angle));
                angle_diff = std::fabs(angle_diff);
                double distance_diff = calculateDistance(previous_target_pos_, target_pos);
                last_selection_angle_diff_ = angle_diff;

                if (distance_diff > LOST_DISTANCE_JUMP && angle_diff > LOST_ANGLE_JUMP) {
                    RCLCPP_WARN(this->get_logger(),
                                "selectReacquisitionTarget: 新しい候補(ID %d)が前回位置と大きく異なるため停止 (距離: %.3f, 角度差: %.2f)",
                                target_id, distance_diff, angle_diff);

                    publishCmdVel(0.0, 0.0);
                    publishLostState(true);
                    current_target_id_ = -1;
                    current_second_id_ = -1;
                    setFollowTrackingState(FollowTrackingState::Lost);

                    last_selection_called_ = true;
                    last_selection_target_id_ = -1;
                    last_selection_reason_ = "jump_detected";
                    last_selection_movement_ = distance_diff;
                    last_selection_distance_to_robot_ = std::hypot(target_pos.x, target_pos.y);
                    last_selection_angle_diff_ = angle_diff;
                    last_rejection_reason_ = "jump_detected";
                    last_selection_timestamp_ = this->now().seconds();
                    has_rejected_candidate_ = true;
                    rejected_candidate_pos_ = target_pos;

                    saveDebugCsv();
                    publishDebugMarkers();
                    start_followme_flag = false;
                    return;
                }
            }
            setFollowTrackingState(FollowTrackingState::Tracking);
            RCLCPP_INFO(this->get_logger(), "新しい追従対象 (ID: %d) を選択", target_id);
        } else {
            RCLCPP_WARN(this->get_logger(), "一時ロスト中: 適切な再捕捉候補が見つかりませんでした。");
            last_rejection_reason_ = "not_found";
            saveDebugCsv();
            return;
        }
    } else {
        setFollowTrackingState(FollowTrackingState::Tracking);
    }

    // 【4】近くにもう1つのクラスタがあるかチェック
    // TODO: 前回のクラスタIDのものがあれば、それに追従する
    int second_id = -1;
    auto second_target = findSecondaryCluster(cluster_centers, target_id, target_pos);
    if (second_target.has_value()) {
        target_pos.x = (target_pos.x + second_target->second.x) / 2.0;
        target_pos.y = (target_pos.y + second_target->second.y) / 2.0;
        second_id = second_target->first;
        // RCLCPP_INFO(this->get_logger(), "2つのクラスタを選択: ID %d と ID %d", target_id, second_id);
    } else {
        // RCLCPP_INFO(this->get_logger(), "単独クラスタを追従: ID %d", target_id);
    }

    // 【5】追従対象を更新
    updateTrackingState(target_id, second_id, target_pos);
    publishPersonMarker(target_pos);

    // 【6】目標地点への移動指令
    double angle_to_target = atan2(target_pos.y, target_pos.x);
    double distance_to_target = sqrt(target_pos.x * target_pos.x + target_pos.y * target_pos.y);

    // RCLCPP_INFO(this->get_logger(), "追従目標位置: (%.2f, %.2f)", target_pos.x, target_pos.y);
    setFollowTrackingState(FollowTrackingState::Tracking);
    clearLostDebugTimer();
    publishCmdVel(distance_to_target, angle_to_target);
    publishLostState(false);

    saveDebugCsv();
}

void LegClusterTracking::resetFollowTarget() {
    is_ready_for_tracking = false;
    is_target_initialized_ = false;
    cluster_velocities_.clear();
    cluster_id_mapping_.clear();
    previous_cluster_info_map_.clear();
    cluster_info_map_.clear();
    lost_clusters_.clear();
    lost_cluster_velocities_.clear();
    cluster_id_history_.clear();
    cluster_velocity_history_.clear();
    cluster_static_frame_count_.clear();
    next_cluster_id_ = 1;
    integral_dist = 0.0;
    integral_angle = 0.0;
    target_id = -1;
    previous_target_id_ = -1;
    previous_second_id_ = -1;
    current_target_id_ = -1;
    current_second_id_ = -1;
    previous_target_pos_ =  geometry_msgs::msg::Point();
    setFollowTrackingState(FollowTrackingState::Idle);
    clearLostDebugTimer();
    clearLastSelectionInfo();
}

void LegClusterTracking::clearLastSelectionInfo() {
    last_selection_called_ = false;
    last_selection_target_id_ = -1;
    last_selection_reason_ = "none";
    last_selection_movement_ = -1.0;
    last_selection_distance_to_robot_ = -1.0;
    last_selection_angle_diff_ = -1.0;
    last_selection_timestamp_ = -1.0;
    last_rejection_reason_ = "none";
    has_rejected_candidate_ = false;
    rejected_candidate_pos_ = geometry_msgs::msg::Point();
}

void LegClusterTracking::setFollowTrackingState(FollowTrackingState state) {
    follow_tracking_state_ = state;
    debug_tracking_state_ = followTrackingStateName(state);
}

const char *LegClusterTracking::followTrackingStateName(FollowTrackingState state) const {
    switch (state) {
        case FollowTrackingState::Idle:
            return "idle";
        case FollowTrackingState::WaitingInitial:
            return "waiting_initial";
        case FollowTrackingState::Tracking:
            return "tracking";
        case FollowTrackingState::TemporarilyLost:
            return "temporarily_lost";
        case FollowTrackingState::Lost:
            return "lost";
        case FollowTrackingState::Stopped:
            return "stopped";
        case FollowTrackingState::EmergencyStop:
            return "emergency_stop";
    }
    return "unknown";
}

void LegClusterTracking::startLostDebugTimer() {
    if (!target_lost_timer_active_) {
        target_lost_start_time_ = this->now();
        target_lost_timer_active_ = true;
    }
}

void LegClusterTracking::clearLostDebugTimer() {
    target_lost_timer_active_ = false;
    target_lost_start_time_ = this->now();
}

double LegClusterTracking::lostElapsedSeconds() const {
    if (!target_lost_timer_active_) {
        return -1.0;
    }
    return (this->now() - target_lost_start_time_).seconds();
}

bool LegClusterTracking::targetLostTimedOut() const {
    const auto elapsed = lostElapsedSeconds();
    return elapsed >= TARGET_LOST_TIMEOUT;
}

geometry_msgs::msg::Point LegClusterTracking::predictedTargetPosition() const {
    auto predicted = previous_target_pos_;
    if (!is_target_initialized_) {
        return predicted;
    }

    const int velocity_id = previous_target_id_ != -1 ? previous_target_id_ : previous_second_id_;
    const auto velocity_it = cluster_info_map_.find(velocity_id);
    if (velocity_it == cluster_info_map_.end()) {
        return predicted;
    }

    const auto elapsed = previous_time_.nanoseconds() == 0 ?
        0.0 : (this->now() - previous_time_).seconds();
    predicted.x += velocity_it->second.velocity.x * elapsed * PREDICTED_VEL_GAIN;
    predicted.y += velocity_it->second.velocity.y * elapsed * PREDICTED_VEL_GAIN;
    return predicted;
}

void LegClusterTracking::saveDebugCsv() {
    csv_logger_->saveClusterData(
        cluster_id_history_,
        cluster_info_map_,
        current_target_id_,
        current_second_id_,
        last_selection_called_,
        last_selection_target_id_,
        last_selection_reason_,
        last_selection_movement_,
        last_selection_distance_to_robot_,
        last_selection_timestamp_,
        debug_tracking_state_,
        lostElapsedSeconds(),
        previous_target_pos_,
        predictedTargetPosition(),
        last_selection_angle_diff_,
        last_rejection_reason_);
}

void LegClusterTracking::publishCmdVel(double target_distance, double target_angle) {
    auto cmd_msg = geometry_msgs::msg::Twist();

    // ジョイスティックで停止が指示されたら何もしない
    if (stop_by_joystick_) {
        RCLCPP_WARN(this->get_logger(), "ジョイスティックの入力により移動を停止中...");
        cmd_vel_publisher_->publish(cmd_msg);
        return;
    }
    // 31cm以内なら停止
    if (target_distance <= STOP_DISTANCE_THRESHOLD) {
        // RCLCPP_INFO(this->get_logger(), "追従対象に到達！ 停止します。");
        cmd_vel_publisher_->publish(cmd_msg); // 速度0を送信
        return;
    }

    // 現在の誤差を計算
    double error_dist = target_distance - STOP_DISTANCE_THRESHOLD;
    double error_angle = target_angle; // radian
    // 誤差の積分項を更新（上限制限付き）
    integral_dist += error_dist;
    integral_dist = std::clamp(integral_dist, -MAX_DIST_INTEGRAL, MAX_DIST_INTEGRAL);
    integral_angle += error_angle;
    integral_angle = std::clamp(integral_angle, -MAX_ANGLE_INTEGRAL, MAX_ANGLE_INTEGRAL);
    // RCLCPP_INFO(this->get_logger(), "誤差距離: %.2f, 誤差角度: %.2f", integral_dist, integral_angle);
    // PID計算
    double linear_velocity = (KP_DIST * error_dist) + (KI_DIST * integral_dist);
    double angular_velocity = (KP_ANGLE * error_angle) + (KI_ANGLE * integral_angle);
    cmd_msg.linear.x = std::clamp(linear_velocity, MIN_SPEED, MAX_SPEED);
    cmd_msg.angular.z = std::clamp(angular_velocity, -MAX_TURN_SPEED, MAX_TURN_SPEED);
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "直進速度: %.2f, 回転速度: %.2f", linear_velocity, angular_velocity);
    // if(abs(error_angle) > M_PI/4) { // BLDC
    if(abs(error_angle) > M_PI/4) { // icart
        cmd_msg.linear.x = 0.0; //　対象との角度が大きい場合は旋回を優先する
        cmd_msg.angular.z = (cmd_msg.angular.z > 0) ? MAX_TURN_SPEED : -MAX_TURN_SPEED;
    }

    // 速度をパブリッシュ
    cmd_vel_publisher_->publish(cmd_msg);
}

void LegClusterTracking::publishClusterMarkers(
    const std::vector<geometry_msgs::msg::Point> &points, 
    const std::vector<int> &clusters) {

    visualization_msgs::msg::MarkerArray marker_array;
    for (size_t i = 0; i < points.size(); i++) {
        if (points[i].x != 0.0 || points[i].y != 0.0) {
            if (cluster_id_mapping_.count(clusters[i]) > 0) {
                    int mapped_cluster_id = cluster_id_mapping_[clusters[i]];
                    visualization_msgs::msg::Marker marker = marker_helper_->createMarker(
                        "cluster_markers", i, visualization_msgs::msg::Marker::CUBE,
                        points[i], 0.01, 0.01, 0.01, mapped_cluster_id);
                        marker_array.markers.push_back(marker);
                    }
            }
        }
        cluster_marker_publisher_->publish(marker_array);
}

void LegClusterTracking::publishMatchedClusterCenters(const std::map<int, geometry_msgs::msg::Point> &current_centers) {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker center_marker = marker_helper_->createMarker(
        "matched_cluster_centers", 1, visualization_msgs::msg::Marker::SPHERE_LIST,
        geometry_msgs::msg::Point(), 0.05, 0.05, 0.05);
    center_marker.color.a = 1.0;

    int marker_id = 100;
    for (const auto &[current_id, current_center] : current_centers) {
        center_marker.points.push_back(current_center);
        center_marker.colors.push_back(marker_helper_->color_palette_[current_id % marker_helper_->color_palette_.size()]);

        visualization_msgs::msg::Marker text_marker = marker_helper_->createMarker(
            "cluster_id_labels", marker_id++, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
            current_center, 0.05, 0.05, 0.05, -1, 1.0, 1.0, 1.0, 1.0);
        text_marker.pose.position.z += 0.1;
        text_marker.text = std::to_string(current_id);
        marker_array.markers.push_back(text_marker);
    }
    marker_array.markers.push_back(center_marker);
    center_marker_publisher_->publish(marker_array);
}

void LegClusterTracking::publishPersonMarker(const geometry_msgs::msg::Point &target_pos) {
    visualization_msgs::msg::MarkerArray markers;
    std::string frame_id = "laser";
    std::string ns = "person_marker";
    int marker_id = 999;

    // BLDC
    // visualization_msgs::msg::Marker body_marker = marker_helper_->createMarker(
    //     ns, marker_id++, visualization_msgs::msg::Marker::CYLINDER,
    //     target_pos, 0.1, 0.1, -0.3, -1, 0.0, 1.0, 0.0, 0.5);
    // body_marker.pose.position.z -= 0.15;
    // icart
    visualization_msgs::msg::Marker body_marker = marker_helper_->createMarker(
        ns, marker_id++, visualization_msgs::msg::Marker::CYLINDER,
        target_pos, 0.1, 0.1, 0.3, -1, 0.0, 1.0, 0.0, 0.5);
    body_marker.pose.position.z += 0.15;
    markers.markers.push_back(body_marker);

    // BLDC
    // visualization_msgs::msg::Marker head_marker = marker_helper_->createMarker(
    //     ns, marker_id++, visualization_msgs::msg::Marker::SPHERE,
    //     target_pos, 0.1, 0.1, -0.1, -1, 0.0, 1.0, 0.0, 0.5);
    // head_marker.pose.position.z -= 0.35;
    // icart
    visualization_msgs::msg::Marker head_marker = marker_helper_->createMarker(
        ns, marker_id++, visualization_msgs::msg::Marker::SPHERE,
        target_pos, 0.1, 0.1, 0.1, -1, 0.0, 1.0, 0.0, 0.5);
    head_marker.pose.position.z += 0.35;
    markers.markers.push_back(head_marker);

    person_marker_publisher_->publish(markers);
}

void LegClusterTracking::publishDebugMarkers() {
    if (!debug_marker_publisher_) {
        return;
    }

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "laser";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    // RVizには最新のTFで表示してほしいため、debug markerはstamp=0のままにする。
    // シミュレーション時刻とwall時刻が混在すると、now()を入れたmarkerだけ
    // 「その時刻の laser->odom TF がない」エラーになりやすい。
    if (is_target_initialized_) {
        auto current_target_marker = marker_helper_->createMarker(
            "debug_current_target", 1, visualization_msgs::msg::Marker::SPHERE,
            previous_target_pos_, 0.16, 0.16, 0.16, -1, 0.0, 1.0, 0.0, 0.8);
        markers.markers.push_back(current_target_marker);

        const auto predicted_target_pos = predictedTargetPosition();
        auto predicted_target_marker = marker_helper_->createMarker(
            "debug_predicted_target", 2, visualization_msgs::msg::Marker::SPHERE,
            predicted_target_pos, 0.12, 0.12, 0.12, -1, 0.1, 0.4, 1.0, 0.8);
        markers.markers.push_back(predicted_target_marker);

        // 再捕捉ゲートは「この範囲外なら同一人物として扱わない」目安をRVizで見るために出す。
        auto reacquire_gate_marker = marker_helper_->createMarker(
            "debug_reacquire_gate", 3, visualization_msgs::msg::Marker::CYLINDER,
            predicted_target_pos, MOVEMENT_THRESHOLD * 2.0, MOVEMENT_THRESHOLD * 2.0,
            0.01, -1, 0.1, 0.4, 1.0, 0.18);
        reacquire_gate_marker.pose.position.z = 0.005;
        markers.markers.push_back(reacquire_gate_marker);
    }

    auto state_text_pos = previous_target_pos_;
    state_text_pos.z += 0.45;
    auto state_text_marker = marker_helper_->createMarker(
        "debug_tracking_state", 4, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
        state_text_pos, 0.0, 0.0, 0.16, -1, 1.0, 1.0, 1.0, 0.9);
    state_text_marker.text = debug_tracking_state_;
    markers.markers.push_back(state_text_marker);

    if (has_rejected_candidate_) {
        auto rejected_candidate_marker = marker_helper_->createMarker(
            "debug_rejected_candidate", 10, visualization_msgs::msg::Marker::CUBE,
            rejected_candidate_pos_, 0.16, 0.16, 0.16, -1, 1.0, 0.0, 0.0, 0.85);
        markers.markers.push_back(rejected_candidate_marker);

        auto rejected_text_pos = rejected_candidate_pos_;
        rejected_text_pos.z += 0.28;
        auto rejected_text_marker = marker_helper_->createMarker(
            "debug_rejected_candidate_text", 11, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
            rejected_text_pos, 0.0, 0.0, 0.13, -1, 1.0, 0.2, 0.2, 0.9);
        rejected_text_marker.text = "reject: " + last_rejection_reason_;
        markers.markers.push_back(rejected_text_marker);
    }

    debug_marker_publisher_->publish(markers);
}

void LegClusterTracking::publishLostState(bool lost) {
    if (!is_lost_target_publisher_) {
        return;
    }
    std_msgs::msg::Bool msg;
    msg.data = lost;
    is_lost_target_publisher_->publish(msg);
}

void LegClusterTracking::publishClusterInfoMap() {
    icart_msg::ClusterInfoArray msg;
    for (const auto &[id, info] : cluster_info_map_) {
        msg.clusters.push_back(info); 
    }
    cluster_info_publisher_->publish(msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegClusterTracking>());
    rclcpp::shutdown();
    return 0;
}
