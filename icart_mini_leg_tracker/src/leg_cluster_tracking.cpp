#include "icart_mini_leg_tracker/leg_cluster_tracking.hpp"

LegClusterTracking::LegClusterTracking() : 
    Node("leg_cluster_tracking_node"),
    next_cluster_id_(1), 
    is_ready_for_tracking(false), 
    start_followme_flag(false),
    is_target_initialized_(false), 
    stop_by_joystick_(false),
    marker_helper_(std::make_shared<MarkerHelper>(1000)), 
    csv_logger_(std::make_shared<CSVLogger>(FILENAME))
    {

    lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(10).best_effort(), 
        std::bind(&LegClusterTracking::scanCallback, this, std::placeholders::_1)
    );
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", rclcpp::QoS(10).best_effort(), std::bind(&LegClusterTracking::joyCallback, this, std::placeholders::_1)
    );

    cluster_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/leg_tracker/cluster_markers", 10);
    center_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/leg_tracker/cluster_centers", 10);
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    person_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/leg_tracker/person_marker", 10);
    cluster_info_publisher_ = this->create_publisher<icart_msg::ClusterInfoArray>("/leg_tracker/cluster_infos", 10);
    is_lost_target_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/leg_tracker/is_lost_target", 10);

    RCLCPP_INFO(this->get_logger(), "Leg cluster and tracking started.");
}

void LegClusterTracking::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->buttons[EMERGENCY_BUTTON] == 1) {
        RCLCPP_WARN(this->get_logger(), "非常停止");
        stop_by_joystick_ = true;
        publishCmdVel(0.0, 0.0);
    }
    else if (msg->buttons[UNLOCK_EMERGENCY_BUTTON] == 1) {
        RCLCPP_WARN(this->get_logger(), "非常停止解除");
        stop_by_joystick_ = false;
    }
    else if (msg->buttons[FOLLOWME_START_BUTTON] == 1) {
        resetFollowTarget();
        start_followme_flag = true;
        RCLCPP_WARN(this->get_logger(), "追従開始");
    }
    else if (msg->buttons[FOLLOWME_STOP_BUTTON] == 1) {
        start_followme_flag = false;
        resetFollowTarget();
        RCLCPP_WARN(this->get_logger(), "追従停止");
    }
}

void LegClusterTracking::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (start_followme_flag == false) return;
    if (stop_by_joystick_ == true) return;
    auto start_time = this->get_clock()->now();
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
        std::cout << "========================================\n\n";
        publishClusterMarkers(points, clusters);
        publishMatchedClusterCenters(cluster_centers);
        publishClusterInfoMap();
    }
    auto end_time = this->get_clock()->now();
    auto duration = (end_time - start_time).seconds();  // 経過時間（秒）
    // RCLCPP_INFO(this->get_logger(), "scanCallback 実行時間: %.6f 秒", duration);
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
    std::vector<int> clusters(points.size(), 0);
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
        // 正面0.6m以内かつ左右0.2m以内のクラスタのみ採用
        if (!(center.x > 0 && center.x < 1.0 && fabs(center.y) < 0.4)) {
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

void LegClusterTracking::matchLostClusters(std::map<int, geometry_msgs::msg::Point> &current_centers, std::map<int, int> &temp_cluster_mapping_) {
    for (auto &[current_id, current_center] : current_centers) {
        double min_distance = std::numeric_limits<double>::max();
        int matched_id = -1;

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
            }
            ++it;
        }

        if (matched_id != -1) {
            temp_cluster_mapping_[current_id] = matched_id;
            lost_clusters_.erase(matched_id);
            lost_cluster_velocities_.erase(matched_id);
            // std::cout << "失われたクラスタID " << matched_id << " が復活" << std::endl;
        }
    }
}

// 前回のクラスタとマッチングを試みる関数
void LegClusterTracking::matchPreviousClusters(std::map<int, geometry_msgs::msg::Point> &current_centers, std::map<int, int> &temp_cluster_mapping_, std::map<int, bool> &matched_previous) {
    for (auto &[current_id, current_center] : current_centers) {
        if (cluster_id_mapping_.count(current_id) > 0) continue;  // すでにマッチ済み

        double min_distance = std::numeric_limits<double>::max();
        int matched_id = -1;

        for (const auto &[prev_id, prev_info] : previous_cluster_info_map_) {
            geometry_msgs::msg::Point predicted_center = prev_info.center;
            if (cluster_info_map_.count(prev_id) > 0) {
                const auto &velocity = cluster_info_map_[prev_id].velocity;
                double delta_time = (this->get_clock()->now() - previous_time_).seconds();
                predicted_center.x += velocity.x * delta_time * PREDICTED_VEL_GAIN;
                predicted_center.y += velocity.y * delta_time * PREDICTED_VEL_GAIN;
                // RCLCPP_INFO(this->get_logger(), "クラスタID: %d | 差分xy: (%.2f, %.2f), 予測位置: (%.2f, %.2f)", prev_id, velocity.x*delta_time*PREDICTED_VEL_GAIN, velocity.y*delta_time*PREDICTED_VEL_GAIN, predicted_center.x, predicted_center.y);
            }
            double dist = calculateDistance(current_center, predicted_center);
            
            if (dist < CLUSTER_MATCHED_THRESH && dist < min_distance) {
                min_distance = dist;
                matched_id = prev_id;
            }
        }
        RCLCPP_INFO(this->get_logger(), "現在のクラスタID: %d | 前回のクラスタID: %d | 距離: %.3f", current_id, matched_id, min_distance);

        // ここでロストクラスタと前回のクラスタの両方を比較し、最適なものを採用
        if (temp_cluster_mapping_.count(current_id) > 0) {
            int lost_matched_id = temp_cluster_mapping_[current_id];
            double lost_dist = calculateDistance(current_center, lost_clusters_[lost_matched_id].first);
            if (matched_id == -1 || lost_dist < min_distance) {
                matched_id = lost_matched_id;  // ロストクラスタのIDを採用
                RCLCPP_INFO(this->get_logger(), "ロストクラスタID: %d | 距離: %.2f が優先されました", matched_id, lost_dist);
            }
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
    matchLostClusters(current_centers, temp_cluster_mapping_);

    // 【2】前回のクラスタと比較して最も近いクラスタを探す, ロストクラスタのほうが近い場合はそちらを採用
    matchPreviousClusters(current_centers, temp_cluster_mapping_, matched_previous);

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
    for (const auto &[current_id, current_center] : cluster_centers) {
        double dist = sqrt(current_center.x * current_center.x + current_center.y * current_center.y);
        if (current_center.x > 0 && dist < min_distance) {  // 前方 (x > 0) のクラスタを選ぶ
            min_distance = dist;
            target_id = current_id;
            target_pos = current_center;
        }
    }
    return target_id;
}

// 追従対象がまだ存在しているか確認する関数
bool LegClusterTracking::verifyPreviousTarget(const std::map<int, geometry_msgs::msg::Point> &cluster_centers, int &target_id, geometry_msgs::msg::Point &target_pos, double &movement) {
    if (previous_target_id_ != -1 && cluster_centers.count(previous_target_id_)) {
        geometry_msgs::msg::Point prev_pos = cluster_centers.at(previous_target_id_);
        target_id = previous_target_id_;
        target_pos = prev_pos;
        movement = sqrt(pow(target_pos.x - prev_pos.x, 2) + pow(target_pos.y - prev_pos.y, 2));
        // RCLCPP_INFO(this->get_logger(), "前回の追従対象1 (ID: %d) を継続 [移動距離: %.3f]", target_id, movement);
        return true;
    } else if (previous_second_id_ != -1 && cluster_centers.count(previous_second_id_)) {
        target_pos = cluster_centers.at(previous_second_id_);
        target_id = previous_second_id_;
        geometry_msgs::msg::Point prev_pos = cluster_centers.at(target_id);
        movement = sqrt(pow(prev_pos.x - target_pos.x, 2) + pow(prev_pos.y - target_pos.y, 2));
        // RCLCPP_INFO(this->get_logger(), "前回の追従対象2 (ID: %d) を継続 [移動距離: %.3f]", target_id, movement);
        return true;
    }
    // RCLCPP_INFO(this->get_logger(), "前回の追従対象（ID: %d）をロスト", target_id);
    movement = 0.0;
    return false;
}

std::optional<std::pair<int, geometry_msgs::msg::Point>> 
LegClusterTracking::selectNewTarget(const std::map<int, geometry_msgs::msg::Point> &cluster_centers, 
                                    bool previous_target_found) { // 前回の追従対象が見つからなかったが、前回の追従対象の位置はわかる
    double min_movement = std::numeric_limits<double>::max();  // 前回の対象との移動距離
    double min_distance = std::numeric_limits<double>::max();  // ロボットとの距離
    int new_target_id = -1;
    geometry_msgs::msg::Point new_target_pos;

    for (const auto &[current_id, current_center] : cluster_centers) {
        double dist = sqrt(current_center.x * current_center.x + current_center.y * current_center.y);  // ロボットとの距離
        double movement_from_prev = std::numeric_limits<double>::max();  // 初期化
        
        // 前回の対象が見つかっている場合、移動距離を計算
        // TODO: 前回の対象が見つからなかった場合は、前回の対象の位置をもとにもっともらしい対象を選ぶ
        movement_from_prev = sqrt(pow(current_center.x - previous_target_pos_.x, 2) + 
                                  pow(current_center.y - previous_target_pos_.y, 2));
        RCLCPP_INFO(this->get_logger(), "計算対象のクラスタID: %d, 中心座標: (%.2f, %.2f), 前回の追従対象の座標: (%.2f, %.2f)", current_id, current_center.x, current_center.y, previous_target_pos_.x, previous_target_pos_.y);
        RCLCPP_INFO(this->get_logger(), "前回の対象との移動距離: %.3f, 計算対象のid: %d, 追従対象のid: %d", movement_from_prev, current_id, target_id);

        // 【優先度】 (1) できるだけ前回の対象に近い → (2) ロボットから近い
        // 前回の対象が見つからなかった場合、または、移動距離がしきい値以下の場合
        if (!previous_target_found || movement_from_prev < MOVEMENT_THRESHOLD) {  
            // より前回の位置に近いクラスタを記録
            if (movement_from_prev < min_movement) {
                min_movement = movement_from_prev;
                new_target_id = current_id;
                new_target_pos = current_center;
            }
        // 前回の対象が遠く離れていた場合、ロボットに近いものを代わりに選ぶ
        } else if (new_target_id == -1 || dist < min_distance) {
            min_distance = dist;
            new_target_id = current_id;
            new_target_pos = current_center;
        }
    }

    if (new_target_id == -1) {
        return std::nullopt;  // 追従対象なし
    }

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

void LegClusterTracking::followTarget(const std::map<int, geometry_msgs::msg::Point> &cluster_centers) {
    // 【1】最初のフレームでは、ロボットの前方（x > 0）の最も近いクラスタを選ぶ
    std_msgs::msg::Bool is_lost_target;
    is_lost_target.data = true;
    if (cluster_centers.empty()) {
        publishCmdVel(0.0, 0.0);  // 停止指令を送信
        is_lost_target_publisher_->publish(is_lost_target);
        return;
    }
    geometry_msgs::msg::Point target_pos;
    if (!is_target_initialized_) {
        int target_id = this->initializeTarget(cluster_centers, target_pos);
        if (target_id == -1) {
            RCLCPP_WARN(this->get_logger(), "初期追従対象が見つかりませんでした。");
            publishCmdVel(0.0, 0.0);
            is_lost_target_publisher_->publish(is_lost_target);
            return;
        }

        is_target_initialized_ = true;
        previous_target_id_ = target_id;
        previous_second_id_ = -1;
        // RCLCPP_INFO(this->get_logger(), "初期追従対象ID: %d", target_id);
        return;
    }

    // 【2】前回の追従対象がまだ存在しているか確認
    double movement = std::numeric_limits<double>::max();  // 移動距離の初期値
    bool previous_target_found = this->verifyPreviousTarget(cluster_centers, target_id, target_pos, movement);

    // 【3】前回の追従対象が見つからなかった場合 or 移動が大きすぎる場合、新しい対象を探す
    if (!previous_target_found || movement > MOVEMENT_THRESHOLD) {
        RCLCPP_INFO(this->get_logger(), "前回の追従対象をロスト, movement: %lf", movement);
        if (auto new_target = selectNewTarget(cluster_centers, previous_target_found)) {
            target_id = new_target->first;
            target_pos = new_target->second;
            RCLCPP_INFO(this->get_logger(), "新しい追従対象 (ID: %d) を選択", target_id);
        } else {
            RCLCPP_WARN(this->get_logger(), "適切な追従対象が見つかりませんでした。");
            publishCmdVel(0.0, 0.0);
            is_lost_target_publisher_->publish(is_lost_target);
            this->current_target_id_ = -1;
            this->current_second_id_ = -1;
            csv_logger_->saveClusterData(cluster_id_history_, cluster_info_map_, current_target_id_, current_second_id_);
            return;
        }
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
    publishCmdVel(distance_to_target, angle_to_target);
    is_lost_target.data = false;
    is_lost_target_publisher_->publish(is_lost_target);

    csv_logger_->saveClusterData(cluster_id_history_, cluster_info_map_, current_target_id_, current_second_id_);
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
    previous_target_pos_ =  geometry_msgs::msg::Point();
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
    RCLCPP_INFO(this->get_logger(), "直進速度: %.2f, 回転速度: %.2f", linear_velocity, angular_velocity);
    // if(abs(error_angle) > M_PI/4) { // BLDC
    if(abs(error_angle) > M_PI/3) { // icart
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