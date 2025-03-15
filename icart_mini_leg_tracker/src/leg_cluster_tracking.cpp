#include "icart_mini_leg_tracker/leg_cluster_tracking.hpp"

void resetCSVFile() {
    std::ofstream csv_file(FILENAME, std::ios::trunc); // ファイルをリセット（上書き）

    if (!csv_file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("CSV_Manager"), "CSVファイルを開けませんでした。");
        return;
    }

    // ヘッダーを書き込む
    csv_file << "追従対象ID1,追従対象ID2,クラスタID,履歴,速度X,速度Y,位置X, 位置Y" << std::endl;

    csv_file.close();
    RCLCPP_INFO(rclcpp::get_logger("CSV_Manager"), "CSVファイルをリセットしました。");
}

LegClusterTracking::LegClusterTracking() : Node("leg_cluster_tracking_node"),
    next_cluster_id_(1), is_ready_for_tracking(false), is_target_initialized_(false), stop_by_joystick_(false) {
    resetCSVFile();

    lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(10).best_effort(), 
        std::bind(&LegClusterTracking::scanCallback, this, std::placeholders::_1)
    );
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", rclcpp::QoS(10).best_effort(), std::bind(&LegClusterTracking::joyCallback, this, std::placeholders::_1)
    );

    cluster_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/leg_tracker/cluster_markers", 10);
    center_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/leg_tracker/cluster_centers", 10);
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    target_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/leg_tracker/target_marker", 10);

    color_palette_ = generateColors(1000);  // 最大100クラスタ用のカラーを事前生成

    RCLCPP_INFO(this->get_logger(), "Leg cluster and tracking started.");
}

void LegClusterTracking::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    int emergency_button = 4;
    if (msg->buttons[emergency_button] == 1) {
        RCLCPP_WARN(this->get_logger(), "ジョイスティック入力でロボットを停止します！");
        stop_by_joystick_ = true;
        publishCmdVel(0.0, 0.0);
    } else if (msg->buttons[emergency_button] == 0) {
        stop_by_joystick_ = false;
    }
}

void LegClusterTracking::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
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

    // PCLのPointCloudに変換
    for (const auto &point : points) {
        double distance = sqrt(point.x * point.x + point.y * point.y);
        if (distance > MAX_CLUSTER_DISTANCE) continue; // 遠すぎる点はスキップ]
        
        pcl::PointXYZ pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = 0.0;
        cloud->points.push_back(pcl_point);
    }
    // KD-Treeを作成
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // クラスタリング実行
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(CLUSTER_TOLERANCE);  // 5cm以内の点を同じクラスタにする
    ec.setMinClusterSize(MIN_CLUSTER_SIZE);      // 最小クラスタサイズ
    ec.setMaxClusterSize(MAX_CLUSTER_SIZE);     // 最大クラスタサイズ
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    std::vector<int> clusters(points.size(), 0);
    int cluster_id = 1;
    for (const auto &indices : cluster_indices) {
        for (int idx : indices.indices) {
            clusters[idx] = cluster_id;
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

void LegClusterTracking::calculateClusterVelocities(
    const std::map<int, geometry_msgs::msg::Point> &current_centers, 
    const rclcpp::Time &current_time) 
{
    if (previous_cluster_centers_.empty() || previous_time_.nanoseconds() == 0) {
        previous_time_ = current_time;
        return;  // 最初のフレームは速度計算をスキップ
    }

    double delta_time = (current_time - previous_time_).seconds();  // 経過時間[s]
    if (delta_time <= 0) return;  // 経過時間が0以下の場合は計算しない

    cluster_velocities_.clear();
    for (const auto &[id, current_center] : current_centers) {
        if (previous_cluster_centers_.count(id) > 0) {
            const auto &prev_center = previous_cluster_centers_[id];
            geometry_msgs::msg::Vector3 velocity;
            velocity.x = (current_center.x - prev_center.x) / delta_time;
            velocity.y = (current_center.y - prev_center.y) / delta_time;
            velocity.z = 0.0;  // 2Dなのでzは0

            // 速度履歴を保存
            if (cluster_velocity_history_.count(id) > 0) {
                cluster_velocity_history_[id].push_back(velocity);
                if (cluster_velocity_history_[id].size() > 5) {  // 最大5フレーム分保持
                    cluster_velocity_history_[id].erase(cluster_velocity_history_[id].begin());
                }
            } else {
                cluster_velocity_history_[id] = {velocity};
            }

            // 平均化して異常値を抑える
            geometry_msgs::msg::Vector3 smoothed_velocity;
            smoothed_velocity.x = 0.0;
            smoothed_velocity.y = 0.0;
            smoothed_velocity.z = 0.0;
            for (const auto &v : cluster_velocity_history_[id]) {
                smoothed_velocity.x += v.x;
                smoothed_velocity.y += v.y;
            }
            int history_size = cluster_velocity_history_[id].size();
            smoothed_velocity.x /= history_size;
            smoothed_velocity.y /= history_size;

            // 速度が異常に大きい場合は制限をかける
            double speed = sqrt(smoothed_velocity.x * smoothed_velocity.x + smoothed_velocity.y * smoothed_velocity.y);
            if (speed > 1.0) {  // 最大1.0 m/s に制限
                smoothed_velocity.x *= (1.0 / speed);
                smoothed_velocity.y *= (1.0 / speed);
            }

            cluster_velocities_[id] = smoothed_velocity;
            RCLCPP_INFO(this->get_logger(), "クラスタID: %d | 速度ベクトル: (%.2f, %.2f)", id, smoothed_velocity.x, smoothed_velocity.y);
        }
    }
    previous_time_ = current_time;  // 次回のために時間を更新
}

bool LegClusterTracking::filterClustersByRegion(std::map<int, geometry_msgs::msg::Point> &cluster_centers) {
    for (auto it = cluster_centers.begin(); it != cluster_centers.end(); ) {
        const auto &center = it->second;
        // 正面0.6m以内かつ左右0.2m以内のクラスタのみ採用
        if (!(center.x > 0 && center.x < 1.0 && fabs(center.y) < 0.4)) {
            std::cout << "クラスタID " << it->first << " は有効領域外のため除外" << std::endl;
            it = cluster_centers.erase(it);  // 条件を満たさないクラスタを削除
        } else {
            ++it;
        }
    }
    if(cluster_centers.empty()) {
        std::cout << "有効なクラスタが見つかりませんでした" << std::endl;
        return false;
    }
    else{
        return true;
    }
}


void LegClusterTracking::saveClusterDataToCSV(const std::map<int, std::vector<int>>& cluster_id_history_,
                          const std::map<int, geometry_msgs::msg::Vector3>& cluster_velocities_, const std::map<int, geometry_msgs::msg::Point>& current_centers) {
    std::ofstream csv_file(FILENAME, std::ios::app);
    if (!csv_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "CSVファイルを開けませんでした。");
        return;
    }

    // 各クラスタの情報を書き込む
    for (const auto& [cluster_id, history] : cluster_id_history_) {
        csv_file << this->current_target_id_  << "," << this->current_second_id_ << "," << cluster_id << ",";
        // 履歴をカンマ区切りで記録
        for (size_t i = 0; i < history.size(); i++) {
            csv_file << history[i];
            if (i < history.size() - 1) csv_file << " ";
        }
        csv_file << ",";
        // 速度情報を書き込む
        if (cluster_velocities_.count(cluster_id)) {
            csv_file << cluster_velocities_.at(cluster_id).x << ","
                     << cluster_velocities_.at(cluster_id).y;
        } else {
            csv_file << "0,0"; // デフォルト値
        }

        if (current_centers.count(cluster_id)) {
            csv_file << "," << current_centers.at(cluster_id).x << ","
                     << current_centers.at(cluster_id).y;
        } else {
            csv_file << "0,0"; // デフォルト値
        }
        csv_file << std::endl;
    }
    csv_file << " ----------------------- " << std::endl;

    csv_file.close();
    RCLCPP_INFO(this->get_logger(), "クラスタデータをCSVに保存しました。");
}

// 前回のクラスタをもとにトラッキング
void LegClusterTracking::trackClusters(std::map<int, geometry_msgs::msg::Point> &current_centers) {
    cluster_id_mapping_.clear();
    std::map<int, bool> matched_previous;

    // 以前のクラスタを初期化
    if (previous_cluster_centers_.empty()) {
        previous_cluster_centers_ = current_centers;
        return;
    }
    
    // 前回のクラスタをまだマッチしていない状態に初期化
    for (const auto &[prev_id, prev_center] : previous_cluster_centers_) {
        matched_previous[prev_id] = false;
    }

    // 【1】まずは失われたクラスタとマッチングを試みる
    for (auto &[current_id, current_center] : current_centers) {
        double min_distance = std::numeric_limits<double>::max();
        int matched_id = -1;

        for (auto it = lost_clusters_.begin(); it != lost_clusters_.end();) {
            int lost_id = it->first;
            geometry_msgs::msg::Point lost_center = it->second.first;
            rclcpp::Time lost_time = it->second.second;

            // 一定時間経過したクラスタは破棄
            double elapsed_time = (this->get_clock()->now() - lost_time).seconds();
            if (elapsed_time > LOST_CLUSTER_TIMEOUT) {
                it = lost_clusters_.erase(it);
                lost_cluster_velocities_.erase(lost_id);
                continue;
            }

            // 失われたクラスタの予測位置を計算
            geometry_msgs::msg::Point predicted_center = lost_center;
            if (lost_cluster_velocities_.count(lost_id) > 0) {
                const auto &velocity = lost_cluster_velocities_[lost_id];
                predicted_center.x += velocity.x * elapsed_time;
                predicted_center.y += velocity.y * elapsed_time;
            }

            // 距離を比較してマッチング候補を決定
            double dist = calculateDistance(current_center, predicted_center);
            if (dist < CLUSTER_MATCHED_THRESH && dist < min_distance) {
                min_distance = dist;
                matched_id = lost_id;
            }
            ++it;
        }

        // マッチング成功ならIDを復活
        if (matched_id != -1) {
            cluster_id_mapping_[current_id] = matched_id;
            lost_clusters_.erase(matched_id);
            lost_cluster_velocities_.erase(matched_id);
            std::cout << "失われたクラスタID " << matched_id << " が復活" << std::endl;
        }
    }

    // 【2】前回のクラスタと比較して最も近いクラスタを探す
    for (auto &[current_id, current_center] : current_centers) {
        if (cluster_id_mapping_.count(current_id) > 0) continue;  // すでにマッチ済み

        double min_distance = std::numeric_limits<double>::max();
        int matched_id = -1;

        for (const auto &[prev_id, prev_center] : previous_cluster_centers_) {
            geometry_msgs::msg::Point predicted_center = prev_center;
            if (cluster_velocities_.count(prev_id) > 0) {
                const auto &velocity = cluster_velocities_[prev_id];
                double delta_time = (this->get_clock()->now() - previous_time_).seconds();
                predicted_center.x += velocity.x * delta_time * PREDICTED_VEL_GAIN;
                predicted_center.y += velocity.y * delta_time * PREDICTED_VEL_GAIN;
            }
            double dist = calculateDistance(current_center, predicted_center);
            
            if (dist < CLUSTER_MATCHED_THRESH && dist < min_distance) {
                min_distance = dist;
                matched_id = prev_id;
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

    // 【3】マッチしなかったクラスタを"失われたクラスタ"として保存
    for (const auto &[prev_id, prev_center] : previous_cluster_centers_) {
        if (!matched_previous[prev_id]) {
            lost_clusters_[prev_id] = {prev_center, this->get_clock()->now()};
            if (cluster_velocities_.count(prev_id) > 0) {
                lost_cluster_velocities_[prev_id] = cluster_velocities_[prev_id];
            }
        }
    }

    // 最終マッピング結果を出力
    std::map<int, geometry_msgs::msg::Point> updated_centers;
    // std::cout << "   前回のクラスタID   |   現在のクラスタID\n";
    // std::cout << "----------------------------------------\n";
    for (const auto &[current_id, previous_id] : cluster_id_mapping_) {
        updated_centers[previous_id] = current_centers[current_id];
        // std::cout << std::setw(10) << previous_id << "           |   "
                //   << std::setw(10) << current_id << "\n";
    }
    for (const auto &[cluster_id, history] : cluster_id_history_) {
        std::cout << "クラスタ " << cluster_id << " の履歴: ";
        for (int past_id : history) {
            std::cout << past_id << " ";
        }
        std::cout << "\n";
    }

    this->calculateClusterVelocities(updated_centers, this->get_clock()->now());
    // 最新のクラスタ中心を保存
    previous_cluster_centers_ = updated_centers;
    current_centers = updated_centers; 
}

void LegClusterTracking::followTarget(const std::map<int, geometry_msgs::msg::Point> &cluster_centers) {
    if (cluster_centers.empty()) {
        publishCmdVel(0.0, 0.0);  // 停止指令を送信
        return;
    }

    double min_distance = std::numeric_limits<double>::max();
    int target_id = -1;
    geometry_msgs::msg::Point target_pos;

    // 最初のフレームでは、ロボットの前方（x > 0）の最も近いクラスタを選ぶ
    if (!is_target_initialized_) {
        for (const auto &[current_id, current_center] : cluster_centers) {
            double dist = sqrt(current_center.x * current_center.x + current_center.y * current_center.y);
            if (current_center.x > 0 && dist < min_distance) {  // 前方 (x > 0) のクラスタを選ぶ
                min_distance = dist;
                target_id = current_id;
                target_pos = current_center;
            }
        }
        if (target_id == -1) {
            RCLCPP_WARN(this->get_logger(), "初期追従対象が見つかりませんでした。");
            publishCmdVel(0.0, 0.0);
            return;
        }

        is_target_initialized_ = true;
        previous_target_id_ = target_id;
        RCLCPP_INFO(this->get_logger(), "初期追従対象ID: %d", target_id);
        return;
    }

    // 前回の追従対象がまだ存在しているか確認
    bool previous_target_found = false;
    if (previous_target_id_ != -1 && cluster_centers.count(previous_target_id_)) {
        target_pos = cluster_centers.at(previous_target_id_);
        target_id = previous_target_id_;
        min_distance = sqrt(target_pos.x * target_pos.x + target_pos.y * target_pos.y);
        previous_target_found = true;
    }

    // 前回の追従対象が見つからない or 距離が大きく変化していたら、新しい対象を探す
    double min_movement = std::numeric_limits<double>::max();
    for (const auto &[current_id, current_center] : cluster_centers) {
        double dist = sqrt(current_center.x * current_center.x + current_center.y * current_center.y);
        // 急激な移動を防ぐため、前回の対象と大きく離れていないかチェック
        double movement = std::numeric_limits<double>::max();

        if (previous_target_found) {
            geometry_msgs::msg::Point prev_pos = cluster_centers.at(previous_target_id_);
            movement = sqrt(pow(current_center.x - prev_pos.x, 2) + pow(current_center.y - prev_pos.y, 2));
        }

        // 優先度: (1) できるだけ前回の対象に近い → (2) ロボットから近い
        if (!previous_target_found || movement < 0.5) {  // 0.5m 以上離れていたら選ばない
            if (movement < min_movement) {
                min_movement = movement;
                target_id = current_id;
                target_pos = current_center;
            }
        } else if (target_id == -1 || dist < min_distance) {
            min_distance = dist;
            target_id = current_id;
            target_pos = current_center;
        }
    }

    if (target_id == -1) {
        RCLCPP_WARN(this->get_logger(), "適切な追従対象が見つかりませんでした。");
        publishCmdVel(0.0, 0.0);
        return;
    }

    // 近くにもう1つのクラスタがあるかチェック
    int second_id = -1;
    geometry_msgs::msg::Point second_center;
    
    for (const auto &[current_id, current_center] : cluster_centers) {
        if (current_id == target_id) continue;

        double dist = sqrt(pow(current_center.x - target_pos.x, 2) + pow(current_center.y - target_pos.y, 2));
        if (dist < FOOT_DISTANCE_THRESHOLD) {
            second_id = current_id;
            second_center = current_center;
            break;
        }
    }

    // 追従位置を決定
    if (second_id != -1) {
        target_pos.x = (target_pos.x + second_center.x) / 2.0;
        target_pos.y = (target_pos.y + second_center.y) / 2.0;
        RCLCPP_INFO(this->get_logger(), "2つのクラスタを選択: ID %d と ID %d", target_id, second_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "単独クラスタを追従: ID %d", target_id);
    }

    // ターゲットを更新
    previous_target_id_ = target_id;
    this->current_target_id_ = target_id;
    this->current_second_id_ = second_id;

    publishTargetMarker(target_pos);

    // 目標地点への移動指令
    double angle_to_target = atan2(target_pos.y, target_pos.x);
    double distance_to_target = sqrt(target_pos.x * target_pos.x + target_pos.y * target_pos.y);

    RCLCPP_INFO(this->get_logger(), "追従目標位置: (%.2f, %.2f)", target_pos.x, target_pos.y);
    publishCmdVel(distance_to_target, angle_to_target);
    this->saveClusterDataToCSV(cluster_id_history_, cluster_velocities_, cluster_centers);
}

void LegClusterTracking::publishCmdVel(double target_distance, double target_angle) {
    auto cmd_msg = geometry_msgs::msg::Twist();

    // ジョイスティックで停止が指示されたら何もしない
    if (stop_by_joystick_) {
        RCLCPP_WARN(this->get_logger(), "ジョイスティックの入力により移動を停止中...");
        cmd_vel_publisher_->publish(cmd_msg);
        return;
    }

    // 30cm以内なら停止
    if (target_distance <= STOP_DISTANCE_THRESHOLD) {
        RCLCPP_INFO(this->get_logger(), "追従対象に到達！ 停止します。");
        cmd_vel_publisher_->publish(cmd_msg); // 速度0を送信
        return;
    }

    // 現在の誤差を計算
    double error_dist = target_distance - STOP_DISTANCE_THRESHOLD;
    double error_angle = target_angle;

    // 誤差の積分項を更新
    integral_dist += error_dist;
    integral_angle += error_angle;

    // PID計算
    double linear_velocity = (KP_DIST * error_dist) + (KI_DIST * integral_dist);
    double angular_velocity = (KP_ANGLE * error_angle) + (KI_ANGLE * integral_angle);

    cmd_msg.linear.x = std::clamp(linear_velocity, MIN_SPEED, MAX_SPEED);
    cmd_msg.angular.z = std::clamp(angular_velocity, -MAX_TURN_SPEED, MAX_TURN_SPEED);

    // 速度をパブリッシュ
    cmd_vel_publisher_->publish(cmd_msg);
}

void LegClusterTracking::publishClusterMarkers(
    const std::vector<geometry_msgs::msg::Point> &points, 
    const std::vector<int> &clusters) {
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

    // mapping後のクラスタ番号を使用してカラーを付与
    for (size_t i = 0; i < points.size(); i++) {
        if (points[i].x != 0.0 || points[i].y != 0.0) {
            // マッピング後のクラスタ番号を取得
            if(cluster_id_mapping_.count(clusters[i]) > 0){
                int mapped_cluster_id = cluster_id_mapping_[clusters[i]];
                marker.points.push_back(points[i]);
                marker.colors.push_back(color_palette_[mapped_cluster_id % color_palette_.size()]);
            }
        }
    }

    cluster_marker_publisher_->publish(marker);
}

void LegClusterTracking::publishMatchedClusterCenters(const std::map<int, geometry_msgs::msg::Point> &current_centers) {
    visualization_msgs::msg::MarkerArray marker_array;  // マーカー配列を作成

    // クラスタ中心を示すマーカー
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

    // ID 表示用のテキストマーカーのテンプレート
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "laser";
    text_marker.header.stamp = this->get_clock()->now();
    text_marker.ns = "cluster_id_labels";
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.scale.z = 0.1;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;

    int marker_id = 100;

    for (const auto &[current_id, current_center] : current_centers) {
        // クラスタ中心マーカーを追加
        center_marker.points.push_back(current_center);
        center_marker.colors.push_back(color_palette_[current_id % color_palette_.size()]);

        // クラスタIDのテキストマーカーを作成
        visualization_msgs::msg::Marker text = text_marker;
        text.id = marker_id++;  // 一意のIDを設定
        text.pose.position = current_center;
        text.pose.position.z += 0.1; // 少し上に配置
        text.text = std::to_string(current_id); // ID を文字列としてセット
        marker_array.markers.push_back(text); // マーカー配列に追加
    }

    // クラスタ中心マーカーを配列に追加
    marker_array.markers.push_back(center_marker);

    // マーカーを一括でパブリッシュ
    center_marker_publisher_->publish(marker_array);
}

void LegClusterTracking::publishTargetMarker(const geometry_msgs::msg::Point &target_pos) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "laser";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "target_marker";
    marker.id = 999;  // ユニークなID
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = target_pos;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;  // 透明度

    target_marker_publisher_->publish(marker);
}

std::vector<std_msgs::msg::ColorRGBA> LegClusterTracking::generateColors(int num_clusters) {
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

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegClusterTracking>());
    rclcpp::shutdown();
    return 0;
}