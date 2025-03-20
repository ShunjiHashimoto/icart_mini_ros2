#include "icart_mini_leg_tracker/utils/csv_logger.hpp"

CSVLogger::CSVLogger(const std::string &filename) : filename_(filename) {
    resetCSVFile();
}

void CSVLogger::resetCSVFile() {
    std::ofstream csv_file(filename_, std::ios::trunc);
    if (!csv_file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("CSV_Manager"), "CSVファイルを開けませんでした。");
        return;
    }
    csv_file << "追従対象ID1,追従対象ID2,クラスタID,履歴,位置X,位置Y,速度X,速度Y" << std::endl;
    csv_file.close();
    RCLCPP_INFO(rclcpp::get_logger("CSV_Manager"), "CSVファイルをリセットしました。");
}

void CSVLogger::saveClusterData(const std::map<int, std::vector<int>>& cluster_id_history_,
                                const std::map<int, geometry_msgs::msg::Vector3>& cluster_velocities_,
                                const std::map<int, geometry_msgs::msg::Point>& current_centers,
                                int current_target_id_, int current_second_id_) {
    std::ofstream csv_file(filename_, std::ios::app);
    if (!csv_file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("CSV_Manager"), "CSVファイルを開けませんでした。");
        return;
    }
    for (const auto& [cluster_id, history] : cluster_id_history_) {
        csv_file << current_target_id_ << "," << current_second_id_ << "," << cluster_id << ",";
        for (size_t i = 0; i < history.size(); i++) {
            csv_file << history[i];
            if (i < history.size() - 1) csv_file << " ";
        }
        csv_file << ",";
        if (current_centers.count(cluster_id)) {
            csv_file << current_centers.at(cluster_id).x << "," << current_centers.at(cluster_id).y;
        } else {
            csv_file << "0,0";
        }
        csv_file << ",";
        if (cluster_velocities_.count(cluster_id)) {
            csv_file << cluster_velocities_.at(cluster_id).x << "," << cluster_velocities_.at(cluster_id).y;
        } else {
            csv_file << "0,0";
        }
        csv_file << std::endl;
    }
    csv_file.close();
    RCLCPP_INFO(rclcpp::get_logger("CSV_Manager"), "クラスタデータをCSVに保存しました。");
}
