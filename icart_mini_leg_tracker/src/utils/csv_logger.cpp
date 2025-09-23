#include "icart_mini_leg_tracker/utils/csv_logger.hpp"

#include <cmath>
#include <limits>

CSVLogger::CSVLogger(const std::string &filename) : filename_(filename) {
    resetCSVFile();
}

void CSVLogger::resetCSVFile() {
    std::ofstream csv_file(filename_, std::ios::trunc);
    if (!csv_file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("CSV_Manager"), "CSVファイルを開けませんでした。");
        return;
    }
    csv_file << "追従対象ID1,追従対象ID2,クラスタID,履歴,位置X,位置Y,速度X,速度Y,select_called,select_target_id,select_reason,select_movement,select_distance_to_robot,select_timestamp" << std::endl;
    csv_file.close();
    RCLCPP_INFO(rclcpp::get_logger("CSV_Manager"), "CSVファイルをリセットしました。");
}

void CSVLogger::saveClusterData(const std::map<int, std::vector<int>>& cluster_id_history_,
                                const std::map<int, icart_msg::ClusterInfo>& cluster_infos_,
                                int current_target_id_, int current_second_id_,
                                bool select_called, int selected_target_id,
                                const std::string &selection_reason,
                                double selection_movement, double selection_distance_to_robot,
                                double selection_timestamp) {
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

        if (cluster_infos_.count(cluster_id)) {
            const auto& info = cluster_infos_.at(cluster_id);
            csv_file << info.center.x << "," << info.center.y << ",";
            csv_file << info.velocity.x << "," << info.velocity.y;
        } else {
            csv_file << "0,0,0,0";
        }

        csv_file << "," << (select_called ? 1 : 0) << ",";
        if (selected_target_id >= 0) {
            csv_file << selected_target_id;
        }
        csv_file << "," << selection_reason << ",";

        if (std::isfinite(selection_movement) && selection_movement >= 0.0) {
            csv_file << selection_movement;
        }
        csv_file << ",";

        if (std::isfinite(selection_distance_to_robot) && selection_distance_to_robot >= 0.0) {
            csv_file << selection_distance_to_robot;
        }
        csv_file << ",";

        if (std::isfinite(selection_timestamp) && selection_timestamp >= 0.0) {
            csv_file << selection_timestamp;
        }
        csv_file << std::endl;
    }
    csv_file << " --------- " << std::endl;
    csv_file.close();
    // RCLCPP_INFO(rclcpp::get_logger("CSV_Manager"), "クラスタデータをCSVに保存しました。");
}
