#ifndef CSV_LOGGER_HPP
#define CSV_LOGGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <map>
#include <vector>
#include <fstream>
#include <string>
#include "icart_mini_leg_tracker/msg/cluster_info.hpp"

namespace icart_msg = icart_mini_leg_tracker::msg;

class CSVLogger {
public:
    explicit CSVLogger(const std::string &filename);

    void resetCSVFile();
    void saveClusterData(const std::map<int, std::vector<int>>& cluster_id_history_,
                                const std::map<int, icart_msg::ClusterInfo>& cluster_infos_,
                                int current_target_id_, int current_second_id_);

private:
    std::string filename_;
};

#endif  // CSV_LOGGER_HPP
