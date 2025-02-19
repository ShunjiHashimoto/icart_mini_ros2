#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarSubscriberPublisher : public rclcpp::Node {
public:
    LidarSubscriberPublisher() : Node("lidar_subscriber_publisher_node") {
        // /scanトピックの購読
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LidarSubscriberPublisher::lidarCallback, this, std::placeholders::_1)
        );

        // 処理後のデータを/leg_tracker/raw_scanにパブリッシュ
        lidar_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/leg_tracker/raw_scan", 10);

        RCLCPP_INFO(this->get_logger(), "Lidar Subscriber and Publisher Node has started.");
    }

private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received Lidar data with [%zu] ranges.", msg->ranges.size());
        lidar_publisher_->publish(*msg);  // そのまま出力
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarSubscriberPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
