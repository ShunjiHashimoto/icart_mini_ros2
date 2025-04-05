#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <ypspur.h>

class ICartMiniYpSpurBridge : public rclcpp::Node
{
public:
    ICartMiniYpSpurBridge() : Node("icart_mini_ypspur_bridge"), tf_broadcaster_(*this)
    {
        // YPSpur の初期化
        if (YPSpur_init() < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize YPSpur.");
            rclcpp::shutdown();
            return;
        }

        // 初期設定
        Spur_set_vel(0.3);
        Spur_set_accel(1.0);
        Spur_set_angvel(M_PI / 2.0);
        Spur_set_angaccel(M_PI / 2.0);
        Spur_set_pos_GL(0, 0, 0);

        // cmd_vel を購読
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&ICartMiniYpSpurBridge::cmdVelCallback, this, std::placeholders::_1));

        // odometry を発行
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // タイマーを設定 (10ms 間隔)
        loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ICartMiniYpSpurBridge::updateOdometry, this));

        RCLCPP_INFO(this->get_logger(), "icart_mini_ypspur_bridge node has started.");
    }

    ~ICartMiniYpSpurBridge()
    {
        Spur_stop();
        Spur_free();
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        Spur_vel(msg->linear.x, msg->angular.z);
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear.x = %.2f, angular.z = %.2f", msg->linear.x, msg->angular.z);
    }

    void updateOdometry()
    {
        double x, y, yaw, v, w;
        Spur_get_pos_GL(&x, &y, &yaw);
        Spur_get_vel(&v, &w);

        // odom メッセージ作成
        auto current_time = this->now();
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));
        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = w;
        odom_pub_->publish(odom);

        // odom TF 送信
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_.sendTransform(odom_trans);

        // Joint State 更新
        sensor_msgs::msg::JointState js;
        js.header.stamp = current_time;
        js.name = {"left_wheel_joint", "right_wheel_joint"};
        double l_ang_pos, r_ang_pos, l_wheel_vel, r_wheel_vel;
        YP_get_wheel_ang(&l_ang_pos, &r_ang_pos);
        YP_get_wheel_vel(&l_wheel_vel, &r_wheel_vel);
        js.position = {-l_ang_pos, -r_ang_pos};
        js.velocity = {l_wheel_vel, r_wheel_vel};
        js_pub_->publish(js);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ICartMiniYpSpurBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}