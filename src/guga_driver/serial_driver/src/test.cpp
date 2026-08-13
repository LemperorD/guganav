#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("tset_20260812_node");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.0;  // Set linear velocity in x direction
    msg.linear.y = 0.0;  // Set linear velocity in y direction
    msg.linear.z = 0.0;  // Set linear velocity in z direction
    msg.angular.x = 0.0; // Set angular velocity around x axis
    msg.angular.y = 0.0; // Set angular velocity around y axis
    msg.angular.z = 0.0; // Set angular velocity around z axis

    rclcpp::Rate loop_rate(10); // 10 Hz
    while (rclcpp::ok())
    {
        publisher->publish(msg);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}