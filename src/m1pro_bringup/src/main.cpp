#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <m1pro_bringup/robot.h>
#include <m1pro_bringup/msg/robot_status.hpp>
#include <m1pro_bringup/msg/tool_vector_actual.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        auto robot = std::make_shared<M1ProRobot>();
        robot->init();

        auto joint_state_pub = robot->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 100);
        auto robot_status_pub = robot->create_publisher<m1pro_bringup::msg::RobotStatus>("msg/RobotStatus", 100);
        auto tool_vector_pub = robot->create_publisher<m1pro_bringup::msg::ToolVectorActual>("msg/ToolVectorActual", 100);

        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.name = {"joint1", "joint2", "joint3", "joint4"};
        joint_state_msg.position.resize(4, 0.0);

        m1pro_bringup::msg::RobotStatus robot_status_msg;
        m1pro_bringup::msg::ToolVectorActual tool_vector_msg;

        double publish_rate = robot->declare_parameter<double>("JointStatePublishRate", 10.0);
        rclcpp::Rate rate(publish_rate);
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(robot);

        RCLCPP_INFO(robot->get_logger(), "M1Pro driver running at %.1f Hz", publish_rate);

        double position[6] = {};
        double tool_vector[6] = {};

        while (rclcpp::ok())
        {
            executor.spin_some();

            robot->getJointState(position);
            joint_state_msg.header.stamp = robot->get_clock()->now();
            joint_state_msg.header.frame_id = "base_link";
            for (int i = 0; i < 4; i++)
                joint_state_msg.position[i] = position[i];
            joint_state_pub->publish(joint_state_msg);

            robot->getToolVectorActual(tool_vector);
            tool_vector_msg.x  = tool_vector[0];
            tool_vector_msg.y  = tool_vector[1];
            tool_vector_msg.z  = tool_vector[2];
            tool_vector_msg.rx = tool_vector[3];
            tool_vector_msg.ry = tool_vector[4];
            tool_vector_msg.rz = tool_vector[5];
            tool_vector_pub->publish(tool_vector_msg);

            robot_status_msg.is_enable    = robot->isEnable();
            robot_status_msg.is_connected = robot->isConnected();
            robot_status_pub->publish(robot_status_msg);

            rate.sleep();
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "%s", e.what());
        rclcpp::shutdown();
        return -1;
    }
    rclcpp::shutdown();
    return 0;
}
