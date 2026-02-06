#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_moveit_node");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    auto gripper = moveit::planning_interface::MoveGroupInterface(node, "finger");

    // Set Pose Target
    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("home");

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success1) {
    //     arm.execute(plan1);
    // }

    // Set Joint Target
    // std::vector<double> joints = {0.05, 0.08, 1.0, 0.07, 1.0, 0.1};
    // arm.setStartStateToCurrentState();
    // arm.setJointValueTarget(joints);
    
    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success1) {
    //     arm.execute(plan1);
    // }
    // tf2::Quaternion q; 
    // q.setRPY(0.0, 1.45, 0.0);
    // q = q.normalize();

    // geometry_msgs::msg::PoseStamped target_pose;
    // target_pose.header.frame_id = "base_link";
    // target_pose.pose.position.x = 0.2;
    // target_pose.pose.position.y = 0.0;
    // target_pose.pose.position.z = 0.2;
    // target_pose.pose.orientation.x = q.getX();
    // target_pose.pose.orientation.y = q.getY();
    // target_pose.pose.orientation.z = q.getZ();
    // target_pose.pose.orientation.w = q.getW();

    // arm.setStartStateToCurrentState();
    // arm.setPoseTarget(target_pose);

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success1) {
    //     arm.execute(plan1);
    // }

    // Cartesian Path
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose;
    pose1.position.z -= 0.1;
    waypoints.push_back(pose1);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction == 1.0) {
        arm.execute(trajectory);
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}