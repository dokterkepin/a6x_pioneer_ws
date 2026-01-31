#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/callback_group.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using namespace std::placeholders;

class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node; 
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        finger_ = std::make_shared<MoveGroupInterface>(node_, "finger");

        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group_;

        open_finger_sub_ = node_->create_subscription<std_msgs::msg::Float64>("open_finger", 10, 
            std::bind(&Commander::openFingerCallback, this, _1), options);
        
        pose_target_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>("pose_target", 10,
            std::bind(&Commander::poseTargetCallback, this, _1), options);
    }

    void goToNamedTarget(const std::string &target_name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(target_name);
        planAndExecute(arm_);
    }

    void goToJointTarget(const std::vector<double> &joint_values)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joint_values);
        planAndExecute(arm_);
    }

    void goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();
        arm_->setPoseTarget(target_pose);
        planAndExecute(arm_);
    }

    void setFingerPosition(double value)
    {
        finger_->setStartStateToCurrentState();
        finger_->setJointValueTarget({value});
        planAndExecute(finger_);
    }

    void goToCartesianPath(double x, double y, double z)
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose start_pose = arm_->getCurrentPose().pose; 
        start_pose.position.x += x;
        start_pose.position.y += y;
        start_pose.position.z += z;
        
        waypoints.push_back(start_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = arm_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
        if (fraction == 1.0) {
            arm_->execute(trajectory);
        }
    }

private:
    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            interface->execute(plan);
        }
    }

    void openFingerCallback(const std_msgs::msg::Float64 &msg)
    {
        if (msg.data) {
            setFingerPosition(msg.data);
        }
    }

    void poseTargetCallback(const std_msgs::msg::Float64MultiArray &msg)
    {
        if (msg.data.size() == 6) {
            goToPoseTarget(msg.data[0], msg.data[1], msg.data[2],
                           msg.data[3], msg.data[4], msg.data[5]);
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> finger_;
    
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr open_finger_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pose_target_sub_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = std::make_shared<Commander>(node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}