#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "position",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("position");

  // Spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");
  auto finger_group_interface = MoveGroupInterface(node, "finger");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Visualization helpers
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  };
  auto const draw_trajectory_tool_path =
    [&moveit_visual_tools,
     jmg = move_group_interface.getRobotModel()->getJointModelGroup(
       "arm")](auto const trajectory) {
      moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
    };

  // Go to home position first
  move_group_interface.setNamedTarget("pose1");
  {
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    auto const home_ok = static_cast<bool>(move_group_interface.plan(home_plan));
    if (home_ok) {
      draw_title("Going Home");
      moveit_visual_tools.trigger();
      move_group_interface.execute(home_plan);
      RCLCPP_INFO(logger, "Moved to home position");
    } else {
      RCLCPP_ERROR(logger, "Failed to plan to home position!");
    }
  }

  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.4;
    msg.position.y = -0.1;
    msg.position.z = 0.2;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Plan to that target pose
  draw_title("Planning");
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success) {
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  } else {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  finger_group_interface.setNamedTarget("open");
  {
    moveit::planning_interface::MoveGroupInterface::Plan open_plan;
    auto const open_ok = static_cast<bool>(finger_group_interface.plan(open_plan));
    if (open_ok) {
      draw_title("Opening Fingers");
      moveit_visual_tools.trigger();
      finger_group_interface.execute(open_plan);
      RCLCPP_INFO(logger, "Fingers opened");
    } else {
      RCLCPP_ERROR(logger, "Failed to plan to open fingers!");
    }
  }

  auto const target_pose2 = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.2;
    msg.position.y = 0.0;
    msg.position.z = 0.3;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose2);
  auto const [success2, plan2] = [&move_group_interface]() {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the second plan
  if (success2) {
    draw_trajectory_tool_path(plan2.trajectory_);
    moveit_visual_tools.trigger();
    draw_title("Executing Second Plan");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan2);
  } else {
    draw_title("Second Plan Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Second plan failed!");
  }
  
    auto const target_pose3 = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.35;
    msg.position.y = -0.4;
    msg.position.z = 0.1;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose3);
  auto const [success3, plan3] = [&move_group_interface]() {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the third plan
  if (success3) {
    draw_trajectory_tool_path(plan3.trajectory_);
    moveit_visual_tools.trigger();
    draw_title("Executing Third Plan");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan3);
  } else {
    draw_title("Third Plan Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Third plan failed!");
  }
  finger_group_interface.setNamedTarget("close");
  {
    moveit::planning_interface::MoveGroupInterface::Plan close_plan;
    auto const close_ok = static_cast<bool>(finger_group_interface.plan(close_plan));
    if (close_ok) {
      draw_title("Closing Fingers");
      moveit_visual_tools.trigger();
      finger_group_interface.execute(close_plan);
      RCLCPP_INFO(logger, "Fingers closed");
    } else {
      RCLCPP_ERROR(logger, "Failed to plan to close fingers!");
    }
  }

  // Go to pose1 position
  move_group_interface.setNamedTarget("home");
  {
    moveit::planning_interface::MoveGroupInterface::Plan pose1_plan;
    auto const pose1_ok = static_cast<bool>(move_group_interface.plan(pose1_plan));
    if (pose1_ok) {
      draw_title("Going to pose1");
      moveit_visual_tools.trigger();
      move_group_interface.execute(pose1_plan);
      RCLCPP_INFO(logger, "Moved to pose1 position");
    } else {
      RCLCPP_ERROR(logger, "Failed to plan to pose1 position!");
    }
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
