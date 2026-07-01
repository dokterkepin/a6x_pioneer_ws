#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
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
  move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group_interface.setPlannerId("PTP");
  
  RCLCPP_INFO(logger, "Planning Pipeline: %s", move_group_interface.getPlanningPipelineId().c_str());
  RCLCPP_INFO(logger, "Planner ID: %s", move_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Finger Planning Pipeline: %s", finger_group_interface.getPlanningPipelineId().c_str());


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

  // Add floor collision object
  {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::msg::CollisionObject floor;
    floor.header.frame_id = move_group_interface.getPlanningFrame();
    floor.id = "floor";
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {2.0, 2.0, 0.01};
    geometry_msgs::msg::Pose floor_pose;
    floor_pose.orientation.w = 1.0;
    floor_pose.position.z = -0.02;
    floor.primitives.push_back(box);
    floor.primitive_poses.push_back(floor_pose);
    floor.operation = floor.ADD;
    planning_scene_interface.applyCollisionObject(floor);
    RCLCPP_INFO(logger, "Floor collision object added");
  }

  // Compute orientation for gripper pointing nearly straight down,
  // accounting for shoulder rotation toward each target position.
  // tilt_deg controls how far from horizontal (75° = nearly vertical, 15° from straight down)
  auto make_down_pose = [](double x, double y, double z, double tilt_deg = 75.0) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    double yaw = std::atan2(y, x);
    double tilt = tilt_deg * M_PI / 180.0;
    double hy = yaw / 2.0;
    double ht = tilt / 2.0;

    // Quaternion for Rz(yaw) * Ry(tilt)
    pose.orientation.w = std::cos(hy) * std::cos(ht);
    pose.orientation.x = -std::sin(hy) * std::sin(ht);
    pose.orientation.y = std::cos(hy) * std::sin(ht);
    pose.orientation.z = std::sin(hy) * std::cos(ht);
    return pose;
  };

  for (int i = 0; i < 100; ++i) {
    RCLCPP_INFO(logger, "=== Iteration %d/100 ===", i + 1);

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
        auto current = move_group_interface.getCurrentPose().pose;
        RCLCPP_INFO(logger, "End-effector at home -> pos(%.3f, %.3f, %.3f) orient(x=%.4f, y=%.4f, z=%.4f, w=%.4f)",
          current.position.x, current.position.y, current.position.z,
          current.orientation.x, current.orientation.y, current.orientation.z, current.orientation.w);
      } else {
        RCLCPP_ERROR(logger, "Failed to plan to home position!");
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Odd iterations: close fingers first (pick at home), then open after target4 (place)
    // Even iterations: go to target1 first, close (pick there), then open after target4 (place)
    if (i % 2 != 0) {
      // Close fingers (pick at home)
      finger_group_interface.setNamedTarget("home");
      {
        moveit::planning_interface::MoveGroupInterface::Plan close_plan;
        auto const close_ok = static_cast<bool>(finger_group_interface.plan(close_plan));
        if (close_ok) {
          draw_title("Closing Fingers (Pick)");
          moveit_visual_tools.trigger();
          finger_group_interface.execute(close_plan);
          std::this_thread::sleep_for(std::chrono::seconds(2));
          RCLCPP_INFO(logger, "Fingers closed (pick at home)");
        } else {
          RCLCPP_ERROR(logger, "Failed to plan to close fingers!");
        }
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Target 1
    {
      auto const target_pose = make_down_pose(0.4, -0.1, 0.07, 20.0);
      move_group_interface.setPoseTarget(target_pose);

      draw_title("Planning Target 1");
      moveit_visual_tools.trigger();
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto const success = static_cast<bool>(move_group_interface.plan(plan));

      if (success) {
        draw_trajectory_tool_path(plan.trajectory_);
        moveit_visual_tools.trigger();
        draw_title("Executing Target 1");
        moveit_visual_tools.trigger();
        move_group_interface.execute(plan);
      } else {
        draw_title("Target 1 Failed!");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Target 1 failed!");
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (i % 2 == 0) {
      // Close fingers (pick at target1)
      finger_group_interface.setNamedTarget("home");
      {
        moveit::planning_interface::MoveGroupInterface::Plan close_plan;
        auto const close_ok = static_cast<bool>(finger_group_interface.plan(close_plan));
        if (close_ok) {
          draw_title("Closing Fingers (Pick)");
          moveit_visual_tools.trigger();
          finger_group_interface.execute(close_plan);
          std::this_thread::sleep_for(std::chrono::seconds(2));
          RCLCPP_INFO(logger, "Fingers closed (pick at target1)");
        } else {
          RCLCPP_ERROR(logger, "Failed to plan to close fingers!");
        }
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Target 2
    {
      auto const target_pose2 = make_down_pose(0.2, 0.0, 0.4);
      move_group_interface.setPoseTarget(target_pose2);
      moveit::planning_interface::MoveGroupInterface::Plan plan2;
      auto const success2 = static_cast<bool>(move_group_interface.plan(plan2));
      if (success2) {
        draw_trajectory_tool_path(plan2.trajectory_);
        moveit_visual_tools.trigger();
        draw_title("Executing Target 2");
        moveit_visual_tools.trigger();
        move_group_interface.execute(plan2);
      } else {
        draw_title("Target 2 Failed!");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Target 2 failed!");
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Target 3
    {
      auto const target_pose3 = make_down_pose(0.1, 0.3, 0.3, 90.0);
      move_group_interface.setPoseTarget(target_pose3);
      moveit::planning_interface::MoveGroupInterface::Plan plan3;
      auto const success3 = static_cast<bool>(move_group_interface.plan(plan3));

      if (success3) {
        draw_trajectory_tool_path(plan3.trajectory_);
        moveit_visual_tools.trigger();
        draw_title("Executing Target 3");
        moveit_visual_tools.trigger();
        move_group_interface.execute(plan3);
      } else {
        draw_title("Target 3 Failed!");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Target 3 failed!");
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Target 4
    {
      auto const target_pose4 = make_down_pose(0.35, -0.25, 0.1, 20.0);
      move_group_interface.setPoseTarget(target_pose4);
      moveit::planning_interface::MoveGroupInterface::Plan plan4;
      auto const success4 = static_cast<bool>(move_group_interface.plan(plan4));

      if (success4) {
        draw_trajectory_tool_path(plan4.trajectory_);
        moveit_visual_tools.trigger();
        draw_title("Executing Target 4");
        moveit_visual_tools.trigger();
        move_group_interface.execute(plan4);
      } else {
        draw_title("Target 4 Failed!");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Target 4 failed!");
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Open fingers (place)
    finger_group_interface.setNamedTarget("open");
    {
      moveit::planning_interface::MoveGroupInterface::Plan open_plan;
      auto const open_ok = static_cast<bool>(finger_group_interface.plan(open_plan));
      if (open_ok) {
        draw_title("Opening Fingers (Place)");
        moveit_visual_tools.trigger();
        finger_group_interface.execute(open_plan);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        RCLCPP_INFO(logger, "Fingers opened (place)");
      } else {
        RCLCPP_ERROR(logger, "Failed to plan to open fingers!");
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Go to home position
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

    RCLCPP_INFO(logger, "=== Iteration %d/100 complete ===", i + 1);
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
