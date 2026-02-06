#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  void cubeTransformCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void targetTransformCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr cube_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr target_sub_;
  
  geometry_msgs::msg::Vector3 cube_position_;
  geometry_msgs::msg::Vector3 target_position_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  cube_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
    "/cube", 10, std::bind(&MTCTaskNode::cubeTransformCallback, this, std::placeholders::_1));

  target_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
    "/target", 10, std::bind(&MTCTaskNode::targetTransformCallback, this, std::placeholders::_1));
}

void MTCTaskNode::cubeTransformCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  for (const auto& transform : msg->transforms) {
    if (transform.child_frame_id == "Cube") {
      cube_position_ = transform.transform.translation;
      return;
    }else{
      RCLCPP_WARN(LOGGER, "Cube Position not Received");
    }
  }
}

void MTCTaskNode::targetTransformCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  for (const auto& transform : msg->transforms) {
    if (transform.child_frame_id == "target") {
      target_position_ = transform.transform.translation;
      return;
    }else{
      RCLCPP_WARN(LOGGER, "Target Position not Received");
    }
  }
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = { 0.035, 0.035, 0.05 }; 

  geometry_msgs::msg::Pose pose;
  if (cube_position_.x != 0.00 || cube_position_.y != 0.00 || cube_position_.z != 0.00) {
    pose.position.x = cube_position_.x;
    pose.position.y = cube_position_.y;
    pose.position.z = cube_position_.z;
    pose.orientation.w = 1.0;
    object.pose = pose;
  }else{
    RCLCPP_WARN(LOGGER, "Cube Position not Received");
  }

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
  
  // Add ground plane to prevent collisions with floor
  moveit_msgs::msg::CollisionObject ground;
  ground.id = "ground";
  ground.header.frame_id = "world";
  ground.primitives.resize(1);
  ground.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  ground.primitives[0].dimensions = { 2.0, 2.0, 0.001 };  // Large thin box
  
  geometry_msgs::msg::Pose ground_pose;
  ground_pose.position.x = 0.0;
  ground_pose.position.y = 0.0;
  ground_pose.position.z = -0.040;
  ground_pose.orientation.w = 1.0;
  ground.pose = ground_pose;
  
  psi.applyCollisionObject(ground);
  
  RCLCPP_INFO(LOGGER, "Added cube collision object at [%.3f, %.3f, %.3f]",
              pose.position.x, pose.position.y, pose.position.z);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5 /* max_solutions */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    
    // Print detailed failure info
    std::stringstream ss;
    task_.explainFailure(ss);
    RCLCPP_ERROR_STREAM(LOGGER, "Failure details:\n" << ss.str());
    
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "finger";
  const auto& hand_frame = "palm_link";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setProperty("max_velocity_scaling_factor", 0.1);  // Slower for carrying object
  sampling_planner->setProperty("max_acceleration_scaling_factor", 0.1);
  
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.1);
  cartesian_planner->setMaxAccelerationScalingFactor(0.1);
  cartesian_planner->setStepSize(.005);  // Smaller step size for more waypoints

  // clang-format off
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  // clang-format on
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  // clang-format off
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  // clang-format on
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  // clang-format off
  mtc::Stage* attach_object_stage =
      nullptr;  // Forward attach_object_stage to place pose generator
  // clang-format on

  // This is an example of SerialContainer usage. It's not strictly needed here.
  // In fact, `task` itself is a SerialContainer by default.
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    
    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      // clang-format on
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.2);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = -1.0;  
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  ---- *               Generate Grasp Pose                *
     ***************************************************/
    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12);  // Sample every 15 degrees around object
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state

      // Transform from object frame to end-effector frame
      // Top-down grasp: gripper points down toward the cube
      Eigen::Isometry3d grasp_frame_transform;
      // Combine two rotations: first around Z, then around X
      Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2 , Eigen::Vector3d::UnitZ()));
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().x() = 0.1;  // Palm above cube  

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      // clang-format on
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      // Allow collision between gripper/finger links and the object
      // Include all gripper-related links since finger group may not include all
      std::vector<std::string> gripper_links = {"palm_link", "left_finger_link", "right_finger_link"};
      stage->allowCollisions("object", gripper_links, true);
      // clang-format on
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal({{"left_finger_prismatic_joint", 0.001}});
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      // clang-format on
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.2);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));
  }

  {
    // clang-format off
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{{ arm_group_name, sampling_planner }});
    // clang-format on
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on

    /****************************************************
  ---- *               Generate Place Pose                *
     ***************************************************/
    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "world";
      target_pose_msg.pose.position.x = target_position_.x;
      target_pose_msg.pose.position.y = target_position_.y;
      target_pose_msg.pose.position.z = target_position_.z;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      std::vector<std::string> gripper_links = {"palm_link", "left_finger_link", "right_finger_link"};
      stage->allowCollisions("object", gripper_links, false);
      // clang-format on
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.2);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("home");
    task.add(std::move(stage));
  }
  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  // Wait a moment for cube topic to be received
  std::this_thread::sleep_for(std::chrono::seconds(2));
  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}

