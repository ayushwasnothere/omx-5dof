#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cmath> // Required for std::atan2
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


// ── Namespaces ──────────────────────────────────────────────────────────────
namespace mtc = moveit::task_constructor;

// ── Robot-specific constants — CHANGE ONLY THESE if your SRDF changes ───────
static const std::string ARM_GROUP       = "arm";
static const std::string GRIPPER_GROUP   = "gripper";
static const std::string EEF_NAME        = "end_effector";   
static const std::string EEF_LINK        = "end_effector_link"; 
static const std::string HAND_FRAME      = "end_effector_link"; 
static const std::string BASE_FRAME      = "world";

static const std::string ARM_HOME_STATE    = "home";
static const std::string GRIPPER_OPEN      = "open";
static const std::string GRIPPER_CLOSE     = "close";

// ── Object / scene constants ─────────────────────────────────────────────────
static const std::string OBJECT_ID  = "target_cylinder";

// ── Helper: spawn scene (cylinder only) ──────────────────────────────────────
void spawnScene(rclcpp::Node::SharedPtr node)
{
  moveit::planning_interface::PlanningSceneInterface psi;

  moveit_msgs::msg::CollisionObject obj;
  obj.id = OBJECT_ID;
  obj.header.frame_id = BASE_FRAME;
  obj.header.stamp = node->now();

  shape_msgs::msg::SolidPrimitive cyl;
  cyl.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  cyl.dimensions = {0.10, 0.01};   // [height, radius]

  geometry_msgs::msg::Pose obj_pose;
  obj_pose.position.x = 0.20;
  obj_pose.position.y = 0.00;
  obj_pose.position.z = 0.05;     
  obj_pose.orientation.w = 1.0;

  obj.primitives.push_back(cyl);
  obj.primitive_poses.push_back(obj_pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  psi.applyCollisionObjects({obj});
  RCLCPP_INFO(node->get_logger(), "Scene spawned: %s", OBJECT_ID.c_str());
}



// ── Build MTC Task ───────────────────────────────────────────────────────────
mtc::Task buildTask(rclcpp::Node::SharedPtr node)
{
  mtc::Task task;
  task.stages()->setName("omx_pick_place");
  task.loadRobotModel(node);

  // ── Shared solvers ─────────────────────────────────────────────────────────
  
  // 1. Pipeline planner (OMPL) for free-space point-to-point moves
  auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(node);
  pipeline->setProperty("goal_joint_tolerance", 1e-3);
  pipeline->setProperty("max_velocity_scaling_factor", 0.1);
  pipeline->setProperty("max_acceleration_scaling_factor", 0.1);

  // 2. Cartesian planner for straight-line moves
  auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian->setStepSize(0.01);
  cartesian->setMaxVelocityScalingFactor(0.1);
  cartesian->setMaxAccelerationScalingFactor(0.1);

  // 3. Joint-space solver for gripper open/close and linear approaches
  auto joint_interpolation = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  joint_interpolation->setMaxVelocityScalingFactor(0.1);
  joint_interpolation->setMaxAccelerationScalingFactor(0.1);

  // ── Set properties that stages inherit ────────────────────────────────────
  task.setProperty("group",        ARM_GROUP);
  task.setProperty("eef",          EEF_NAME);
  task.setProperty("hand",         GRIPPER_GROUP);
  task.setProperty("hand_grasping_frame", HAND_FRAME);
  task.setProperty("ik_frame",     HAND_FRAME);

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  // Stage 0 — Current state 
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  mtc::Stage* current_state_ptr = nullptr;
  {
    auto current = std::make_unique<mtc::stages::CurrentState>("current state");
    current_state_ptr = current.get();  
    task.add(std::move(current));
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  // Stage 1 — Open gripper 
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  {
    auto open = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_interpolation);
    open->setGroup(GRIPPER_GROUP);
    open->setGoal(GRIPPER_OPEN);
    task.add(std::move(open));
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  // Stage 2 — Connect (Move arm to pre-grasp vicinity)
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  {
    auto connect = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{
        {ARM_GROUP,     pipeline},
        {GRIPPER_GROUP, joint_interpolation}
      }
    );
    connect->setTimeout(10.0);
    connect->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(connect));
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  // Stage 3 — Pick container (approach → grasp → lift)
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  mtc::Stage* attach_stage_ptr = nullptr;  
  {
    auto pick = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(pick->properties(), {"eef", "hand", "group", "ik_frame"});
    pick->properties().configureInitFrom(mtc::Stage::PARENT,
                                          {"eef", "hand", "group", "ik_frame"});

    // 3a — Approach object
    {
      auto approach = std::make_unique<mtc::stages::MoveRelative>("approach object", joint_interpolation);
      approach->setGroup(ARM_GROUP);
      approach->setMinMaxDistance(0.005, 0.10);  
      approach->setIKFrame(HAND_FRAME);
      approach->properties().set("marker_ns", "approach");

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = BASE_FRAME; 
      direction.vector.z = -1.0;              // Move down
      approach->setDirection(direction);
      pick->insert(std::move(approach));
    }
    
    // 3b — Generate Grasp Pose (Dynamic Yaw for 5-DOF)
    {
      auto grasp_stage = std::make_unique<mtc::stages::GeneratePose>("generate grasp pose");

      double obj_x = 0.20;
      double obj_y = 0.00;
      double obj_z = 0.05;

      // 1. Calculate the required yaw FIRST
      double required_yaw = std::atan2(obj_y, obj_x);

      // 2. THE DEEP GRASP FIX:
      // 'offset_dist' is how far back the wrist stops from the center of the cylinder.
      // 0.02m (2cm) was barely touching. 0.00m puts the frame right in the center.
      // (You can safely tune this to 0.01 or even -0.01 if you need it deeper/shallower)
      double offset_dist = 0.00; 

      geometry_msgs::msg::PoseStamped grasp_pose;
      grasp_pose.header.frame_id = BASE_FRAME;
      
      // 3. Dynamically apply the offset along the approach angle
      grasp_pose.pose.position.x = obj_x - (offset_dist * std::cos(required_yaw));
      grasp_pose.pose.position.y = obj_y - (offset_dist * std::sin(required_yaw));
      grasp_pose.pose.position.z = obj_z;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.15, required_yaw); // Roll=0, Pitch=0.15, Yaw=Dynamic
      grasp_pose.pose.orientation = tf2::toMsg(q);

      grasp_stage->setPose(grasp_pose);
      grasp_stage->setMonitoredStage(current_state_ptr);

      auto ik = std::make_unique<mtc::stages::ComputeIK>("grasp IK", std::move(grasp_stage));
      ik->setMaxIKSolutions(20);
      ik->setTimeout(0.5);
      ik->setIKFrame(HAND_FRAME);
      ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      pick->insert(std::move(ik));
    }

    // 3c — Allow collision
    {
      auto allow_col = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (gripper,object)");
      allow_col->allowCollisions(
        OBJECT_ID,
        task.getRobotModel()
          ->getJointModelGroup(GRIPPER_GROUP)
          ->getLinkModelNamesWithCollisionGeometry(),
        true);
      pick->insert(std::move(allow_col));
    }

    // 3d — Close gripper
    {
      auto close = std::make_unique<mtc::stages::MoveTo>("close gripper", joint_interpolation);
      close->setGroup(GRIPPER_GROUP);
      close->setGoal(GRIPPER_CLOSE);
      pick->insert(std::move(close));
    }

    // 3e — Attach object
    {
      auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      attach_stage_ptr = attach.get();  
      attach->attachObject(OBJECT_ID, EEF_LINK);
      pick->insert(std::move(attach));
    }

    // 3f — Lift object 
    {
      auto lift = std::make_unique<mtc::stages::MoveRelative>("lift object", joint_interpolation);
      lift->setGroup(ARM_GROUP);
      lift->setMinMaxDistance(0.005, 0.10);
      lift->setIKFrame(HAND_FRAME);
      lift->properties().set("marker_ns", "lift");

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = BASE_FRAME;
      direction.vector.z = 1.0;   // Lift up
      lift->setDirection(direction);
      pick->insert(std::move(lift));
    }

    task.add(std::move(pick));
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  // Stage 4 — Connect (Move to place location)
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  {
    auto connect = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{
        {ARM_GROUP,     pipeline},
        {GRIPPER_GROUP, joint_interpolation}
      }
    );
    connect->setTimeout(10.0);
    connect->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(connect));
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  // Stage 5 — Place container 
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "hand", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                           {"eef", "hand", "group", "ik_frame"});

    // NEW: 5a — Approach Place (Hover, then move straight down)
    {
      auto approach_place = std::make_unique<mtc::stages::MoveRelative>("approach place", joint_interpolation);
      approach_place->setGroup(ARM_GROUP);
      approach_place->setMinMaxDistance(0.005, 0.10);
      approach_place->setIKFrame(HAND_FRAME);
      approach_place->properties().set("marker_ns", "approach_place");

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = BASE_FRAME;
      direction.vector.z = -1.0;  // Move down to place
      approach_place->setDirection(direction);
      place->insert(std::move(approach_place));
    }

    // 5b — Generate place pose (Dynamic Yaw for 5-DOF)
    {
      double place_x = 0.20;
      double place_y = 0.15; 
      
      // FIXED: Z=0.05 so the bottom of the 10cm cylinder rests perfectly on the floor
      double place_z = 0.05; 

      geometry_msgs::msg::PoseStamped place_pose;
      place_pose.header.frame_id = BASE_FRAME;
      place_pose.pose.position.x = place_x;
      place_pose.pose.position.y = place_y;   
      place_pose.pose.position.z = place_z;   

      double place_yaw = std::atan2(place_y, place_x);
      
      tf2::Quaternion q_place;
      q_place.setRPY(0.0, 0.15, place_yaw); 
      place_pose.pose.orientation = tf2::toMsg(q_place);

      auto place_stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      place_stage->properties().configureInitFrom(mtc::Stage::PARENT);
      place_stage->setObject(OBJECT_ID);
      place_stage->setPose(place_pose);
      place_stage->setMonitoredStage(attach_stage_ptr);

      auto place_ik = std::make_unique<mtc::stages::ComputeIK>("place IK", std::move(place_stage));
      place_ik->setMaxIKSolutions(20);
      place_ik->setTimeout(0.5);
      place_ik->setIKFrame(HAND_FRAME);
      place_ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      place_ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(place_ik));
    }

    // 5c — Open gripper 
    {
      auto open = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_interpolation);
      open->setGroup(GRIPPER_GROUP);
      open->setGoal(GRIPPER_OPEN);
      place->insert(std::move(open));
    }

    // 5d — Detach object
    {
      auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      detach->detachObject(OBJECT_ID, EEF_LINK);
      place->insert(std::move(detach));
    }

    // 5e — Restore collision
    {
      auto forbid_col = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "forbid collision (gripper,object)");
      forbid_col->allowCollisions(
        OBJECT_ID,
        task.getRobotModel()
          ->getJointModelGroup(GRIPPER_GROUP)
          ->getLinkModelNamesWithCollisionGeometry(),
        false);
      place->insert(std::move(forbid_col));
    }

    // 5f — Retreat (Must go straight UP for 5-DOF to keep Yaw math valid!)
    {
      auto retreat = std::make_unique<mtc::stages::MoveRelative>("retreat", joint_interpolation);
      retreat->setGroup(ARM_GROUP);
      retreat->setMinMaxDistance(0.01, 0.10);  
      retreat->setIKFrame(HAND_FRAME);
      retreat->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped direction;
      // CRITICAL: Retreat straight up in the world frame!
      direction.header.frame_id = BASE_FRAME; 
      direction.vector.z = 1.0;              
      retreat->setDirection(direction);
      place->insert(std::move(retreat));
    }
    task.add(std::move(place));
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  // Stage 6 — Return home
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  {
    auto home = std::make_unique<mtc::stages::MoveTo>("return home", pipeline);
    home->setGroup(ARM_GROUP);
    home->setGoal(ARM_HOME_STATE);
    task.add(std::move(home));
  }

  return task;
}

// ── Main ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("mtc_node", options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spin_thread = std::thread([&executor]() { executor.spin(); });

  rclcpp::sleep_for(std::chrono::seconds(2));

  spawnScene(node);
  rclcpp::sleep_for(std::chrono::seconds(1));

  auto task = buildTask(node);

  try {
    task.init();
  } catch (const mtc::InitStageException& e) {
    RCLCPP_ERROR(node->get_logger(), "Task init failed:");
    std::ostringstream oss;
    oss << e;
    RCLCPP_ERROR(node->get_logger(), "%s", oss.str().c_str());
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  if (!task.plan(10)) { 
    RCLCPP_ERROR(node->get_logger(), "Task planning failed");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Planning succeeded! Publishing solution for RViz...");
  task.introspection().publishSolution(*task.solutions().front());
  RCLCPP_INFO(node->get_logger(), "Solution published to RViz.");

  // Give controllers a moment to settle before executing
  rclcpp::sleep_for(std::chrono::seconds(2));

  // ── Execute via MoveIt's ExecuteTaskSolution action (through move_group) ──
  RCLCPP_INFO(node->get_logger(), "Executing solution via MoveIt...");
  auto result = task.execute(*task.solutions().front());
  if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Execution succeeded!");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Execution failed (MoveIt error code: %d)", result.val);
  }

  RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit the node.");

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}