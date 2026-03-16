#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cmath>
#include <set>
#include <mutex>
#include <random>

// MoveIt Task Constructor
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Custom service interfaces
#include <omx_interfaces/srv/pick.hpp>
#include <omx_interfaces/srv/place.hpp>
#include <omx_interfaces/srv/pour.hpp>
#include <omx_interfaces/srv/rotate.hpp>
#include <omx_interfaces/srv/go_home.hpp>
#include <omx_interfaces/srv/add_objects.hpp>
#include <omx_interfaces/srv/remove_objects.hpp>
#include <omx_interfaces/srv/get_objects.hpp>

namespace mtc = moveit::task_constructor;

// ── Robot constants ─────────────────────────────────────────────────────────
static const std::string ARM_GROUP     = "arm";
static const std::string GRIPPER_GROUP = "gripper";
static const std::string EEF_NAME      = "end_effector";
static const std::string EEF_LINK      = "end_effector_link";
static const std::string HAND_FRAME    = "end_effector_link";
static const std::string BASE_FRAME    = "world";
static const std::string CAMERA_FRAME  = "camera_color_optical_frame";
static const std::string GRIPPER_OPEN  = "open";
static const std::string GRIPPER_CLOSE = "close";
static const std::string ARM_HOME      = "home";

// ═══════════════════════════════════════════════════════════════════════════
// ManipulatorNode — service-based MTC action server
// ═══════════════════════════════════════════════════════════════════════════
class ManipulatorNode : public rclcpp::Node
{
public:
  ManipulatorNode(const rclcpp::NodeOptions& opts)
    : Node("manipulator_node", opts)
  {
    // Solvers are created lazily in initSolvers() because
    // PipelinePlanner needs shared_from_this() which is unavailable here.

    cartesian_ = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_->setStepSize(0.01);
    cartesian_->setMaxVelocityScalingFactor(0.025);
    cartesian_->setMaxAccelerationScalingFactor(0.025);

    joint_interp_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    joint_interp_->setMaxVelocityScalingFactor(0.025);
    joint_interp_->setMaxAccelerationScalingFactor(0.025);

    // Motion services
    pick_srv_ = create_service<omx_interfaces::srv::Pick>(
      "~/pick", std::bind(&ManipulatorNode::pickCb, this,
        std::placeholders::_1, std::placeholders::_2));

    place_srv_ = create_service<omx_interfaces::srv::Place>(
      "~/place", std::bind(&ManipulatorNode::placeCb, this,
        std::placeholders::_1, std::placeholders::_2));

    pour_srv_ = create_service<omx_interfaces::srv::Pour>(
      "~/pour", std::bind(&ManipulatorNode::pourCb, this,
        std::placeholders::_1, std::placeholders::_2));

    rotate_srv_ = create_service<omx_interfaces::srv::Rotate>(
      "~/rotate", std::bind(&ManipulatorNode::rotateCb, this,
        std::placeholders::_1, std::placeholders::_2));

    go_home_srv_ = create_service<omx_interfaces::srv::GoHome>(
      "~/go_home", std::bind(&ManipulatorNode::goHomeCb, this,
        std::placeholders::_1, std::placeholders::_2));

    // Scene services
    add_obj_srv_ = create_service<omx_interfaces::srv::AddObjects>(
      "~/add_objects", std::bind(&ManipulatorNode::addObjectsCb, this,
        std::placeholders::_1, std::placeholders::_2));

    remove_obj_srv_ = create_service<omx_interfaces::srv::RemoveObjects>(
      "~/remove_objects", std::bind(&ManipulatorNode::removeObjectsCb, this,
        std::placeholders::_1, std::placeholders::_2));

    get_obj_srv_ = create_service<omx_interfaces::srv::GetObjects>(
      "~/get_objects", std::bind(&ManipulatorNode::getObjectsCb, this,
        std::placeholders::_1, std::placeholders::_2));

    // TF2 for camera frame transforms
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "ManipulatorNode ready. Services advertised.");
  }

  /// Create a fresh PipelinePlanner (must be called per-task to match robot model)
  mtc::solvers::PipelinePlannerPtr makePipeline()
  {
    auto p = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    p->setProperty("goal_joint_tolerance", 1e-3);
    p->setProperty("max_velocity_scaling_factor", 0.025);
    p->setProperty("max_acceleration_scaling_factor", 0.025);
    return p;
  }

private:
  // ── Solvers (cartesian & joint_interp are stateless and can be shared;
  //    pipeline must be created per-task to match the task's robot model) ──
  mtc::solvers::CartesianPathPtr cartesian_;
  mtc::solvers::JointInterpolationPlannerPtr joint_interp_;

  // ── TF2 for frame transforms ────────────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── Task mutex (one MTC task at a time) ─────────────────────────────────
  std::mutex task_mutex_;

  // ── Scene tracker ───────────────────────────────────────────────────────
  struct TrackedObject {
    std::string id;
    std::string type;
    geometry_msgs::msg::PoseStamped pose;
    std::vector<double> dimensions;
  };
  std::mutex scene_mutex_;
  std::map<std::string, TrackedObject> tracked_objects_;
  std::string attached_object_id_;   // currently grasped object

  // ── Helpers ─────────────────────────────────────────────────────────────
  // Extract height from tracked object dimensions
  //   Cylinder: dims = [height, radius]
  //   Box:      dims = [dx, dy, dz]  → height = dz
  //   Sphere:   dims = [radius]      → height = 2*radius
  static double objectHeight(const TrackedObject& obj)
  {
    const auto& d = obj.dimensions;
    if (obj.type == "cylinder" && d.size() >= 1) return d[0];
    if (obj.type == "box"      && d.size() >= 3) return d[2];
    if (obj.type == "sphere"   && d.size() >= 1) return 2.0 * d[0];
    return 0.0;
  }

  // Extract grip width (diameter) from tracked object dimensions
  //   Cylinder: dims = [height, radius] → width = 2*radius
  //   Box:      dims = [dx, dy, dz]     → width = min(dx, dy)
  //   Sphere:   dims = [radius]          → width = 2*radius
  static double objectWidth(const TrackedObject& obj)
  {
    const auto& d = obj.dimensions;
    if (obj.type == "cylinder" && d.size() >= 2) return 2.0 * d[1];
    if (obj.type == "box"      && d.size() >= 2) return std::min(d[0], d[1]);
    if (obj.type == "sphere"   && d.size() >= 1) return 2.0 * d[0];
    return 0.0;
  }

  // ── Service handles ─────────────────────────────────────────────────────
  rclcpp::Service<omx_interfaces::srv::Pick>::SharedPtr pick_srv_;
  rclcpp::Service<omx_interfaces::srv::Place>::SharedPtr place_srv_;
  rclcpp::Service<omx_interfaces::srv::Pour>::SharedPtr pour_srv_;
  rclcpp::Service<omx_interfaces::srv::Rotate>::SharedPtr rotate_srv_;
  rclcpp::Service<omx_interfaces::srv::GoHome>::SharedPtr go_home_srv_;
  rclcpp::Service<omx_interfaces::srv::AddObjects>::SharedPtr add_obj_srv_;
  rclcpp::Service<omx_interfaces::srv::RemoveObjects>::SharedPtr remove_obj_srv_;
  rclcpp::Service<omx_interfaces::srv::GetObjects>::SharedPtr get_obj_srv_;

  // ════════════════════════════════════════════════════════════════════════
  //  Helper: transform POSITION ONLY to world frame via TF2
  //  Orientation is NOT transformed — it is determined by object dimensions
  //  and the grasp logic (graspOrientation), not by the camera frame.
  // ════════════════════════════════════════════════════════════════════════
  bool transformToWorld(geometry_msgs::msg::PoseStamped& pose, std::string& err_msg)
  {
    if (pose.header.frame_id.empty() || pose.header.frame_id == BASE_FRAME) {
      pose.header.frame_id = BASE_FRAME;
      return true;
    }
    try {
      // Transform only the position (as a PointStamped)
      geometry_msgs::msg::PointStamped pt_in;
      pt_in.header = pose.header;
      pt_in.point = pose.pose.position;

      auto pt_out = tf_buffer_->transform(pt_in, BASE_FRAME,
                                           tf2::durationFromSec(5.0));

      RCLCPP_INFO(get_logger(),
                  "Transformed position from '%s' to '%s': [%.3f, %.3f, %.3f]",
                  pose.header.frame_id.c_str(), BASE_FRAME.c_str(),
                  pt_out.point.x, pt_out.point.y, pt_out.point.z);

      // Rebuild pose: transformed position + identity orientation
      pose.header.frame_id = BASE_FRAME;
      pose.pose.position = pt_out.point;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      return true;
    } catch (const tf2::TransformException& e) {
      err_msg = std::string("TF2 transform failed: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", err_msg.c_str());
      return false;
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  //  Helper: setup common task properties
  // ════════════════════════════════════════════════════════════════════════
  void setupTask(mtc::Task& task, const std::string& name)
  {
    task.stages()->setName(name);
    task.loadRobotModel(shared_from_this());
    task.setProperty("group",              ARM_GROUP);
    task.setProperty("eef",                EEF_NAME);
    task.setProperty("hand",               GRIPPER_GROUP);
    task.setProperty("hand_grasping_frame", HAND_FRAME);
    task.setProperty("ik_frame",           HAND_FRAME);
  }

  // ════════════════════════════════════════════════════════════════════════
  //  Helper: plan + execute a task
  // ════════════════════════════════════════════════════════════════════════
  bool planAndExecute(mtc::Task& task, std::string& err_msg, int max_plans = 10)
  {
    try { task.init(); }
    catch (const mtc::InitStageException& e) {
      std::ostringstream oss; oss << e;
      err_msg = "Task init failed: " + oss.str();
      RCLCPP_ERROR(get_logger(), "%s", err_msg.c_str());
      return false;
    }

    if (!task.plan(max_plans)) {
      // Collect per-stage failure details
      std::ostringstream details;
      task.explainFailure(details);
      err_msg = "Planning failed:\n" + details.str();
      RCLCPP_ERROR(get_logger(), "%s", err_msg.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "Planning succeeded. Executing...");
    auto result = task.execute(*task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      err_msg = "Execution failed (code: " + std::to_string(result.val) + ")";
      RCLCPP_ERROR(get_logger(), "%s", err_msg.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Execution succeeded.");
    return true;
  }

  // ════════════════════════════════════════════════════════════════════════
  //  Helper: compute 5-DOF grasp orientation toward an XY position
  // ════════════════════════════════════════════════════════════════════════
  geometry_msgs::msg::Quaternion graspOrientation(double x, double y)
  {
    double yaw = std::atan2(y, x);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.15, yaw);  // slight pitch for stable grasp
    return tf2::toMsg(q);
  }

  // ════════════════════════════════════════════════════════════════════════
  //  PICK service
  // ════════════════════════════════════════════════════════════════════════
  void pickCb(const omx_interfaces::srv::Pick::Request::SharedPtr req,
              omx_interfaces::srv::Pick::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    RCLCPP_INFO(get_logger(), "Pick requested: '%s'", req->object_id.c_str());

    if (!attached_object_id_.empty()) {
      res->success = false;
      res->message = "Already holding object '" + attached_object_id_ + "'. Place it first.";
      return;
    }

    // Look up object pose and dimensions from tracker
    geometry_msgs::msg::PoseStamped obj_pose;
    double obj_height = 0.0;
    double obj_width  = 0.0;
    {
      std::lock_guard<std::mutex> sl(scene_mutex_);
      auto it = tracked_objects_.find(req->object_id);
      if (it == tracked_objects_.end()) {
        res->success = false;
        res->message = "Object '" + req->object_id + "' not found in scene.";
        return;
      }
      obj_pose = it->second.pose;
      obj_height = objectHeight(it->second);
      obj_width  = objectWidth(it->second);
    }

    double ox = obj_pose.pose.position.x;
    double oy = obj_pose.pose.position.y;
    // Grasp at the object centroid — gives maximum grip contact and keeps
    // the place height consistent (no gripper-to-center offset to manage)
    double oz = obj_pose.pose.position.z;
    RCLCPP_INFO(get_logger(), "Grasp at centroid: center_z=%.3f, obj_height=%.3f",
                oz, obj_height);

    // Build pick task
    auto pipeline = makePipeline();
    mtc::Task task;
    setupTask(task, "pick_" + req->object_id);

    // Current State
    mtc::Stage* current_state_ptr = nullptr;
    {
      auto s = std::make_unique<mtc::stages::CurrentState>("current state");
      current_state_ptr = s.get();
      task.add(std::move(s));
    }

    // Open gripper
    {
      auto s = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_interp_);
      s->setGroup(GRIPPER_GROUP);
      s->setGoal(GRIPPER_OPEN);
      task.add(std::move(s));
    }

    // Connect to pre-grasp (arm only — gripper stays fully open)
    {
      auto s = std::make_unique<mtc::stages::Connect>("move to pick",
        mtc::stages::Connect::GroupPlannerVector{{ARM_GROUP, pipeline}});
      s->setTimeout(10.0);
      s->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(s));
    }

    // Pick container
    {
      auto pick = std::make_unique<mtc::SerialContainer>("pick object");
      task.properties().exposeTo(pick->properties(), {"eef", "hand", "group", "ik_frame"});
      pick->properties().configureInitFrom(mtc::Stage::PARENT,
                                            {"eef", "hand", "group", "ik_frame"});

      // Zero wrist roll before approach so object stays vertical
      {
        auto s = std::make_unique<mtc::stages::MoveTo>("zero wrist roll", joint_interp_);
        s->setGroup(ARM_GROUP);
        std::map<std::string, double> joints;
        joints["joint5_roll"] = 0.0;
        s->setGoal(joints);
        pick->insert(std::move(s));
      }

      // Approach
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>("approach object", joint_interp_);
        s->setGroup(ARM_GROUP);
        s->setMinMaxDistance(0.005, 0.10);
        s->setIKFrame(HAND_FRAME);
        geometry_msgs::msg::Vector3Stamped dir;
        dir.header.frame_id = BASE_FRAME;
        dir.vector.z = -1.0;
        s->setDirection(dir);
        pick->insert(std::move(s));
      }

      // Grasp pose
      {
        auto gen = std::make_unique<mtc::stages::GeneratePose>("generate grasp pose");
        geometry_msgs::msg::PoseStamped grasp_pose;
        grasp_pose.header.frame_id = BASE_FRAME;
        grasp_pose.pose.position.x = ox;
        grasp_pose.pose.position.y = oy;
        grasp_pose.pose.position.z = oz;
        grasp_pose.pose.orientation = graspOrientation(ox, oy);
        gen->setPose(grasp_pose);
        gen->setMonitoredStage(current_state_ptr);

        auto ik = std::make_unique<mtc::stages::ComputeIK>("grasp IK", std::move(gen));
        ik->setMaxIKSolutions(20);
        ik->setTimeout(0.5);
        ik->setIKFrame(HAND_FRAME);
        ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        pick->insert(std::move(ik));
      }

      // Allow collision
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
        s->allowCollisions(req->object_id,
          task.getRobotModel()->getJointModelGroup(GRIPPER_GROUP)
            ->getLinkModelNamesWithCollisionGeometry(), true);
        pick->insert(std::move(s));
      }

      // Close gripper to object width + 5% extra squeeze
      {
        auto s = std::make_unique<mtc::stages::MoveTo>("close gripper", joint_interp_);
        s->setGroup(GRIPPER_GROUP);
        // Gripper gap = 0.042 + 2*joint_val  →  joint_val = (gap - 0.042)/2
        // Target gap = object_width * 0.95 (5% tighter for sturdy grip)
        double target_gap = obj_width * 0.95;
        double grip_joint = (target_gap - 0.042) / 2.0;
        // Clamp to joint limits [-0.011, 0.02]
        grip_joint = std::clamp(grip_joint, -0.011, 0.02);
        RCLCPP_INFO(get_logger(),
          "Gripper: obj_width=%.4f target_gap=%.4f joint=%.4f",
          obj_width, target_gap, grip_joint);
        std::map<std::string, double> grip_goal;
        grip_goal["gripper_left_joint"] = grip_joint;
        s->setGoal(grip_goal);
        pick->insert(std::move(s));
      }

      // Attach object
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
        s->attachObject(req->object_id, EEF_LINK);
        pick->insert(std::move(s));
      }

      // Lift
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>("lift object", joint_interp_);
        s->setGroup(ARM_GROUP);
        s->setMinMaxDistance(0.005, 0.10);
        s->setIKFrame(HAND_FRAME);
        geometry_msgs::msg::Vector3Stamped dir;
        dir.header.frame_id = BASE_FRAME;
        dir.vector.z = 1.0;
        s->setDirection(dir);
        pick->insert(std::move(s));
      }

      task.add(std::move(pick));
    }

    std::string err;
    if (planAndExecute(task, err)) {
      attached_object_id_ = req->object_id;
      res->success = true;
      res->message = "Picked '" + req->object_id + "'";
    } else {
      res->success = false;
      res->message = err;
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  //  PLACE service
  // ════════════════════════════════════════════════════════════════════════
  void placeCb(const omx_interfaces::srv::Place::Request::SharedPtr req,
               omx_interfaces::srv::Place::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    RCLCPP_INFO(get_logger(), "Place requested.");

    if (attached_object_id_.empty()) {
      res->success = false;
      res->message = "No object currently held.";
      return;
    }

    const std::string obj_id = attached_object_id_;

    // Retrieve object height for place Z computation
    double half_h = 0.0;
    {
      std::lock_guard<std::mutex> sl(scene_mutex_);
      auto it = tracked_objects_.find(obj_id);
      if (it != tracked_objects_.end()) {
        half_h = objectHeight(it->second) / 2.0;
      }
    }

    // Determine place pose
    geometry_msgs::msg::PoseStamped place_pose;
    if (req->use_target_pose) {
      place_pose = req->target_pose;
      if (req->use_camera_frame) {
        place_pose.header.frame_id = CAMERA_FRAME;
      }
      std::string tf_err;
      if (!transformToWorld(place_pose, tf_err)) {
        res->success = false;
        res->message = tf_err;
        return;
      }
    } else {
      // Choose the closest reachable position: sample a few candidates
      // and pick the one nearest to the current EEF position.
      // Current EEF position is approximated from the object's tracked pose.
      double cur_x = 0.0, cur_y = 0.0;
      {
        std::lock_guard<std::mutex> sl(scene_mutex_);
        auto it = tracked_objects_.find(obj_id);
        if (it != tracked_objects_.end()) {
          cur_x = it->second.pose.pose.position.x;
          cur_y = it->second.pose.pose.position.y;
        }
      }

      std::mt19937 rng(std::random_device{}());
      std::uniform_real_distribution<double> r_dist(0.15, 0.22);
      std::uniform_real_distribution<double> a_dist(-1.2, 1.2);

      double best_x = 0.0, best_y = 0.0, best_dist = 1e9;
      for (int i = 0; i < 10; ++i) {
        double r = r_dist(rng);
        double a = a_dist(rng);
        double cx = r * std::cos(a);
        double cy = r * std::sin(a);
        double d = std::hypot(cx - cur_x, cy - cur_y);
        if (d < best_dist) {
          best_dist = d;
          best_x = cx;
          best_y = cy;
        }
      }

      place_pose.header.frame_id = BASE_FRAME;
      place_pose.pose.position.x = best_x;
      place_pose.pose.position.y = best_y;
      place_pose.pose.position.z = 0.0;
      place_pose.pose.orientation.w = 1.0;
    }

    double px = place_pose.pose.position.x;
    double py = place_pose.pose.position.y;
    // EEF target Z = half object height + small gap (3mm) above table
    double pz = half_h + 0.003;

    RCLCPP_INFO(get_logger(),
      "Place: obj='%s' px=%.3f py=%.3f pz=%.3f half_h=%.4f",
      obj_id.c_str(), px, py, pz, half_h);

    // ── Place task — mirrors pick exactly ──
    // Pick:  CurrentState → Open → Connect → Container{ Approach↓ → GeneratePose+IK → Allow → Close → Attach → Lift↑ }
    // Place: CurrentState →        Connect → Container{ Lower↓   → GeneratePose+IK → Open  → Forbid → Detach → Retreat↑ }
    auto pipeline = makePipeline();
    mtc::Task task;
    setupTask(task, "place_" + obj_id);

    // Current state
    mtc::Stage* current_state_ptr = nullptr;
    {
      auto s = std::make_unique<mtc::stages::CurrentState>("current state");
      current_state_ptr = s.get();
      task.add(std::move(s));
    }

    // Connect — bridges current (holding object) to the place container
    {
      auto s = std::make_unique<mtc::stages::Connect>("move to place",
        mtc::stages::Connect::GroupPlannerVector{
          {ARM_GROUP, pipeline}, {GRIPPER_GROUP, joint_interp_}});
      s->setTimeout(10.0);
      s->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(s));
    }

    // Place container — exact mirror of pick container
    {
      auto place = std::make_unique<mtc::SerialContainer>("place object");
      task.properties().exposeTo(place->properties(), {"eef", "hand", "group", "ik_frame"});
      place->properties().configureInitFrom(mtc::Stage::PARENT,
                                             {"eef", "hand", "group", "ik_frame"});

      // Zero wrist roll before lowering so object is vertical at release
      {
        auto s = std::make_unique<mtc::stages::MoveTo>("zero wrist roll", joint_interp_);
        s->setGroup(ARM_GROUP);
        std::map<std::string, double> joints;
        joints["joint5_roll"] = 0.0;
        s->setGoal(joints);
        place->insert(std::move(s));
      }

      // Lower — mirrors pick's "approach" (MoveRelative down, same solver)
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>("lower to place", joint_interp_);
        s->setGroup(ARM_GROUP);
        s->setMinMaxDistance(0.005, 0.10);
        s->setIKFrame(HAND_FRAME);
        geometry_msgs::msg::Vector3Stamped dir;
        dir.header.frame_id = BASE_FRAME;
        dir.vector.z = -1.0;
        s->setDirection(dir);
        place->insert(std::move(s));
      }

      // Place pose — same pattern as pick's GeneratePose+ComputeIK
      {
        auto gen = std::make_unique<mtc::stages::GeneratePose>("generate place pose");
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = BASE_FRAME;
        target.pose.position.x = px;
        target.pose.position.y = py;
        target.pose.position.z = pz;
        // Zero pitch for place — the pick pitch (0.15) tilts the object
        // relative to the EEF; placing with 0 pitch keeps the object vertical
        {
          double yaw = std::atan2(py, px);
          tf2::Quaternion q;
          q.setRPY(0.0, 0.0, yaw);
          target.pose.orientation = tf2::toMsg(q);
        }
        gen->setPose(target);
        gen->setMonitoredStage(current_state_ptr);

        auto ik = std::make_unique<mtc::stages::ComputeIK>("place IK", std::move(gen));
        ik->setMaxIKSolutions(20);
        ik->setTimeout(0.5);
        ik->setIKFrame(HAND_FRAME);
        ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        place->insert(std::move(ik));
      }

      // Open gripper — mirrors pick's "close gripper"
      {
        auto s = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_interp_);
        s->setGroup(GRIPPER_GROUP);
        s->setGoal(GRIPPER_OPEN);
        place->insert(std::move(s));
      }

      // Forbid collision — mirrors pick's "allow collision"
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision");
        s->allowCollisions(obj_id,
          task.getRobotModel()->getJointModelGroup(GRIPPER_GROUP)
            ->getLinkModelNamesWithCollisionGeometry(), false);
        place->insert(std::move(s));
      }

      // Detach object — mirrors pick's "attach object"
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
        s->detachObject(obj_id, EEF_LINK);
        place->insert(std::move(s));
      }

      // Retreat up — mirrors pick's "lift"
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>("retreat", joint_interp_);
        s->setGroup(ARM_GROUP);
        s->setMinMaxDistance(0.005, 0.10);
        s->setIKFrame(HAND_FRAME);
        geometry_msgs::msg::Vector3Stamped dir;
        dir.header.frame_id = BASE_FRAME;
        dir.vector.z = 1.0;
        s->setDirection(dir);
        place->insert(std::move(s));
      }

      task.add(std::move(place));
    }

    std::string err;
    if (planAndExecute(task, err)) {
      {
        std::lock_guard<std::mutex> sl(scene_mutex_);
        auto it = tracked_objects_.find(obj_id);
        if (it != tracked_objects_.end()) {
          // Update tracked pose with correct centroid Z so next pick
          // grasps at the centroid, not the ground.
          it->second.pose = place_pose;
          it->second.pose.pose.position.z = pz;
        }
      }
      attached_object_id_.clear();
      res->success = true;
      res->message = "Placed '" + obj_id + "'";
    } else {
      res->success = false;
      res->message = err;
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  //  POUR service
  // ════════════════════════════════════════════════════════════════════════
  void pourCb(const omx_interfaces::srv::Pour::Request::SharedPtr req,
              omx_interfaces::srv::Pour::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    RCLCPP_INFO(get_logger(), "Pour requested into '%s'", req->target_object_id.c_str());

    if (attached_object_id_.empty()) {
      res->success = false;
      res->message = "No object currently held.";
      return;
    }

    // Look up held object height
    double held_height = 0.0;
    {
      std::lock_guard<std::mutex> sl(scene_mutex_);
      auto it = tracked_objects_.find(attached_object_id_);
      if (it != tracked_objects_.end()) {
        held_height = objectHeight(it->second);
      }
    }

    // Look up target object position and height
    geometry_msgs::msg::PoseStamped target_pose;
    double target_height = 0.0;
    {
      std::lock_guard<std::mutex> sl(scene_mutex_);
      auto it = tracked_objects_.find(req->target_object_id);
      if (it == tracked_objects_.end()) {
        res->success = false;
        res->message = "Target '" + req->target_object_id + "' not in scene.";
        return;
      }
      target_pose = it->second.pose;
      target_height = objectHeight(it->second);
    }

    double tx = target_pose.pose.position.x;
    double ty = target_pose.pose.position.y;
    double tz = target_pose.pose.position.z;
    double pour_angle = (req->pour_angle > 0.01) ? req->pour_angle : 2.0;

    // Compute pour position:
    //   target top  = tz + target_height / 2
    //   held tip    = eef_z - held_height / 2   (bottom of held object)
    //   want: held tip = target top + 0.05      (5 cm gap)
    //   => eef_z = target top + 0.05 + held_height / 2
    double target_top_z = tz + target_height / 2.0;
    double pour_z = target_top_z + 0.05 + held_height / 2.0;

    RCLCPP_INFO(get_logger(),
      "Pour: held_h=%.3f target_top=%.3f pour_z=%.3f",
      held_height, target_top_z, pour_z);

    // Build pour task
    auto pipeline = makePipeline();
    mtc::Task task;
    setupTask(task, "pour");

    // Current state
    mtc::Stage* current_state_ptr = nullptr;
    {
      auto s = std::make_unique<mtc::stages::CurrentState>("current state");
      current_state_ptr = s.get();
      task.add(std::move(s));
    }

    // Connect — bridges current state to the pour container
    {
      auto s = std::make_unique<mtc::stages::Connect>("move to pour",
        mtc::stages::Connect::GroupPlannerVector{{ARM_GROUP, pipeline}});
      s->setTimeout(10.0);
      s->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(s));
    }

    // Pour container
    {
      auto pour = std::make_unique<mtc::SerialContainer>("pour sequence");
      task.properties().exposeTo(pour->properties(), {"eef", "hand", "group", "ik_frame"});
      pour->properties().configureInitFrom(mtc::Stage::PARENT,
                                            {"eef", "hand", "group", "ik_frame"});

      // Pour pose — EEF directly above target, held object tip 5 cm above target top
      {
        auto gen = std::make_unique<mtc::stages::GeneratePose>("generate pour pose");
        geometry_msgs::msg::PoseStamped pour_pose;
        pour_pose.header.frame_id = BASE_FRAME;
        pour_pose.pose.position.x = tx;
        pour_pose.pose.position.y = ty;
        pour_pose.pose.position.z = pour_z;
        pour_pose.pose.orientation = graspOrientation(tx, ty);
        gen->setPose(pour_pose);
        gen->setMonitoredStage(current_state_ptr);

        auto ik = std::make_unique<mtc::stages::ComputeIK>("pour IK", std::move(gen));
        ik->setMaxIKSolutions(20);
        ik->setTimeout(0.5);
        ik->setIKFrame(HAND_FRAME);
        ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        pour->insert(std::move(ik));
      }

      // Tilt wrist to pour
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>("pour tilt", joint_interp_);
        s->setGroup(ARM_GROUP);
        std::map<std::string, double> joint_deltas;
        joint_deltas["joint5_roll"] = pour_angle;
        s->setDirection(joint_deltas);
        pour->insert(std::move(s));
      }

      // Untilt wrist
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>("pour untilt", joint_interp_);
        s->setGroup(ARM_GROUP);
        std::map<std::string, double> joint_deltas;
        joint_deltas["joint5_roll"] = -pour_angle;
        s->setDirection(joint_deltas);
        pour->insert(std::move(s));
      }

      task.add(std::move(pour));
    }

    std::string err;
    if (planAndExecute(task, err)) {
      res->success = true;
      res->message = "Pour completed";
    } else {
      res->success = false;
      res->message = err;
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  //  ROTATE / MIX service
  // ════════════════════════════════════════════════════════════════════════
  void rotateCb(const omx_interfaces::srv::Rotate::Request::SharedPtr req,
                omx_interfaces::srv::Rotate::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    RCLCPP_INFO(get_logger(), "Rotate requested: angle=%.2f, cycles=%d",
                req->angle, req->cycles);

    double angle = (std::abs(req->angle) > 0.01) ? req->angle : M_PI;
    int cycles = std::max(req->cycles, 0);

    mtc::Task task;
    setupTask(task, "rotate");

    {
      auto s = std::make_unique<mtc::stages::CurrentState>("current state");
      task.add(std::move(s));
    }

    if (cycles == 0) {
      // Single rotation
      auto s = std::make_unique<mtc::stages::MoveRelative>("rotate", joint_interp_);
      s->setGroup(ARM_GROUP);
      std::map<std::string, double> deltas;
      deltas["joint5_roll"] = angle;
      s->setDirection(deltas);
      task.add(std::move(s));
    } else {
      // Oscillate back and forth
      for (int i = 0; i < cycles; ++i) {
        {
          auto s = std::make_unique<mtc::stages::MoveRelative>(
            "rotate_fwd_" + std::to_string(i), joint_interp_);
          s->setGroup(ARM_GROUP);
          std::map<std::string, double> d;
          d["joint5_roll"] = angle;
          s->setDirection(d);
          task.add(std::move(s));
        }
        {
          auto s = std::make_unique<mtc::stages::MoveRelative>(
            "rotate_rev_" + std::to_string(i), joint_interp_);
          s->setGroup(ARM_GROUP);
          std::map<std::string, double> d;
          d["joint5_roll"] = -angle;
          s->setDirection(d);
          task.add(std::move(s));
        }
      }
    }

    std::string err;
    if (planAndExecute(task, err)) {
      res->success = true;
      res->message = "Rotation completed";
    } else {
      res->success = false;
      res->message = err;
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  //  GO HOME service
  // ════════════════════════════════════════════════════════════════════════
  void goHomeCb(const omx_interfaces::srv::GoHome::Request::SharedPtr req,
                omx_interfaces::srv::GoHome::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    std::string state = req->state_name.empty() ? ARM_HOME : req->state_name;
    RCLCPP_INFO(get_logger(), "GoHome requested: '%s'", state.c_str());

    auto pipeline = makePipeline();
    mtc::Task task;
    setupTask(task, "go_home");

    {
      auto s = std::make_unique<mtc::stages::CurrentState>("current state");
      task.add(std::move(s));
    }
    {
      auto s = std::make_unique<mtc::stages::MoveTo>("go to " + state, pipeline);
      s->setGroup(ARM_GROUP);
      s->setGoal(state);
      task.add(std::move(s));
    }

    std::string err;
    if (planAndExecute(task, err)) {
      res->success = true;
      res->message = "Reached '" + state + "'";
    } else {
      res->success = false;
      res->message = err;
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  //  ADD OBJECTS service
  // ════════════════════════════════════════════════════════════════════════
  void addObjectsCb(const omx_interfaces::srv::AddObjects::Request::SharedPtr req,
                    omx_interfaces::srv::AddObjects::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> sl(scene_mutex_);

    moveit::planning_interface::PlanningSceneInterface psi;
    std::vector<moveit_msgs::msg::CollisionObject> cos;

    for (auto obj : req->objects) {
      // If use_camera_frame flag is set, override frame_id to camera frame
      if (obj.use_camera_frame) {
        obj.pose.header.frame_id = CAMERA_FRAME;
      }
      // Transform to world frame if needed
      std::string tf_err;
      if (!transformToWorld(obj.pose, tf_err)) {
        res->success = false;
        res->message = "Object '" + obj.id + "': " + tf_err;
        return;
      }

      moveit_msgs::msg::CollisionObject co;
      co.id = obj.id;
      co.header.frame_id = BASE_FRAME;
      co.operation = moveit_msgs::msg::CollisionObject::ADD;

      shape_msgs::msg::SolidPrimitive prim;
      if (obj.type == "box") {
        prim.type = shape_msgs::msg::SolidPrimitive::BOX;
      } else if (obj.type == "cylinder") {
        prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      } else if (obj.type == "sphere") {
        prim.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      } else {
        res->success = false;
        res->message = "Unknown type '" + obj.type + "' for object '" + obj.id + "'.";
        return;
      }
      prim.dimensions.assign(obj.dimensions.begin(), obj.dimensions.end());
      co.primitives.push_back(prim);

      // Clamp z so the object bottom never sinks below the table (z=0).
      // Compute half-height from dimensions, then ensure centroid z >= half_h.
      {
        double half_h = 0.0;
        const auto& d = obj.dimensions;
        if (obj.type == "cylinder" && d.size() >= 1) half_h = d[0] / 2.0;
        else if (obj.type == "box" && d.size() >= 3) half_h = d[2] / 2.0;
        else if (obj.type == "sphere" && d.size() >= 1) half_h = d[0];
        if (obj.pose.pose.position.z < half_h) {
          RCLCPP_WARN(get_logger(),
            "Object '%s' centroid z=%.3f is below table surface (half_h=%.3f). "
            "Clamping to z=%.3f so it sits on the table.",
            obj.id.c_str(), obj.pose.pose.position.z, half_h, half_h);
          obj.pose.pose.position.z = half_h;
        }
      }

      co.primitive_poses.push_back(obj.pose.pose);
      cos.push_back(co);

      TrackedObject to;
      to.id = obj.id;
      to.type = obj.type;
      to.pose = obj.pose;
      to.dimensions = std::vector<double>(obj.dimensions.begin(), obj.dimensions.end());
      tracked_objects_[obj.id] = to;
    }

    psi.applyCollisionObjects(cos);

    size_t n = req->objects.size();
    res->success = true;
    res->message = "Added " + std::to_string(n) + " object(s).";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  // ════════════════════════════════════════════════════════════════════════
  //  REMOVE OBJECTS service
  // ════════════════════════════════════════════════════════════════════════
  void removeObjectsCb(const omx_interfaces::srv::RemoveObjects::Request::SharedPtr req,
                       omx_interfaces::srv::RemoveObjects::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> sl(scene_mutex_);
    moveit::planning_interface::PlanningSceneInterface psi;

    std::vector<std::string> to_remove;
    if (req->clear_all) {
      for (auto& [id, _] : tracked_objects_)
        to_remove.push_back(id);
    } else {
      to_remove = std::vector<std::string>(req->object_ids.begin(), req->object_ids.end());
    }

    std::vector<moveit_msgs::msg::CollisionObject> cos;
    for (const auto& id : to_remove) {
      moveit_msgs::msg::CollisionObject co;
      co.id = id;
      co.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      cos.push_back(co);
      tracked_objects_.erase(id);
    }

    if (!cos.empty())
      psi.applyCollisionObjects(cos);

    res->success = true;
    res->message = "Removed " + std::to_string(to_remove.size()) + " object(s).";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  // ════════════════════════════════════════════════════════════════════════
  //  GET OBJECTS service
  // ════════════════════════════════════════════════════════════════════════
  void getObjectsCb(const omx_interfaces::srv::GetObjects::Request::SharedPtr /*req*/,
                    omx_interfaces::srv::GetObjects::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> sl(scene_mutex_);
    for (auto& [id, obj] : tracked_objects_) {
      res->object_ids.push_back(id);
      res->poses.push_back(obj.pose);
      res->object_types.push_back(obj.type);
    }
  }
};

// ── Main ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<ManipulatorNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
