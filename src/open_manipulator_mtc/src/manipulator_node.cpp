#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cmath>
#include <set>
#include <limits>
#include <mutex>
#include <random>

// MoveIt Task Constructor
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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
static constexpr double PLACE_BASE_CLEARANCE = 0.015;

// ═══════════════════════════════════════════════════════════════════════════
// ManipulatorNode
// ═══════════════════════════════════════════════════════════════════════════
class ManipulatorNode : public rclcpp::Node
{
public:
  ManipulatorNode(const rclcpp::NodeOptions& opts)
    : Node("manipulator_node", opts)
  {
    cartesian_ = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_->setStepSize(0.01);
    cartesian_->setMaxVelocityScalingFactor(0.025);
    cartesian_->setMaxAccelerationScalingFactor(0.025);

    joint_interp_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    joint_interp_->setMaxVelocityScalingFactor(0.025);
    joint_interp_->setMaxAccelerationScalingFactor(0.025);

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
    add_obj_srv_ = create_service<omx_interfaces::srv::AddObjects>(
      "~/add_objects", std::bind(&ManipulatorNode::addObjectsCb, this,
        std::placeholders::_1, std::placeholders::_2));
    remove_obj_srv_ = create_service<omx_interfaces::srv::RemoveObjects>(
      "~/remove_objects", std::bind(&ManipulatorNode::removeObjectsCb, this,
        std::placeholders::_1, std::placeholders::_2));
    get_obj_srv_ = create_service<omx_interfaces::srv::GetObjects>(
      "~/get_objects", std::bind(&ManipulatorNode::getObjectsCb, this,
        std::placeholders::_1, std::placeholders::_2));

    // Track latest arm joint values
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 20,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(joint_state_mutex_);
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
          const auto& n = msg->name[i];
          if (n == "joint1" || n == "joint2" || n == "joint3" ||
              n == "joint4" || n == "joint5_roll") {
            latest_arm_joints_[n] = msg->position[i];
          }
        }
      });

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "ManipulatorNode ready.");
  }

  mtc::solvers::PipelinePlannerPtr makePipeline()
  {
    auto p = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    p->setProperty("goal_joint_tolerance",             1e-3);
    p->setProperty("max_velocity_scaling_factor",      0.025);
    p->setProperty("max_acceleration_scaling_factor",  0.025);
    return p;
  }

private:
  mtc::solvers::CartesianPathPtr             cartesian_;
  mtc::solvers::JointInterpolationPlannerPtr joint_interp_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex task_mutex_;

  // ── Scene ────────────────────────────────────────────────────────────────
  struct TrackedObject {
    std::string id, type;
    geometry_msgs::msg::PoseStamped pose;
    std::vector<double> dimensions;
  };
  std::mutex scene_mutex_;
  std::map<std::string, TrackedObject> tracked_objects_;
  std::string attached_object_id_;

  // ── Pick memory ──────────────────────────────────────────────────────────
  struct LastPickInfo {
    bool valid            = false;
    std::string object_id;
    geometry_msgs::msg::PoseStamped grasp_pose;
    bool have_arm_joints  = false;
    std::map<std::string, double> approach_joints;
    double obj_half_h = 0.0;
  };
  LastPickInfo last_pick_;

  // ── Joint state subscription ─────────────────────────────────────────────
  std::mutex joint_state_mutex_;
  std::map<std::string, double> latest_arm_joints_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // ── Helpers ──────────────────────────────────────────────────────────────
  static double objectHeight(const TrackedObject& obj)
  {
    const auto& d = obj.dimensions;
    if (obj.type == "cylinder" && d.size() >= 1) return d[0];
    if (obj.type == "box"      && d.size() >= 3) return d[2];
    if (obj.type == "sphere"   && d.size() >= 1) return 2.0 * d[0];
    return 0.0;
  }
  static double objectWidth(const TrackedObject& obj)
  {
    const auto& d = obj.dimensions;
    if (obj.type == "cylinder" && d.size() >= 2) return 2.0 * d[1];
    if (obj.type == "box"      && d.size() >= 2) return std::min(d[0], d[1]);
    if (obj.type == "sphere"   && d.size() >= 1) return 2.0 * d[0];
    return 0.0;
  }

  static geometry_msgs::msg::PoseStamped rotatePoseAroundWorldZ(
    const geometry_msgs::msg::PoseStamped& in, double delta)
  {
    geometry_msgs::msg::PoseStamped out = in;
    const double c = std::cos(delta), s = std::sin(delta);
    const double x = in.pose.position.x, y = in.pose.position.y;
    out.pose.position.x = c * x - s * y;
    out.pose.position.y = s * x + c * y;
    tf2::Quaternion q_in, q_delta;
    tf2::fromMsg(in.pose.orientation, q_in);
    q_delta.setRPY(0.0, 0.0, delta);
    out.pose.orientation = tf2::toMsg((q_delta * q_in).normalized());
    return out;
  }

  static double normalizeAngle(double a)
  { return std::atan2(std::sin(a), std::cos(a)); }

  bool isSafePlaceXY(double x, double y, const std::string& ignore_id,
                     double min_clearance = 0.07,
                     double r_min = 0.14, double r_max = 0.24)
  {
    const double r = std::hypot(x, y);
    if (r < r_min || r > r_max) return false;
    std::lock_guard<std::mutex> sl(scene_mutex_);
    for (const auto& kv : tracked_objects_) {
      if (kv.first == ignore_id) continue;
      const auto& p = kv.second.pose.pose.position;
      if (std::hypot(x - p.x, y - p.y) < min_clearance) return false;
    }
    return true;
  }

  bool getLatestArmJointSnapshot(std::map<std::string, double>& out)
  {
    std::lock_guard<std::mutex> lk(joint_state_mutex_);
    if (!latest_arm_joints_.count("joint1") || !latest_arm_joints_.count("joint2") ||
        !latest_arm_joints_.count("joint3") || !latest_arm_joints_.count("joint4") ||
        !latest_arm_joints_.count("joint5_roll")) return false;
    out = latest_arm_joints_;
    return true;
  }

  // ── Service handles ───────────────────────────────────────────────────────
  rclcpp::Service<omx_interfaces::srv::Pick>::SharedPtr        pick_srv_;
  rclcpp::Service<omx_interfaces::srv::Place>::SharedPtr       place_srv_;
  rclcpp::Service<omx_interfaces::srv::Pour>::SharedPtr        pour_srv_;
  rclcpp::Service<omx_interfaces::srv::Rotate>::SharedPtr      rotate_srv_;
  rclcpp::Service<omx_interfaces::srv::GoHome>::SharedPtr      go_home_srv_;
  rclcpp::Service<omx_interfaces::srv::AddObjects>::SharedPtr  add_obj_srv_;
  rclcpp::Service<omx_interfaces::srv::RemoveObjects>::SharedPtr remove_obj_srv_;
  rclcpp::Service<omx_interfaces::srv::GetObjects>::SharedPtr  get_obj_srv_;

  // ════════════════════════════════════════════════════════════════════════
  bool transformToWorld(geometry_msgs::msg::PoseStamped& pose, std::string& err)
  {
    if (pose.header.frame_id.empty() || pose.header.frame_id == BASE_FRAME) {
      pose.header.frame_id = BASE_FRAME; return true;
    }
    try {
      geometry_msgs::msg::PointStamped pt_in;
      pt_in.header = pose.header;
      pt_in.point  = pose.pose.position;
      auto pt_out  = tf_buffer_->transform(pt_in, BASE_FRAME, tf2::durationFromSec(5.0));
      RCLCPP_INFO(get_logger(), "TF: '%s'→'%s' [%.3f %.3f %.3f]",
        pose.header.frame_id.c_str(), BASE_FRAME.c_str(),
        pt_out.point.x, pt_out.point.y, pt_out.point.z);
      pose.header.frame_id      = BASE_FRAME;
      pose.pose.position        = pt_out.point;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w   = 1.0;
      return true;
    } catch (const tf2::TransformException& e) {
      err = std::string("TF2 failed: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", err.c_str());
      return false;
    }
  }

  void setupTask(mtc::Task& task, const std::string& name)
  {
    task.stages()->setName(name);
    task.loadRobotModel(shared_from_this());
    task.setProperty("group",               ARM_GROUP);
    task.setProperty("eef",                 EEF_NAME);
    task.setProperty("hand",                GRIPPER_GROUP);
    task.setProperty("hand_grasping_frame", HAND_FRAME);
    task.setProperty("ik_frame",            HAND_FRAME);
  }

  bool planAndExecute(mtc::Task& task, std::string& err, int max_plans = 10)
  {
    try { task.init(); }
    catch (const mtc::InitStageException& e) {
      std::ostringstream o; o << e;
      err = "Task init failed: " + o.str();
      RCLCPP_ERROR(get_logger(), "%s", err.c_str()); return false;
    }
    if (!task.plan(max_plans)) {
      std::ostringstream o; task.explainFailure(o);
      err = "Planning failed:\n" + o.str();
      RCLCPP_ERROR(get_logger(), "%s", err.c_str()); return false;
    }
    RCLCPP_INFO(get_logger(), "Planning succeeded. Executing…");
    auto result = task.execute(*task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      err = "Execution failed (code: " + std::to_string(result.val) + ")";
      RCLCPP_ERROR(get_logger(), "%s", err.c_str()); return false;
    }
    RCLCPP_INFO(get_logger(), "Execution succeeded.");
    return true;
  }

  geometry_msgs::msg::Quaternion graspOrientation(double x, double y)
  {
    tf2::Quaternion q; q.setRPY(0.0, 0.15, std::atan2(y, x));
    return tf2::toMsg(q);
  }

  // ════════════════════════════════════════════════════════════════════════
  //  PICK
  // ════════════════════════════════════════════════════════════════════════
  void pickCb(const omx_interfaces::srv::Pick::Request::SharedPtr req,
              omx_interfaces::srv::Pick::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    RCLCPP_INFO(get_logger(), "Pick requested: '%s'", req->object_id.c_str());

    if (!attached_object_id_.empty()) {
      res->success = false;
      res->message = "Already holding '" + attached_object_id_ + "'. Place first.";
      return;
    }

    geometry_msgs::msg::PoseStamped obj_pose;
    double obj_height = 0.0, obj_width = 0.0;
    {
      std::lock_guard<std::mutex> sl(scene_mutex_);
      auto it = tracked_objects_.find(req->object_id);
      if (it == tracked_objects_.end()) {
        res->success = false;
        res->message = "Object '" + req->object_id + "' not found.";
        return;
      }
      obj_pose   = it->second.pose;
      obj_height = objectHeight(it->second);
      obj_width  = objectWidth(it->second);
    }

    const double ox = obj_pose.pose.position.x;
    const double oy = obj_pose.pose.position.y;
    const double oz = obj_pose.pose.position.z;

    geometry_msgs::msg::PoseStamped grasp_pose;
    grasp_pose.header.frame_id  = BASE_FRAME;
    grasp_pose.pose.position.x  = ox;
    grasp_pose.pose.position.y  = oy;
    grasp_pose.pose.position.z  = oz;
    grasp_pose.pose.orientation = graspOrientation(ox, oy);

    RCLCPP_INFO(get_logger(), "Grasp centroid z=%.3f  obj_h=%.3f", oz, obj_height);

    auto pipeline = makePipeline();
    mtc::Task task;
    setupTask(task, "pick_" + req->object_id);

    mtc::Stage* current_state_ptr = nullptr;
    { auto s = std::make_unique<mtc::stages::CurrentState>("current state");
      current_state_ptr = s.get(); task.add(std::move(s)); }

    { auto s = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_interp_);
      s->setGroup(GRIPPER_GROUP); s->setGoal(GRIPPER_OPEN); task.add(std::move(s)); }

    { auto s = std::make_unique<mtc::stages::Connect>("move to pick",
        mtc::stages::Connect::GroupPlannerVector{{ARM_GROUP, pipeline}});
      s->setTimeout(10.0); s->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(s)); }

    {
      auto pick = std::make_unique<mtc::SerialContainer>("pick object");
      task.properties().exposeTo(pick->properties(), {"eef","hand","group","ik_frame"});
      pick->properties().configureInitFrom(mtc::Stage::PARENT, {"eef","hand","group","ik_frame"});

      { auto s = std::make_unique<mtc::stages::MoveTo>("zero wrist roll", joint_interp_);
        s->setGroup(ARM_GROUP);
        std::map<std::string,double> j; j["joint5_roll"] = 0.0; s->setGoal(j);
        pick->insert(std::move(s)); }

      { auto s = std::make_unique<mtc::stages::MoveRelative>("approach object", joint_interp_);
        s->setGroup(ARM_GROUP); s->setMinMaxDistance(0.005, 0.10); s->setIKFrame(HAND_FRAME);
        geometry_msgs::msg::Vector3Stamped d; d.header.frame_id = BASE_FRAME; d.vector.z = -1.0;
        s->setDirection(d); pick->insert(std::move(s)); }

      { auto gen = std::make_unique<mtc::stages::GeneratePose>("generate grasp pose");
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

      { auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
        s->allowCollisions(req->object_id,
          task.getRobotModel()->getJointModelGroup(GRIPPER_GROUP)
            ->getLinkModelNamesWithCollisionGeometry(), true);
        pick->insert(std::move(s)); }

      {
        auto s = std::make_unique<mtc::stages::MoveTo>("close gripper", joint_interp_);
        s->setGroup(GRIPPER_GROUP);
        double target_gap = obj_width * 0.95;
        double grip_joint = (target_gap - 0.042) / 2.0;
        grip_joint = std::clamp(grip_joint, -0.011, 0.02);
        RCLCPP_INFO(get_logger(),
          "Gripper: obj_width=%.4f target_gap=%.4f joint=%.4f",
          obj_width, target_gap, grip_joint);
        std::map<std::string, double> grip_goal;
        grip_goal["gripper_left_joint"] = grip_joint;
        s->setGoal(grip_goal);
        pick->insert(std::move(s));
      }

      { auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
        s->attachObject(req->object_id, EEF_LINK); pick->insert(std::move(s)); }

      { auto s = std::make_unique<mtc::stages::MoveRelative>("lift object", joint_interp_);
        s->setGroup(ARM_GROUP); s->setMinMaxDistance(0.005, 0.10); s->setIKFrame(HAND_FRAME);
        geometry_msgs::msg::Vector3Stamped d; d.header.frame_id = BASE_FRAME; d.vector.z = 1.0;
        s->setDirection(d); pick->insert(std::move(s)); }

      task.add(std::move(pick));
    }

    std::string err;
    if (planAndExecute(task, err)) {
  attached_object_id_ = req->object_id;
  last_pick_.valid      = true;
  last_pick_.object_id  = req->object_id;
  last_pick_.grasp_pose = grasp_pose;
  last_pick_.obj_half_h = obj_height / 2.0;
  last_pick_.have_arm_joints = false;

  // ── Read joints from MTC solution's end scene ──────────────────────
  // SolutionBase::end() returns the InterfaceState at the end of the
  // solution, whose scene() holds the exact robot state after lift.
  // This is guaranteed accurate — no /joint_states timing dependency.
  if (!task.solutions().empty()) {
    try {
      const mtc::SolutionBase& solution = *task.solutions().front();
      const mtc::InterfaceState* end_state = solution.end();
      if (end_state && end_state->scene()) {
        const moveit::core::RobotState& robot_state =
          end_state->scene()->getCurrentState();
        const moveit::core::JointModelGroup* jmg =
          robot_state.getJointModelGroup(ARM_GROUP);
        if (jmg) {
          std::vector<double> joint_vals;
          robot_state.copyJointGroupPositions(jmg, joint_vals);
          const std::vector<std::string>& joint_names =
            jmg->getActiveJointModelNames();
          last_pick_.approach_joints.clear();
          for (size_t i = 0; i < joint_names.size(); ++i)
            last_pick_.approach_joints[joint_names[i]] = joint_vals[i];
          last_pick_.have_arm_joints = true;
          RCLCPP_INFO(get_logger(),
            "Approach joints (MTC end scene): j1=%.3f j2=%.3f j3=%.3f j4=%.3f j5=%.3f",
            last_pick_.approach_joints["joint1"],
            last_pick_.approach_joints["joint2"],
            last_pick_.approach_joints["joint3"],
            last_pick_.approach_joints["joint4"],
            last_pick_.approach_joints["joint5_roll"]);
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "MTC end scene read failed: %s", e.what());
    }
  }

  // ── Fallback: /joint_states if MTC scene read failed ───────────────
  if (!last_pick_.have_arm_joints) {
    rclcpp::sleep_for(std::chrono::milliseconds(1500));
    last_pick_.have_arm_joints =
      getLatestArmJointSnapshot(last_pick_.approach_joints);
    if (last_pick_.have_arm_joints) {
      RCLCPP_WARN(get_logger(),
        "Approach joints (/joint_states fallback): j1=%.3f j2=%.3f j3=%.3f j4=%.3f j5=%.3f",
        last_pick_.approach_joints["joint1"],
        last_pick_.approach_joints["joint2"],
        last_pick_.approach_joints["joint3"],
        last_pick_.approach_joints["joint4"],
        last_pick_.approach_joints["joint5_roll"]);
    } else {
      RCLCPP_WARN(get_logger(), "Could not snapshot approach joints.");
    }
  }

  res->success = true;
  res->message = "Picked '" + req->object_id + "'";
} else {
  res->success = false; res->message = err;
}
  }

  // ════════════════════════════════════════════════════════════════════════
  //  PLACE
  // ════════════════════════════════════════════════════════════════════════
  void placeCb(const omx_interfaces::srv::Place::Request::SharedPtr req,
               omx_interfaces::srv::Place::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    RCLCPP_INFO(get_logger(), "Place requested.");

    if (attached_object_id_.empty()) {
      res->success = false; res->message = "Not holding anything."; return;
    }
    const std::string obj_id = attached_object_id_;

    double half_h = 0.0;
    {
      std::lock_guard<std::mutex> sl(scene_mutex_);
      auto it = tracked_objects_.find(obj_id);
      if (it != tracked_objects_.end()) half_h = objectHeight(it->second) / 2.0;
    }

    geometry_msgs::msg::PoseStamped place_pose;
    bool place_back_to_pick   = false;
    bool semicircle_mode      = false;
    std::map<std::string,double> sc_joints_goal;
    bool used_legacy_fallback = false;
    std::string fallback_reason;

    // ── Determine placement mode ──────────────────────────────────────────
    if (req->use_target_pose) {
      place_pose = req->target_pose;
      if (req->use_camera_frame) place_pose.header.frame_id = CAMERA_FRAME;
      std::string tf_err;
      if (!transformToWorld(place_pose, tf_err)) {
        res->success = false; res->message = tf_err; return;
      }

    } else if (!req->random) {
      if (!last_pick_.valid || last_pick_.object_id != obj_id) {
        res->success = false; res->message = "No saved pick pose."; return;
      }
      place_pose         = last_pick_.grasp_pose;
      place_back_to_pick = true;

    } else {
      // ── Random / Semicircle ───────────────────────────────────────────
      if (req->semicircle && last_pick_.valid && last_pick_.object_id == obj_id) {
        if (!last_pick_.have_arm_joints) {
          res->success = false;
          res->message = "Need fresh pick for semicircle mode."; return;
        }

        const double pick_j1 = last_pick_.approach_joints.at("joint1");

        // ── Generate evenly-spaced j1 samples across (-75°, +75°) ──────
        // Exclude ±15° deadzone around pick_j1 to avoid placing back on origin.
        constexpr double FRONT_MIN = -75.0 * M_PI / 180.0;
        constexpr double FRONT_MAX =  75.0 * M_PI / 180.0;
        constexpr double DEADZONE  =  15.0 * M_PI / 180.0;
        constexpr int    N_SAMPLES =  20;

        std::vector<double> j1_samples;
        for (int i = 0; i < N_SAMPLES; ++i) {
          double j1 = FRONT_MIN + i * (FRONT_MAX - FRONT_MIN) / (N_SAMPLES - 1);
          if (std::abs(normalizeAngle(j1 - pick_j1)) < DEADZONE) continue;
          j1_samples.push_back(j1);
        }

        RCLCPP_INFO(get_logger(),
          "Semicircle: pick_j1=%.3f  testing %zu candidates (deadzone ±%.0f°)",
          pick_j1, j1_samples.size(), DEADZONE * 180.0 / M_PI);

        // ── For each sample, plan rotation + lower and keep valid ones ──
        // A candidate is valid only if OMPL can plan the rotation to the
        // exact 5-joint goal AND JointInterp can lower afterwards.
        // This pre-validates before committing to any motion.
        struct ValidCandidate {
          double j1;
          double angular_dist;               // |j1 - pick_j1|, for middle selection
          std::map<std::string, double> joints_goal;
        };
        std::vector<ValidCandidate> valid_candidates;

        for (double j1 : j1_samples) {
          // Build full 5-joint goal: j2-4 fixed from pick, only j1 changes
          std::map<std::string, double> goal = last_pick_.approach_joints;
          goal["joint1"]      = normalizeAngle(j1);
          goal["joint5_roll"] = 0.0;

          // Plan-only test task (no execution)
          mtc::Task test_task;
          setupTask(test_task, "sc_test_" + std::to_string(static_cast<int>(j1 * 1000)));

          { auto s = std::make_unique<mtc::stages::CurrentState>("current state");
            test_task.add(std::move(s)); }

          // Stage 1: OMPL rotates to all 5 joint targets
          {
            auto ompl = makePipeline();
            auto s = std::make_unique<mtc::stages::MoveTo>("rotate", ompl);
            s->setGroup(ARM_GROUP);
            s->setTimeout(5.0);
            s->setGoal(goal);
            test_task.add(std::move(s));
          }

          // Stage 2: JointInterp lowers in world-Z
          {
            auto s = std::make_unique<mtc::stages::MoveRelative>("lower", joint_interp_);
            s->setGroup(ARM_GROUP);
            s->setMinMaxDistance(0.03, 0.15);
            s->setIKFrame(HAND_FRAME);
            geometry_msgs::msg::Vector3Stamped dir;
            dir.header.frame_id = BASE_FRAME;
            dir.vector.z = -1.0;
            s->setDirection(dir);
            test_task.add(std::move(s));
          }

          try {
            test_task.init();
            if (test_task.plan(3)) {
              const double dist = std::abs(normalizeAngle(j1 - pick_j1));
              valid_candidates.push_back({normalizeAngle(j1), dist, goal});
              RCLCPP_INFO(get_logger(),
                "  SC candidate j1=%.3f (dist=%.3f rad) → valid", j1, dist);
            } else {
              RCLCPP_DEBUG(get_logger(), "  SC candidate j1=%.3f → plan failed", j1);
            }
          } catch (const std::exception& e) {
            RCLCPP_DEBUG(get_logger(), "  SC candidate j1=%.3f → exception: %s", j1, e.what());
          }
        }

        if (valid_candidates.empty()) {
          used_legacy_fallback = true;
          fallback_reason      = "no collision-free semicircle candidate found";
          RCLCPP_WARN(get_logger(), "Semicircle: no valid candidates → legacy fallback");
        } else {
          // Sort by angular distance from pick_j1 (ascending = closest first)
          // then pick the middle index → not too close to pick, not at extreme
          std::sort(valid_candidates.begin(), valid_candidates.end(),
            [](const ValidCandidate& a, const ValidCandidate& b) {
              return a.angular_dist < b.angular_dist;
            });

          const auto& chosen = valid_candidates[valid_candidates.size() / 2];
          sc_joints_goal     = chosen.joints_goal;
          semicircle_mode    = true;
          place_back_to_pick = true;

          // Compute place_pose for scene tracking (collision obj update after place)
          const double delta = normalizeAngle(chosen.j1 - pick_j1);
          place_pose = rotatePoseAroundWorldZ(last_pick_.grasp_pose, delta);
          place_pose.header.frame_id = BASE_FRAME;

          RCLCPP_INFO(get_logger(),
            "Semicircle chosen: j1=%.3f (dist=%.3f rad) from %zu valid candidates",
            chosen.j1, chosen.angular_dist, valid_candidates.size());
          RCLCPP_INFO(get_logger(),
            "Semicircle goal: j1=%.3f j2=%.3f j3=%.3f j4=%.3f j5=%.3f",
            sc_joints_goal["joint1"],   sc_joints_goal["joint2"],
            sc_joints_goal["joint3"],   sc_joints_goal["joint4"],
            sc_joints_goal["joint5_roll"]);
        }

      } else if (req->semicircle) {
        used_legacy_fallback = true;
        fallback_reason      = "missing valid last-pick pose";
        RCLCPP_WARN(get_logger(), "Semicircle: no pick info → legacy fallback");
      }

      if (!semicircle_mode && !place_back_to_pick) {
        // ── Legacy random fallback ──────────────────────────────────────
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
        bool found = false; double bx = 0, by = 0, bd = 1e9;
        for (int i = 0; i < 20; ++i) {
          double r = r_dist(rng), a = a_dist(rng);
          double cx = r*std::cos(a), cy = r*std::sin(a);
          if (!isSafePlaceXY(cx, cy, obj_id, 0.06)) continue;
          double d = std::hypot(cx-cur_x, cy-cur_y);
          if (d < bd) { bd=d; bx=cx; by=cy; found=true; }
        }
        if (!found) { res->success=false; res->message="No safe place candidate."; return; }
        place_pose.header.frame_id    = BASE_FRAME;
        place_pose.pose.position.x    = bx;
        place_pose.pose.position.y    = by;
        place_pose.pose.position.z    = 0.0;
        place_pose.pose.orientation.w = 1.0;
      }
    }

    const double px = place_pose.pose.position.x;
    const double py = place_pose.pose.position.y;
    const double pz = place_back_to_pick
                        ? place_pose.pose.position.z
                        : (half_h + PLACE_BASE_CLEARANCE);

    RCLCPP_INFO(get_logger(),
      "Place '%s': px=%.3f py=%.3f pz=%.3f half_h=%.4f mode=%s",
      obj_id.c_str(), px, py, pz, half_h,
      semicircle_mode ? "semicircle" : (place_back_to_pick ? "back-to-pick" : "legacy"));

    auto pipeline = makePipeline();
    mtc::Task task;
    setupTask(task, "place_" + obj_id);

    // ── Compute lower distance from FK at the chosen joint goal ──────────
    double semicircle_lower_distance = 0.06;
    if (semicircle_mode) {
      moveit::core::RobotState state(task.getRobotModel());
      state.setToDefaultValues();
      for (const auto& kv : sc_joints_goal)
        state.setVariablePosition(kv.first, kv.second);
      state.update();

      const double hand_z            = state.getGlobalLinkTransform(HAND_FRAME).translation().z();
      const double target_centroid_z = half_h + PLACE_BASE_CLEARANCE;
      semicircle_lower_distance = std::clamp(hand_z - target_centroid_z, 0.005, 0.15);

      RCLCPP_INFO(get_logger(),
        "Semicircle lower: hand_z=%.3f target_z=%.3f lower=%.3f",
        hand_z, target_centroid_z, semicircle_lower_distance);
    }

    mtc::Stage* current_state_ptr = nullptr;
    { auto s = std::make_unique<mtc::stages::CurrentState>("current state");
      current_state_ptr = s.get(); task.add(std::move(s)); }

    // ════════════════════════════════════════════════════════════════════
    if (semicircle_mode) {
      // ── SEMICIRCLE PLACE (joint-space, OMPL-validated) ────────────────
      //
      // OMPL plans freely through joint space but must END at exactly the
      // 5-joint goal (j1=chosen, j2-4=pick values, j5=0).
      // This guarantees the arm shape at place matches the pick approach,
      // so the object lands at the correct height when we lower.

      {
        auto ompl = makePipeline();
        auto s = std::make_unique<mtc::stages::MoveTo>("rotate to place angle", ompl);
        s->setGroup(ARM_GROUP);
        s->setTimeout(15.0);
        s->setGoal(sc_joints_goal);  // all 5 joints — OMPL must end here exactly
        task.add(std::move(s));
      }

      {
        auto s = std::make_unique<mtc::stages::MoveRelative>("lower to place", joint_interp_);
        s->setGroup(ARM_GROUP);
        s->setMinMaxDistance(std::max(0.005, semicircle_lower_distance - 0.002),
                             std::min(0.15,  semicircle_lower_distance + 0.002));
        s->setIKFrame(HAND_FRAME);
        geometry_msgs::msg::Vector3Stamped dir;
        dir.header.frame_id = BASE_FRAME;
        dir.vector.z = -1.0;
        s->setDirection(dir);
        task.add(std::move(s));
      }

      {
        auto s = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_interp_);
        s->setGroup(GRIPPER_GROUP);
        s->setGoal(GRIPPER_OPEN);
        task.add(std::move(s));
      }

      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision");
        s->allowCollisions(obj_id,
          task.getRobotModel()->getJointModelGroup(GRIPPER_GROUP)
            ->getLinkModelNamesWithCollisionGeometry(), false);
        task.add(std::move(s));
      }

      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
        s->detachObject(obj_id, EEF_LINK);
        task.add(std::move(s));
      }

      {
        auto s = std::make_unique<mtc::stages::MoveRelative>("retreat", joint_interp_);
        s->setGroup(ARM_GROUP);
        s->setMinMaxDistance(0.005, 0.12);
        s->setIKFrame(HAND_FRAME);
        geometry_msgs::msg::Vector3Stamped dir;
        dir.header.frame_id = BASE_FRAME;
        dir.vector.z = 1.0;
        s->setDirection(dir);
        task.add(std::move(s));
      }

    } else {
      // ════════════════════════════════════════════════════════════════
      // ── STANDARD PLACE (Cartesian target + IK) ───────────────────
      // ════════════════════════════════════════════════════════════════
      {
        auto s = std::make_unique<mtc::stages::Connect>("move to place",
          mtc::stages::Connect::GroupPlannerVector{
            {ARM_GROUP, pipeline}, {GRIPPER_GROUP, joint_interp_}});
        s->setTimeout(10.0);
        s->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(s));
      }
      {
        auto place = std::make_unique<mtc::SerialContainer>("place object");
        task.properties().exposeTo(place->properties(), {"eef","hand","group","ik_frame"});
        place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef","hand","group","ik_frame"});

        { auto s = std::make_unique<mtc::stages::MoveTo>("zero wrist roll", joint_interp_);
          s->setGroup(ARM_GROUP);
          std::map<std::string,double> j; j["joint5_roll"]=0.0; s->setGoal(j);
          place->insert(std::move(s)); }

        { auto s = std::make_unique<mtc::stages::MoveRelative>("lower to place", joint_interp_);
          s->setGroup(ARM_GROUP); s->setMinMaxDistance(0.005,0.10); s->setIKFrame(HAND_FRAME);
          geometry_msgs::msg::Vector3Stamped d; d.header.frame_id=BASE_FRAME; d.vector.z=-1.0;
          s->setDirection(d); place->insert(std::move(s)); }

        {
          auto gen = std::make_unique<mtc::stages::GeneratePose>("generate place pose");
          geometry_msgs::msg::PoseStamped target;
          target.header.frame_id = BASE_FRAME;
          target.pose.position.x = px;
          target.pose.position.y = py;
          target.pose.position.z = pz;
          if (place_back_to_pick) {
            target.pose.orientation = place_pose.pose.orientation;
          } else {
            tf2::Quaternion q; q.setRPY(0.0, 0.0, std::atan2(py, px));
            target.pose.orientation = tf2::toMsg(q);
          }
          gen->setPose(target); gen->setMonitoredStage(current_state_ptr);
          auto ik = std::make_unique<mtc::stages::ComputeIK>("place IK", std::move(gen));
          ik->setMaxIKSolutions(20); ik->setTimeout(0.5); ik->setIKFrame(HAND_FRAME);
          ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef","group"});
          ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
          place->insert(std::move(ik));
        }

        { auto s = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_interp_);
          s->setGroup(GRIPPER_GROUP); s->setGoal(GRIPPER_OPEN);
          place->insert(std::move(s)); }

        { auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision");
          s->allowCollisions(obj_id,
            task.getRobotModel()->getJointModelGroup(GRIPPER_GROUP)
              ->getLinkModelNamesWithCollisionGeometry(), false);
          place->insert(std::move(s)); }

        { auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
          s->detachObject(obj_id, EEF_LINK); place->insert(std::move(s)); }

        { auto s = std::make_unique<mtc::stages::MoveRelative>("retreat", joint_interp_);
          s->setGroup(ARM_GROUP); s->setMinMaxDistance(0.005,0.10); s->setIKFrame(HAND_FRAME);
          geometry_msgs::msg::Vector3Stamped d; d.header.frame_id=BASE_FRAME; d.vector.z=1.0;
          s->setDirection(d); place->insert(std::move(s)); }

        task.add(std::move(place));
      }
    }

    std::string err;
    if (planAndExecute(task, err)) {
      {
        std::lock_guard<std::mutex> sl(scene_mutex_);
        auto it = tracked_objects_.find(obj_id);
        if (it != tracked_objects_.end()) {
          it->second.pose = place_pose;
          it->second.pose.pose.position.z = half_h + PLACE_BASE_CLEARANCE;

          moveit::planning_interface::PlanningSceneInterface psi;
          moveit_msgs::msg::CollisionObject co;
          co.id = obj_id;
          co.header.frame_id = BASE_FRAME;
          co.operation = moveit_msgs::msg::CollisionObject::MOVE;
          co.primitive_poses.push_back(it->second.pose.pose);
          shape_msgs::msg::SolidPrimitive prim;
          const auto& d = it->second.dimensions;
          if (it->second.type == "cylinder" && d.size() >= 2) {
            prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
            prim.dimensions.assign(d.begin(), d.end());
          } else if (it->second.type == "box" && d.size() >= 3) {
            prim.type = shape_msgs::msg::SolidPrimitive::BOX;
            prim.dimensions.assign(d.begin(), d.end());
          } else if (it->second.type == "sphere" && d.size() >= 1) {
            prim.type = shape_msgs::msg::SolidPrimitive::SPHERE;
            prim.dimensions.assign(d.begin(), d.end());
          }
          co.primitives.push_back(prim);
          psi.applyCollisionObject(co);
        }
      }
      attached_object_id_.clear();
      res->success = true;
      res->message = "Placed '" + obj_id + "'";
      if (used_legacy_fallback)
        res->message += " (fallback: " + fallback_reason + ")";
      else if (semicircle_mode)
        res->message += " (mode: semicircle joint-space)";
    } else {
      res->success = false; res->message = err;
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  //  POUR
  // ════════════════════════════════════════════════════════════════════════
  void pourCb(const omx_interfaces::srv::Pour::Request::SharedPtr req,
              omx_interfaces::srv::Pour::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    RCLCPP_INFO(get_logger(), "MTC pour into '%s'", req->target_object_id.c_str());

    if (attached_object_id_.empty()) {
      res->success=false; res->message="Not holding anything."; return;
    }

    TrackedObject src, tgt;
    {
      std::lock_guard<std::mutex> sl(scene_mutex_);
      auto it1 = tracked_objects_.find(attached_object_id_);
      auto it2 = tracked_objects_.find(req->target_object_id);
      if (it1 == tracked_objects_.end() || it2 == tracked_objects_.end()) {
        res->success=false; res->message="Source/Target not found."; return;
      }
      src = it1->second;
      tgt = it2->second;
    }

    const double src_h = std::max(0.04, objectHeight(src));
    const double tgt_h = std::max(0.02, objectHeight(tgt));

    double src_r = 0.03;
    if (src.type == "cylinder" && src.dimensions.size() >= 2) src_r = src.dimensions[1];

    double tgt_r = 0.03;
    if (tgt.type == "cylinder" && tgt.dimensions.size() >= 2) tgt_r = tgt.dimensions[1];

    geometry_msgs::msg::Point target_opening = tgt.pose.pose.position;
    target_opening.z += tgt_h / 2.0;

    const double target_r = std::hypot(target_opening.x, target_opening.y);
    constexpr double POUR_R_MIN = 0.12;
    constexpr double POUR_R_MAX = 0.24;

    geometry_msgs::msg::Point anchor = target_opening;
    if (target_r > 1e-6) {
      if (target_r > POUR_R_MAX) {
        const double s = POUR_R_MAX / target_r;
        anchor.x *= s; anchor.y *= s;
        RCLCPP_WARN(get_logger(),
          "Pour target r=%.3f exceeds reach band; using projected anchor r=%.3f",
          target_r, std::hypot(anchor.x, anchor.y));
      } else if (target_r < POUR_R_MIN) {
        const double s = POUR_R_MIN / target_r;
        anchor.x *= s; anchor.y *= s;
        RCLCPP_WARN(get_logger(),
          "Pour target r=%.3f too close; using projected anchor r=%.3f",
          target_r, std::hypot(anchor.x, anchor.y));
      }
    }

    const double base_yaw    = std::atan2(anchor.y, anchor.x);
    const double base_offset = std::max(0.02, src_r * 0.9);

    double tilt = req->pour_angle;
    if (std::abs(tilt) < 1e-3) tilt = std::atan(src_h / (src_r + 1e-3));
    tilt = std::clamp(tilt, 0.30, 1.20);
    if (tgt_r < src_r) tilt = std::min(1.30, tilt * 1.1);

    const std::vector<double> clearances    {0.06, 0.09, 0.12};
    const std::vector<double> offset_scales {1.0, 1.25, 0.75};
    const std::vector<double> yaw_biases    {0.0, 0.35, -0.35};

    std::string last_err = "No MTC pour candidate succeeded.";

    for (double c : clearances) {
      for (double os : offset_scales) {
        for (double yb : yaw_biases) {
          const double yaw_try = base_yaw + yb;
          const double off     = base_offset * os;

          geometry_msgs::msg::PoseStamped pre_pour;
          pre_pour.header.frame_id = BASE_FRAME;
          pre_pour.pose.position   = anchor;
          pre_pour.pose.position.z += c;
          pre_pour.pose.position.x -= off * std::cos(yaw_try);
          pre_pour.pose.position.y -= off * std::sin(yaw_try);
          tf2::Quaternion q_pre;
          q_pre.setRPY(0.0, 0.0, yaw_try);
          pre_pour.pose.orientation = tf2::toMsg(q_pre.normalized());

          auto pour_pipeline = makePipeline();
          mtc::Task task;
          setupTask(task, "pour_" + req->target_object_id);

          mtc::Stage* current_state_ptr = nullptr;
          { auto s = std::make_unique<mtc::stages::CurrentState>("current state");
            current_state_ptr = s.get(); task.add(std::move(s)); }

          { auto s = std::make_unique<mtc::stages::Connect>(
              "move to pour", mtc::stages::Connect::GroupPlannerVector{{ARM_GROUP, pour_pipeline}});
            s->setTimeout(10.0);
            s->properties().configureInitFrom(mtc::Stage::PARENT);
            task.add(std::move(s)); }

          {
            auto pour = std::make_unique<mtc::SerialContainer>("pour sequence");
            task.properties().exposeTo(pour->properties(), {"eef","hand","group","ik_frame"});
            pour->properties().configureInitFrom(mtc::Stage::PARENT, {"eef","hand","group","ik_frame"});

            { auto gen = std::make_unique<mtc::stages::GeneratePose>("generate pour pose");
              gen->setPose(pre_pour);
              gen->setMonitoredStage(current_state_ptr);
              auto ik = std::make_unique<mtc::stages::ComputeIK>("pour IK", std::move(gen));
              ik->setMaxIKSolutions(20); ik->setTimeout(0.5); ik->setIKFrame(HAND_FRAME);
              ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
              ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
              pour->insert(std::move(ik)); }

            { auto s = std::make_unique<mtc::stages::MoveTo>("zero wrist roll", joint_interp_);
              s->setGroup(ARM_GROUP);
              std::map<std::string,double> j; j["joint5_roll"] = 0.0;
              s->setGoal(j);
              pour->insert(std::move(s)); }

            { auto s = std::make_unique<mtc::stages::MoveRelative>("tilt to pour", joint_interp_);
              s->setGroup(ARM_GROUP);
              std::map<std::string,double> d; d["joint5_roll"] = tilt;
              s->setDirection(d);
              pour->insert(std::move(s)); }

            { auto s = std::make_unique<mtc::stages::MoveRelative>("untilt", joint_interp_);
              s->setGroup(ARM_GROUP);
              std::map<std::string,double> d; d["joint5_roll"] = -tilt;
              s->setDirection(d);
              pour->insert(std::move(s)); }

            { auto s = std::make_unique<mtc::stages::MoveRelative>("retreat", joint_interp_);
              s->setGroup(ARM_GROUP);
              s->setMinMaxDistance(0.005, 0.08); s->setIKFrame(HAND_FRAME);
              geometry_msgs::msg::Vector3Stamped dir;
              dir.header.frame_id = BASE_FRAME; dir.vector.z = 1.0;
              s->setDirection(dir);
              pour->insert(std::move(s)); }

            task.add(std::move(pour));
          }

          std::string err;
          if (planAndExecute(task, err, 8)) {
            res->success = true;
            res->message = "Pour completed (MTC)";
            return;
          }
          last_err = err;
        }
      }
    }

    res->success = false;
    res->message = "MTC pour failed: " + last_err;
  }

  // ════════════════════════════════════════════════════════════════════════
  //  ROTATE
  // ════════════════════════════════════════════════════════════════════════
  void rotateCb(const omx_interfaces::srv::Rotate::Request::SharedPtr req,
                omx_interfaces::srv::Rotate::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    RCLCPP_INFO(get_logger(), "Rotate: angle=%.2f cycles=%d", req->angle, req->cycles);
    double angle = (std::abs(req->angle) > 0.01) ? req->angle : M_PI;
    int cycles = std::max(req->cycles, 0);

    mtc::Task task; setupTask(task, "rotate");
    { auto s = std::make_unique<mtc::stages::CurrentState>("current state");
      task.add(std::move(s)); }

    if (cycles == 0) {
      auto s = std::make_unique<mtc::stages::MoveRelative>("rotate", joint_interp_);
      s->setGroup(ARM_GROUP);
      std::map<std::string,double> d; d["joint5_roll"]=angle; s->setDirection(d);
      task.add(std::move(s));
    } else {
      for (int i = 0; i < cycles; ++i) {
        { auto s = std::make_unique<mtc::stages::MoveRelative>(
            "fwd_"+std::to_string(i), joint_interp_);
          s->setGroup(ARM_GROUP);
          std::map<std::string,double> d; d["joint5_roll"]=angle; s->setDirection(d);
          task.add(std::move(s)); }
        { auto s = std::make_unique<mtc::stages::MoveRelative>(
            "rev_"+std::to_string(i), joint_interp_);
          s->setGroup(ARM_GROUP);
          std::map<std::string,double> d; d["joint5_roll"]=-angle; s->setDirection(d);
          task.add(std::move(s)); }
      }
    }
    std::string err;
    if (planAndExecute(task, err)) { res->success=true; res->message="Rotation completed"; }
    else { res->success=false; res->message=err; }
  }

  // ════════════════════════════════════════════════════════════════════════
  //  GO HOME
  // ════════════════════════════════════════════════════════════════════════
  void goHomeCb(const omx_interfaces::srv::GoHome::Request::SharedPtr req,
                omx_interfaces::srv::GoHome::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    std::string state = req->state_name.empty() ? ARM_HOME : req->state_name;
    RCLCPP_INFO(get_logger(), "GoHome: '%s'", state.c_str());

    auto pipeline = makePipeline();
    mtc::Task task; setupTask(task, "go_home");
    { auto s = std::make_unique<mtc::stages::CurrentState>("current state");
      task.add(std::move(s)); }
    { auto s = std::make_unique<mtc::stages::MoveTo>("go to "+state, pipeline);
      s->setGroup(ARM_GROUP); s->setGoal(state); task.add(std::move(s)); }

    std::string err;
    if (planAndExecute(task, err)) { res->success=true; res->message="Reached '"+state+"'"; }
    else { res->success=false; res->message=err; }
  }

  // ════════════════════════════════════════════════════════════════════════
  //  ADD OBJECTS
  // ════════════════════════════════════════════════════════════════════════
  void addObjectsCb(const omx_interfaces::srv::AddObjects::Request::SharedPtr req,
                    omx_interfaces::srv::AddObjects::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> sl(scene_mutex_);
    moveit::planning_interface::PlanningSceneInterface psi;
    std::vector<moveit_msgs::msg::CollisionObject> cos;

    for (auto obj : req->objects) {
      if (obj.use_camera_frame) obj.pose.header.frame_id = CAMERA_FRAME;
      std::string tf_err;
      if (!transformToWorld(obj.pose, tf_err)) {
        res->success=false; res->message="Object '"+obj.id+"': "+tf_err; return;
      }
      moveit_msgs::msg::CollisionObject co;
      co.id = obj.id; co.header.frame_id = BASE_FRAME;
      co.operation = moveit_msgs::msg::CollisionObject::ADD;
      shape_msgs::msg::SolidPrimitive prim;
      if      (obj.type=="box")      prim.type = shape_msgs::msg::SolidPrimitive::BOX;
      else if (obj.type=="cylinder") prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      else if (obj.type=="sphere")   prim.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      else { res->success=false; res->message="Unknown type '"+obj.type+"'"; return; }
      prim.dimensions.assign(obj.dimensions.begin(), obj.dimensions.end());
      co.primitives.push_back(prim);

      { double hh=0.0; const auto& d=obj.dimensions;
        if (obj.type=="cylinder"&&d.size()>=1) hh=d[0]/2.0;
        else if (obj.type=="box"&&d.size()>=3) hh=d[2]/2.0;
        else if (obj.type=="sphere"&&d.size()>=1) hh=d[0];
        if (obj.pose.pose.position.z < hh) {
          RCLCPP_WARN(get_logger(), "Clamping '%s' z %.3f→%.3f",
            obj.id.c_str(), obj.pose.pose.position.z, hh);
          obj.pose.pose.position.z = hh;
        }
      }
      co.primitive_poses.push_back(obj.pose.pose);
      cos.push_back(co);

      TrackedObject to; to.id=obj.id; to.type=obj.type; to.pose=obj.pose;
      to.dimensions = std::vector<double>(obj.dimensions.begin(), obj.dimensions.end());
      tracked_objects_[obj.id] = to;
    }
    psi.applyCollisionObjects(cos);
    res->success=true;
    res->message="Added "+std::to_string(req->objects.size())+" object(s).";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  // ════════════════════════════════════════════════════════════════════════
  //  REMOVE OBJECTS
  // ════════════════════════════════════════════════════════════════════════
  void removeObjectsCb(const omx_interfaces::srv::RemoveObjects::Request::SharedPtr req,
                       omx_interfaces::srv::RemoveObjects::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> sl(scene_mutex_);
    moveit::planning_interface::PlanningSceneInterface psi;
    std::vector<std::string> to_rm;
    if (req->clear_all) for (auto& [id,_]:tracked_objects_) to_rm.push_back(id);
    else to_rm = std::vector<std::string>(req->object_ids.begin(),req->object_ids.end());

    std::vector<moveit_msgs::msg::CollisionObject> cos;
    for (const auto& id : to_rm) {
      moveit_msgs::msg::CollisionObject co;
      co.id=id; co.operation=moveit_msgs::msg::CollisionObject::REMOVE;
      cos.push_back(co); tracked_objects_.erase(id);
    }
    if (!cos.empty()) psi.applyCollisionObjects(cos);
    res->success=true;
    res->message="Removed "+std::to_string(to_rm.size())+" object(s).";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  // ════════════════════════════════════════════════════════════════════════
  //  GET OBJECTS
  // ════════════════════════════════════════════════════════════════════════
  void getObjectsCb(const omx_interfaces::srv::GetObjects::Request::SharedPtr,
                    omx_interfaces::srv::GetObjects::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> sl(scene_mutex_);
    for (auto& [id,obj]:tracked_objects_) {
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
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<ManipulatorNode>(opts);
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node); exec.spin();
  rclcpp::shutdown(); return 0;
}