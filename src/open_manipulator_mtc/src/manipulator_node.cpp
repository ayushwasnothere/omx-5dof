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

// ── Robot constants ──────────────────────────────────────────────────────────
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
    // Fine Cartesian steps for smooth pour arc
    cartesian_ = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_->setStepSize(0.005);
    cartesian_->setMaxVelocityScalingFactor(0.02);
    cartesian_->setMaxAccelerationScalingFactor(0.02);

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

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 20,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(joint_state_mutex_);
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
          const auto& n = msg->name[i];
          if (n == "joint1" || n == "joint2" || n == "joint3" ||
              n == "joint4" || n == "joint5_roll")
            latest_arm_joints_[n] = msg->position[i];
        }
      });

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    RCLCPP_INFO(get_logger(), "ManipulatorNode ready.");
  }

  mtc::solvers::PipelinePlannerPtr makePipeline()
  {
    auto p = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    p->setProperty("goal_joint_tolerance",            1e-3);
    p->setProperty("max_velocity_scaling_factor",     0.025);
    p->setProperty("max_acceleration_scaling_factor", 0.025);
    return p;
  }

private:
  mtc::solvers::CartesianPathPtr             cartesian_;
  mtc::solvers::JointInterpolationPlannerPtr joint_interp_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex task_mutex_;

  // ── Scene ─────────────────────────────────────────────────────────────────
  struct TrackedObject {
    std::string id, type;
    geometry_msgs::msg::PoseStamped pose;
    std::vector<double> dimensions;
  };
  std::mutex scene_mutex_;
  std::map<std::string, TrackedObject> tracked_objects_;
  std::string attached_object_id_;

  // ── Pick memory ───────────────────────────────────────────────────────────
  struct LastPickInfo {
    bool valid           = false;
    std::string object_id;
    geometry_msgs::msg::PoseStamped grasp_pose;
    bool have_arm_joints = false;
    std::map<std::string, double> approach_joints;
    double obj_half_h    = 0.0;
  };
  LastPickInfo last_pick_;

  // ── Joint state subscription ──────────────────────────────────────────────
  std::mutex joint_state_mutex_;
  std::map<std::string, double> latest_arm_joints_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // ── Helpers ───────────────────────────────────────────────────────────────
  static double objectHeight(const TrackedObject& obj)
  {
    const auto& d = obj.dimensions;
    if (obj.type == "cylinder" && d.size() >= 1) return d[0];
    if (obj.type == "box"      && d.size() >= 3) return d[2];
    if (obj.type == "sphere"   && d.size() >= 1) return 2.0 * d[0];
    return 0.0;
  }
  static double objectRadius(const TrackedObject& obj)
  {
    const auto& d = obj.dimensions;
    if (obj.type == "cylinder" && d.size() >= 2) return d[1];
    if (obj.type == "box"      && d.size() >= 2) return std::min(d[0], d[1]) * 0.5;
    if (obj.type == "sphere"   && d.size() >= 1) return d[0];
    return 0.03;
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
  rclcpp::Service<omx_interfaces::srv::Pick>::SharedPtr         pick_srv_;
  rclcpp::Service<omx_interfaces::srv::Place>::SharedPtr        place_srv_;
  rclcpp::Service<omx_interfaces::srv::Pour>::SharedPtr         pour_srv_;
  rclcpp::Service<omx_interfaces::srv::Rotate>::SharedPtr       rotate_srv_;
  rclcpp::Service<omx_interfaces::srv::GoHome>::SharedPtr       go_home_srv_;
  rclcpp::Service<omx_interfaces::srv::AddObjects>::SharedPtr   add_obj_srv_;
  rclcpp::Service<omx_interfaces::srv::RemoveObjects>::SharedPtr remove_obj_srv_;
  rclcpp::Service<omx_interfaces::srv::GetObjects>::SharedPtr   get_obj_srv_;

  // ═══════════════════════════════════════════════════════════════════════════
  bool transformToWorld(geometry_msgs::msg::PoseStamped& pose, std::string& err)
  {
    if (pose.header.frame_id.empty() || pose.header.frame_id == BASE_FRAME) {
      pose.header.frame_id = BASE_FRAME; return true;
    }
    try {
      geometry_msgs::msg::PointStamped pt_in;
      pt_in.header = pose.header; pt_in.point = pose.pose.position;
      auto pt_out = tf_buffer_->transform(pt_in, BASE_FRAME, tf2::durationFromSec(5.0));
      RCLCPP_INFO(get_logger(), "TF: '%s'→'%s' [%.3f %.3f %.3f]",
        pose.header.frame_id.c_str(), BASE_FRAME.c_str(),
        pt_out.point.x, pt_out.point.y, pt_out.point.z);
      pose.header.frame_id    = BASE_FRAME;
      pose.pose.position      = pt_out.point;
      pose.pose.orientation.x = 0.0; pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0; pose.pose.orientation.w = 1.0;
      return true;
    } catch (const tf2::TransformException& e) {
      err = std::string("TF2 failed: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", err.c_str()); return false;
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

  // ═══════════════════════════════════════════════════════════════════════════
  //  PICK
  // ═══════════════════════════════════════════════════════════════════════════
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
        res->message = "Object '" + req->object_id + "' not found."; return;
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

    mtc::Stage* open_gripper_ptr = nullptr;  
    
    { auto s = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_interp_);
      s->setGroup(GRIPPER_GROUP); s->setGoal(GRIPPER_OPEN);
      open_gripper_ptr = s.get();  
      task.add(std::move(s)); }

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
        gen->setPose(grasp_pose); gen->setMonitoredStage(current_state_ptr);
        gen->setMonitoredStage(open_gripper_ptr);
        auto ik = std::make_unique<mtc::stages::ComputeIK>("grasp IK", std::move(gen));
        ik->setMaxIKSolutions(20); ik->setTimeout(0.5); ik->setIKFrame(HAND_FRAME);
        ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef","group"});
        ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        pick->insert(std::move(ik)); }

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
        RCLCPP_INFO(get_logger(), "Gripper: obj_width=%.4f target_gap=%.4f joint=%.4f",
          obj_width, target_gap, grip_joint);
        std::map<std::string,double> grip_goal;
        grip_goal["gripper_left_joint"] = grip_joint;
        s->setGoal(grip_goal); pick->insert(std::move(s));
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
      attached_object_id_        = req->object_id;
      last_pick_.valid           = true;
      last_pick_.object_id       = req->object_id;
      last_pick_.grasp_pose      = grasp_pose;
      last_pick_.obj_half_h      = obj_height / 2.0;
      last_pick_.have_arm_joints = false;

      if (!task.solutions().empty()) {
        try {
          const mtc::SolutionBase& solution = *task.solutions().front();
          const mtc::InterfaceState* end_state = solution.end();
          if (end_state && end_state->scene()) {
            const moveit::core::RobotState& rs = end_state->scene()->getCurrentState();
            const moveit::core::JointModelGroup* jmg = rs.getJointModelGroup(ARM_GROUP);
            if (jmg) {
              std::vector<double> jv;
              rs.copyJointGroupPositions(jmg, jv);
              const std::vector<std::string>& jn = jmg->getActiveJointModelNames();
              last_pick_.approach_joints.clear();
              for (size_t i = 0; i < jn.size(); ++i)
                last_pick_.approach_joints[jn[i]] = jv[i];
              last_pick_.have_arm_joints = true;
              RCLCPP_INFO(get_logger(),
                "Approach joints (MTC end scene): j1=%.3f j2=%.3f j3=%.3f j4=%.3f j5=%.3f",
                last_pick_.approach_joints["joint1"], last_pick_.approach_joints["joint2"],
                last_pick_.approach_joints["joint3"], last_pick_.approach_joints["joint4"],
                last_pick_.approach_joints["joint5_roll"]);
            }
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN(get_logger(), "MTC end scene read failed: %s", e.what());
        }
      }

      if (!last_pick_.have_arm_joints) {
        rclcpp::sleep_for(std::chrono::milliseconds(1500));
        last_pick_.have_arm_joints = getLatestArmJointSnapshot(last_pick_.approach_joints);
        if (last_pick_.have_arm_joints)
          RCLCPP_WARN(get_logger(),
            "Approach joints (/joint_states fallback): j1=%.3f j2=%.3f j3=%.3f j4=%.3f j5=%.3f",
            last_pick_.approach_joints["joint1"], last_pick_.approach_joints["joint2"],
            last_pick_.approach_joints["joint3"], last_pick_.approach_joints["joint4"],
            last_pick_.approach_joints["joint5_roll"]);
        else
          RCLCPP_WARN(get_logger(), "Could not snapshot approach joints.");
      }

      res->success = true; res->message = "Picked '" + req->object_id + "'";
    } else {
      res->success = false; res->message = err;
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  PLACE
  // ═══════════════════════════════════════════════════════════════════════════
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
      place_pose = last_pick_.grasp_pose; place_back_to_pick = true;

    } else {
      if (req->semicircle && last_pick_.valid && last_pick_.object_id == obj_id) {
        if (!last_pick_.have_arm_joints) {
          res->success = false; res->message = "Need fresh pick for semicircle mode."; return;
        }

        const double pick_j1 = last_pick_.approach_joints.at("joint1");
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

        struct ValidCandidate {
          double j1, angular_dist;
          std::map<std::string,double> joints_goal;
        };
        std::vector<ValidCandidate> valid_candidates;

        for (double j1 : j1_samples) {
          std::map<std::string,double> goal = last_pick_.approach_joints;
          goal["joint1"]      = normalizeAngle(j1);
          goal["joint5_roll"] = 0.0;

          mtc::Task test_task;
          setupTask(test_task, "sc_test_" + std::to_string(static_cast<int>(j1 * 1000)));
          { auto s = std::make_unique<mtc::stages::CurrentState>("current state");
            test_task.add(std::move(s)); }
          { auto ompl = makePipeline();
            auto s = std::make_unique<mtc::stages::MoveTo>("rotate", ompl);
            s->setGroup(ARM_GROUP); s->setTimeout(5.0); s->setGoal(goal);
            test_task.add(std::move(s)); }
          { auto s = std::make_unique<mtc::stages::MoveRelative>("lower", joint_interp_);
            s->setGroup(ARM_GROUP); s->setMinMaxDistance(0.03, 0.15); s->setIKFrame(HAND_FRAME);
            geometry_msgs::msg::Vector3Stamped dir;
            dir.header.frame_id = BASE_FRAME; dir.vector.z = -1.0;
            s->setDirection(dir); test_task.add(std::move(s)); }

          try {
            test_task.init();
            if (test_task.plan(3)) {
              const double dist = std::abs(normalizeAngle(j1 - pick_j1));
              valid_candidates.push_back({normalizeAngle(j1), dist, goal});
              RCLCPP_INFO(get_logger(), "  SC candidate j1=%.3f (dist=%.3f rad) → valid", j1, dist);
            } else {
              RCLCPP_DEBUG(get_logger(), "  SC candidate j1=%.3f → plan failed", j1);
            }
          } catch (const std::exception& e) {
            RCLCPP_DEBUG(get_logger(), "  SC candidate j1=%.3f → exception: %s", j1, e.what());
          }
        }

        if (valid_candidates.empty()) {
          used_legacy_fallback = true;
          fallback_reason = "no collision-free semicircle candidate found";
          RCLCPP_WARN(get_logger(), "Semicircle: no valid candidates → legacy fallback");
        } else {
          std::sort(valid_candidates.begin(), valid_candidates.end(),
            [](const ValidCandidate& a, const ValidCandidate& b) {
              return a.angular_dist < b.angular_dist; });

          const auto& chosen = valid_candidates[valid_candidates.size() / 2];
          sc_joints_goal = chosen.joints_goal;
          semicircle_mode = true; place_back_to_pick = true;

          const double delta = normalizeAngle(chosen.j1 - pick_j1);
          place_pose = rotatePoseAroundWorldZ(last_pick_.grasp_pose, delta);
          place_pose.header.frame_id = BASE_FRAME;

          RCLCPP_INFO(get_logger(),
            "Semicircle chosen: j1=%.3f (dist=%.3f rad) from %zu valid candidates",
            chosen.j1, chosen.angular_dist, valid_candidates.size());
          RCLCPP_INFO(get_logger(),
            "Semicircle goal: j1=%.3f j2=%.3f j3=%.3f j4=%.3f j5=%.3f",
            sc_joints_goal["joint1"], sc_joints_goal["joint2"],
            sc_joints_goal["joint3"], sc_joints_goal["joint4"],
            sc_joints_goal["joint5_roll"]);
        }

      } else if (req->semicircle) {
        used_legacy_fallback = true; fallback_reason = "missing valid last-pick pose";
        RCLCPP_WARN(get_logger(), "Semicircle: no pick info → legacy fallback");
      }

      if (!semicircle_mode && !place_back_to_pick) {
        double cur_x = 0.0, cur_y = 0.0;
        { std::lock_guard<std::mutex> sl(scene_mutex_);
          auto it = tracked_objects_.find(obj_id);
          if (it != tracked_objects_.end()) {
            cur_x = it->second.pose.pose.position.x;
            cur_y = it->second.pose.pose.position.y;
          } }
        std::mt19937 rng(std::random_device{}());
        std::uniform_real_distribution<double> r_dist(0.15, 0.22);
        std::uniform_real_distribution<double> a_dist(-1.2, 1.2);
        bool found = false; double bx=0, by=0, bd=1e9;
        for (int i = 0; i < 20; ++i) {
          double r=r_dist(rng), a=a_dist(rng);
          double cx=r*std::cos(a), cy=r*std::sin(a);
          if (!isSafePlaceXY(cx, cy, obj_id, 0.06)) continue;
          double d=std::hypot(cx-cur_x, cy-cur_y);
          if (d<bd){bd=d;bx=cx;by=cy;found=true;}
        }
        if (!found){res->success=false;res->message="No safe place candidate.";return;}
        place_pose.header.frame_id=BASE_FRAME;
        place_pose.pose.position.x=bx; place_pose.pose.position.y=by;
        place_pose.pose.position.z=0.0; place_pose.pose.orientation.w=1.0;
      }
    }

    const double px = place_pose.pose.position.x;
    const double py = place_pose.pose.position.y;
    const double pz = place_back_to_pick ? place_pose.pose.position.z
                                         : (half_h + PLACE_BASE_CLEARANCE);

    RCLCPP_INFO(get_logger(),
      "Place '%s': px=%.3f py=%.3f pz=%.3f half_h=%.4f mode=%s",
      obj_id.c_str(), px, py, pz, half_h,
      semicircle_mode ? "semicircle" : (place_back_to_pick ? "back-to-pick" : "legacy"));

    auto pipeline = makePipeline();
    mtc::Task task;
    setupTask(task, "place_" + obj_id);

    double semicircle_lower_distance = 0.06;
    if (semicircle_mode) {
      moveit::core::RobotState state(task.getRobotModel());
      state.setToDefaultValues();
      for (const auto& kv : sc_joints_goal) state.setVariablePosition(kv.first, kv.second);
      state.update();
      const double hand_z = state.getGlobalLinkTransform(HAND_FRAME).translation().z();
      const double tgt_z  = half_h + PLACE_BASE_CLEARANCE;
      semicircle_lower_distance = std::clamp(hand_z - tgt_z, 0.005, 0.15);
      RCLCPP_INFO(get_logger(), "Semicircle lower: hand_z=%.3f target_z=%.3f lower=%.3f",
        hand_z, tgt_z, semicircle_lower_distance);
    }

    mtc::Stage* current_state_ptr = nullptr;
    { auto s = std::make_unique<mtc::stages::CurrentState>("current state");
      current_state_ptr = s.get(); task.add(std::move(s)); }

    if (semicircle_mode) {
      { auto ompl = makePipeline();
        auto s = std::make_unique<mtc::stages::MoveTo>("rotate to place angle", ompl);
        s->setGroup(ARM_GROUP); s->setTimeout(15.0); s->setGoal(sc_joints_goal);
        task.add(std::move(s)); }
      { auto s = std::make_unique<mtc::stages::MoveRelative>("lower to place", joint_interp_);
        s->setGroup(ARM_GROUP);
        s->setMinMaxDistance(std::max(0.005, semicircle_lower_distance - 0.002),
                             std::min(0.15,  semicircle_lower_distance + 0.002));
        s->setIKFrame(HAND_FRAME);
        geometry_msgs::msg::Vector3Stamped dir;
        dir.header.frame_id=BASE_FRAME; dir.vector.z=-1.0;
        s->setDirection(dir); task.add(std::move(s)); }
      { auto s = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_interp_);
        s->setGroup(GRIPPER_GROUP); s->setGoal(GRIPPER_OPEN); task.add(std::move(s)); }
      { auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision");
        s->allowCollisions(obj_id,
          task.getRobotModel()->getJointModelGroup(GRIPPER_GROUP)
            ->getLinkModelNamesWithCollisionGeometry(), false);
        task.add(std::move(s)); }
      { auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
        s->detachObject(obj_id, EEF_LINK); task.add(std::move(s)); }
      { auto s = std::make_unique<mtc::stages::MoveRelative>("retreat", joint_interp_);
        s->setGroup(ARM_GROUP); s->setMinMaxDistance(0.005, 0.12); s->setIKFrame(HAND_FRAME);
        geometry_msgs::msg::Vector3Stamped dir;
        dir.header.frame_id=BASE_FRAME; dir.vector.z=1.0;
        s->setDirection(dir); task.add(std::move(s)); }

    } else {
      { auto s = std::make_unique<mtc::stages::Connect>("move to place",
          mtc::stages::Connect::GroupPlannerVector{
            {ARM_GROUP, pipeline}, {GRIPPER_GROUP, joint_interp_}});
        s->setTimeout(10.0); s->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(s)); }
      {
        auto place = std::make_unique<mtc::SerialContainer>("place object");
        task.properties().exposeTo(place->properties(), {"eef","hand","group","ik_frame"});
        place->properties().configureInitFrom(mtc::Stage::PARENT,{"eef","hand","group","ik_frame"});

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
          target.header.frame_id=BASE_FRAME;
          target.pose.position.x=px; target.pose.position.y=py; target.pose.position.z=pz;
          if (place_back_to_pick) { target.pose.orientation = place_pose.pose.orientation; }
          else { tf2::Quaternion q; q.setRPY(0.0,0.0,std::atan2(py,px));
                 target.pose.orientation = tf2::toMsg(q); }
          gen->setPose(target); gen->setMonitoredStage(current_state_ptr);
          auto ik = std::make_unique<mtc::stages::ComputeIK>("place IK", std::move(gen));
          ik->setMaxIKSolutions(20); ik->setTimeout(0.5); ik->setIKFrame(HAND_FRAME);
          ik->properties().configureInitFrom(mtc::Stage::PARENT,{"eef","group"});
          ik->properties().configureInitFrom(mtc::Stage::INTERFACE,{"target_pose"});
          place->insert(std::move(ik));
        }
        { auto s = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_interp_);
          s->setGroup(GRIPPER_GROUP); s->setGoal(GRIPPER_OPEN); place->insert(std::move(s)); }
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
      { std::lock_guard<std::mutex> sl(scene_mutex_);
        auto it = tracked_objects_.find(obj_id);
        if (it != tracked_objects_.end()) {
          it->second.pose = place_pose;
          it->second.pose.pose.position.z = half_h + PLACE_BASE_CLEARANCE;
          moveit::planning_interface::PlanningSceneInterface psi;
          moveit_msgs::msg::CollisionObject co;
          co.id=obj_id; co.header.frame_id=BASE_FRAME;
          co.operation=moveit_msgs::msg::CollisionObject::MOVE;
          co.primitive_poses.push_back(it->second.pose.pose);
          shape_msgs::msg::SolidPrimitive prim;
          const auto& d=it->second.dimensions;
          if (it->second.type=="cylinder"&&d.size()>=2){
            prim.type=shape_msgs::msg::SolidPrimitive::CYLINDER;prim.dimensions.assign(d.begin(),d.end());}
          else if(it->second.type=="box"&&d.size()>=3){
            prim.type=shape_msgs::msg::SolidPrimitive::BOX;prim.dimensions.assign(d.begin(),d.end());}
          else if(it->second.type=="sphere"&&d.size()>=1){
            prim.type=shape_msgs::msg::SolidPrimitive::SPHERE;prim.dimensions.assign(d.begin(),d.end());}
          co.primitives.push_back(prim);
          psi.applyCollisionObject(co);
        } }
      attached_object_id_.clear();
      res->success = true; res->message = "Placed '" + obj_id + "'";
      if (used_legacy_fallback) res->message += " (fallback: " + fallback_reason + ")";
      else if (semicircle_mode) res->message += " (mode: semicircle joint-space)";
    } else { res->success = false; res->message = err; }
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  POUR — dynamic Cartesian arc pour
  //
  //  Geometry:
  //    source_tip  = EEF z + src_h/2        (top rim of held source)
  //    target_rim  = tgt centroid z + tgt_h/2
  //    gap         = clamp(max(src_r, tgt_r)*0.8, 0.02, 0.06)
  //
  //  Pre-pour EEF:
  //    XY: offset from target center by src_r along pour_yaw axis
  //        so the source tip starts directly above the target center
  //    Z:  EEF z = target_rim + gap - src_h/2
  //        → source tip = EEF z + src_h/2 = target_rim + gap  ✓
  //
  //  Tilt arc (N Cartesian waypoints):
  //    At tilt angle θ_i:
  //      tip_drop(θ) = src_r * (1 - cos(θ))   ← tip descends as it swings
  //      EEF Z       = pre_z - tip_drop(θ)     ← arm lowers to compensate
  //      EEF roll    = θ                        ← wrist tilts source
  //    Result: source tip stays at CONSTANT height (target_rim + gap)
  //            while rotating toward target center — liquid arc always
  //            lands inside the target container.
  //
  //  max_tilt = atan2(src_h/2, src_r) * 0.85  (85% of tip-over angle)
  //
  //  Motion sequence:
  //    1. OMPL  → approach pre-pour pose (upright, above target)
  //    2. Cartesian IK → reach pre-pour EEF pose exactly
  //    3. Cartesian arc → tilt (coupled roll + Z descent, N waypoints)
  //    4. Cartesian arc → untilt (reverse, coupled roll + Z ascent)
  //    5. JointInterp  → retreat upward
  // ═══════════════════════════════════════════════════════════════════════════
  void pourCb(const omx_interfaces::srv::Pour::Request::SharedPtr req,
              omx_interfaces::srv::Pour::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    RCLCPP_INFO(get_logger(), "Dynamic pour into '%s'", req->target_object_id.c_str());

    if (attached_object_id_.empty()) {
      res->success = false; res->message = "Not holding anything."; return;
    }

    // ── Fetch geometry ──────────────────────────────────────────────────────
    TrackedObject src, tgt;
    {
      std::lock_guard<std::mutex> sl(scene_mutex_);
      auto it1 = tracked_objects_.find(attached_object_id_);
      auto it2 = tracked_objects_.find(req->target_object_id);
      if (it1 == tracked_objects_.end() || it2 == tracked_objects_.end()) {
        res->success = false; res->message = "Source/Target not found."; return;
      }
      src = it1->second; tgt = it2->second;
    }

    const double src_h = std::max(0.04, objectHeight(src));
    const double src_r = std::max(0.01, objectRadius(src));
    const double tgt_h = std::max(0.02, objectHeight(tgt));
    const double tgt_r = std::max(0.01, objectRadius(tgt));

    // ── Pour geometry ───────────────────────────────────────────────────────
    // Gap scales with container size: large containers need more clearance
    const double gap = std::clamp(std::max(src_r, tgt_r) * 0.8, 0.02, 0.06);

    // Target rim and center
    const double tgt_rim_z = tgt.pose.pose.position.z + tgt_h / 2.0;
    const double tgt_cx    = tgt.pose.pose.position.x;
    const double tgt_cy    = tgt.pose.pose.position.y;

    // Direction from base toward target
    const double pour_yaw = std::atan2(tgt_cy, tgt_cx);

    // Pre-pour EEF position:
    // Offset EEF back by src_r along pour axis so that when the wrist tilts
    // by max_tilt, the source tip swings forward and lands over target center.
    // EEF z set so source tip is exactly gap above target rim.
    const double pour_offset = src_r + tgt_r;
    const double pre_x = tgt_cx - pour_offset * std::cos(pour_yaw);
    const double pre_y = tgt_cy - pour_offset * std::sin(pour_yaw);
    const double pre_z = tgt_rim_z + gap + src_h / 2.0;
    // Max tilt: 85% of angle to bring source fully horizontal
    const double max_tilt = std::atan2(src_h / 2.0, src_r) * 0.85;

    RCLCPP_INFO(get_logger(),
      "Pour: src_h=%.3f src_r=%.3f tgt_h=%.3f tgt_r=%.3f "
      "gap=%.3f pre=[%.3f,%.3f,%.3f] yaw=%.3f max_tilt=%.3f rad",
      src_h, src_r, tgt_h, tgt_r, gap, pre_x, pre_y, pre_z, pour_yaw, max_tilt);

    // ── Build Cartesian waypoints for tilt arc ──────────────────────────────
    // Each waypoint encodes both the wrist roll (tilt) and the compensating
    // Z descent so the source tip stays at constant height above target rim.
    constexpr int N_STEPS = 12;

    auto makeWaypoints = [&](double yaw_offset) {
      const double yaw    = pour_yaw + yaw_offset;
      const double off_x  = tgt_cx - src_r * std::cos(yaw);
      const double off_y  = tgt_cy - src_r * std::sin(yaw);

      std::vector<geometry_msgs::msg::PoseStamped> tilt_wps, untilt_wps;

      for (int i = 0; i <= N_STEPS; ++i) {
        const double theta    = max_tilt * static_cast<double>(i) / N_STEPS;
        // tip_drop: how far the source tip descends as wrist rolls by theta
        const double tip_drop = src_r * (1.0 - std::cos(theta));
        // arm lowers by tip_drop to keep tip at constant height
        const double wp_z     = pre_z - tip_drop;

        // Orientation: pour_yaw (face target) + theta (wrist roll = tilt)
        // RPY(roll=theta, pitch=0, yaw=pour_yaw) in world frame
        tf2::Quaternion q; q.setRPY(theta, 0.0, yaw);

        geometry_msgs::msg::PoseStamped wp;
        wp.header.frame_id  = BASE_FRAME;
        wp.pose.position.x  = off_x;
        wp.pose.position.y  = off_y;
        wp.pose.position.z  = wp_z;
        wp.pose.orientation = tf2::toMsg(q.normalized());
        tilt_wps.push_back(wp);
      }

      // Untilt: reverse of tilt (index N → 0)
      for (int i = N_STEPS; i >= 0; --i)
        untilt_wps.push_back(tilt_wps[i]);

      return std::make_pair(tilt_wps, untilt_wps);
    };

    // Try a few yaw offsets for IK feasibility
    const std::vector<double> yaw_offsets {0.0, 0.2, -0.2, 0.35, -0.35};
    std::string last_err = "No pour candidate succeeded.";

    for (double yaw_off : yaw_offsets) {
      auto [tilt_wps, untilt_wps] = makeWaypoints(yaw_off);

      // Pre-pour pose = first waypoint (upright, theta=0)
      const geometry_msgs::msg::PoseStamped& pre_pour_pose = tilt_wps.front();
      // Full-tilt pose = last tilt waypoint
      const geometry_msgs::msg::PoseStamped& full_tilt_pose = tilt_wps.back();
      // Back-upright = last untilt waypoint (same as pre-pour)

      auto pipeline = makePipeline();
      mtc::Task task;
      setupTask(task, "pour_" + req->target_object_id);

      mtc::Stage* current_state_ptr = nullptr;
      { auto s = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state_ptr = s.get(); task.add(std::move(s)); }

      // Phase 1: OMPL global move to vicinity of pre-pour pose
      { auto s = std::make_unique<mtc::stages::Connect>("approach pre-pour",
          mtc::stages::Connect::GroupPlannerVector{{ARM_GROUP, pipeline}});
        s->setTimeout(10.0); s->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(s)); }

      {
        auto pour_seq = std::make_unique<mtc::SerialContainer>("pour sequence");
        task.properties().exposeTo(pour_seq->properties(), {"eef","hand","group","ik_frame"});
        pour_seq->properties().configureInitFrom(mtc::Stage::PARENT,
          {"eef","hand","group","ik_frame"});

        // Phase 2: IK to reach exact pre-pour EEF pose (source upright above target)
        {
          auto gen = std::make_unique<mtc::stages::GeneratePose>("pre-pour pose");
          gen->setPose(pre_pour_pose); gen->setMonitoredStage(current_state_ptr);
          auto ik = std::make_unique<mtc::stages::ComputeIK>("pre-pour IK", std::move(gen));
          ik->setMaxIKSolutions(20); ik->setTimeout(0.5); ik->setIKFrame(HAND_FRAME);
          ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef","group"});
          ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
          pour_seq->insert(std::move(ik));
        }

        // Phase 3: Cartesian tilt arc — coupled wrist roll + Z descent
        // CartesianPath with step_size=0.005 interpolates through all
        // intermediate poses, giving smooth human-like tilting motion.
        // The Z descent exactly compensates the tip drop: source tip
        // stays at constant height (target_rim + gap) throughout the arc.
        {
          auto s = std::make_unique<mtc::stages::MoveRelative>("tilt arc", cartesian_);
          s->setGroup(ARM_GROUP); s->setIKFrame(HAND_FRAME);
          // Allow generous min/max distance — Cartesian covers the arc length
          s->setMinMaxDistance(0.001, 1.0);
          // Direction: displacement from pre-pour to full-tilt pose
          // CartesianPath interpolates orientation (roll) smoothly alongside position
          geometry_msgs::msg::TwistStamped twist;
          twist.header.frame_id = BASE_FRAME;
          twist.twist.linear.x  = full_tilt_pose.pose.position.x - pre_pour_pose.pose.position.x;
          twist.twist.linear.y  = full_tilt_pose.pose.position.y - pre_pour_pose.pose.position.y;
          twist.twist.linear.z  = full_tilt_pose.pose.position.z - pre_pour_pose.pose.position.z;
          // Angular: total roll = max_tilt around world X axis (tilt direction)
          twist.twist.angular.x = max_tilt;
          twist.twist.angular.y = 0.0;
          twist.twist.angular.z = 0.0;
          s->setDirection(twist);
          pour_seq->insert(std::move(s));
        }

        // Phase 4: Cartesian untilt arc — reverse (roll back, Z ascent)
        // Exact mirror of Phase 3: arm rises as wrist un-tilts.
        // Source tip stays at constant height throughout.
        {
          auto s = std::make_unique<mtc::stages::MoveRelative>("untilt arc", cartesian_);
          s->setGroup(ARM_GROUP); s->setIKFrame(HAND_FRAME);
          s->setMinMaxDistance(0.001, 1.0);
          geometry_msgs::msg::TwistStamped twist;
          twist.header.frame_id = BASE_FRAME;
          // Rise back up by the same tip_drop amount
          twist.twist.linear.z  = src_r * (1.0 - std::cos(max_tilt));
          twist.twist.linear.x  = 0.0;
          twist.twist.linear.y  = 0.0;
          twist.twist.angular.x = -max_tilt;  // roll back to 0
          twist.twist.angular.y = 0.0;
          twist.twist.angular.z = 0.0;
          s->setDirection(twist);
          pour_seq->insert(std::move(s));
        }

        task.add(std::move(pour_seq));
      }

      // Phase 5: retreat upward after pour
      { auto s = std::make_unique<mtc::stages::MoveRelative>("retreat", joint_interp_);
        s->setGroup(ARM_GROUP); s->setMinMaxDistance(0.05, 0.15); s->setIKFrame(HAND_FRAME);
        geometry_msgs::msg::Vector3Stamped dir;
        dir.header.frame_id=BASE_FRAME; dir.vector.z=1.0;
        s->setDirection(dir); task.add(std::move(s)); }

      std::string err;
      if (planAndExecute(task, err, 8)) {
        res->success = true;
        res->message = "Pour completed (dynamic Cartesian arc, yaw_off=" +
                       std::to_string(yaw_off) + ")";
        return;
      }
      last_err = err;
      RCLCPP_WARN(get_logger(), "Pour yaw_off=%.2f failed: %s", yaw_off, err.c_str());
    }

    res->success = false;
    res->message = "Dynamic pour failed: " + last_err;
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  ROTATE
  // ═══════════════════════════════════════════════════════════════════════════
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

  // ═══════════════════════════════════════════════════════════════════════════
  //  GO HOME
  // ═══════════════════════════════════════════════════════════════════════════
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

  // ═══════════════════════════════════════════════════════════════════════════
  //  ADD OBJECTS
  // ═══════════════════════════════════════════════════════════════════════════
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
      co.id=obj.id; co.header.frame_id=BASE_FRAME;
      co.operation=moveit_msgs::msg::CollisionObject::ADD;
      shape_msgs::msg::SolidPrimitive prim;
      if      (obj.type=="box")      prim.type=shape_msgs::msg::SolidPrimitive::BOX;
      else if (obj.type=="cylinder") prim.type=shape_msgs::msg::SolidPrimitive::CYLINDER;
      else if (obj.type=="sphere")   prim.type=shape_msgs::msg::SolidPrimitive::SPHERE;
      else { res->success=false; res->message="Unknown type '"+obj.type+"'"; return; }
      prim.dimensions.assign(obj.dimensions.begin(), obj.dimensions.end());
      co.primitives.push_back(prim);

      { double hh=0.0; const auto& d=obj.dimensions;
        if (obj.type=="cylinder"&&d.size()>=1) hh=d[0]/2.0;
        else if(obj.type=="box"&&d.size()>=3) hh=d[2]/2.0;
        else if(obj.type=="sphere"&&d.size()>=1) hh=d[0];
        if(obj.pose.pose.position.z<hh){
          RCLCPP_WARN(get_logger(),"Clamping '%s' z %.3f→%.3f",
            obj.id.c_str(),obj.pose.pose.position.z,hh);
          obj.pose.pose.position.z=hh;
        } }
      co.primitive_poses.push_back(obj.pose.pose);
      cos.push_back(co);

      TrackedObject to; to.id=obj.id; to.type=obj.type; to.pose=obj.pose;
      to.dimensions=std::vector<double>(obj.dimensions.begin(),obj.dimensions.end());
      tracked_objects_[obj.id]=to;
    }
    psi.applyCollisionObjects(cos);
    res->success=true;
    res->message="Added "+std::to_string(req->objects.size())+" object(s).";
    RCLCPP_INFO(get_logger(),"%s",res->message.c_str());
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  REMOVE OBJECTS
  // ═══════════════════════════════════════════════════════════════════════════
  void removeObjectsCb(const omx_interfaces::srv::RemoveObjects::Request::SharedPtr req,
                       omx_interfaces::srv::RemoveObjects::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> sl(scene_mutex_);
    moveit::planning_interface::PlanningSceneInterface psi;
    std::vector<std::string> to_rm;
    if(req->clear_all) for(auto&[id,_]:tracked_objects_) to_rm.push_back(id);
    else to_rm=std::vector<std::string>(req->object_ids.begin(),req->object_ids.end());
    std::vector<moveit_msgs::msg::CollisionObject> cos;
    for(const auto& id:to_rm){
      moveit_msgs::msg::CollisionObject co;
      co.id=id; co.operation=moveit_msgs::msg::CollisionObject::REMOVE;
      cos.push_back(co); tracked_objects_.erase(id);
    }
    if(!cos.empty()) psi.applyCollisionObjects(cos);
    res->success=true;
    res->message="Removed "+std::to_string(to_rm.size())+" object(s).";
    RCLCPP_INFO(get_logger(),"%s",res->message.c_str());
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  GET OBJECTS
  // ═══════════════════════════════════════════════════════════════════════════
  void getObjectsCb(const omx_interfaces::srv::GetObjects::Request::SharedPtr,
                    omx_interfaces::srv::GetObjects::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> sl(scene_mutex_);
    for(auto&[id,obj]:tracked_objects_){
      res->object_ids.push_back(id);
      res->poses.push_back(obj.pose);
      res->object_types.push_back(obj.type);
    }
  }
};

// ── Main ──────────────────────────────────────────────────────────────────────
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