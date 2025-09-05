#include "ros_interface.h"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>

#include "config_loader.h"
#include "rclcpp/rclcpp.hpp"

// Constants
const int kDofFloatingBase = 6;        // Number of DoF for floating base
const int kNumFloatingBaseJoints = 7;  // Number of joints for floating base (quaternion + xyz)
const int kDimQuaternion = 4;          // Dimension of a quaternion

namespace mujoco {

RosInterface::RosInterface(const rclcpp::Node::SharedPtr& node, std::shared_ptr<ConfigLoader> config_loader)
    : node_(node), config_loader_(config_loader), model_(nullptr), data_(nullptr), is_floating_base_(false) {}

RosInterface::~RosInterface() {}

bool RosInterface::Initialize() {
  // Declare tunable parameters (can be extended later)
  if (!node_->has_parameter("base_height")) {
    node_->declare_parameter<double>("base_height", base_height_);
  }
  node_->get_parameter("base_height", base_height_);
  base_height_ = std::clamp(base_height_, 0.3, 1.5);
  // Create publishers
  joint_state_pub_ =
      node_->create_publisher<interface_protocol::msg::JointState>(config_loader_->GetJointStateTopic(), 10);

  imu_pub_ = node_->create_publisher<interface_protocol::msg::ImuInfo>(config_loader_->GetImuTopic(), 10);
  
  // Create publisher for motion state
  motion_state_pub_ = node_->create_publisher<interface_protocol::msg::MotionState>("/motion/motion_state", 10);

  // Create subscriber with more compatible QoS settings
  using std::placeholders::_1;

  // 创建更兼容的QoS设置
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

  joint_cmd_sub_ = node_->create_subscription<interface_protocol::msg::JointCommand>(
    config_loader_->GetJointCommandTopic(), qos, std::bind(&RosInterface::JointCommandCallback, this, _1));
  body_vel_sub_ = node_->create_subscription<interface_protocol::msg::BodyVelCmd>(
    "/motion/body_vel_cmd", qos, std::bind(&RosInterface::BodyVelCmdCallback, this, _1));

  // Get number of joints from config loader
  num_total_joints_ = config_loader_->GetNumTotalJoints();

  // Initialize commanded values with a default stand pose (set later after first state)
  joint_command_.position.resize(num_total_joints_, 0.0);
  joint_command_.velocity.resize(num_total_joints_, 0.0);
  joint_command_.torque.resize(num_total_joints_, 0.0);
  joint_command_.feed_forward_torque.resize(num_total_joints_, 0.0);
  joint_command_.stiffness.resize(num_total_joints_, 300.0);
  joint_command_.damping.resize(num_total_joints_, 5.0);

  // Create timers
  motion_state_timer_ = node_->create_wall_timer(
    std::chrono::seconds(1), std::bind(&RosInterface::MotionStateTimerCallback, this));
  gait_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(10),  // 100 Hz gait update
    std::bind(&RosInterface::GaitTimerCallback, this));

  // Initialize time stamps to consistent clock type to avoid subtraction errors
  last_joint_cmd_time_ = node_->now();
  body_vel_state_.stamp = node_->now();

  RCLCPP_INFO(node_->get_logger(), "MuJoCo ROS interface initialized successfully");
  return true;
}

interface_protocol::msg::JointCommand RosInterface::GetCommandedSafe() {
  std::lock_guard<std::mutex> lock(mtx_);
  return joint_command_;
}

void RosInterface::JointCommandCallback(const interface_protocol::msg::JointCommand::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_);

  // Update commanded values
  joint_command_ = *msg;

  // Ensure all vectors are properly sized
  if (joint_command_.position.size() > num_total_joints_) {
    joint_command_.position.resize(num_total_joints_);
  }
  if (joint_command_.velocity.size() > num_total_joints_) {
    joint_command_.velocity.resize(num_total_joints_);
  }
  if (joint_command_.torque.size() > num_total_joints_) {
    joint_command_.torque.resize(num_total_joints_);
  }
  if (joint_command_.feed_forward_torque.size() > num_total_joints_) {
    joint_command_.feed_forward_torque.resize(num_total_joints_);
  }
  if (joint_command_.stiffness.size() > num_total_joints_) {
    joint_command_.stiffness.resize(num_total_joints_);
  }
  if (joint_command_.damping.size() > num_total_joints_) {
    joint_command_.damping.resize(num_total_joints_);
  }
  received_explicit_joint_cmd_ = true;
  last_joint_cmd_time_ = node_->now();
}

void RosInterface::BodyVelCmdCallback(const interface_protocol::msg::BodyVelCmd::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  body_vel_state_.vx = msg->linear_velocity.size() > 0 ? msg->linear_velocity[0] : 0.0;
  body_vel_state_.vy = msg->linear_velocity.size() > 1 ? msg->linear_velocity[1] : 0.0;
  body_vel_state_.yaw = msg->yaw_velocity;
  body_vel_state_.stamp = node_->now();
}

void RosInterface::UpdateSimState(const mjModel* m, mjData* d) {
  is_floating_base_ = (m->nv != m->nu);

  // Create messages
  auto joint_state_msg = std::make_unique<interface_protocol::msg::JointState>();
  auto imu_msg = std::make_unique<interface_protocol::msg::ImuInfo>();

  // Set timestamp
  joint_state_msg->header.stamp = node_->now();
  imu_msg->header.stamp = node_->now();

  // Set joint states
  joint_state_msg->position.resize(num_total_joints_);
  joint_state_msg->velocity.resize(num_total_joints_);
  joint_state_msg->torque.resize(num_total_joints_);

  if (is_floating_base_) {
    // Skip the floating base joints
    for (int i = 0; i < num_total_joints_; ++i) {
      joint_state_msg->position[i] = d->qpos[i + kNumFloatingBaseJoints];
      joint_state_msg->velocity[i] = d->qvel[i + kDofFloatingBase];
      joint_state_msg->torque[i] = d->actuator_force[i];
    }
  } else {
    for (int i = 0; i < num_total_joints_; ++i) {
      joint_state_msg->position[i] = d->qpos[i];
      joint_state_msg->velocity[i] = d->qvel[i];
      joint_state_msg->torque[i] = d->actuator_force[i];
    }
  }

  // IMU data typically comes from sensors in MuJoCo
  int index = 0;

  // Set IMU quaternion
  imu_msg->quaternion.w = d->sensordata[index + 0];
  imu_msg->quaternion.x = d->sensordata[index + 1];
  imu_msg->quaternion.y = d->sensordata[index + 2];
  imu_msg->quaternion.z = d->sensordata[index + 3];
  index += kDimQuaternion;

  // Set RPY values from the sensor data
  // Assuming the RPY values are the next three values after the quaternion
  imu_msg->rpy.x = d->sensordata[index + 0];  // Roll
  imu_msg->rpy.y = d->sensordata[index + 1];  // Pitch
  imu_msg->rpy.z = d->sensordata[index + 2];  // Yaw
  index += 3;

  // Linear acceleration
  imu_msg->linear_acceleration.x = d->sensordata[index + 0];
  imu_msg->linear_acceleration.y = d->sensordata[index + 1];
  imu_msg->linear_acceleration.z = d->sensordata[index + 2];
  index += 3;

  // Angular velocity
  imu_msg->angular_velocity.x = d->sensordata[index + 0];
  imu_msg->angular_velocity.y = d->sensordata[index + 1];
  imu_msg->angular_velocity.z = d->sensordata[index + 2];

  // Publish messages
  joint_state_pub_->publish(std::move(joint_state_msg));
  imu_pub_->publish(std::move(imu_msg));
}

void RosInterface::SetModelAndData(mjModel* model, mjData* data) {
  std::lock_guard<std::mutex> lock(mtx_);
  model_ = model;
  data_ = data;
  neutral_initialized_ = false; // defer initialization to gait timer to avoid recursive callback deadlock
}

void RosInterface::MotionStateTimerCallback() {
  // Create a motion state message
  auto motion_state_msg = std::make_unique<interface_protocol::msg::MotionState>();
  
  // Set the current_motion_task field to "joint_bridge"
  motion_state_msg->current_motion_task = "joint_bridge";
  
  // Publish the message
  motion_state_pub_->publish(std::move(motion_state_msg));
}

void RosInterface::InitializeNeutralStandPose() {
  if (!data_ || !model_ || num_total_joints_ == 0) return;
  if (num_total_joints_ < 0) return;
  // Build neutral pose from joint_test.yaml target positions (concatenated groups)
  std::vector<double> pose = {0.0,  0.5,  1.57, 0.6,  -0.3, 0.0,  // leg group 1
                              -0.0, -0.5, -1.57, 0.6, -0.3, 0.0,  // leg group 2
                              0.0,                                 // torso
                              0.0,  0.3,  0.0,  -0.4, 0.0,         // left arm
                              0.0, -0.2,  0.0,  -0.3, 0.0,         // right arm
                              0.0};                                // head
  if ((int)pose.size() == num_total_joints_) {
    neutral_pose_ = pose;
  } else {
    neutral_pose_.assign(num_total_joints_, 0.0);
  }
  // Floating base: set upright base orientation and height
  if (model_->nv != model_->nu && model_->nq >= 7 && data_->qpos) {
    // Ensure index access inside bounds (MuJoCo guarantees nQ >=7 for free joint) but add guard.
    data_->qpos[0] = 1.0; // qw
    data_->qpos[1] = 0.0; // qx
    data_->qpos[2] = 0.0; // qy
    data_->qpos[3] = 0.0; // qz
    data_->qpos[4] = 0.0; // x
    data_->qpos[5] = 0.0; // y
  data_->qpos[6] = base_height_; // initial guess (may be adjusted below)
  }
  // Apply neutral pose to commanded joints
  for (int i = 0; i < num_total_joints_; ++i) {
    joint_command_.position[i] = neutral_pose_[i];
    joint_command_.velocity[i] = 0.0;
  }

  // (simplified) no ground alignment logic
}


void RosInterface::ApplyStandPoseIfIdle() {
  // If no explicit joint command in last 0.5s, hold stand pose
  if (!last_joint_cmd_time_.nanoseconds()) {
    last_joint_cmd_time_ = node_->now();
    return;
  }
  if ((node_->now() - last_joint_cmd_time_).seconds() > 0.5) {
    // velocities stay zero, stiffness/damping already set
  }
}

void RosInterface::GaitTimerCallback() {
  std::unique_lock<std::mutex> lock(mtx_);
  if (!neutral_initialized_) {
    if (model_ && data_) {
      InitializeNeutralStandPose();
      // store local copies and release lock before mj_forward to avoid recursive lock via control callback
      auto* m = model_;
      auto* d = data_;
      lock.unlock();
      mj_forward(m, d);
      lock.lock();
  neutral_initialized_ = true;
    } else {
      return; // wait
    }
  }
  ApplyStandPoseIfIdle();

  // Gait modulation around neutral pose without drift
  bool have_recent_body_cmd = body_vel_state_.stamp.nanoseconds() != 0 &&
      (node_->now() - body_vel_state_.stamp).seconds() < 0.2;
  if (!neutral_pose_.empty() && have_recent_body_cmd && num_total_joints_ >= 12) {
    double speed_scale = std::clamp(std::abs(body_vel_state_.vx) / 0.5, 0.2, 1.0);
    gait_phase_ += 0.02 * speed_scale;  // phase increment
    double step_amp = 0.15 * std::clamp(body_vel_state_.vx / 0.5, -1.0, 1.0);
    constexpr double kPi = 3.141592653589793;
    // Use hip pitch joints (indices 2 and 8) as simple example
    int hip1 = 2;
    int hip2 = 8;
    if (hip1 < num_total_joints_) {
      joint_command_.position[hip1] = neutral_pose_[hip1] + step_amp * std::sin(gait_phase_);
    }
    if (hip2 < num_total_joints_) {
      joint_command_.position[hip2] = neutral_pose_[hip2] + step_amp * std::sin(gait_phase_ + kPi);
    }
  } else if (!neutral_pose_.empty()) {
    // Idle limb animation (e.g., shoulders) to show simulation is responsive
    gait_phase_ += 0.01;
    int l_shoulder = 13; // approximate index for first arm joint after torso
    int r_shoulder = 18; // approximate index for right arm counterpart (depends on model ordering)
    if (l_shoulder < num_total_joints_) {
      joint_command_.position[l_shoulder] = neutral_pose_[l_shoulder] + 0.2 * std::sin(gait_phase_);
    }
    if (r_shoulder < num_total_joints_) {
      joint_command_.position[r_shoulder] = neutral_pose_[r_shoulder] + 0.2 * std::sin(gait_phase_ + 3.14159);
    }
  }
}

}  // namespace mujoco
