#ifndef MUJOCO_ROS_INTERFACE_H_
#define MUJOCO_ROS_INTERFACE_H_

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "interface_protocol/msg/imu_info.hpp"
#include "interface_protocol/msg/joint_command.hpp"
#include "interface_protocol/msg/body_vel_cmd.hpp"
#include "interface_protocol/msg/joint_state.hpp"
#include "interface_protocol/msg/motion_state.hpp"
#include "rclcpp/rclcpp.hpp"

// MuJoCo includes
#include <mujoco/mujoco.h>

// Forward declarations
namespace mujoco {
class Simulate;
}
class ConfigLoader;

namespace mujoco {

class RosInterface {
 public:
  RosInterface(const rclcpp::Node::SharedPtr& node, std::shared_ptr<ConfigLoader> config_loader);
  ~RosInterface();

  // Initialize the MuJoCo interface
  bool Initialize();

  // Callback for joint command messages
  void JointCommandCallback(const interface_protocol::msg::JointCommand::SharedPtr msg);
  // Callback for high-level body velocity command
  void BodyVelCmdCallback(const interface_protocol::msg::BodyVelCmd::SharedPtr msg);

  // Update the simulation state to publish to ROS
  void UpdateSimState(const mjModel* m, mjData* d);

  // Get joint command values (thread-safe)
  interface_protocol::msg::JointCommand GetCommandedSafe();

  // Set the current mjModel and mjData
  void SetModelAndData(mjModel* model, mjData* data);

  // Get the ROS node
  rclcpp::Node::SharedPtr GetNode() const { return node_; }

 private:
  // ROS2 node
  rclcpp::Node::SharedPtr node_;

  // Publishers
  rclcpp::Publisher<interface_protocol::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<interface_protocol::msg::ImuInfo>::SharedPtr imu_pub_;
  rclcpp::Publisher<interface_protocol::msg::MotionState>::SharedPtr motion_state_pub_;

  // Subscribers
  rclcpp::Subscription<interface_protocol::msg::JointCommand>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<interface_protocol::msg::BodyVelCmd>::SharedPtr body_vel_sub_;

  // Config loader
  std::shared_ptr<ConfigLoader> config_loader_;

  // Number of joints
  int num_total_joints_ = 0;

  // Current joint command
  interface_protocol::msg::JointCommand joint_command_;
  // High-level body velocity state
  struct BodyVelState {
    double vx{0.0};
    double vy{0.0};
    double yaw{0.0};
    rclcpp::Time stamp{};
  } body_vel_state_;

  // MuJoCo model and data
  mjModel* model_;
  mjData* data_;

  // Timers
  rclcpp::TimerBase::SharedPtr motion_state_timer_;
  rclcpp::TimerBase::SharedPtr gait_timer_;
  
  // Motion state timer callback
  void MotionStateTimerCallback();
  // Periodic gait synthesis based on high-level body velocity
  void GaitTimerCallback();
  void InitializeNeutralStandPose();
  void ApplyStandPoseIfIdle();
  // (simplified) no automatic ground alignment; base height taken directly from parameter

  // Mutex for thread safety
  std::mutex mtx_;

  // Flag indicating if we have a floating base robot
  bool is_floating_base_;
  bool received_explicit_joint_cmd_{false};
  rclcpp::Time last_joint_cmd_time_;
  double gait_phase_{0.0};
  std::vector<double> neutral_pose_;
  bool neutral_initialized_{false};
  double base_height_{1.1};  // spawn height
  bool pd_active_{false};
  rclcpp::Time neutral_init_time_;
};

}  // namespace mujoco

#endif  // MUJOCO_ROS_INTERFACE_H_
