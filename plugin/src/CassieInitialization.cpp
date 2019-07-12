#include "CassieInitialization.h"

#include <ignition/math/Vector3.hh>

using ignition::math::Vector3d;
using ignition::math::Quaterniond;

namespace gazebo {
void CassieInitialization::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf_elem) {
  // Store the pointer to the model
  model_ = _parent;

  // Initialize the link
  Vector3d position = Vector3d(0, 0, 2);
  Quaterniond quat = Quaterniond(1, 0, 0, 0);
  model_->SetLinkWorldPose(ignition::math::Pose3d(position, quat),
                                "pelvis");

  std::array<physics::JointPtr, 10> motors;
  motors = {
      model_->GetJoint("left-roll-op"),
      model_->GetJoint("left-yaw-op"),
      model_->GetJoint("left-pitch-op"),
      model_->GetJoint("left-knee-op"),
      model_->GetJoint("left-foot-op"),
      model_->GetJoint("right-roll-op"),
      model_->GetJoint("right-yaw-op"),
      model_->GetJoint("right-pitch-op"),
      model_->GetJoint("right-knee-op"),
      model_->GetJoint("right-foot-op")
  };

  std::array<physics::JointPtr, 6> unactuated_joints;
  unactuated_joints = {
        model_->GetJoint("left-knee-shin-joint"),
        model_->GetJoint("left-shin-tarsus-joint"),
        model_->GetJoint("left-foot-op"),
        model_->GetJoint("right-knee-shin-joint"),
        model_->GetJoint("right-shin-tarsus-joint"),
        model_->GetJoint("right-foot-op")
  };

  std::array<double, 10> motor_positions;
  std::array<double, 10> motor_velocities;
  motor_positions = {0.1, 0.1, 0.1, -2.07694, -2.4435, 0.1, 0.1, 0.1, -2.07694, -2.4435};
  motor_velocities = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  std::array<double, 6> unactuated_joint_positions;
  std::array<double, 6> unactuated_joint_velocities;
  unactuated_joint_positions = {0, 0, -1, 0, 0, -1};
  unactuated_joint_velocities = {1, 1, -1, 1, 1, -1};

  for(int i=0; i<motors.size(); i++) {
    std::cout << motors[i]->GetName() << std::endl;
    if (motors[i]->LowerLimit(0) <= motor_positions[i] &&
        motors[i]->UpperLimit(0) >= motor_positions[i]) {
      std::cout << "Is success: "
                << motors[i]->SetPosition(0, motor_positions[i]) << std::endl;
    } else std::cout << "Joint limit violated for " << motors[i]->GetName() << std::endl;
    motors[i]->SetVelocity(0, motor_velocities[i]);
  }

  for(int i=0; i<unactuated_joints.size(); i++) {
    std::cout << unactuated_joints[i]->GetName() << std::endl;
    if (unactuated_joints[i]->LowerLimit(0) <= unactuated_joint_positions[i] &&
        unactuated_joints[i]->UpperLimit(0) >= unactuated_joint_positions[i]) {
      std::cout << "Is success: "
                << unactuated_joints[i]->SetPosition(
                       0, unactuated_joint_positions[i])
                << std::endl;
    } else
      std::cout << "Joint limit violated for " << unactuated_joints[i]->GetName()
                << std::endl;
    unactuated_joints[i]->SetVelocity(0, unactuated_joint_velocities[i]);
  }
}
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(CassieInitialization)
}  // namespace gazebo
