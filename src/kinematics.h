#pragma once

#include <BasicLinearAlgebra.h>

struct KinematicsConfig
{
  float hip_offset;
  float l1;
  float l2;
};

// BLA::Matrix<3> forward_kinematics(BLA::Matrix<3> joint_angles, KinematicsConfig config)
// {
//   return BLA::Matrix<3>(0, 0, 0);
// }

BLA::Matrix<3> inverse_kinematics(BLA::Matrix<3> target_location, KinematicsConfig config)
{
  return BLA::Matrix<3>(0, 0, 0);
}

BLA::Matrix<3, 3> RotateX(float theta) {
  return {1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta)};
}

BLA::Matrix<3> forward_kinematics(BLA::Matrix<3> joint_angles,
                                 KinematicsConfig leg_params) {
  float l1 = leg_params.l1;
  float l2 = leg_params.l2;

  float alpha = joint_angles(0);
  float theta = joint_angles(1);
  float phi = joint_angles(2);

  float px = -l1 * sin(theta) - l2 * sin(theta + phi);
  float py = leg_params.hip_offset; // UPDATE
  float pz = -l1 * cos(theta) - l2 * cos(theta + phi);

  BLA::Matrix<3> tilted_frame_coordinates = {px, py, pz};
  BLA::Matrix<3> cartesian_coordinates =
      RotateX(alpha) * tilted_frame_coordinates;
  return cartesian_coordinates;
}