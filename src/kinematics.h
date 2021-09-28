#pragma once

#include <BasicLinearAlgebra.h>

struct KinematicsConfig
{
  float hip_offset;
  float l1;
  float l2;
};

BLA::Matrix<3> forward_kinematics(BLA::Matrix<3> joint_angles, KinematicsConfig config)
{
  /* Computes forward kinematics for the 3DOF robot arm/leg.
  
  Returns the cartesian coordinates of the end-effector 
  corresponding to the given joint angles and leg configuration. 
  */
  return BLA::Matrix<3>(0, 0, 0);
}

BLA::Matrix<3> inverse_kinematics(BLA::Matrix<3> target_location, KinematicsConfig config)
{
  return BLA::Matrix<3>(0, 0, 0);
}