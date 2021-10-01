#pragma once

#include <BasicLinearAlgebra.h>
#include "utils.h"

struct KinematicsConfig
{
  float l1;
  float l2;
  float l3;
};

BLA::Matrix<3> forward_kinematics(const BLA::Matrix<3> &joint_angles, const KinematicsConfig &config)
{
  /* Computes forward kinematics for the 3DOF robot arm/leg.
  
  Returns the cartesian coordinates of the end-effector 
  corresponding to the given joint angles and leg configuration. 
  */
  return BLA::Matrix<3>(0, 0, 0);
}

BLA::Matrix<3> inverse_kinematics(const BLA::Matrix<3> &target_location, const KinematicsConfig &config)
{
  return BLA::Matrix<3>(0, 0, 0);
}

enum class BodySide
{
  kLeft,
  kRight,
  kUnspecified
};

BLA::Matrix<3> correct_for_actuator_direction(const BLA::Matrix<3> &joint_vector, const BodySide &side)
{
  if (side == BodySide::kLeft)
  {
    return {-joint_vector(0), joint_vector(1), -joint_vector(2)};
  }
  else if (side == BodySide::kRight)
  {
    return {-joint_vector(0), -joint_vector(1), joint_vector(2)};
  }
  else
  {
    Serial.println("ERROR. Unspecified body side.");
    assert(false);
    return {0, 0, 0};
  }
}
