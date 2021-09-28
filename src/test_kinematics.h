/*
 Copyright (c) 2014-present PlatformIO <contact@platformio.org>
 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
**/
#pragma once
#include <kinematics.h>
#include <utils.h>

void test_neutral()
{
  auto config = KinematicsConfig{0.5, 1.0, 1.0};
  auto joint_angles = BLA::Matrix<3>(0, 0, 0);
  Serial << "Testing kinematics for joint angles = " << joint_angles << "\n";
  auto r = forward_kinematics(joint_angles, config);
  assert_close(r(0), 0.0);
  assert_close(r(1), 0.5);
  assert_close(r(2), -2.0);
}

void test_bent()
{
  auto config = KinematicsConfig{0.5, 1.0, 1.0};
  auto joint_angles = BLA::Matrix<3>(0, PI / 4, -PI / 2);
  Serial << "Testing kinematics for joint angles = " << joint_angles << "\n";
  auto r = forward_kinematics(joint_angles, config);
  assert_close(r(0), 0.0);
  assert_close(r(1), 0.5);
  assert_close(r(2), -sqrt(2.0));
}

void test_outward()
{
  auto config = KinematicsConfig{0.5, 1.0, 1.0};
  auto joint_angles = BLA::Matrix<3>(PI / 2, 0, 0);
  Serial << "Testing kinematics for joint angles = " << joint_angles << "\n";
  auto r = forward_kinematics(joint_angles, config);
  assert_close(r(0), 0.0);
  assert_close(r(1), 2.0);
  assert_close(r(2), 0.5);
}

void test_kinematics()
{
  test_neutral();
  test_bent();
  test_outward();
}