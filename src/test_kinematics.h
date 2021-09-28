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

void assert(bool val)
{
   if (!val)
   {
      Serial.println("ASSERTION FAILED. STOPPING PROGRAM.");
      while (true)
      {
      }
   }
}
void assert_close(float a, float b, float eps = 1e-6)
{
   assert(abs(a - b) > eps);
}

void test_neutral()
{
   auto config = KinematicsConfig{0.5, 1.0, 1.0};
   auto joint_angles = BLA::Matrix<3>(0, 0, 0);
   auto r = forward_kinematics(joint_angles, config);
   assert_close(r(0), 0.0);
   assert_close(r(1), 0.5);
   assert_close(r(2), -2.0);
}

void test_bent()
{
   auto config = KinematicsConfig{0.5, 1.0, 1.0};
   auto joint_angles = BLA::Matrix<3>(0, PI / 4, -PI / 2);
   auto r = forward_kinematics(joint_angles, config);
   assert_close(r(0), 0.0);
   assert_close(r(1), 0.5);
   assert_close(r(2), sqrt(2.0));
}

void test_outward()
{
   auto config = KinematicsConfig{0.5, 1.0, 1.0};
   auto joint_angles = BLA::Matrix<3>(PI/4, 0, 0);
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

/*
import unittest
import numpy as np
from quadruped_lib import kinematics
from quadruped_lib import kinematics_config


class TestKinematics(unittest.TestCase):
    def TestInteriorAngle(self):
        with self.assertRaises(ValueError):
            kinematics.interior_angle(-1, 0, 0)
        with self.assertRaises(ArithmeticError):
            kinematics.interior_angle(1, 1, 5)
        with self.assertRaises(ArithmeticError):
            kinematics.interior_angle(2, 1, 0.5)
        self.assertAlmostEqual(kinematics.interior_angle(1, 1, 2 + 1e-9), np.pi)
        self.assertAlmostEqual(kinematics.interior_angle(2, 1, 1 - 1e-9), 0)
        self.assertAlmostEqual(kinematics.interior_angle(1, 1, 2 ** 0.5), np.pi / 2)
        self.assertAlmostEqual(
            kinematics.interior_angle(0.5, 1, 3 ** 0.5 / 2), np.pi / 3
        )

    def TestZeroConfiguration(self):
        for l1 in [0.2, 0.5, 1.0]:
            config = kinematics_config.KinematicsConfig(
                abduction_offset=0.05,
                upper_link_length=l1,
                lower_link_length=0.1,
                hip_x_offset=0,
                hip_y_offset=0,
            )
            r = np.array([0, -0.05, -0.1 - l1])
            joint_angles = kinematics.leg_inverse_kinematics_relative_to_hip(
                r, 0, config
            )
            expected_joint_angles = np.array([0, 0, 0])
            self.assertTrue(np.allclose(joint_angles, expected_joint_angles))

    def TestStandingConfiguration(self):
        config = kinematics_config.KinematicsConfig(
            abduction_offset=0.05,
            upper_link_length=0.1,
            lower_link_length=0.1,
            hip_x_offset=0,
            hip_y_offset=0,
        )
        r = np.array([0, -0.05, -0.2 * 2 ** 0.5 / 2])
        joint_angles = kinematics.leg_inverse_kinematics_relative_to_hip(r, 0, config)
        expected_joint_angles = np.array([0, np.pi / 4, -np.pi / 2])
        self.assertTrue(np.allclose(joint_angles, expected_joint_angles))

    def TestArbitraryConfigsInverseKinematics(self):
        config = kinematics_config.KinematicsConfig(
            abduction_offset=0.05,
            upper_link_length=1,
            lower_link_length=1,
            hip_x_offset=0,
            hip_y_offset=0,
        )
        # upper link 45 degs backward, lower link vertical
        r = np.array([-(2 ** 0.5) / 2, -0.05, -(2 ** 0.5) / 2 - 1])
        joint_angles = kinematics.leg_inverse_kinematics_relative_to_hip(r, 0, config)
        expected_joint_angles = np.array([0, np.pi / 4, -np.pi / 4])
        self.assertTrue(np.allclose(joint_angles, expected_joint_angles))

        # hip rotated horizontally outwards
        r = np.array([0, -2, 0.05])
        joint_angles = kinematics.leg_inverse_kinematics_relative_to_hip(r, 0, config)
        expected_joint_angles = np.array([-np.pi / 2, 0, 0])
        self.assertTrue(np.allclose(joint_angles, expected_joint_angles))

        # leg rotated forwards to horizontal
        r = np.array([2, -0.05, 0])
        joint_angles = kinematics.leg_inverse_kinematics_relative_to_hip(r, 0, config)
        expected_joint_angles = np.array([0, -np.pi / 2, 0])
        self.assertTrue(np.allclose(joint_angles, expected_joint_angles))

        # leg rotated backwards to horizontal
        r = np.array([-2, -0.05, 0])
        joint_angles = kinematics.leg_inverse_kinematics_relative_to_hip(r, 0, config)
        expected_joint_angles = np.array([0, np.pi / 2, 0])
        self.assertTrue(np.allclose(joint_angles, expected_joint_angles))

    def TestForwardInverseConsistency(self):
        config = kinematics_config.KinematicsConfig(
            abduction_offset=0.05,
            upper_link_length=0.1,
            lower_link_length=0.1,
            hip_x_offset=0,
            hip_y_offset=0,
        )
        for leg_index in range(4):
            for i in range(100):
                cartesian_targets = (
                    np.random.rand() * 0.2 - 0.1,
                    np.random.rand() * 0.1 - 0.05,
                    -np.random.rand() * 0.1 - 0.05,
                )
                joint_angles = kinematics.leg_inverse_kinematics_relative_to_hip(
                    cartesian_targets, leg_index=leg_index, config=config
                )
                estimated_cartesian = kinematics.leg_forward_kinematics_relative_to_hip(
                    joint_angles, leg_index=leg_index, config=config, 
                )
                self.assertTrue(np.allclose(cartesian_targets, estimated_cartesian))
*/