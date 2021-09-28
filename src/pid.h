#pragma once

#include "Arduino.h"
#include "BasicLinearAlgebra.h"

// Step 5. Implement your own PD controller here.
float pd_control(float pos,
                 float vel,
                 float target,
                 float Kp,
                 float Kd)
{
  return -Kp * (pos - target) - Kd * vel;
}

template <int n>
BLA::Matrix<n> vectorized_pd(const BLA::Matrix<n> &pos,
                             const BLA::Matrix<n> &vel,
                             const BLA::Matrix<n> &target,
                             float Kp,
                             float Kd)
{
  return -(pos - target) * Kp - vel * Kd;
}

float sanitize_current_command(float command,
                               float pos,
                               float vel,
                               float max_current = 2000,
                               float max_pos = 3.141,
                               float max_vel = 30)
{
  /* Sanitize current command to make it safer.

  Clips current command between bounds. Reduces command if actuator outside of position or velocity bounds.
  Max current defaults to 1000mA. Max position defaults to +-180degs. Max velocity defaults to +-5rotations/s.
  */
  command = command > max_current ? max_current : command;
  command = command < -max_current ? -max_current : command;
  if (pos > max_pos || pos < -max_pos)
  {
    Serial.println("WARNING: Actuator position outside of allowed bounds.");
  }
  if (vel > max_vel || vel < -max_vel)
  {
    Serial.println("ERROR: Actuactor velocity outside of allowed bounds. Setting torque to 0.");
    return 0.0;
  }
  return command;
}

template <int n>
BLA::Matrix<n> vectorized_sanitize(const BLA::Matrix<n> &command,
                                   const BLA::Matrix<n> &angles,
                                   const BLA::Matrix<n> &velocities,
                                   float max_current = 2000,
                                   float max_pos = 3.141,
                                   float max_vel = 30)
{
  BLA::Matrix<n> command_copy;
  for (int i = 0; i < n; i++)
  {
    command_copy(i) = sanitize_current_command(command(i), angles(i), velocities(i), max_current, max_pos, max_vel);
  }
  return command_copy;
}