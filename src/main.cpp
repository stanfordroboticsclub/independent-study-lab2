#include "Arduino.h"
#include "C610Bus.h"
#include <array>
#include "BasicLinearAlgebra.h"
#include "pid.h"
#include "kinematics.h"
#include "test_kinematics.h"
#include "utils.h"

#define DO_TESTS // comment out to skip running tests before executing main program

/*********** STUDENT CONFIGURATION NECESSARY *************/
// Examine your robot leg to see if you built a left leg or a right leg.
// Then replace kUnspecified with the correct side.
const BodySide kLegSide = BodySide::kUnspecified; // Replace with BodySide::kLeft or BodySide::kRight
/*********************************************************/

long last_command = 0; // To keep track of when we last commanded the motors
C610Bus<CAN2> bus;     // Initialize the Teensy's CAN bus to talk to the rear Pupper motors

const int LOOP_DELAY_MILLIS = 5; // Wait for 0.005s between motor updates.

const float Kp = 2000;
const float Kd = 100;
const float kMaxCurrent = 3000;

// Define the signed hip offset and link lengths
const KinematicsConfig pupper_leg_config = (kLegSide == BodySide::kLeft) ? KinematicsConfig{0.035, 0.08, 0.12} : KinematicsConfig{-0.035, 0.08, 0.12};

BLA::Matrix<3> actuator_angles{0, 0, 0};     // rad
BLA::Matrix<3> actuator_velocities{0, 0, 0}; // rad/s
BLA::Matrix<3> actuator_commands{0, 0, 0};   // mA

void setup()
{
  clear_serial_buffer();
  wait_for_key('s', "Remember to start the arm extended out horizontally.");
#ifdef DO_TESTS
  test_kinematics();
#endif
}

void loop()
{
  bus.PollCAN(); // Check for messages from the motors.

  long now = millis();

  // Check to see if we received a 's' and if so, stop the program.
  if (Serial.available())
  {
    if (Serial.read() == 's')
    {
      bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks);
      Serial.println("Stopping.");
      while (true)
      {
      }
    }
  }

  // Check to see if it's time to run our control loop again.
  if (now - last_command >= LOOP_DELAY_MILLIS)
  {
    for (int i = 0; i < 3; i++)
    {
      actuator_angles(i) = bus.Get(i).Position();
      actuator_velocities(i) = bus.Get(i).Velocity();
    }
    actuator_angles = correct_for_actuator_direction(actuator_angles, kLegSide);
    actuator_velocities = correct_for_actuator_direction(actuator_angles, kLegSide);

    BLA::Matrix<3> cartesian_coordinates = forward_kinematics(actuator_angles, pupper_leg_config);

    Serial.print("Cart. pos: ");
    print_vector(cartesian_coordinates, ' ', '\0');
    Serial.print("Angles: ");
    print_vector(actuator_angles, ' ', '\0');
    Serial.print("\tVel: ");
    print_vector(actuator_velocities, ' ', '\0');
    Serial.println("\tPress s to stop.");

    // Insert your code for checking if the robot end-effector is outside the safety box here
    // if(your code here) {
    //   Serial.println("OUTSIDE SAFETY BOX.");

      // Insert your code for providing haptic feedback here
      // actuator_commands = blah;
    // }

    actuator_commands = vectorized_sanitize(actuator_commands,
                                            actuator_angles,
                                            actuator_velocities,
                                            kMaxCurrent);

    // Put the torque commands to be the each motors local coordinate frame
    actuator_commands = correct_for_actuator_direction(actuator_commands, kLegSide);
    // Only call CommandTorques once per loop! Calling it multiple times will override the last command.
    bus.CommandTorques(actuator_commands(0),
                       actuator_commands(1),
                       actuator_commands(2),
                       0,
                       C610Subbus::kOneToFourBlinks);

    // Once you have motors with ID=4 to 7, use this command
    // bus.CommandTorques(0, 0, 0, 0, C610Subbus::kFiveToEightBlinks);

    last_command = now;
    Serial.println();
  }
}