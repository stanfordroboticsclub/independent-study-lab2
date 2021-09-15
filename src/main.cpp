#include "Arduino.h"
#include "C610Bus.h"

long last_command = 0; // To keep track of when we last commanded the motors
C610Bus<CAN2> bus; // Initialize the Teensy's CAN bus to talk to the motors

int LOOP_DELAY_MILLIS = 2; // Wait for 0.002s between motor updates.

void setup()
{
}
void loop()
{
    bus.PollCAN(); // Check for messages from the motors.

    long now = millis();
    if (now - last_command >= LOOP_DELAY_MILLIS)
    {
        float m0_pos = bus.Get(0).Position(); // Get the shaft position of motor 0 in radians.
        float m0_vel = bus.Get(0).Velocity(); // Get the shaft velocity of motor 0 in radians/sec.

        Serial.print(m0_pos);
        Serial.print(" ");
        Serial.println(m0_vel);

        float m0_current = 0.0;

        // Step 4. Uncomment for bang-bang control
        // if(m0_pos < 0) {
        //   m0_current = 800;
        // } else {
        //   m0_current = -800;
        // }

        // Step 8. Change the target position to something periodic
        // float time = millis() / 1000.0; // millis() returns the time in milliseconds since start of program


        // Step 5. Write your own PD controller here.
        float Kp = 1000.0;
        float Kd = 100.0;
        float target_position = 0.5;
        // m0_current = 0.0; // REPLACE WITH YOUR LOGIC
        m0_current = -Kp * (m0_pos - target_position) - Kd*m0_vel;
        
        // This next line will cause the motor to turn. Make sure it is mounted safely. 
        bus.CommandTorques(m0_current, 0, 0, 0, C610Subbus::kIDZeroToThree);

        last_command = now;
    }
}