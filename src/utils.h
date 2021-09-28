#include "Arduino.h"

template <class T>
inline Print &operator<<(Print &obj, T arg) // no-cost stream operator as described at http://arduiniana.org/libraries/streaming/
{
  obj.print(arg);
  return obj;
}

void wait_for_key(char s)
{
    long last_print = millis();
    while (true)
    {
        char c = Serial.read();
        if (c == s)
        {
            Serial.println("Starting...");
            break;
        }
        if (millis() - last_print > 2000)
        {
            Serial.print("Press ");
            Serial.print(s);
            Serial.println(" to start...");
            last_print = millis();
        }
    }
}

void clear_serial_buffer()
{
    // Remove all characters that might have been stored up in the serial input buffer prior to running this program
    while (Serial.available())
    {
        Serial.read();
    }
}