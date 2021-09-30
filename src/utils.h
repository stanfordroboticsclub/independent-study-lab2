#pragma once

#include "Arduino.h"
#include <string.h>

template <class T>
inline Print &operator<<(Print &obj, T arg) // no-cost stream operator as described at http://arduiniana.org/libraries/streaming/
{
  obj.print(arg);
  return obj;
}

void assert(bool val, int print_delay = 2000)
{
  if (!val)
  {
    while (true)
    {
      Serial.println("ASSERTION FAILED. PROGRAM STOPPED.");
      delay(print_delay);
    }
  }
}

template <class T>
T absolute_val(T a)
{
  return a > 0 ? a : -a;
}

void assert_close(float a, float b, float eps = 0.000001)
{
  Serial << "ASSERTING " << a << " == " << b << "\n";
  assert(absolute_val(a - b) < eps);
}

void wait_for_key(char s, const String &prompt = "")
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
      Serial.print("Click inside this console and then press ");
      Serial.print(s);
      Serial.print(" to start...");
      if (prompt != "")
      {
        Serial.print(prompt);
      }
      Serial.println();
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

template <int N>
void print_vector(BLA::Matrix<N, 1> vec, char delimiter = ' ', char end = '\n')
{
  for (int i = 0; i < N; i++)
  {
    Serial << vec(i) << delimiter;
  }
}