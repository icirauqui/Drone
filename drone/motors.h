const int MIN_PWM = 1000;
const int MAX_PWM = 2000;

#include <Servo.h>

Servo esc1, esc2, esc3, esc4; // Create servo objects for each ESC
int escPins[4] = {2, 3, 4, 5}; // PWM pins connected to the ESCs


void motors_setup() {
  esc1.attach(escPins[0]);
  esc2.attach(escPins[1]);
  esc3.attach(escPins[2]);
  esc4.attach(escPins[3]);
}

void motors_loop(float &throttle, float &pitch, float &roll, float &yaw) {

  // Map throttle from 0-100 to 1000-2000 microseconds
  float baseThrottle = map(throttle, 50, 100, MIN_PWM, MAX_PWM);

  // Calculate individual motor speeds based on yaw, pitch, and roll
  float m1Speed = baseThrottle + pitch + roll + yaw;
  float m2Speed = baseThrottle + pitch - roll - yaw;
  float m3Speed = baseThrottle - pitch + roll - yaw;
  float m4Speed = baseThrottle - pitch - roll + yaw;

  // Constrain motor speeds to stay within the ESC signal range
  m1Speed = constrain(m1Speed, MIN_PWM, MAX_PWM);
  m2Speed = constrain(m2Speed, MIN_PWM, MAX_PWM);
  m3Speed = constrain(m3Speed, MIN_PWM, MAX_PWM);
  m4Speed = constrain(m4Speed, MIN_PWM, MAX_PWM);

  // Send signals to ESCs
  esc1.writeMicroseconds((int)m1Speed);
  esc2.writeMicroseconds((int)m2Speed);
  esc3.writeMicroseconds((int)m3Speed);
  esc4.writeMicroseconds((int)m4Speed);
}