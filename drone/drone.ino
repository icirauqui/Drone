// Drone control

//#include "mpu.h"
#include "controller.h"
//#include "battery.h"
#include "motors.h"


float rc_t = 0.0;
float rc_p = 0.0;
float rc_r = 0.0;
float rc_y = 0.0;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(115200);
  while(!Serial);


  //mpu_setup();
  controller_setup();
  motors_setup();
}

void loop() {
  //mpu_loop(false);
  controller_loop(rc_t, rc_p, rc_r, rc_y);
  motors_loop(rc_t, rc_p, rc_r, rc_y);

  //Serial.print("  - Angle {Pitch, Roll} = {");
  //Serial.print(ang_p);
  //Serial.print(", ");
  //Serial.print(ang_r);
  //Serial.println("} deg");

  //delay(1000);
}
