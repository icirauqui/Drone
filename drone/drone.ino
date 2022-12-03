// Drone control

#include "mpu.h"
#include "controller.h"


void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  //setup_mpu();
  mpu_setup();
  controller_setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  //digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  //delay(1000);                      // wait for a second
  //digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  //delay(1000);                      // wait for a second

  //char ag = mpu_read(true);

  //Serial.print(ag);

  //mpu_loop();
  controller_loop();
}
