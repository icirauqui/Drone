#define BAT_CYCLE 50000

long timer_loop;
float bat_v, bat_read = 0.0;

float bat_offset = 4.8;

void bat_setup() {
  //Serial.begin(115200);
}

void bat_loop() {
  while (micros() - loop_timer < BAT_CYCLE);

  timer_loop = micros();

  bat_read = analogRead(A6);
  bat_v = 2.5 * (bat_read * bat_offset / 1023.0);
}