#define PIN_M1 2
#define PIN_M2 3
#define PIN_M3 4
#define PIN_M4 5


float esc1, esc2, esc3, esc4 = 0.0;
float throttle_cons = 0.0;

long time_eng_start = 0;

const int rc_t_min = 1000;
const int rc_t_max = 2000;

const int rc_t_min_raw = 0;
const int rc_t_max_raw = 100;


void motors_setup() {
  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_M2, OUTPUT);
  pinMode(PIN_M3, OUTPUT);
  pinMode(PIN_M4, OUTPUT);

  digitalWrite(PIN_M1, LOW);
  digitalWrite(PIN_M2, LOW);
  digitalWrite(PIN_M3, LOW);
  digitalWrite(PIN_M4, LOW);
}



void motors_loop(float &rc_t, float &rc_p, float &rc_r, float &rc_y) {

  throttle_cons = map(rc_t, rc_t_min_raw, rc_t_max_raw, rc_t_min, rc_t_max);

  esc1 = throttle_cons;
  esc2 = throttle_cons;
  esc3 = throttle_cons;
  esc4 = throttle_cons;
  
  digitalWrite(PIN_M1, HIGH);
  digitalWrite(PIN_M2, HIGH);
  digitalWrite(PIN_M3, HIGH);
  digitalWrite(PIN_M4, HIGH);

  while (digitalRead(PIN_M1) == HIGH || 
         digitalRead(PIN_M2) == HIGH || 
         digitalRead(PIN_M3) == HIGH || 
         digitalRead(PIN_M4) == HIGH) {
    if (time_eng_start + esc1 <= micros()) digitalWrite(PIN_M1, LOW); // MOTOR 1
    if (time_eng_start + esc2 <= micros()) digitalWrite(PIN_M2, LOW); // MOTOR 2
    if (time_eng_start + esc3 <= micros()) digitalWrite(PIN_M3, LOW); // MOTOR 3
    if (time_eng_start + esc4 <= micros()) digitalWrite(PIN_M4, LOW); // MOTOR 4
  }
}