/* 
 *  
 *
 * 
 * Roll -> R/r
 * Pitch -> P/p
 * Throttle -> T/t
 * Yaw -> Y/y
 * 
 * 
 */

#define PIN_R 6
#define PIN_P 7
#define PIN_T 8
#define PIN_Y 9


long loop_timer, exec_time;

// Consignment values
float rc_t_cons, rc_p_cons, rc_r_cons, rc_y_cons;

bool cal = false;
int cal_loops = 0;
int cal_loops_req = 3;
int center_delay = 0;
int center_delay_req = 200;
int center_raw_cum = 0;
int center_raw_cum_req = 200;

bool tl1 = false;
bool tr1 = false;
bool bl1 = false;
bool br1 = false;

bool tl2 = false;
bool tr2 = false;
bool bl2 = false;
bool br2 = false;





// INTERRUPT ROLL
volatile long r_high_us;
volatile int rc_r_raw;
int range_r_raw[2] = {1500, 1500};
int center_r_raw = 1500;
int center_r_raw_cum = 0;
void IntR() {
  if (digitalRead(PIN_R) == HIGH) r_high_us = micros();
  if (digitalRead(PIN_R) == LOW)  rc_r_raw  = micros() - r_high_us;
}

// INTERRUPT PITCH
volatile long p_high_us;
volatile int rc_p_raw;
int range_p_raw[2] = {1500, 1500};
int center_p_raw = 1500;
int center_p_raw_cum = 0;
void IntP() {
  if (digitalRead(PIN_P) == HIGH) p_high_us = micros();
  if (digitalRead(PIN_P) == LOW)  rc_p_raw  = micros() - p_high_us;
}

// INTERRUPT THROTTLE
volatile long t_high_us;
volatile int rc_t_raw;
int range_t_raw[2] = {1500, 1500};
int center_t_raw = 1500;
int center_t_raw_cum = 0;
void IntT() {
  if (digitalRead(PIN_T) == HIGH) t_high_us = micros();
  if (digitalRead(PIN_T) == LOW)  rc_t_raw  = micros() - t_high_us;
}

// INTERRUPT YAW
volatile long y_high_us;
volatile int rc_y_raw;
int range_y_raw[2] = {1500, 1500};
int center_y_raw = 1500;
int center_y_raw_cum = 0;
void IntY() {
  if (digitalRead(PIN_Y) == HIGH) y_high_us = micros();
  if (digitalRead(PIN_Y) == LOW)  rc_y_raw  = micros() - y_high_us;
}



void controller_setup() {
  pinMode(PIN_R, INPUT_PULLUP);
  attachInterrupt(PIN_R, IntR, CHANGE);
  
  pinMode(PIN_P, INPUT_PULLUP);
  attachInterrupt(PIN_P, IntP, CHANGE);
  
  pinMode(PIN_T, INPUT_PULLUP);
  attachInterrupt(PIN_T, IntT, CHANGE);
  
  pinMode(PIN_Y, INPUT_PULLUP);
  attachInterrupt(PIN_Y, IntY, CHANGE);

  Serial.println("Controller setup complete");
  cal_loops = 0;
  cal = false;


  Serial.begin(115200);
}

void CalibrateChannels() {
  if (rc_r_raw < range_r_raw[0] && rc_r_raw > 800) 
    range_r_raw[0] = rc_r_raw;
  if (rc_r_raw > range_r_raw[1] && rc_r_raw < 2200) 
    range_r_raw[1] = rc_r_raw;
 
  if (rc_p_raw < range_p_raw[0] && rc_p_raw > 800) 
    range_p_raw[0] = rc_p_raw;
  if (rc_p_raw > range_p_raw[1] && rc_p_raw < 2200) 
    range_p_raw[1] = rc_p_raw;
 
  if (rc_t_raw < range_t_raw[0] && rc_t_raw > 800) 
    range_t_raw[0] = rc_t_raw;
  if (rc_t_raw > range_t_raw[1] && rc_t_raw < 2200) 
    range_t_raw[1] = rc_t_raw;
 
  if (rc_y_raw < range_y_raw[0] && rc_y_raw > 800) 
    range_y_raw[0] = rc_y_raw;
  if (rc_y_raw > range_y_raw[1] && rc_y_raw < 2200) 
    range_y_raw[1] = rc_y_raw;
}


void PrintCalInfo(bool new_line) {
  // Monitor Serie
  Serial.print("Calibration (");
  Serial.print(cal_loops);
  Serial.print("/");
  Serial.print(cal_loops_req);
  Serial.print(") : ");

  Serial.print("R: [");
  Serial.print(range_r_raw[0]);
  Serial.print(" - ");
  Serial.print(rc_r_raw);
  Serial.print(" - ");
  Serial.print(range_r_raw[1]);
  Serial.print("] ");
  
  Serial.print("P: [");
  Serial.print(range_p_raw[0]);
  Serial.print(" - ");
  Serial.print(rc_p_raw);
  Serial.print(" - ");
  Serial.print(range_p_raw[1]);
  Serial.print("] ");

  Serial.print("T: [");
  Serial.print(range_t_raw[0]);
  Serial.print(" - ");
  Serial.print(rc_t_raw);
  Serial.print(" - ");
  Serial.print(range_t_raw[1]);
  Serial.print("] ");

  Serial.print("Y: [");
  Serial.print(range_y_raw[0]);
  Serial.print(" - ");
  Serial.print(rc_y_raw);
  Serial.print(" - ");
  Serial.print(range_y_raw[1]);
  Serial.print("] ");

  Serial.print(tl1);
  Serial.print(" ");
  Serial.print(tr1);
  Serial.print(" ");
  Serial.print(br1);
  Serial.print(" ");
  Serial.print(bl1);
  Serial.print(" ");

  Serial.print(tl2);
  Serial.print(" ");
  Serial.print(tr2);
  Serial.print(" ");
  Serial.print(br2);
  Serial.print(" ");
  Serial.print(bl2);
  Serial.print(" ");

  
  if (new_line) {
    Serial.println();
  }

}

void PrintConsignment(bool new_line) {
  Serial.print("R: ");
  Serial.print(rc_r_cons);
  Serial.print("\t");
  Serial.print("P: ");
  Serial.print(rc_p_cons);
  Serial.print("\t");
  Serial.print("T: ");
  Serial.print(rc_t_cons);
  Serial.print("\t");
  Serial.print("Y: ");
  Serial.print(rc_y_cons);
  Serial.print("\t");
  if (new_line) {
    Serial.println();
  }
}



bool check_loops() {
  if (rc_t_raw > 1800 && rc_t_raw < 2200 && rc_y_raw > 800 && rc_y_raw < 1200) {
    tl1 = true;
  }
  if (tl1 && rc_t_raw > 1800 && rc_t_raw < 2200 && rc_y_raw > 1800  && rc_y_raw < 2200) {
    tr1 = true;
    tl1 = false;
  }
  if (tr1 && rc_t_raw > 800 && rc_t_raw < 1200 && rc_y_raw > 1800  && rc_y_raw < 2200) {
    br1 = true;
  } 
  if (br1 && rc_t_raw > 800 && rc_t_raw < 1200 && rc_y_raw > 800  && rc_y_raw < 1200) {
    bl1 = true;
  } 
  if (bl1 && rc_t_raw > 1800 && rc_t_raw < 2200 && rc_y_raw > 800 && rc_y_raw < 1200) {
    tl1 = true;
  }

  if (rc_p_raw > 1800 && rc_p_raw < 2200 && rc_r_raw > 800  && rc_r_raw < 1200) {
    tl2 = true;
  }
  if (tl2 && rc_p_raw > 1800 && rc_p_raw < 2200 && rc_r_raw > 1800  && rc_r_raw < 2200) {
    tr2 = true;
    tl2 = false;
  }
  if (tr2 && rc_p_raw > 800 && rc_p_raw < 1200 && rc_r_raw > 1800  && rc_r_raw < 2200) {
    br2 = true;
  } 
  if (br2 && rc_p_raw > 800 && rc_p_raw < 1200 && rc_r_raw > 800  && rc_r_raw < 1200) {
    bl2 = true;
  } 
  if (bl2 && rc_p_raw > 1800 && rc_p_raw < 2200 && rc_r_raw > 800  && rc_r_raw < 1200) {
    tl2 = true;
  }

  if (tl1 && tr1 && br1 && bl1 && tl2 && tr2 && br2 && bl2) {
    tl1 = false;
    tr1 = false;
    br1 = false;
    bl1 = false;
    tl2 = false;
    tr2 = false;
    br2 = false;
    bl2 = false;
    return true;
  } else {
    return false;
  }

}



void controller_loop(float &rc_t, float &rc_p, float &rc_r, float &rc_y) {

  while (micros() - loop_timer < 10000);
  exec_time = (micros() - loop_timer) / 1000;
  loop_timer = micros();

  if (cal == false) {
    CalibrateChannels();

    if (check_loops()) {
      cal_loops++;
    }

    if (cal_loops >= cal_loops_req) {

      // Set center
      if ((rc_t_raw > 1400 && rc_t_raw < 1600 && rc_y_raw > 1400 && rc_y_raw < 1600) && 
          (rc_p_raw > 1400 && rc_p_raw < 1600 && rc_r_raw > 1400 && rc_r_raw < 1600)) {
        center_delay++;
        Serial.print("Recording standby position. DO NOT MOVE the controller!  ");
        Serial.print("[ ");
        Serial.print(center_delay);
        Serial.print(" / ");
        Serial.print(center_delay_req);
        Serial.print(" ] - [ ");
        Serial.print(center_raw_cum);
        Serial.print(" / ");
        Serial.print(center_raw_cum_req);
        Serial.println(" ]");

        if (center_delay >= center_delay_req) {
          center_t_raw_cum += rc_t_raw;
          center_y_raw_cum += rc_y_raw;
          center_p_raw_cum += rc_p_raw;
          center_r_raw_cum += rc_r_raw;
          center_raw_cum++;

          if (center_raw_cum == center_raw_cum_req) {
            center_t_raw = center_t_raw_cum / center_raw_cum_req;
            center_y_raw = center_y_raw_cum / center_raw_cum_req;
            center_p_raw = center_p_raw_cum / center_raw_cum_req;
            center_r_raw = center_r_raw_cum / center_raw_cum_req;
            Serial.println("Standby position recorded");
            Serial.println("You can now move the controller");
            cal = true;
          }
        }
      }
      else {
        center_delay = 0;
        center_raw_cum = 0;
        center_t_raw_cum = 0;
        center_y_raw_cum = 0;
        center_p_raw_cum = 0;
        center_r_raw_cum = 0;
      }
    } 
    else {
      PrintCalInfo(true);
    }

    if (cal) {
      Serial.println("Calibration complete");
      Serial.println("Starting controller loop");
    }
  
    return;
  } 
  

  rc_t_cons = map(rc_t_raw, range_t_raw[0], range_t_raw[1], 0, 100);
  rc_p_cons = map(rc_p_raw, range_p_raw[0], range_p_raw[1], -30, 30);
  rc_r_cons = map(rc_r_raw, range_r_raw[0], range_r_raw[1], -30, 30);
  rc_y_cons = map(rc_y_raw, range_y_raw[0], range_y_raw[1], -30, 30);

  
  rc_t = rc_t_cons;
  rc_p = rc_p_cons;
  rc_r = rc_r_cons;
  rc_y = rc_y_cons;
  
  PrintConsignment(true);

}

