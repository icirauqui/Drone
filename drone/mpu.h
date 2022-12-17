// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
#define MPU6050_adress 0x68
MPU6050 mpu;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t gx, gy, gz = 0;
int64_t gx_cal, gy_cal, gz_cal = 0;
float wx, wy, wz = 0.0;

float a_xyz = 0.0;
int16_t ax, ay, az = 0;
int64_t ax_cal, ay_cal, az_cal = 0;

float ang_p, ang_r = 0.0;
float ang_p_a, ang_r_a = 0.0;

bool set_gyro_ang, acc_calib_ok = false;

float timer_run, timer_loop = 0.0;



// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

#define MPU_CYCLE 5000




void mpu_init() {
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x6B);                          // PWR_MGMT_1 registro 6B hex
  Wire.write(0x00);                          // 00000000 para activar
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1B);                          // GYRO_CONFIG registro 1B hex
  Wire.write(0x08);                          // 00001000: 500dps
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1C);                          // ACCEL_CONFIG registro 1C hex
  Wire.write(0x10);                          // 00010000: +/- 8g
  Wire.endTransmission();
}



void mpu_print_calib() {
  Serial.print("  * ");
  Serial.print(ax_cal); Serial.print("\t");
  Serial.print(ay_cal); Serial.print("\t");
  Serial.print(az_cal); Serial.print("\t");
  Serial.print(gx_cal); Serial.print("\t");
  Serial.print(gy_cal); Serial.print("\t");
  Serial.print(gz_cal);
  Serial.println("");
}



void mpu_print() {
  Serial.print("  - ");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.print(gz);
  Serial.println("");
}


void mpu_calib(int calib_loops = 3000, int delay = 100) {
  for (unsigned int i=0; i < calib_loops; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_cal += ax;
    ay_cal += ay;
    az_cal += az;
    gx_cal += gx;
    gy_cal += gy;
    gz_cal += gz;
    delayMicroseconds(delay);
  }

  ax_cal /= calib_loops;
  ay_cal /= calib_loops;
  az_cal /= calib_loops;
  gx_cal /= calib_loops;
  gy_cal /= calib_loops;
  gz_cal /= calib_loops;

  timer_loop = micros();

  acc_calib_ok = true;
}



void mpu_print_ang_speed() {
  Serial.print("  - Angular speeds (deg/s): ");
  Serial.print(wx); Serial.print("\t");
  Serial.print(wy); Serial.print("\t");
  Serial.print(wz);
  Serial.println("");
}

void mpu_process() {

  //Serial.println("Process");
  //mpu_print();
  //mpu_print_calib();

  ax -= ax_cal;
  ay -= ay_cal;
  az -= az_cal;
  az += 4096;

  gx -= gx_cal;
  gy -= gy_cal;
  gz -= gz_cal;

  //mpu_print();

  wx = gx / 65.5;
  wy = gy / 65.5;
  wz = gz / 65.5;

  //mpu_print_ang_speed();

  // 0.000000266 = tiempo_ejecucion / 1000 / 65.5 * PI / 180
  ang_p += wx * timer_run / 1000.0;
  ang_r += wy * timer_run / 1000.0;

  ang_p += (ang_r * sin((gz - gz_cal) * timer_run * 0.000000266));
  ang_r -= (ang_p * sin((gz - gz_cal) * timer_run * 0.000000266));

  // Compute acceleration vector
  // Rad to Deg = * 180 / PI
  a_xyz = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2));
  ang_p_a = asin((float)ay / a_xyz) * 57.2957795;
  ang_r_a = asin((float)ax / a_xyz) * -57.2957795;

  if (set_gyro_ang) {
    ang_p = ang_p * 0.99 + ang_p_a * 0.01;
    ang_r = ang_r * 0.99 + ang_r_a * 0.01;
  }
  else {
    ang_p = ang_p_a;
    ang_r = ang_r_a;
    set_gyro_ang = true;
  }
}



char mpu_read(bool verbose = false) {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  if (verbose) {
    Serial.print("\ta/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz);
  }

  char ag = (uint8_t)(ax >> 8) + (uint8_t)(ax & 0xFF)
          + (uint8_t)(ay >> 8) + (uint8_t)(ay & 0xFF)
          + (uint8_t)(az >> 8) + (uint8_t)(az & 0xFF)
          + (uint8_t)(gx >> 8) + (uint8_t)(gx & 0xFF)
          + (uint8_t)(gy >> 8) + (uint8_t)(gy & 0xFF)
          + (uint8_t)(gz >> 8) + (uint8_t)(gz & 0xFF);

  return ag;
}









void mpu_setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  
  // initialize device
  Serial.print("Initializing MPU6050... ");
  mpu.initialize();
  Serial.println("Done");

  // verify connection
  Serial.print("Testing device connection... ");
  Serial.println(mpu.testConnection() ? "OK" : "Fail");

  // Load default params
  Serial.print("Setting default params... ");
  mpu_init();
  Serial.println("Done");

  // Calibrate
  Serial.print("Calibrating MPU6050... ");
  mpu_calib(3000, 100);
  Serial.println("Done");
}



void mpu_loop(void verbose = false) {
  while (micros() - timer_loop < MPU_CYCLE) {
    // do nothing
  }

  timer_run = (micros() - timer_loop) / 1000.0;
  timer_loop = micros();

  char ag = mpu_read(false);

  mpu_process();

  if (verbose) {
    Serial.print("  - Angle {Pitch, Roll} = {");
    Serial.print(ang_p);
    Serial.print(", ");
    Serial.print(ang_r);
    Serial.println("} deg");
  }
}



