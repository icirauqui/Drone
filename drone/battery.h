#define BAT_CYCLE 50000  // Time interval in microseconds between battery checks
#define REFERENCE_VOLTAGE 5.0  // Default reference voltage
#define VOLTAGE_DIVIDER_RATIO 2.5  // Ratio defined by the voltage divider circuit
#define ADC_RESOLUTION 1023.0  // Resolution of ADC in Arduino

long timer_loop;  // Stores the last time the battery was checked
float bat_v, bat_read = 0.0;  // Variables to store battery read and voltage

float bat_offset = 4.8;  // Offset to calibrate voltage reading

void bat_setup() {
  Serial.begin(115200);  // Begin serial communication at 115200 baud
}

void bat_loop() {
  // Check if enough time has passed since the last battery check
  if (micros() - timer_loop >= BAT_CYCLE) {
    timer_loop = micros();  // Update last checked time

    // Read battery voltage from pin A6
    bat_read = analogRead(A6);
    // Calculate actual battery voltage
    bat_v = VOLTAGE_DIVIDER_RATIO * (bat_read * bat_offset / ADC_RESOLUTION) * REFERENCE_VOLTAGE;

    // Output the battery voltage to the serial monitor
    Serial.print("Battery Voltage: ");
    Serial.println(bat_v);
  }
}
