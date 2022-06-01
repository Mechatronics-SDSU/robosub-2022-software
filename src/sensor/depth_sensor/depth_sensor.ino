#include <Wire.h>
#include "MS5837.h"
#include "depth-serial.h"

MS5837 sensor;
Depthsensor depthsensor;
float* depthptr = &depthsensor._depth_value;

void setup() {

  Serial.begin(9600);

  Serial.println("Depth Sensor online...");

  Wire.begin();

  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(500);
  }

  // .init sets the sensor model for us but we can override it if required.
  // Uncomment the next line to force the sensor model to the MS5837_30BA.
  sensor.setModel(MS5837::MS5837_02BA);

  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

void loop() {
  // Update pressure and temperature readings
  sensor.read();
  *depthptr = sensor.depth();
  depthsensor.serialUpdate();
}
