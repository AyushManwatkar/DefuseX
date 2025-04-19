#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Store offsets
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

// Filter buffer size
#define FILTER_SIZE 5

// Create buffers for filtering
float axBuffer[FILTER_SIZE], ayBuffer[FILTER_SIZE], azBuffer[FILTER_SIZE];
float gxBuffer[FILTER_SIZE], gyBuffer[FILTER_SIZE], gzBuffer[FILTER_SIZE];

// Index for buffer insertion
int bufferIndex = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Initializing MPU6050...");

  Wire.begin(21, 22);  // SDA, SCL for ESP32
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }

  Serial.println("MPU6050 Initialized!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // Calibrate the sensors
  calibrateSensors();

  // Initialize the buffers to 0
  memset(axBuffer, 0, sizeof(axBuffer));
  memset(ayBuffer, 0, sizeof(ayBuffer));
  memset(azBuffer, 0, sizeof(azBuffer));
  memset(gxBuffer, 0, sizeof(gxBuffer));
  memset(gyBuffer, 0, sizeof(gyBuffer));
  memset(gzBuffer, 0, sizeof(gzBuffer));

  delay(100);
}

void calibrateSensors() {
  Serial.println("Calibrating sensors...");

  const int calibrationSamples = 1000;
  float ax_sum = 0, ay_sum = 0, az_sum = 0;
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;

  // Collect samples to calculate the offsets (average value)
  for (int i = 0; i < calibrationSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    ax_sum += a.acceleration.x;
    ay_sum += a.acceleration.y;
    az_sum += a.acceleration.z;
    
    gx_sum += g.gyro.x;
    gy_sum += g.gyro.y;
    gz_sum += g.gyro.z;
    
    delay(10);  // Small delay between samples
  }

  // Calculate the average offsets
  ax_offset = ax_sum / calibrationSamples;
  ay_offset = ay_sum / calibrationSamples;
  az_offset = az_sum / calibrationSamples;
  
  gx_offset = gx_sum / calibrationSamples;
  gy_offset = gy_sum / calibrationSamples;
  gz_offset = gz_sum / calibrationSamples;

  Serial.println("Calibration complete!");
  Serial.print("Accel Offsets -> X: "); Serial.print(ax_offset);
  Serial.print(" Y: "); Serial.print(ay_offset);
  Serial.print(" Z: "); Serial.println(az_offset);
  Serial.print("Gyro Offsets -> X: "); Serial.print(gx_offset);
  Serial.print(" Y: "); Serial.print(gy_offset);
  Serial.print(" Z: "); Serial.println(gz_offset);
}

float computeAverage(float* buffer) {
  float sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += buffer[i];
  }
  return sum / FILTER_SIZE;
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply the calculated offsets to the sensor readings
  float ax = a.acceleration.x - ax_offset;
  float ay = a.acceleration.y - ay_offset;
  float az = a.acceleration.z - az_offset;

  float gx = g.gyro.x - gx_offset;
  float gy = g.gyro.y - gy_offset;
  float gz = g.gyro.z - gz_offset;

  // Insert the new reading into the buffer
  axBuffer[bufferIndex] = ax;
  ayBuffer[bufferIndex] = ay;
  azBuffer[bufferIndex] = az;
  gxBuffer[bufferIndex] = gx;
  gyBuffer[bufferIndex] = gy;
  gzBuffer[bufferIndex] = gz;

  // Increment the buffer index, and reset it if it reaches the filter size
  bufferIndex = (bufferIndex + 1) % FILTER_SIZE;

  // Compute the filtered averages
  float ax_filtered = computeAverage(axBuffer);
  float ay_filtered = computeAverage(ayBuffer);
  float az_filtered = computeAverage(azBuffer);
  float gx_filtered = computeAverage(gxBuffer);
  float gy_filtered = computeAverage(gyBuffer);
  float gz_filtered = computeAverage(gzBuffer);

  // Only print accelerometer and gyroscope values
  Serial.print("Accel -> X: ");
  Serial.print(ax_filtered);
  Serial.print(" Y: ");
  Serial.print(ay_filtered);
  Serial.print(" Z: ");
  Serial.println(az_filtered);

  Serial.print("Gyro -> X: ");
  Serial.print(gx_filtered);
  Serial.print(" Y: ");
  Serial.print(gy_filtered);
  Serial.print(" Z: ");
  Serial.println(gz_filtered);

  delay(500);  // Delay to avoid excessive output
}
