#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <esp_now.h>

MPU6050 mpu;

// Slave MAC Address (your other ESP32)
uint8_t slaveAddress[] = { 0xF4, 0x65, 0x0B, 0x49, 0x8E, 0x64 };

// Data structure to send
typedef struct struct_message {
  float angle;
} struct_message;

struct_message dataToSend;

float totalAngle = 0;
const float maxAngle = 360;
float offsetAngle = 0;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  // Initial calibration for "flat = 0°"
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  offsetAngle = atan2(ax, az) * (180.0 / PI);

  lastTime = millis();

  // Init WiFi & ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, slaveAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_add_peer(&peerInfo)) {
    Serial.println("Peer added.");
  }

  esp_now_register_send_cb([](const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
  });
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Tilt angle from accelerometer
  float tiltAngle = atan2(ax, az) * (180.0 / PI) - offsetAngle;

  // Clamp for stability
  if (tiltAngle > 90) tiltAngle = 90;
  if (tiltAngle < -90) tiltAngle = -90;

  // Convert tilt to speed
  float speed = tiltAngle * 2.0;

  // Integrate to get total angle
  totalAngle += speed * dt;

  if (totalAngle > maxAngle) totalAngle = maxAngle;
  if (totalAngle < -maxAngle) totalAngle = -maxAngle;
  //long targetSteps = totalAngle * degreeToStep;
  // Prepare data and send
  dataToSend.angle = tiltAngle;
  esp_now_send(slaveAddress, (uint8_t*)&dataToSend, sizeof(dataToSend));

  // Debug
  Serial.print("Tilt: "); Serial.print(tiltAngle);
  Serial.print(" | Total angle: "); Serial.println(totalAngle);

}
