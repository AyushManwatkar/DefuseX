#include <WiFi.h>
#include <esp_now.h>
#include <AccelStepper.h>

#define MOTOR_INTERFACE_TYPE 4
AccelStepper stepper(MOTOR_INTERFACE_TYPE, 14, 27, 26, 25);

float receivedTilt = 0;
// const float stepsPerRevolution = 200;
// const float degreeToStep = stepsPerRevolution / 360.0;

// Structure must match sender
typedef struct struct_message {
  float angle;
} struct_message;

struct_message incomingData;

// ✅ NEW: receive callback for ESP-IDF v5.x
void onReceiveData(const esp_now_recv_info_t *recv_info, const uint8_t *incomingDataRaw, int len) {
  if (len == sizeof(incomingData)) {
    memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
    receivedTilt = incomingData.angle;

    //Serial.print("Received from: ");
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
             recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
    //Serial.print(macStr);

    Serial.print(" | Tilt: ");
    Serial.println(receivedTilt);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // ✅ Register the updated callback
  esp_now_register_recv_cb(onReceiveData);

  // Stepper setup
  stepper.setMaxSpeed(800);
  stepper.setAcceleration(300);

}

void loop() {
  //float speed = receivedTilt * 2.0;
  stepper.moveTo(receivedTilt);
  stepper.run();
}
