#include <Arduino.h>
#include "settings.h"
#include <WiFi.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
  #include "driver/rtc_io.h"
  #include "driver/adc.h"
}
#include <AsyncMqttClient.h>
#include "settings.h"

RTC_DATA_ATTR int bootCount = 0;

AsyncMqttClient mqttClient;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          Serial.println("WiFi connected");
          Serial.println("IP address: ");
          Serial.println(WiFi.localIP());
          connectToMqtt();
          break;
      default:
          break;

    }
}



void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  pinMode(REED_PIN, INPUT_PULLUP);
  bool isOpen = digitalRead(REED_PIN);

  String message = "{\"bootCount\": " + String(bootCount) 
    + ", \"doorOpen\": " + String(isOpen ? "true" : "false") 
    + ", \"millis\": " + String(millis()) + "}";
  Serial.println(message);
  uint16_t messageId = mqttClient.publish(MQTT_TOPIC, 2, false, message.c_str());
  Serial.printf("Message id: %d\n", messageId);
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  
  bool wakeupTrigger = !digitalRead(REED_PIN);
  // After the message has been sent to the MQTT broker 
  // Prepare the device for sleep. The ESP32 should wake up, when
  // the state of the door switch changes. Sadly the API doesn't offer
  // a wake-up trigger on state change. It can only wake up when a certain
  // state is monitored on the given PIN. So we are prepping the ESP32 to
  // wake up on a state different from the current one.

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_15, wakeupTrigger);
  adc_power_off();
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();

}

void setup() {
  Serial.begin(115200);
  bootCount++;
  pinMode(REED_PIN, INPUT_PULLUP);
  
  // Setup callbacks for event driven handling
  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();

}

void loop() {
  // not used
}