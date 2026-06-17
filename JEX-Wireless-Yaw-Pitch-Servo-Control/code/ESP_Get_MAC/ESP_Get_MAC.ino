#ifdef ESP32
#include <WiFi.h>
#include <esp_wifi.h>
#else
#include <ESP8266WiFi.h>
#endif

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.print("ESP Board MAC Address: ");
  
#ifdef ESP32
  WiFi.mode(WIFI_STA);
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
#else
  Serial.println(WiFi.macAddress());
#endif
}

void loop() {
  // Nothing to do here
}
