#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

bool otaEnabled = true;

void beginOTA(const char* ssid, const char* passwd) {
WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, passwd);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("OTA Connection Failed! continuing without ota");
    otaEnabled = false;
  } else {
    ArduinoOTA.setHostname("Flying breadboard X");
    ArduinoOTA.setPassword("admin");
  
    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
    ArduinoOTA.begin();
  
    Serial.println("OTA Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

void handleOTA() {
  if(otaEnabled) {
    ArduinoOTA.handle();  
  }
}