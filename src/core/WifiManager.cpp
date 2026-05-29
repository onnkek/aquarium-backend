#include "core/WifiManager.h"

void WifiManager::begin(const char* ssid, const char* password) {
  ssid_ = ssid;
  password_ = password;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid_, password_);
  xTaskCreatePinnedToCore(WifiManager::taskEntry, "wifi", 4096, this, 1, nullptr, 0);
}

void WifiManager::taskEntry(void* arg) { static_cast<WifiManager*>(arg)->task(); }
void WifiManager::task() {
  for (;;) {
    const bool ok = WiFi.status() == WL_CONNECTED;
    if (ok) {
      if (!wasConnected_) logger_.info(LogCategory::Wifi, "connected");
      wasConnected_ = true;
      retryDelayMs_ = 5000;
    } else {
      if (wasConnected_) logger_.warn(LogCategory::Wifi, "disconnected");
      wasConnected_ = false;
      if (millis() - lastTryMs_ >= retryDelayMs_) {
        logger_.warn(LogCategory::Wifi, "reconnect attempt");
        WiFi.disconnect(false);
        WiFi.begin(ssid_, password_);
        lastTryMs_ = millis();
        retryDelayMs_ = min<uint32_t>(retryDelayMs_ * 2, 60000);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}
