#include "hardware/SensorService.h"
#include "config/PinMap.h"

void SensorService::begin() {
  // I2C bus is initialized once in App::begin() before RTC and sensors.
  ahtOk_ = aht_.begin();
  logger_.log(ahtOk_ ? LogLevel::Info : LogLevel::Warn, LogCategory::Sensor,
              ahtOk_ ? "AHT20 initialized" : "AHT20 unavailable");
  ds_.requestTemp();
}

void SensorService::tick(RuntimeState& state) {
  const uint32_t now = millis();

  if (now - lastWaterRequestMs_ >= 1000) {
    if (ds_.readTemp()) {
      const float value = ds_.getTemp();
      if (value > -20.0f && value < 80.0f) {
        state.waterTemp = value;
        state.waterTempValid = true;
      } else {
        state.waterTempValid = false;
      }
    } else {
      state.waterTempValid = false;
    }

    ds_.requestTemp();
    lastWaterRequestMs_ = now;
  }

  if (ahtOk_ && now - lastAhtReadMs_ >= 1000) {
    if (aht_.available()) {
      state.airTemp = aht_.getTemperature();
      state.humidity = aht_.getHumidity();
      state.airTempValid = true;
      state.humidityValid = true;
    } else {
      state.airTempValid = false;
      state.humidityValid = false;
    }
    lastAhtReadMs_ = now;
  } else if (!ahtOk_) {
    state.airTempValid = false;
    state.humidityValid = false;
  }
}
