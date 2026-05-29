#pragma once
#include <Arduino.h>
#include <SparkFun_Qwiic_Humidity_AHT20.h>
#include <GyverDS18.h>
#include "models/AppTypes.h"
#include "core/Logger.h"

class SensorService {
public:
  explicit SensorService(Logger& logger) : logger_(logger), ds_(Pins::DS18B20) {}
  void begin();
  void tick(RuntimeState& state);

private:
  Logger& logger_;
  AHT20 aht_;
  GyverDS18Single ds_;
  bool ahtOk_ = false;
  uint32_t lastWaterRequestMs_ = 0;
  uint32_t lastAhtReadMs_ = 0;
};
