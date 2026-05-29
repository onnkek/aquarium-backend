#include "hardware/RelayController.h"
#include "config/PinMap.h"
#include "config/HardwareSafety.h"

static const uint8_t EXTRA_PINS[HardwareConfig::EXTRA_RELAY_COUNT] = {
  Pins::RELAY_CO2, Pins::RELAY_O2, Pins::RELAY_FILTER, Pins::RELAY_LIGHT
};

static const uint8_t DOSER_PINS[HardwareConfig::DOSER_COUNT] = {
  Pins::DOSER_1, Pins::DOSER_2, Pins::DOSER_3, Pins::DOSER_4
};

void RelayController::begin() {
  // Configure pins first, then immediately force the same safe OFF level
  // as the live legacy firmware: LOW = OFF, HIGH = ON.
  pinMode(Pins::RELAY_COOL, OUTPUT);
  pinMode(Pins::RELAY_HEAT, OUTPUT);

  for (uint8_t i = 0; i < HardwareConfig::EXTRA_RELAY_COUNT; ++i) {
    pinMode(EXTRA_PINS[i], OUTPUT);
  }

  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    pinMode(DOSER_PINS[i], OUTPUT);
  }

  allOff();
  delay(HardwareSafety::SAFE_BOOT_SETTLE_MS);
}

void RelayController::allOff() {
  tempOff();
  extraOff();
  dosersOff();
}

void RelayController::extraOff() {
  for (uint8_t i = 0; i < HardwareConfig::EXTRA_RELAY_COUNT; ++i) {
    setExtra(i, false);
  }
}

void RelayController::dosersOff() {
  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    setDoser(i, false);
  }
}

void RelayController::tempOff() {
  setCool(false);
  setHeat(false);
}

void RelayController::writeExtraRaw(uint8_t index, bool on) {
  if (index >= HardwareConfig::EXTRA_RELAY_COUNT) return;
  digitalWrite(EXTRA_PINS[index], on ? HardwareSafety::RELAY_ON_LEVEL : HardwareSafety::RELAY_OFF_LEVEL);
}

void RelayController::writeDoserRaw(uint8_t index, bool on) {
  if (index >= HardwareConfig::DOSER_COUNT) return;
  digitalWrite(DOSER_PINS[index], on ? HardwareSafety::DOSER_ON_LEVEL : HardwareSafety::DOSER_OFF_LEVEL);
}

void RelayController::writeCoolRaw(bool on) {
  digitalWrite(Pins::RELAY_COOL, on ? HardwareSafety::TEMP_ON_LEVEL : HardwareSafety::TEMP_OFF_LEVEL);
}

void RelayController::writeHeatRaw(bool on) {
  digitalWrite(Pins::RELAY_HEAT, on ? HardwareSafety::TEMP_ON_LEVEL : HardwareSafety::TEMP_OFF_LEVEL);
}

void RelayController::setExtra(uint8_t index, bool on) {
  if (index >= HardwareConfig::EXTRA_RELAY_COUNT) return;
  extraState_[index] = on;
  writeExtraRaw(index, on);
}

void RelayController::setDoser(uint8_t index, bool on) {
  if (index >= HardwareConfig::DOSER_COUNT) return;
  doserState_[index] = on;
  writeDoserRaw(index, on);
}

void RelayController::setCool(bool on) {
  coolState_ = on;
  writeCoolRaw(on);
}

void RelayController::setHeat(bool on) {
  heatState_ = on;
  writeHeatRaw(on);
}

bool RelayController::extra(uint8_t index) const {
  return index < HardwareConfig::EXTRA_RELAY_COUNT && extraState_[index];
}

bool RelayController::doser(uint8_t index) const {
  return index < HardwareConfig::DOSER_COUNT && doserState_[index];
}
