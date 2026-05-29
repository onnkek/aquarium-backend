#include "hardware/FanController.h"
#include "config/PinMap.h"
#include "config/HardwareSafety.h"

volatile uint32_t FanController::pulses_ = 0;

void FanController::tachIsr() { pulses_++; }

void FanController::begin() {
  pinMode(Pins::TACH, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Pins::TACH), FanController::tachIsr, FALLING);
  ledcSetup(HardwareConfig::PWM_CHANNEL, HardwareConfig::PWM_FREQ, HardwareConfig::PWM_RES);
  ledcAttachPin(Pins::FAN_PWM, HardwareConfig::PWM_CHANNEL);
  setSpeedPercent(HardwareSafety::FAN_BOOT_PERCENT);
}

uint16_t FanController::toPwm(uint8_t percent) const {
  if (percent == 0) return 0;
  if (percent > 100) percent = 100;
  constexpr uint16_t pwmMin = 200;
  constexpr uint16_t pwmMax = 1023;
  return pwmMin + (uint32_t)(percent - 1) * (pwmMax - pwmMin) / 99;
}

void FanController::setSpeedPercent(uint8_t percent) { ledcWrite(HardwareConfig::PWM_CHANNEL, toPwm(percent)); }

uint32_t FanController::rpm() const { return rpm_; }
