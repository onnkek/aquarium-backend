#pragma once
#include <Arduino.h>
#include "models/AppTypes.h"

class RelayController {
public:
  void begin();

  void setExtra(uint8_t index, bool on);
  void setDoser(uint8_t index, bool on);
  void setCool(bool on);
  void setHeat(bool on);

  void allOff();
  void dosersOff();
  void extraOff();
  void tempOff();

  bool extra(uint8_t index) const;
  bool doser(uint8_t index) const;

private:
  void writeExtraRaw(uint8_t index, bool on);
  void writeDoserRaw(uint8_t index, bool on);
  void writeCoolRaw(bool on);
  void writeHeatRaw(bool on);

  bool extraState_[HardwareConfig::EXTRA_RELAY_COUNT] = {false};
  bool doserState_[HardwareConfig::DOSER_COUNT] = {false};
  bool coolState_ = false;
  bool heatState_ = false;
};
