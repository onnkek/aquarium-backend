#pragma once

#include <Arduino.h>

/*
  HardwareSafety documents the electrical semantics extracted from the legacy
  live project. The legacy firmware uses:

    LOW  = OFF
    HIGH = ON

  for:
    - extra relays: CO2 / O2 / filter / light
    - doser outputs 1..4
    - thermostat outputs: cool / heat

  Do not change these constants unless the relay board/wiring is changed.
*/
namespace HardwareSafety {
static constexpr uint8_t RELAY_OFF_LEVEL = LOW;
static constexpr uint8_t RELAY_ON_LEVEL  = HIGH;

static constexpr uint8_t DOSER_OFF_LEVEL = LOW;
static constexpr uint8_t DOSER_ON_LEVEL  = HIGH;

static constexpr uint8_t TEMP_OFF_LEVEL = LOW;
static constexpr uint8_t TEMP_ON_LEVEL  = HIGH;

// In the legacy project ledcWrite(..., 0) was documented as fan off.
static constexpr uint8_t FAN_BOOT_PERCENT = 0;

// On boot every dangerous output is forced OFF before config automation starts.
static constexpr uint32_t SAFE_BOOT_SETTLE_MS = 100;
}
