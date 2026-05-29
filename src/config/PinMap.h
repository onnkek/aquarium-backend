#pragma once
#include <stdint.h>

namespace Pins {
static constexpr uint8_t CLK = 10;
static constexpr uint8_t MISO = 2;
static constexpr uint8_t MOSI = 9;
static constexpr uint8_t SD_CS = 11;

static constexpr uint8_t RELAY_COOL = 7;
static constexpr uint8_t RELAY_CO2 = 17;
static constexpr uint8_t RELAY_O2 = 1;
static constexpr uint8_t RELAY_FILTER = 4;
static constexpr uint8_t RELAY_LIGHT = 5;
static constexpr uint8_t RELAY_HEAT = 6;

static constexpr uint8_t DOSER_1 = 18;
static constexpr uint8_t DOSER_2 = 21;
static constexpr uint8_t DOSER_3 = 14;
static constexpr uint8_t DOSER_4 = 44;

static constexpr uint8_t ARGB = 12;
static constexpr uint8_t DS18B20 = 43;
static constexpr uint8_t SDA = 3;
static constexpr uint8_t SCL = 8;
static constexpr uint8_t FAN_PWM = 13;
static constexpr uint8_t TACH = 36;
}

namespace HardwareConfig {
static constexpr uint8_t DOSER_COUNT = 4;
static constexpr uint8_t EXTRA_RELAY_COUNT = 4;
static constexpr uint8_t NUM_LEDS = 24;
static constexpr uint8_t PWM_CHANNEL = 0;
static constexpr uint32_t PWM_FREQ = 30000;
static constexpr uint8_t PWM_RES = 10;
}
