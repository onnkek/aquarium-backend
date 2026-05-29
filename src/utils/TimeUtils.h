#pragma once
#include <stdint.h>
#include <RTClib.h>
#include "models/AppTypes.h"

namespace TimeUtils {
bool parseHHMM(const char* value, TimeOfDay& out);
uint16_t minuteOfDay(const TimeOfDay& t);
bool isActiveWindow(const TimeOfDay& on, const TimeOfDay& off, const DateTime& now);
uint8_t weekDayBit(uint8_t rtclibDayOfWeek);
bool periodAllows(uint8_t periodMask, uint8_t rtclibDayOfWeek);
void ymd(const DateTime& now, char* out, size_t outLen);
}
