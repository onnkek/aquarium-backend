#include "utils/TimeUtils.h"
#include <stdio.h>

namespace TimeUtils {
bool parseHHMM(const char* value, TimeOfDay& out) {
  if (!value || value[0] == '\0' || value[0] == 'n') {
    out.hour = 0;
    out.minute = 0;
    return false;
  }
  int h = 0, m = 0;
  if (sscanf(value, "%d:%d", &h, &m) != 2) return false;
  if (h < 0 || h > 23 || m < 0 || m > 59) return false;
  out.hour = static_cast<uint8_t>(h);
  out.minute = static_cast<uint8_t>(m);
  return true;
}

uint16_t minuteOfDay(const TimeOfDay& t) {
  return static_cast<uint16_t>(t.hour) * 60u + t.minute;
}

bool isActiveWindow(const TimeOfDay& on, const TimeOfDay& off, const DateTime& now) {
  const uint16_t n = now.hour() * 60u + now.minute();
  const uint16_t a = minuteOfDay(on);
  const uint16_t b = minuteOfDay(off);
  if (a == b) return true;
  if (a < b) return n >= a && n < b;
  return n >= a || n < b;
}

uint8_t weekDayBit(uint8_t rtclibDayOfWeek) {
  return static_cast<uint8_t>(1u << (rtclibDayOfWeek % 7));
}

bool periodAllows(uint8_t periodMask, uint8_t rtclibDayOfWeek) {
  return (periodMask & weekDayBit(rtclibDayOfWeek)) != 0;
}

void ymd(const DateTime& now, char* out, size_t outLen) {
  snprintf(out, outLen, "%04d-%02d-%02d", now.year(), now.month(), now.day());
}
}
