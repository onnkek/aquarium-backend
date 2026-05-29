#pragma once
#include <Arduino.h>
#include <RTClib.h>

class TimeService {
public:
  bool begin();
  DateTime now();
  bool adjust(const DateTime& dt);
  bool valid() const { return valid_; }
  bool present() const { return present_; }
  static void taskEntry(void* arg);

private:
  static bool saneDate(const DateTime& dt);

  RTC_DS3231 rtc_;
  SemaphoreHandle_t mutex_ = nullptr;
  DateTime cached_ = DateTime(2000, 1, 1, 0, 0, 0);
  bool present_ = false;
  bool valid_ = false;
  void task();
};
