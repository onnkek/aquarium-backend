#include "core/TimeService.h"

bool TimeService::saneDate(const DateTime& dt) {
  // If the DS3231 battery is dead or the clock was never adjusted, many
  // modules return year 2000. For aquarium automation this is not a valid
  // scheduling source, so scheduled relays/doser must be blocked.
  return dt.year() >= 2024 && dt.year() <= 2099;
}

bool TimeService::begin() {
  mutex_ = xSemaphoreCreateMutex();
  present_ = rtc_.begin();

  if (present_) {
    cached_ = rtc_.now();
    valid_ = !rtc_.lostPower() && saneDate(cached_);
  } else {
    valid_ = false;
  }

  xTaskCreatePinnedToCore(TimeService::taskEntry, "rtc", 2048, this, 3, nullptr, 1);
  return valid_;
}

DateTime TimeService::now() {
  DateTime copy(2000, 1, 1, 0, 0, 0);
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
    copy = cached_;
    xSemaphoreGive(mutex_);
  }
  return copy;
}

bool TimeService::adjust(const DateTime& dt) {
  if (!present_) return false;

  rtc_.adjust(dt);

  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
    cached_ = dt;
    valid_ = saneDate(dt);
    xSemaphoreGive(mutex_);
  } else {
    valid_ = saneDate(dt);
  }

  return valid_;
}

void TimeService::taskEntry(void* arg) { static_cast<TimeService*>(arg)->task(); }

void TimeService::task() {
  for (;;) {
    if (present_) {
      DateTime n = rtc_.now();
      const bool isValid = !rtc_.lostPower() && saneDate(n);

      if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        cached_ = n;
        valid_ = isValid;
        xSemaphoreGive(mutex_);
      } else {
        valid_ = isValid;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
