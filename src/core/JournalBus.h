#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

struct JournalRecord {
  char path[80] = {0};
  char line[192] = {0};
};

class JournalBus {
public:
  bool begin(uint8_t depth) {
    queue_ = xQueueCreate(depth, sizeof(JournalRecord));
    return queue_ != nullptr;
  }

  bool publish(const JournalRecord& record, TickType_t timeout = 0) {
    return queue_ && xQueueSend(queue_, &record, timeout) == pdTRUE;
  }

  bool receive(JournalRecord& record, TickType_t timeout = portMAX_DELAY) {
    return queue_ && xQueueReceive(queue_, &record, timeout) == pdTRUE;
  }

  bool ready() const { return queue_ != nullptr; }

private:
  QueueHandle_t queue_ = nullptr;
};
