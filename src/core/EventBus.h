#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

enum class AquariumEventTopic : uint8_t {
  Relay = 1,
  Doser = 2,
  Pid = 3,
  System = 4,
};

struct AquariumEvent {
  AquariumEventTopic topic = AquariumEventTopic::System;
  uint8_t subtype = 0;
  float value = 0.0f;
  uint32_t unixTime = 0;
};

class EventBus {
public:
  bool begin(uint8_t depth = 32) {
    queue_ = xQueueCreate(depth, sizeof(AquariumEvent));
    return queue_ != nullptr;
  }

  bool publish(const AquariumEvent& event, TickType_t timeout = 0) {
    return queue_ && xQueueSend(queue_, &event, timeout) == pdTRUE;
  }

  bool receive(AquariumEvent& event, TickType_t timeout = 0) {
    return queue_ && xQueueReceive(queue_, &event, timeout) == pdTRUE;
  }

  bool ready() const { return queue_ != nullptr; }

private:
  QueueHandle_t queue_ = nullptr;
};
