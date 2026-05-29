#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

enum class CommandType : uint8_t {
  None = 0,

  DoserRun,
  DoserStop,
  DoserRefill,

  RelayOn,
  RelayOff,

  ReloadConfig,
  SaveConfig,
  EmergencyOff,
  EmergencyClear,
  EmergencyOverrideClear,
};

struct Command {
  CommandType type = CommandType::None;
  uint8_t target = 0;
  uint32_t value = 0;
};

class CommandBus {
public:
  bool begin(uint8_t depth = 16) {
    queue_ = xQueueCreate(depth, sizeof(Command));
    return queue_ != nullptr;
  }

  bool send(const Command& command, TickType_t timeout = 0) {
    if (!queue_) return false;
    return xQueueSend(queue_, &command, timeout) == pdTRUE;
  }

  bool receive(Command& command, TickType_t timeout = 0) {
    if (!queue_) return false;
    return xQueueReceive(queue_, &command, timeout) == pdTRUE;
  }

  bool available() const {
    return queue_ && uxQueueMessagesWaiting(queue_) > 0;
  }

private:
  QueueHandle_t queue_ = nullptr;
};
