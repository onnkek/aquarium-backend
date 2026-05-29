#pragma once

#include "core/EventBus.h"
#include "core/Logger.h"
#include "storage/EventStorage.h"

class EventWriterTask {
public:
  EventWriterTask(
    EventBus& bus,
    EventStorage& doserEvents,
    EventStorage& relayEvents,
    EventStorage& pidEvents,
    Logger& logger
  );

  bool begin();

private:
  EventBus& bus_;
  EventStorage& doserEvents_;
  EventStorage& relayEvents_;
  EventStorage& pidEvents_;
  Logger& logger_;
  TaskHandle_t handle_ = nullptr;

  static void taskMain(void* arg);
  void task();
  void write(const AquariumEvent& event);
};
