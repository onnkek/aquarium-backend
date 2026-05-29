#include "runtime/EventWriterTask.h"
#include <RTClib.h>

EventWriterTask::EventWriterTask(
  EventBus& bus,
  EventStorage& doserEvents,
  EventStorage& relayEvents,
  EventStorage& pidEvents,
  Logger& logger
)
  : bus_(bus),
    doserEvents_(doserEvents),
    relayEvents_(relayEvents),
    pidEvents_(pidEvents),
    logger_(logger) {}

bool EventWriterTask::begin() {
  return xTaskCreatePinnedToCore(
    EventWriterTask::taskMain,
    "event-writer",
    4096,
    this,
    1,
    &handle_,
    1
  ) == pdPASS;
}

void EventWriterTask::taskMain(void* arg) {
  static_cast<EventWriterTask*>(arg)->task();
}

void EventWriterTask::task() {
  AquariumEvent event{};
  for (;;) {
    if (!bus_.receive(event, portMAX_DELAY)) {
      continue;
    }
    write(event);
  }
}

void EventWriterTask::write(const AquariumEvent& event) {
  DateTime now(event.unixTime == 0 ? 946684800UL : event.unixTime);

  switch (event.topic) {
    case AquariumEventTopic::Doser:
      doserEvents_.write(event.subtype, event.value, now);
      break;

    case AquariumEventTopic::Relay:
      relayEvents_.write(event.subtype, event.value, now);
      break;

    case AquariumEventTopic::Pid:
      pidEvents_.write(event.subtype, event.value, now);
      break;

    case AquariumEventTopic::System:
    default:
      logger_.debug(LogCategory::System, "system event ignored by EventWriterTask");
      break;
  }
}
