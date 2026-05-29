#include "automation/ScheduledRelayService.h"
#include "utils/TimeUtils.h"

void ScheduledRelayService::tick(uint8_t index, const ScheduleRelayConfig& cfg, const DateTime& now, RuntimeState& state) {
  bool target = false;
  const char* reason = "manual";
  switch (cfg.mode) {
    case Mode::Off: target = false; reason = "manual off"; break;
    case Mode::On: target = true; reason = "manual on"; break;
    case Mode::Auto: target = TimeUtils::isActiveWindow(cfg.on, cfg.off, now); reason = target ? "auto on" : "auto off"; break;
  }
  set(index, target, reason, now, state);
}

void ScheduledRelayService::force(uint8_t index, bool on, const char* reason, const DateTime& now, RuntimeState& state) {
  set(index, on, reason, now, state);
}

void ScheduledRelayService::set(uint8_t index, bool on, const char* reason, const DateTime& now, RuntimeState& state) {
  if (index >= HardwareConfig::EXTRA_RELAY_COUNT) return;
  const RelayStatus current = state.extra[index];
  const RelayStatus next = on ? RelayStatus::On : RelayStatus::Off;
  if (current == next) return;
  relay_.setExtra(index, on);
  state.extra[index] = next;
  AquariumEvent event{};
  event.topic = AquariumEventTopic::Relay;
  event.subtype = index + 1;
  event.value = on ? 1.0f : 0.0f;
  event.unixTime = now.unixtime();
  events_.publish(event, 0);
  logger_.info(LogCategory::Relay, reason);
}
