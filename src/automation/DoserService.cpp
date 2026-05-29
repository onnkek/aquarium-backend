#include "automation/DoserService.h"
#include "utils/TimeUtils.h"
#include <string.h>

void DoserService::begin(RuntimeState& state) {
  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    relay_.setDoser(i, false);
    state.doser[i] = DoserRuntime{};
  }
}

bool DoserService::shouldStart(const DoserConfig& cfg, const DateTime& now) {
  if (cfg.mode != Mode::Auto) return false;
  if (!TimeUtils::periodAllows(cfg.period.bits, now.dayOfTheWeek())) return false;

  char today[11];
  TimeUtils::ymd(now, today, sizeof(today));

  // lastRunYmd is persistent doser state stored in /state/doser.json, not in config.
  if (strcmp(cfg.lastRunYmd, today) == 0) return false;

  const uint16_t nowMin = now.hour() * 60u + now.minute();
  const uint16_t startMin = TimeUtils::minuteOfDay(cfg.time);

  // >= is intentional: if ESP32 was rebooting at the exact scheduled minute,
  // the dose still runs once later that day instead of being skipped forever.
  return nowMin >= startMin;
}

uint32_t DoserService::durationMs(const DoserConfig& cfg) {
  if (cfg.rateMlPerSecX10 == 0) return 0;

  // Legacy config stores rate as ml/sec, not ml/min.
  // Example: dosage=7ml, rate=0.5ml/sec => 14 seconds.
  return (static_cast<uint32_t>(cfg.dosageMlX10) * 1000u) / cfg.rateMlPerSecX10;
}

void DoserService::normalizeLegacyDailyFlags(const DateTime& now) {
  AppConfig& cfg = config_.data();
  bool changed = false;
  char today[11];
  TimeUtils::ymd(now, today, sizeof(today));

  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    // Migration path from old config: hasRunToday=true used to be runtime state persisted in JSON.
    // ConfigService maps it to "legacy" because it does not know current RTC date while parsing.
    // Here, when RTC is available, we convert it to today's date once.
    if (strcmp(cfg.doser[i].lastRunYmd, "legacy") == 0) {
      strncpy(cfg.doser[i].lastRunYmd, today, sizeof(cfg.doser[i].lastRunYmd) - 1);
      cfg.doser[i].lastRunYmd[sizeof(cfg.doser[i].lastRunYmd) - 1] = '\0';
      changed = true;
    }
  }

  if (changed) {
    config_.saveDoserState();
    logger_.info(LogCategory::Doser, "legacy hasRunToday migrated to doser state");
  }
}

void DoserService::tick(const DateTime& now, RuntimeState& state) {
  normalizeLegacyDailyFlags(now);
  AppConfig& cfg = config_.data();
  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    DoserConfig& d = cfg.doser[i];
    DoserRuntime& r = state.doser[i];

    if (d.mode == Mode::Off) {
      if (r.status != RelayStatus::Off) stop(i, now, state);
      continue;
    }
    if (d.mode == Mode::On) {
      if (r.status != RelayStatus::On) {
        relay_.setDoser(i, true);
        r.status = RelayStatus::On;
        r.running = false;
        logger_.info(LogCategory::Doser, "pump manual on");
      }
      continue;
    }

    if (!r.running && shouldStart(d, now)) start(i, now, state);

    if (r.running) {
      const uint32_t elapsed = millis() - r.startedAtMs;
      r.progress = r.durationMs == 0 ? 100 : min<uint32_t>(100, elapsed * 100u / r.durationMs);
      if (elapsed >= r.durationMs) stop(i, now, state);
    }
  }
}

bool DoserService::startManual(uint8_t index, const DateTime& now, RuntimeState& state) {
  if (index >= HardwareConfig::DOSER_COUNT) return false;
  if (state.doser[index].running) return false;
  start(index, now, state);
  return state.doser[index].running;
}

bool DoserService::stopManual(uint8_t index, const DateTime& now, RuntimeState& state) {
  if (index >= HardwareConfig::DOSER_COUNT) return false;
  stop(index, now, state);
  return true;
}

void DoserService::start(uint8_t index, const DateTime& now, RuntimeState& state) {
  DoserConfig& d = config_.data().doser[index];
  DoserRuntime& r = state.doser[index];
  const uint32_t ms = durationMs(d);
  if (ms == 0 || d.dosageMlX10 == 0 || d.currentVolumeMlX10 < d.dosageMlX10) {
    char msg[96];
    snprintf(msg, sizeof(msg), "pump %u skipped: invalid dose/rate/volume", index);
    logger_.warn(LogCategory::Doser, msg);
    config_.markDoserRun(index, 0, now);
    return;
  }
  r.running = true;
  r.status = RelayStatus::Running;
  r.startedAtMs = millis();
  r.durationMs = ms;
  r.progress = 0;
  relay_.setDoser(index, true);
  char msg[96];
  snprintf(msg, sizeof(msg), "pump %u auto start, duration=%lu ms", index, static_cast<unsigned long>(ms));
  logger_.info(LogCategory::Doser, msg);
}

void DoserService::stop(uint8_t index, const DateTime& now, RuntimeState& state) {
  DoserConfig& d = config_.data().doser[index];
  DoserRuntime& r = state.doser[index];
  relay_.setDoser(index, false);
  const bool wasRunning = r.running;
  r.running = false;
  r.status = RelayStatus::Off;
  r.progress = wasRunning ? 100 : 0;
  if (wasRunning) {
    AquariumEvent event{};
    event.topic = AquariumEventTopic::Doser;
    event.subtype = index + 1;
    event.value = d.dosageMlX10 / 10.0f;
    event.unixTime = now.unixtime();
    events_.publish(event, 0);
    config_.markDoserRun(index, d.dosageMlX10, now);
    char msg[96];
    snprintf(msg, sizeof(msg), "pump %u auto stop, volume=%.1f ml", index, d.dosageMlX10 / 10.0f);
    logger_.info(LogCategory::Doser, msg);
  }
}
