#include "core/Logger.h"
#include "core/SdGuard.h"
#include "config/BuildConfig.h"
#include <SD.h>

void Logger::begin() {
  journal_.begin(AQUARIUM_LOG_QUEUE_SIZE);
  xTaskCreatePinnedToCore(Logger::taskEntry, "journal", 4096, this, 1, nullptr, 1);
}

void Logger::setTimeProvider(DateTime (*provider)()) { timeProvider_ = provider; }
void Logger::setStorageEnabled(bool enabled) { storageEnabled_ = enabled; }
void Logger::setStoragePaused(bool paused) { storagePaused_ = paused; }

void Logger::info(LogCategory c, const char* msg) { log(LogLevel::Info, c, msg); }
void Logger::warn(LogCategory c, const char* msg) { log(LogLevel::Warn, c, msg); }
void Logger::error(LogCategory c, const char* msg) { log(LogLevel::Error, c, msg); }
void Logger::debug(LogCategory c, const char* msg) {
#if AQUARIUM_ENABLE_DEBUG_LOGS
  log(LogLevel::Debug, c, msg);
#endif
}

void Logger::log(LogLevel level, LogCategory category, const char* msg) {
  if (!journal_.ready()) return;
  DateTime now = timeProvider_ ? timeProvider_() : DateTime(2000, 1, 1, 0, 0, 0);
  JournalRecord e{};
  buildPath(category, now, e.path, sizeof(e.path));
  snprintf(e.line, sizeof(e.line), "[%04d-%02d-%02d %02d:%02d:%02d][%s]: %s",
           now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(),
           levelName(level), msg ? msg : "");
  Serial.println(e.line);
  journal_.publish(e, 0);
}

void Logger::taskEntry(void* arg) { static_cast<Logger*>(arg)->task(); }

void Logger::task() {
  JournalRecord e{};
  for (;;) {
    if (!journal_.receive(e, portMAX_DELAY)) continue;
    const uint32_t nowMs = millis();
    if (!storageEnabled_ || storagePaused_ || nowMs < storageDisabledUntilMs_) {
      continue;
    }

    SdGuard guard(200);
    if (!guard.locked()) {
      storageDisabledUntilMs_ = nowMs + AQUARIUM_STORAGE_BACKOFF_MS;
      continue;
    }

    if (!SD.mkdir("/logs") && !SD.exists("/logs")) {
      storageDisabledUntilMs_ = nowMs + AQUARIUM_STORAGE_BACKOFF_MS;
      continue;
    }
    char y[24], m[32], d[40];
    int yy = 2000, mm = 1, dd = 1;
    sscanf(e.path, "/logs/%d/%d/%d/", &yy, &mm, &dd);
    snprintf(y, sizeof(y), "/logs/%04d", yy);
    snprintf(m, sizeof(m), "/logs/%04d/%02d", yy, mm);
    snprintf(d, sizeof(d), "/logs/%04d/%02d/%02d", yy, mm, dd);
    if ((!SD.exists(y) && !SD.mkdir(y)) ||
        (!SD.exists(m) && !SD.mkdir(m)) ||
        (!SD.exists(d) && !SD.mkdir(d))) {
      storageDisabledUntilMs_ = nowMs + AQUARIUM_STORAGE_BACKOFF_MS;
      continue;
    }

    File f = SD.open(e.path, FILE_APPEND);
    if (f) {
      f.println(e.line);
      f.close();
    } else {
      storageDisabledUntilMs_ = nowMs + AQUARIUM_STORAGE_BACKOFF_MS;
    }
  }
}

void Logger::buildPath(LogCategory category, DateTime now, char* out, size_t outLen) {
  snprintf(out, outLen, "/logs/%04d/%02d/%02d/%s.log",
           now.year(), now.month(), now.day(), categoryFile(category));
}

const char* Logger::levelName(LogLevel level) {
  switch (level) {
    case LogLevel::Debug: return "DEBUG";
    case LogLevel::Info: return "INFO";
    case LogLevel::Warn: return "WARN";
    case LogLevel::Error: return "ERROR";
  }
  return "UNK";
}

const char* Logger::categoryFile(LogCategory category) {
  switch (category) {
    case LogCategory::System: return "system";
    case LogCategory::Relay: return "relay";
    case LogCategory::Doser: return "doser";
    case LogCategory::Storage: return "storage";
    case LogCategory::Api: return "api";
    case LogCategory::Sensor: return "sensor";
    case LogCategory::Wifi: return "wifi";
  }
  return "unknown";
}
