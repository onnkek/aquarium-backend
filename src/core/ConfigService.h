#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <RTClib.h>
#include "models/AppTypes.h"
#include "core/Logger.h"

class ConfigService {
public:
  explicit ConfigService(Logger& logger) : logger_(logger) {}

  bool begin();

  AppConfig& data() { return config_; }
  const AppConfig& data() const { return config_; }

  bool load();
  bool save();

  // Split-config persistence. These write only one subsystem file.
  bool saveSystem();
  bool saveDoser();
  bool saveRelays();
  bool saveTemp();
  bool saveArgb();
  bool saveDoserState();

  // Full-config compatibility helpers. Keep for diagnostics/export only.
  bool updateFromJsonBytes(const uint8_t* data, size_t len, char* error, size_t errorLen);
  bool serializeToJson(String& out) const;

  // Normalized production API: small config documents per subsystem.
  bool serializeSystem(String& out) const;
  bool updateSystemFromJsonBytes(const uint8_t* data, size_t len, char* error, size_t errorLen);

  bool serializeDoserAll(String& out) const;
  bool serializeDoser(uint8_t index, String& out) const;
  bool serializeDoserStateAll(String& out) const;
  bool serializeDoserState(uint8_t index, String& out) const;
  bool resetDoserLastRun(uint8_t index);
  bool resetAllDoserLastRuns();
  bool markDoserLastRunToday(uint8_t index, const DateTime& now);
  bool markAllDoserLastRunsToday(const DateTime& now);
  bool updateDoserFromJsonBytes(uint8_t index, const uint8_t* data, size_t len, char* error, size_t errorLen);

  bool serializeRelayAll(String& out) const;
  bool serializeRelay(const char* id, String& out) const;
  bool updateRelayFromJsonBytes(const char* id, const uint8_t* data, size_t len, char* error, size_t errorLen);

  bool serializeArgb(String& out) const;
  bool updateArgbFromJsonBytes(const uint8_t* data, size_t len, char* error, size_t errorLen);

  bool serializeTemp(String& out) const;
  bool updateTempFromJsonBytes(const uint8_t* data, size_t len, char* error, size_t errorLen);

  bool markDoserRun(uint8_t index, uint16_t dosageMlX10, const DateTime& now);
  bool refillDoser(uint8_t index, uint16_t volumeMlX10 = 0);
  bool setRelayMode(uint8_t index, Mode mode);
  bool applyEmergencyMode();
  bool restorePreEmergencySnapshot();
  bool hasPreEmergencySnapshot() const;
  bool clearPreEmergencySnapshot();
  bool forceAllOutputsOff(); // Backward-compatible alias: applies emergency mode policy.

private:
  Logger& logger_;
  AppConfig config_;

  bool loadSplitConfig();
  bool loadLegacyConfigAndMigrate();
  bool loadDoserState();
  bool savePreEmergencySnapshotIfNeeded();
  bool loadJsonDocument(const char* path, JsonDocument& doc, char* error, size_t errorLen);
  bool writeJsonDocumentAtomic(const char* path, JsonDocument& doc);
  bool ensureDocumentDir(const char* path);

  bool parseDocument(JsonDocument& doc, char* error, size_t errorLen);
  void writeDocument(JsonDocument& doc) const;

  bool parseSystemObject(JsonObject system, char* error, size_t errorLen);
  bool parseDoserDocument(JsonDocument& doc, char* error, size_t errorLen);
  bool parseRelaysDocument(JsonDocument& doc, char* error, size_t errorLen);
  bool parseArgbDocument(JsonDocument& doc, char* error, size_t errorLen);
  bool parseTempDocument(JsonDocument& doc, char* error, size_t errorLen);

  void writeSystemDocument(JsonDocument& doc) const;
  void writeDoserDocument(JsonDocument& doc) const;
  void writeDoserStateDocument(JsonDocument& doc) const;
  void parseDoserStateDocument(JsonDocument& doc);
  void writeRelaysDocument(JsonDocument& doc) const;
  void writeArgbDocument(JsonDocument& doc) const;
  void writeTempDocument(JsonDocument& doc) const;

  bool validate(char* error, size_t errorLen) const;
  bool validateDoser(uint8_t index, char* error, size_t errorLen) const;
  bool validateRelay(const ScheduleRelayConfig& relay, char* error, size_t errorLen) const;
  bool validateArgb(char* error, size_t errorLen) const;
  bool validateTemp(char* error, size_t errorLen) const;

  void parseRelay(JsonObject obj, ScheduleRelayConfig& out, const char* defaultName);
  void writeRelay(JsonObject obj, const ScheduleRelayConfig& in) const;

  void parseDoserPartial(JsonObject obj, DoserConfig& out);
  void writeDoser(JsonObject obj, const DoserConfig& in) const;

  void parseArgbPartial(JsonObject obj, ArgbConfig& out);
  void writeArgb(JsonObject obj) const;

  void parseTempPartial(JsonObject obj, TempConfig& out);
  void writeTemp(JsonObject obj) const;

  bool emergencySnapshotExists() const;
  void writeEmergencySnapshot(JsonDocument& doc) const;
  bool parseEmergencySnapshot(JsonDocument& doc);

  ScheduleRelayConfig* relayById(const char* id);
  const ScheduleRelayConfig* relayById(const char* id) const;

  uint8_t parsePeriod(JsonObject p) const;
  void writePeriod(JsonObject p, uint8_t mask) const;

  static uint16_t mlToX10(float v);
  static float x10ToMl(uint16_t v);
  static bool parseSmallJson(const uint8_t* data, size_t len, JsonDocument& doc, char* error, size_t errorLen);
};
