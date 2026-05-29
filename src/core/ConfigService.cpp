#include "core/ConfigService.h"
#include "config/BuildConfig.h"
#include "utils/TimeUtils.h"
#include "utils/FixedString.h"
#include <SPIFFS.h>
#include <string.h>
#include <Arduino.h>

static const char* dayKeys[7] = {"su", "mo", "tu", "we", "th", "fr", "sa"};

template <typename T>
static T clampValue(T value, T minValue, T maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

static void setRgbColor(RgbColor& dst, uint8_t r, uint8_t g, uint8_t b) {
  dst.r = r;
  dst.g = g;
  dst.b = b;
}

bool ConfigService::begin() { return load(); }

uint16_t ConfigService::mlToX10(float v) {
  if (v < 0) v = 0;
  if (v > 6553.5f) v = 6553.5f;
  return static_cast<uint16_t>(v * 10.0f + 0.5f);
}

float ConfigService::x10ToMl(uint16_t v) { return v / 10.0f; }

bool ConfigService::parseSmallJson(
  const uint8_t* data,
  size_t len,
  JsonDocument& doc,
  char* error,
  size_t errorLen
) {
  if (!data || len == 0) {
    snprintf(error, errorLen, "empty body");
    return false;
  }
  if (len > AQUARIUM_MAX_CONFIG_JSON_BYTES) {
    snprintf(error, errorLen, "body too large");
    return false;
  }
  DeserializationError err = deserializeJson(doc, data, len);
  if (err) {
    snprintf(error, errorLen, "bad json: %s", err.c_str());
    return false;
  }
  if (!doc.is<JsonObject>()) {
    snprintf(error, errorLen, "json object expected");
    return false;
  }
  return true;
}

bool ConfigService::load() {
  if (loadSplitConfig()) {
    logger_.info(LogCategory::System, "split config loaded");
    return true;
  }

  if (loadLegacyConfigAndMigrate()) {
    logger_.info(LogCategory::System, "legacy config migrated to split config");
    return true;
  }

  logger_.error(LogCategory::System, "config load failed");
  return false;
}

bool ConfigService::save() {
  return saveSystem() && saveDoser() && saveRelays() && saveTemp() && saveArgb();
}

bool ConfigService::saveSystem() {
  StaticJsonDocument<192> doc;
  writeSystemDocument(doc);
  return writeJsonDocumentAtomic(AQUARIUM_CONFIG_SYSTEM_PATH, doc);
}

bool ConfigService::saveDoser() {
  StaticJsonDocument<1536> doc;
  writeDoserDocument(doc);
  return writeJsonDocumentAtomic(AQUARIUM_CONFIG_DOSER_PATH, doc);
}

bool ConfigService::saveDoserState() {
  StaticJsonDocument<384> doc;
  writeDoserStateDocument(doc);
  return writeJsonDocumentAtomic(AQUARIUM_STATE_DOSER_PATH, doc);
}

bool ConfigService::saveRelays() {
  StaticJsonDocument<1024> doc;
  writeRelaysDocument(doc);
  return writeJsonDocumentAtomic(AQUARIUM_CONFIG_RELAYS_PATH, doc);
}

bool ConfigService::saveTemp() {
  StaticJsonDocument<256> doc;
  writeTempDocument(doc);
  return writeJsonDocumentAtomic(AQUARIUM_CONFIG_TEMP_PATH, doc);
}

bool ConfigService::saveArgb() {
  DynamicJsonDocument doc(4096);
  writeArgbDocument(doc);
  return writeJsonDocumentAtomic(AQUARIUM_CONFIG_ARGB_PATH, doc);
}

bool ConfigService::loadSplitConfig() {
  if (!SPIFFS.exists(AQUARIUM_CONFIG_SYSTEM_PATH) ||
      !SPIFFS.exists(AQUARIUM_CONFIG_DOSER_PATH) ||
      !SPIFFS.exists(AQUARIUM_CONFIG_RELAYS_PATH) ||
      !SPIFFS.exists(AQUARIUM_CONFIG_TEMP_PATH) ||
      !SPIFFS.exists(AQUARIUM_CONFIG_ARGB_PATH)) {
    return false;
  }

  AppConfig old = config_;
  char error[128] = {0};

  StaticJsonDocument<256> systemDoc;
  if (!loadJsonDocument(AQUARIUM_CONFIG_SYSTEM_PATH, systemDoc, error, sizeof(error)) ||
      !parseSystemObject(systemDoc.as<JsonObject>(), error, sizeof(error))) {
    config_ = old;
    logger_.error(LogCategory::System, error);
    return false;
  }

  StaticJsonDocument<1536> doserDoc;
  if (!loadJsonDocument(AQUARIUM_CONFIG_DOSER_PATH, doserDoc, error, sizeof(error)) ||
      !parseDoserDocument(doserDoc, error, sizeof(error))) {
    config_ = old;
    logger_.error(LogCategory::System, error);
    return false;
  }

  StaticJsonDocument<1024> relaysDoc;
  if (!loadJsonDocument(AQUARIUM_CONFIG_RELAYS_PATH, relaysDoc, error, sizeof(error)) ||
      !parseRelaysDocument(relaysDoc, error, sizeof(error))) {
    config_ = old;
    logger_.error(LogCategory::System, error);
    return false;
  }

  StaticJsonDocument<384> tempDoc;
  if (!loadJsonDocument(AQUARIUM_CONFIG_TEMP_PATH, tempDoc, error, sizeof(error)) ||
      !parseTempDocument(tempDoc, error, sizeof(error))) {
    config_ = old;
    logger_.error(LogCategory::System, error);
    return false;
  }

  DynamicJsonDocument argbDoc(4096);
  if (!loadJsonDocument(AQUARIUM_CONFIG_ARGB_PATH, argbDoc, error, sizeof(error)) ||
      !parseArgbDocument(argbDoc, error, sizeof(error))) {
    config_ = old;
    logger_.error(LogCategory::System, error);
    return false;
  }

  if (!validate(error, sizeof(error))) {
    config_ = old;
    logger_.error(LogCategory::System, error);
    return false;
  }

  // Doser last-run dates are persistent state, not config. Missing state is OK.
  loadDoserState();

  return true;
}

bool ConfigService::loadLegacyConfigAndMigrate() {
  const char* path = AQUARIUM_LEGACY_CONFIG_PATH;

  if (!SPIFFS.exists(path) && SPIFFS.exists(AQUARIUM_LEGACY_CONFIG_BAK_PATH)) {
    SPIFFS.rename(AQUARIUM_LEGACY_CONFIG_BAK_PATH, path);
  }

  if (!SPIFFS.exists(path)) return false;

  File f = SPIFFS.open(path, FILE_READ);
  if (!f) return false;

  DynamicJsonDocument doc(AQUARIUM_MAX_CONFIG_JSON_BYTES);
  DeserializationError err = deserializeJson(doc, f);
  f.close();

  if (err) {
    logger_.error(LogCategory::System, err.c_str());
    return false;
  }

  char error[128] = {0};
  if (!parseDocument(doc, error, sizeof(error))) {
    logger_.error(LogCategory::System, error);
    return false;
  }

  return save() && saveDoserState();
}

bool ConfigService::loadDoserState() {
  if (!SPIFFS.exists(AQUARIUM_STATE_DOSER_PATH)) {
    return true;
  }

  char error[128] = {0};
  StaticJsonDocument<384> doc;
  if (!loadJsonDocument(AQUARIUM_STATE_DOSER_PATH, doc, error, sizeof(error))) {
    logger_.warn(LogCategory::System, error);
    return false;
  }

  parseDoserStateDocument(doc);
  return true;
}

bool ConfigService::loadJsonDocument(const char* path, JsonDocument& doc, char* error, size_t errorLen) {
  File f = SPIFFS.open(path, FILE_READ);
  if (!f) {
    snprintf(error, errorLen, "open failed: %s", path);
    return false;
  }

  DeserializationError err = deserializeJson(doc, f);
  f.close();

  if (err) {
    snprintf(error, errorLen, "%s: %s", path, err.c_str());
    return false;
  }

  return true;
}

bool ConfigService::writeJsonDocumentAtomic(const char* path, JsonDocument& doc) {
  if (!path || !*path) return false;

  if (!ensureDocumentDir(path)) {
    logger_.error(LogCategory::System, "document dir create failed");
    return false;
  }

  char tmp[80];
  char bak[80];
  snprintf(tmp, sizeof(tmp), "%s.tmp", path);
  snprintf(bak, sizeof(bak), "%s.bak", path);

  File f = SPIFFS.open(tmp, FILE_WRITE);
  if (!f) {
    logger_.error(LogCategory::System, "config tmp open failed");
    return false;
  }

  const size_t written = serializeJson(doc, f);
  f.flush();
  f.close();

  if (written == 0) {
    SPIFFS.remove(tmp);
    logger_.error(LogCategory::System, "config serialize failed");
    return false;
  }

  if (SPIFFS.exists(bak)) SPIFFS.remove(bak);

  if (SPIFFS.exists(path)) {
    SPIFFS.rename(path, bak);
  }

  if (!SPIFFS.rename(tmp, path)) {
    if (SPIFFS.exists(bak)) SPIFFS.rename(bak, path);
    logger_.error(LogCategory::System, "config rename failed");
    return false;
  }

  if (SPIFFS.exists(bak)) SPIFFS.remove(bak);
  return true;
}

bool ConfigService::ensureDocumentDir(const char* path) {
  if (!path || path[0] != '/') return false;

  char dir[32] = {0};
  const char* slash = strrchr(path + 1, '/');
  if (!slash) {
    return true;
  }

  size_t len = slash - path;
  if (len >= sizeof(dir)) return false;
  memcpy(dir, path, len);
  dir[len] = '\0';

  if (SPIFFS.exists(dir)) return true;
  return SPIFFS.mkdir(dir);
}

bool ConfigService::updateFromJsonBytes(const uint8_t* data, size_t len, char* error, size_t errorLen) {
  if (len == 0 || len > AQUARIUM_MAX_CONFIG_JSON_BYTES) {
    snprintf(error, errorLen, "config too large");
    return false;
  }

  DynamicJsonDocument doc(AQUARIUM_MAX_CONFIG_JSON_BYTES);
  DeserializationError err = deserializeJson(doc, data, len);
  if (err) {
    snprintf(error, errorLen, "bad json: %s", err.c_str());
    return false;
  }

  AppConfig old = config_;
  if (!parseDocument(doc, error, errorLen)) {
    config_ = old;
    return false;
  }

  if (!save()) {
    snprintf(error, errorLen, "save failed");
    config_ = old;
    return false;
  }

  return true;
}

bool ConfigService::serializeToJson(String& out) const {
  DynamicJsonDocument doc(AQUARIUM_MAX_CONFIG_JSON_BYTES);
  writeDocument(doc);
  out.reserve(measureJson(doc) + 16);
  serializeJson(doc, out);
  return true;
}

bool ConfigService::serializeSystem(String& out) const {
  StaticJsonDocument<192> doc;
  JsonObject system = doc.to<JsonObject>();
  system["name"] = config_.system.name;
  system["update"] = config_.system.updateSec;
  system["pwm"] = config_.system.fanPwmPercent;
  serializeJson(doc, out);
  return true;
}

bool ConfigService::updateSystemFromJsonBytes(const uint8_t* data, size_t len, char* error, size_t errorLen) {
  StaticJsonDocument<256> doc;
  if (!parseSmallJson(data, len, doc, error, errorLen)) return false;

  AppConfig old = config_;
  JsonObject obj = doc.as<JsonObject>();

  if (obj.containsKey("name")) copyString(config_.system.name, obj["name"] | config_.system.name);
  if (obj.containsKey("update")) config_.system.updateSec = clampValue<int>(obj["update"] | config_.system.updateSec, 1, 3600);
  if (obj.containsKey("pwm")) config_.system.fanPwmPercent = clampValue<int>(obj["pwm"] | config_.system.fanPwmPercent, 0, 100);

  if (!validate(error, errorLen)) {
    config_ = old;
    return false;
  }
  if (!saveSystem()) {
    config_ = old;
    snprintf(error, errorLen, "save system failed");
    return false;
  }
  return true;
}

bool ConfigService::serializeDoserAll(String& out) const {
  StaticJsonDocument<1536> doc;
  JsonArray arr = doc.to<JsonArray>();
  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    JsonObject obj = arr.add<JsonObject>();
    obj["id"] = i;
    writeDoser(obj, config_.doser[i]);
  }
  serializeJson(doc, out);
  return true;
}

bool ConfigService::serializeDoser(uint8_t index, String& out) const {
  if (index >= HardwareConfig::DOSER_COUNT) return false;
  StaticJsonDocument<512> doc;
  JsonObject obj = doc.to<JsonObject>();
  obj["id"] = index;
  writeDoser(obj, config_.doser[index]);
  serializeJson(doc, out);
  return true;
}



bool ConfigService::serializeDoserStateAll(String& out) const {
  StaticJsonDocument<768> doc;
  JsonArray arr = doc.to<JsonArray>();

  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    JsonObject obj = arr.add<JsonObject>();
    obj["id"] = i;
    obj["lastRunYmd"] = config_.doser[i].lastRunYmd;
  }

  serializeJson(doc, out);
  return true;
}

bool ConfigService::serializeDoserState(uint8_t index, String& out) const {
  if (index >= HardwareConfig::DOSER_COUNT) return false;

  StaticJsonDocument<192> doc;
  JsonObject obj = doc.to<JsonObject>();
  obj["id"] = index;
  obj["lastRunYmd"] = config_.doser[index].lastRunYmd;

  serializeJson(doc, out);
  return true;
}

bool ConfigService::resetDoserLastRun(uint8_t index) {
  if (index >= HardwareConfig::DOSER_COUNT) return false;

  config_.doser[index].lastRunYmd[0] = '\0';
  return saveDoserState();
}

bool ConfigService::resetAllDoserLastRuns() {
  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    config_.doser[i].lastRunYmd[0] = '\0';
  }

  return saveDoserState();
}

bool ConfigService::markDoserLastRunToday(uint8_t index, const DateTime& now) {
  if (index >= HardwareConfig::DOSER_COUNT) return false;

  TimeUtils::ymd(now, config_.doser[index].lastRunYmd, sizeof(config_.doser[index].lastRunYmd));
  return saveDoserState();
}

bool ConfigService::markAllDoserLastRunsToday(const DateTime& now) {
  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    TimeUtils::ymd(now, config_.doser[i].lastRunYmd, sizeof(config_.doser[i].lastRunYmd));
  }

  return saveDoserState();
}

bool ConfigService::updateDoserFromJsonBytes(uint8_t index, const uint8_t* data, size_t len, char* error, size_t errorLen) {
  if (index >= HardwareConfig::DOSER_COUNT) {
    snprintf(error, errorLen, "bad doser index");
    return false;
  }

  StaticJsonDocument<768> doc;
  if (!parseSmallJson(data, len, doc, error, errorLen)) return false;

  AppConfig old = config_;
  parseDoserPartial(doc.as<JsonObject>(), config_.doser[index]);

  if (!validateDoser(index, error, errorLen)) {
    config_ = old;
    return false;
  }
  if (!saveDoser()) {
    config_ = old;
    snprintf(error, errorLen, "save doser failed");
    return false;
  }
  return true;
}

bool ConfigService::serializeRelayAll(String& out) const {
  StaticJsonDocument<1024> doc;
  JsonObject root = doc.to<JsonObject>();

  JsonObject co2 = root["co2"].to<JsonObject>();
  co2["id"] = "co2";
  writeRelay(co2, config_.co2);

  JsonObject o2 = root["o2"].to<JsonObject>();
  o2["id"] = "o2";
  writeRelay(o2, config_.o2);

  JsonObject filter = root["filter"].to<JsonObject>();
  filter["id"] = "filter";
  writeRelay(filter, config_.filter);

  JsonObject light = root["light"].to<JsonObject>();
  light["id"] = "light";
  writeRelay(light, config_.light);

  serializeJson(doc, out);
  return true;
}

bool ConfigService::serializeRelay(const char* id, String& out) const {
  const ScheduleRelayConfig* relay = relayById(id);
  if (!relay) return false;

  StaticJsonDocument<256> doc;
  JsonObject obj = doc.to<JsonObject>();
  obj["id"] = id;
  writeRelay(obj, *relay);
  serializeJson(doc, out);
  return true;
}

bool ConfigService::updateRelayFromJsonBytes(const char* id, const uint8_t* data, size_t len, char* error, size_t errorLen) {
  ScheduleRelayConfig* relay = relayById(id);
  if (!relay) {
    snprintf(error, errorLen, "bad relay id");
    return false;
  }

  StaticJsonDocument<384> doc;
  if (!parseSmallJson(data, len, doc, error, errorLen)) return false;

  AppConfig old = config_;
  JsonObject obj = doc.as<JsonObject>();

  if (obj.containsKey("name")) copyString(relay->name, obj["name"] | relay->name);
  if (obj.containsKey("on")) TimeUtils::parseHHMM(obj["on"] | "00:00", relay->on);
  if (obj.containsKey("off")) TimeUtils::parseHHMM(obj["off"] | "00:00", relay->off);
  if (obj.containsKey("mode")) relay->mode = static_cast<Mode>(clampValue<int>(obj["mode"] | static_cast<uint8_t>(relay->mode), 0, 2));

  if (!validateRelay(*relay, error, errorLen)) {
    config_ = old;
    return false;
  }
  if (!saveRelays()) {
    config_ = old;
    snprintf(error, errorLen, "save relays failed");
    return false;
  }
  return true;
}

bool ConfigService::serializeArgb(String& out) const {
  DynamicJsonDocument doc(4096);
  JsonObject obj = doc.to<JsonObject>();
  writeArgb(obj);
  serializeJson(doc, out);
  return true;
}

bool ConfigService::updateArgbFromJsonBytes(const uint8_t* data, size_t len, char* error, size_t errorLen) {
  DynamicJsonDocument doc(4096);
  if (!parseSmallJson(data, len, doc, error, errorLen)) return false;

  AppConfig old = config_;
  parseArgbPartial(doc.as<JsonObject>(), config_.argb);

  if (!validateArgb(error, errorLen)) {
    config_ = old;
    return false;
  }
  if (!saveArgb()) {
    config_ = old;
    snprintf(error, errorLen, "save argb failed");
    return false;
  }
  return true;
}

bool ConfigService::serializeTemp(String& out) const {
  StaticJsonDocument<256> doc;
  JsonObject obj = doc.to<JsonObject>();
  writeTemp(obj);
  serializeJson(doc, out);
  return true;
}

bool ConfigService::updateTempFromJsonBytes(const uint8_t* data, size_t len, char* error, size_t errorLen) {
  StaticJsonDocument<384> doc;
  if (!parseSmallJson(data, len, doc, error, errorLen)) return false;

  AppConfig old = config_;
  parseTempPartial(doc.as<JsonObject>(), config_.temp);

  if (!validateTemp(error, errorLen)) {
    config_ = old;
    return false;
  }
  if (!saveTemp()) {
    config_ = old;
    snprintf(error, errorLen, "save temp failed");
    return false;
  }
  return true;
}

bool ConfigService::parseSystemObject(JsonObject system, char* error, size_t errorLen) {
  if (system.isNull()) {
    snprintf(error, errorLen, "system config missing");
    return false;
  }

  copyString(config_.system.name, system["name"] | "System");
  config_.system.updateSec = clampValue<int>(system["update"] | 1, 1, 3600);
  config_.system.fanPwmPercent = clampValue<int>(system["pwm"] | 30, 0, 100);
  return true;
}

bool ConfigService::parseDoserDocument(JsonDocument& doc, char* error, size_t errorLen) {
  JsonObject doser = doc.as<JsonObject>();
  if (doser.isNull()) {
    snprintf(error, errorLen, "doser config missing");
    return false;
  }

  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    char key[2] = {static_cast<char>('0' + i), 0};
    parseDoserPartial(doser[key], config_.doser[i]);
    if (!validateDoser(i, error, errorLen)) return false;
  }

  return true;
}

bool ConfigService::parseRelaysDocument(JsonDocument& doc, char* error, size_t errorLen) {
  JsonObject relays = doc.as<JsonObject>();
  if (relays.isNull()) {
    snprintf(error, errorLen, "relays config missing");
    return false;
  }

  parseRelay(relays["co2"], config_.co2, "CO2 System");
  parseRelay(relays["o2"], config_.o2, "O2 System");
  parseRelay(relays["filter"], config_.filter, "Filtering");
  parseRelay(relays["light"], config_.light, "Lighting");

  if (!validateRelay(config_.co2, error, errorLen)) return false;
  if (!validateRelay(config_.o2, error, errorLen)) return false;
  if (!validateRelay(config_.filter, error, errorLen)) return false;
  if (!validateRelay(config_.light, error, errorLen)) return false;
  return true;
}

bool ConfigService::parseArgbDocument(JsonDocument& doc, char* error, size_t errorLen) {
  parseArgbPartial(doc.as<JsonObject>(), config_.argb);
  return validateArgb(error, errorLen);
}

bool ConfigService::parseTempDocument(JsonDocument& doc, char* error, size_t errorLen) {
  parseTempPartial(doc.as<JsonObject>(), config_.temp);
  return validateTemp(error, errorLen);
}

void ConfigService::writeSystemDocument(JsonDocument& doc) const {
  JsonObject system = doc.to<JsonObject>();
  system["name"] = config_.system.name;
  system["update"] = config_.system.updateSec;
  system["pwm"] = config_.system.fanPwmPercent;
}

void ConfigService::writeDoserDocument(JsonDocument& doc) const {
  JsonObject doser = doc.to<JsonObject>();
  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    char key[2] = {static_cast<char>('0' + i), 0};
    JsonObject p = doser[key].to<JsonObject>();
    writeDoser(p, config_.doser[i]);
  }
}

void ConfigService::writeDoserStateDocument(JsonDocument& doc) const {
  JsonObject root = doc.to<JsonObject>();
  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    char key[2] = {static_cast<char>('0' + i), 0};
    root[key]["lastRunYmd"] = config_.doser[i].lastRunYmd;
  }
}

void ConfigService::parseDoserStateDocument(JsonDocument& doc) {
  JsonObject root = doc.as<JsonObject>();
  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    char key[2] = {static_cast<char>('0' + i), 0};
    JsonObject obj = root[key];
    if (!obj.isNull() && obj["lastRunYmd"].is<const char*>()) {
      copyString(config_.doser[i].lastRunYmd, obj["lastRunYmd"] | "");
    }
  }
}

void ConfigService::writeRelaysDocument(JsonDocument& doc) const {
  JsonObject relays = doc.to<JsonObject>();
  writeRelay(relays["co2"].to<JsonObject>(), config_.co2);
  writeRelay(relays["o2"].to<JsonObject>(), config_.o2);
  writeRelay(relays["filter"].to<JsonObject>(), config_.filter);
  writeRelay(relays["light"].to<JsonObject>(), config_.light);
}

void ConfigService::writeArgbDocument(JsonDocument& doc) const {
  writeArgb(doc.to<JsonObject>());
}

void ConfigService::writeTempDocument(JsonDocument& doc) const {
  writeTemp(doc.to<JsonObject>());
}

bool ConfigService::parseDocument(JsonDocument& doc, char* error, size_t errorLen) {
  JsonObject system = doc["system"];
  copyString(config_.system.name, system["name"] | "System");
  config_.system.updateSec = clampValue<int>(system["update"] | 1, 1, 3600);
  config_.system.fanPwmPercent = clampValue<int>(system["pwm"] | 30, 0, 100);

  JsonObject doser = doc["doser"];
  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    char key[2] = {static_cast<char>('0' + i), 0};
    DoserConfig& out = config_.doser[i];
    parseDoserPartial(doser[key], out);
  }

  parseRelay(doc["co2"], config_.co2, "CO2 System");
  parseRelay(doc["o2"], config_.o2, "O2 System");
  parseRelay(doc["filter"], config_.filter, "Filtering");
  parseRelay(doc["light"], config_.light, "Lighting");
  parseArgbPartial(doc["argb"], config_.argb);
  parseTempPartial(doc["temp"], config_.temp);

  return validate(error, errorLen);
}

void ConfigService::writeDocument(JsonDocument& doc) const {
  JsonObject system = doc["system"].to<JsonObject>();
  system["name"] = config_.system.name;
  system["update"] = config_.system.updateSec;
  system["pwm"] = config_.system.fanPwmPercent;

  JsonObject doser = doc["doser"].to<JsonObject>();
  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    char key[2] = {static_cast<char>('0' + i), 0};
    JsonObject p = doser[key].to<JsonObject>();
    writeDoser(p, config_.doser[i]);
  }

  writeRelay(doc["co2"].to<JsonObject>(), config_.co2);
  writeRelay(doc["o2"].to<JsonObject>(), config_.o2);
  writeRelay(doc["filter"].to<JsonObject>(), config_.filter);
  writeRelay(doc["light"].to<JsonObject>(), config_.light);
  writeArgb(doc["argb"].to<JsonObject>());
  writeTemp(doc["temp"].to<JsonObject>());
}

void ConfigService::parseDoserPartial(JsonObject obj, DoserConfig& out) {
  if (obj.isNull()) return;

  if (obj.containsKey("name")) copyString(out.name, obj["name"] | out.name);
  if (obj.containsKey("period")) out.period.bits = parsePeriod(obj["period"]);
  if (obj.containsKey("time")) TimeUtils::parseHHMM(obj["time"] | "00:00", out.time);
  if (obj.containsKey("currentVolume")) out.currentVolumeMlX10 = mlToX10(obj["currentVolume"] | 0.0f);
  if (obj.containsKey("maxVolume")) out.maxVolumeMlX10 = mlToX10(obj["maxVolume"] | 0.0f);
  if (obj.containsKey("mode")) out.mode = static_cast<Mode>(clampValue<int>(obj["mode"] | 0, 0, 2));
  if (obj.containsKey("dosage")) out.dosageMlX10 = mlToX10(obj["dosage"] | 0.0f);
  if (obj.containsKey("rate")) out.rateMlPerSecX10 = mlToX10(obj["rate"] | 1.0f);
  if (obj.containsKey("lastRunYmd")) copyString(out.lastRunYmd, obj["lastRunYmd"] | "");

  if ((obj["hasRunToday"] | false) && out.lastRunYmd[0] == '\0') {
    copyString(out.lastRunYmd, "legacy");
  }
}

void ConfigService::writeDoser(JsonObject obj, const DoserConfig& in) const {
  obj["name"] = in.name;
  writePeriod(obj["period"].to<JsonObject>(), in.period.bits);

  char time[6];
  snprintf(time, sizeof(time), "%02u:%02u", in.time.hour, in.time.minute);
  obj["time"] = time;
  obj["currentVolume"] = x10ToMl(in.currentVolumeMlX10);
  obj["maxVolume"] = x10ToMl(in.maxVolumeMlX10);
  obj["mode"] = static_cast<uint8_t>(in.mode);
  obj["dosage"] = x10ToMl(in.dosageMlX10);
  obj["rate"] = x10ToMl(in.rateMlPerSecX10);
}

void ConfigService::parseRelay(JsonObject obj, ScheduleRelayConfig& out, const char* defaultName) {
  if (out.name[0] == '\0') copyString(out.name, defaultName);
  if (obj.isNull()) return;

  if (obj.containsKey("name")) copyString(out.name, obj["name"] | defaultName);
  if (obj.containsKey("on")) TimeUtils::parseHHMM(obj["on"] | "00:00", out.on);
  if (obj.containsKey("off")) TimeUtils::parseHHMM(obj["off"] | "00:00", out.off);
  if (obj.containsKey("mode")) out.mode = static_cast<Mode>(clampValue<int>(obj["mode"] | 0, 0, 2));
}

void ConfigService::writeRelay(JsonObject obj, const ScheduleRelayConfig& in) const {
  obj["name"] = in.name;
  char on[6], off[6];
  snprintf(on, sizeof(on), "%02u:%02u", in.on.hour, in.on.minute);
  snprintf(off, sizeof(off), "%02u:%02u", in.off.hour, in.off.minute);
  obj["on"] = on;
  obj["off"] = off;
  obj["mode"] = static_cast<uint8_t>(in.mode);
}

void ConfigService::parseArgbPartial(JsonObject a, ArgbConfig& out) {
  if (a.isNull()) return;

  if (a.containsKey("name")) copyString(out.name, a["name"] | out.name);
  if (a.containsKey("mode")) out.mode = static_cast<Mode>(clampValue<int>(a["mode"] | static_cast<uint8_t>(out.mode), 0, 2));
  if (a.containsKey("style")) out.style = clampValue<int>(a["style"] | out.style, 1, 4);
  if (a.containsKey("brightness")) out.brightness = clampValue<int>(a["brightness"] | out.brightness, 0, 255);
  if (a.containsKey("on")) TimeUtils::parseHHMM(a["on"] | "00:00", out.on);
  if (a.containsKey("off")) TimeUtils::parseHHMM(a["off"] | "00:00", out.off);

  if (a.containsKey("static")) {
    setRgbColor(
      out.staticColor,
      static_cast<uint8_t>(a["static"]["r"] | out.staticColor.r),
      static_cast<uint8_t>(a["static"]["g"] | out.staticColor.g),
      static_cast<uint8_t>(a["static"]["b"] | out.staticColor.b)
    );
  }

  if (a.containsKey("gradient")) {
    setRgbColor(
      out.gradientStart,
      static_cast<uint8_t>(a["gradient"]["start"]["r"] | out.gradientStart.r),
      static_cast<uint8_t>(a["gradient"]["start"]["g"] | out.gradientStart.g),
      static_cast<uint8_t>(a["gradient"]["start"]["b"] | out.gradientStart.b)
    );
    setRgbColor(
      out.gradientEnd,
      static_cast<uint8_t>(a["gradient"]["end"]["r"] | out.gradientEnd.r),
      static_cast<uint8_t>(a["gradient"]["end"]["g"] | out.gradientEnd.g),
      static_cast<uint8_t>(a["gradient"]["end"]["b"] | out.gradientEnd.b)
    );
  }

  if (a.containsKey("cycle")) {
    out.cycleSpeedMs = clampValue<int>(a["cycle"]["speed"] | out.cycleSpeedMs, 1, 60000);
  }

  if (a.containsKey("custom")) {
    JsonArray custom = a["custom"].as<JsonArray>();
    for (uint8_t i = 0; i < HardwareConfig::NUM_LEDS; ++i) {
      JsonObject c = custom[i];
      if (c.isNull()) continue;
      setRgbColor(
        out.custom[i],
        static_cast<uint8_t>(c["r"] | out.custom[i].r),
        static_cast<uint8_t>(c["g"] | out.custom[i].g),
        static_cast<uint8_t>(c["b"] | out.custom[i].b)
      );
    }
  }
}

void ConfigService::writeArgb(JsonObject a) const {
  a["name"] = config_.argb.name;
  a["mode"] = static_cast<uint8_t>(config_.argb.mode);
  a["style"] = config_.argb.style;
  a["brightness"] = config_.argb.brightness;

  char on[6], off[6];
  snprintf(on, sizeof(on), "%02u:%02u", config_.argb.on.hour, config_.argb.on.minute);
  snprintf(off, sizeof(off), "%02u:%02u", config_.argb.off.hour, config_.argb.off.minute);
  a["on"] = on;
  a["off"] = off;

  a["static"]["r"] = config_.argb.staticColor.r;
  a["static"]["g"] = config_.argb.staticColor.g;
  a["static"]["b"] = config_.argb.staticColor.b;

  a["gradient"]["start"]["r"] = config_.argb.gradientStart.r;
  a["gradient"]["start"]["g"] = config_.argb.gradientStart.g;
  a["gradient"]["start"]["b"] = config_.argb.gradientStart.b;
  a["gradient"]["end"]["r"] = config_.argb.gradientEnd.r;
  a["gradient"]["end"]["g"] = config_.argb.gradientEnd.g;
  a["gradient"]["end"]["b"] = config_.argb.gradientEnd.b;

  JsonArray custom = a["custom"].to<JsonArray>();
  for (uint8_t i = 0; i < HardwareConfig::NUM_LEDS; ++i) {
    JsonObject c = custom.add<JsonObject>();
    c["r"] = config_.argb.custom[i].r;
    c["g"] = config_.argb.custom[i].g;
    c["b"] = config_.argb.custom[i].b;
  }

  a["cycle"]["speed"] = config_.argb.cycleSpeedMs;
}

void ConfigService::parseTempPartial(JsonObject t, TempConfig& out) {
  if (t.isNull()) return;

  if (t.containsKey("name")) copyString(out.name, t["name"] | out.name);
  if (t.containsKey("setting")) out.setting = t["setting"] | out.setting;
  if (t.containsKey("hysteresis")) out.hysteresis = t["hysteresis"] | out.hysteresis;
  if (t.containsKey("k")) out.k = t["k"] | out.k;
  if (t.containsKey("timeout")) out.timeoutSec = clampValue<int>(t["timeout"] | out.timeoutSec, 1, 3600);
  if (t.containsKey("mode")) out.mode = static_cast<TempMode>(clampValue<int>(t["mode"] | static_cast<uint8_t>(out.mode), 0, 4));
}

void ConfigService::writeTemp(JsonObject t) const {
  t["name"] = config_.temp.name;
  t["setting"] = config_.temp.setting;
  t["hysteresis"] = config_.temp.hysteresis;
  t["k"] = config_.temp.k;
  t["timeout"] = config_.temp.timeoutSec;
  t["mode"] = static_cast<uint8_t>(config_.temp.mode);
}

bool ConfigService::validate(char* error, size_t errorLen) const {
  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    if (!validateDoser(i, error, errorLen)) return false;
  }

  if (!validateRelay(config_.co2, error, errorLen)) return false;
  if (!validateRelay(config_.o2, error, errorLen)) return false;
  if (!validateRelay(config_.filter, error, errorLen)) return false;
  if (!validateRelay(config_.light, error, errorLen)) return false;
  if (!validateArgb(error, errorLen)) return false;
  if (!validateTemp(error, errorLen)) return false;

  return true;
}

bool ConfigService::validateDoser(uint8_t index, char* error, size_t errorLen) const {
  if (index >= HardwareConfig::DOSER_COUNT) {
    snprintf(error, errorLen, "bad doser index");
    return false;
  }

  const DoserConfig& d = config_.doser[index];

  if (d.mode == Mode::Auto && d.rateMlPerSecX10 == 0) {
    snprintf(error, errorLen, "doser.%u rate must be > 0", index);
    return false;
  }

  if (d.currentVolumeMlX10 > d.maxVolumeMlX10 && d.maxVolumeMlX10 > 0) {
    snprintf(error, errorLen, "doser.%u volume > maxVolume", index);
    return false;
  }

  return true;
}

bool ConfigService::validateRelay(const ScheduleRelayConfig&, char*, size_t) const {
  return true;
}

bool ConfigService::validateArgb(char* error, size_t errorLen) const {
  if (config_.argb.brightness > 255) {
    snprintf(error, errorLen, "argb brightness invalid");
    return false;
  }
  return true;
}

bool ConfigService::validateTemp(char* error, size_t errorLen) const {
  if (config_.temp.hysteresis < 0.1f || config_.temp.hysteresis > 10.0f) {
    snprintf(error, errorLen, "temp hysteresis out of range");
    return false;
  }
  if (config_.temp.setting < 0.0f || config_.temp.setting > 40.0f) {
    snprintf(error, errorLen, "temp setting out of range");
    return false;
  }
  return true;
}

ScheduleRelayConfig* ConfigService::relayById(const char* id) {
  if (!id) return nullptr;
  if (strcmp(id, "co2") == 0) return &config_.co2;
  if (strcmp(id, "o2") == 0) return &config_.o2;
  if (strcmp(id, "filter") == 0) return &config_.filter;
  if (strcmp(id, "light") == 0) return &config_.light;
  return nullptr;
}

const ScheduleRelayConfig* ConfigService::relayById(const char* id) const {
  if (!id) return nullptr;
  if (strcmp(id, "co2") == 0) return &config_.co2;
  if (strcmp(id, "o2") == 0) return &config_.o2;
  if (strcmp(id, "filter") == 0) return &config_.filter;
  if (strcmp(id, "light") == 0) return &config_.light;
  return nullptr;
}

uint8_t ConfigService::parsePeriod(JsonObject p) const {
  if (p.isNull()) return 0x7F;

  uint8_t mask = 0;
  for (uint8_t i = 0; i < 7; ++i) {
    if (p[dayKeys[i]] | false) mask |= (1u << i);
  }
  return mask;
}

void ConfigService::writePeriod(JsonObject p, uint8_t mask) const {
  for (uint8_t i = 0; i < 7; ++i) {
    p[dayKeys[i]] = (mask & (1u << i)) != 0;
  }
}


bool ConfigService::emergencySnapshotExists() const {
  return SPIFFS.exists(AQUARIUM_STATE_EMERGENCY_PATH);
}

bool ConfigService::hasPreEmergencySnapshot() const {
  return emergencySnapshotExists();
}

void ConfigService::writeEmergencySnapshot(JsonDocument& doc) const {
  JsonObject root = doc.to<JsonObject>();
  root["active"] = true;

  JsonArray doser = root["doser"].to<JsonArray>();
  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    doser.add(static_cast<uint8_t>(config_.doser[i].mode));
  }

  JsonObject relays = root["relays"].to<JsonObject>();
  relays["co2"] = static_cast<uint8_t>(config_.co2.mode);
  relays["o2"] = static_cast<uint8_t>(config_.o2.mode);
  relays["filter"] = static_cast<uint8_t>(config_.filter.mode);
  relays["light"] = static_cast<uint8_t>(config_.light.mode);

  root["tempMode"] = static_cast<uint8_t>(config_.temp.mode);

  JsonObject argb = root["argb"].to<JsonObject>();
  argb["mode"] = static_cast<uint8_t>(config_.argb.mode);
  argb["style"] = config_.argb.style;
  argb["brightness"] = config_.argb.brightness;
  JsonObject color = argb["static"].to<JsonObject>();
  color["r"] = config_.argb.staticColor.r;
  color["g"] = config_.argb.staticColor.g;
  color["b"] = config_.argb.staticColor.b;
}

bool ConfigService::savePreEmergencySnapshotIfNeeded() {
  if (emergencySnapshotExists()) {
    return true;
  }

  StaticJsonDocument<768> doc;
  writeEmergencySnapshot(doc);
  return writeJsonDocumentAtomic(AQUARIUM_STATE_EMERGENCY_PATH, doc);
}

bool ConfigService::parseEmergencySnapshot(JsonDocument& doc) {
  JsonObject root = doc.as<JsonObject>();
  if (!root["active"].as<bool>()) {
    return false;
  }

  JsonArray doser = root["doser"].as<JsonArray>();
  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    if (i < doser.size()) {
      config_.doser[i].mode = static_cast<Mode>(clampValue<int>(doser[i] | static_cast<uint8_t>(config_.doser[i].mode), 0, 2));
    }
  }

  JsonObject relays = root["relays"].as<JsonObject>();
  if (!relays.isNull()) {
    config_.co2.mode = static_cast<Mode>(clampValue<int>(relays["co2"] | static_cast<uint8_t>(config_.co2.mode), 0, 2));
    config_.o2.mode = static_cast<Mode>(clampValue<int>(relays["o2"] | static_cast<uint8_t>(config_.o2.mode), 0, 2));
    config_.filter.mode = static_cast<Mode>(clampValue<int>(relays["filter"] | static_cast<uint8_t>(config_.filter.mode), 0, 2));
    config_.light.mode = static_cast<Mode>(clampValue<int>(relays["light"] | static_cast<uint8_t>(config_.light.mode), 0, 2));
  }

  config_.temp.mode = static_cast<TempMode>(clampValue<int>(root["tempMode"] | static_cast<uint8_t>(config_.temp.mode), 0, 4));

  JsonObject argb = root["argb"].as<JsonObject>();
  if (!argb.isNull()) {
    config_.argb.mode = static_cast<Mode>(clampValue<int>(argb["mode"] | static_cast<uint8_t>(config_.argb.mode), 0, 2));
    config_.argb.style = clampValue<int>(argb["style"] | config_.argb.style, 1, 4);
    config_.argb.brightness = clampValue<int>(argb["brightness"] | config_.argb.brightness, 0, 255);

    JsonObject color = argb["static"].as<JsonObject>();
    if (!color.isNull()) {
      setRgbColor(
        config_.argb.staticColor,
        clampValue<int>(color["r"] | config_.argb.staticColor.r, 0, 255),
        clampValue<int>(color["g"] | config_.argb.staticColor.g, 0, 255),
        clampValue<int>(color["b"] | config_.argb.staticColor.b, 0, 255)
      );
    }
  }

  return true;
}

bool ConfigService::restorePreEmergencySnapshot() {
  if (!emergencySnapshotExists()) {
    logger_.warn(LogCategory::System, "no emergency snapshot to restore");
    return false;
  }

  char error[128] = {0};
  StaticJsonDocument<768> doc;
  if (!loadJsonDocument(AQUARIUM_STATE_EMERGENCY_PATH, doc, error, sizeof(error))) {
    logger_.warn(LogCategory::System, error);
    return false;
  }

  AppConfig old = config_;
  if (!parseEmergencySnapshot(doc)) {
    config_ = old;
    logger_.warn(LogCategory::System, "bad emergency snapshot");
    return false;
  }

  const bool saved = saveDoser() && saveRelays() && saveTemp() && saveArgb();
  if (!saved) {
    config_ = old;
    logger_.error(LogCategory::System, "emergency snapshot restore save failed");
    return false;
  }

  clearPreEmergencySnapshot();
  logger_.info(LogCategory::System, "pre-emergency config restored");
  return true;
}

bool ConfigService::clearPreEmergencySnapshot() {
  if (SPIFFS.exists(AQUARIUM_STATE_EMERGENCY_PATH)) {
    return SPIFFS.remove(AQUARIUM_STATE_EMERGENCY_PATH);
  }
  return true;
}

bool ConfigService::markDoserRun(uint8_t index, uint16_t dosageMlX10, const DateTime& now) {
  if (index >= HardwareConfig::DOSER_COUNT) return false;

  DoserConfig& d = config_.doser[index];

  if (d.currentVolumeMlX10 >= dosageMlX10) {
    d.currentVolumeMlX10 -= dosageMlX10;
  } else {
    d.currentVolumeMlX10 = 0;
  }

  TimeUtils::ymd(now, d.lastRunYmd, sizeof(d.lastRunYmd));
  return saveDoser() && saveDoserState();
}

bool ConfigService::refillDoser(uint8_t index, uint16_t volumeMlX10) {
  if (index >= HardwareConfig::DOSER_COUNT) return false;
  DoserConfig& d = config_.doser[index];
  d.currentVolumeMlX10 = volumeMlX10 == 0 ? d.maxVolumeMlX10 : min<uint16_t>(volumeMlX10, d.maxVolumeMlX10);
  return saveDoser();
}

bool ConfigService::setRelayMode(uint8_t index, Mode mode) {
  ScheduleRelayConfig* relay = nullptr;
  switch (index) {
    case 0: relay = &config_.co2; break;
    case 1: relay = &config_.o2; break;
    case 2: relay = &config_.filter; break;
    case 3: relay = &config_.light; break;
    default: return false;
  }
  relay->mode = mode;
  return saveRelays();
}

bool ConfigService::applyEmergencyMode() {
  savePreEmergencySnapshotIfNeeded();

  // Aquarium emergency policy:
  // - dangerous outputs OFF: CO2, heat, cool, all dosers
  // - life-support ON: O2, filter and light fixture cooling fans
  // - visual alarm ON: ARGB static red
  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    config_.doser[i].mode = Mode::Off;
  }

  config_.co2.mode = Mode::Off;
  config_.o2.mode = Mode::On;
  config_.filter.mode = Mode::On;
  config_.light.mode = Mode::On;

  config_.temp.mode = TempMode::Off;

  config_.argb.mode = Mode::On;
  config_.argb.style = 1;
  config_.argb.brightness = 96;
  setRgbColor(config_.argb.staticColor, 255, 0, 0);

  // Persist only affected split-config files. System config does not contain
  // dangerous output state.
  const bool doserOk = saveDoser();
  const bool relaysOk = saveRelays();
  const bool tempOk = saveTemp();
  const bool argbOk = saveArgb();

  return doserOk && relaysOk && tempOk && argbOk;
}

bool ConfigService::forceAllOutputsOff() {
  return applyEmergencyMode();
}

