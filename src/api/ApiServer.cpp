#include "api/ApiServer.h"
#include "api/ApiResponse.h"
#include "api/ApiUtils.h"
#include "config/BuildConfig.h"
#include "secrets/secrets.h"
#include "storage/MetricStorage.h"
#include "storage/EventStorage.h"
#include "core/SdGuard.h"
#include <ArduinoJson.h>
#include <SD.h>
#include <string.h>

static bool buildNotePath(const char* uid, char* out, size_t outLen) {
  if (!uid || !*uid || !out || outLen == 0) return false;
  snprintf(out, outLen, "/notes/%s.json", uid);
  return true;
}

static bool relayIndexById(const char* id, uint8_t& out) {
  if (!id) return false;
  if (strcmp(id, "co2") == 0) { out = 0; return true; }
  if (strcmp(id, "o2") == 0) { out = 1; return true; }
  if (strcmp(id, "filter") == 0) { out = 2; return true; }
  if (strcmp(id, "light") == 0) { out = 3; return true; }
  return false;
}

void ApiServer::begin() {
  setupRoutes();

#if AQUARIUM_ENABLE_SD_STATIC_WEB
  server_.serveStatic("/", SD, "/").setDefaultFile("index.html");
#endif

  server_.onNotFound([](AsyncWebServerRequest* req) {
    if (req->method() == HTTP_OPTIONS) {
      req->send(ApiResponse::withCors(req->beginResponse(204)));
    } else {
      ApiResponse::text(req, 404, "not found");
    }
  });

  server_.begin();
  logger_.info(LogCategory::Api, "api started");
}

void ApiServer::tick() {
  // HTTP server is async; no periodic work here.
}

bool ApiServer::collectBody(
  AsyncWebServerRequest* req,
  uint8_t* data,
  size_t len,
  size_t index,
  size_t total,
  uint8_t** body,
  size_t* used,
  size_t maxBytes
) {
  if (index == 0) {
    delete[] *body;
    *body = nullptr;
    *used = 0;

    if (total == 0) {
      ApiResponse::error(req, 400, "empty body");
      return false;
    }

    if (total > maxBytes) {
      ApiResponse::error(req, 413, "body too large");
      return false;
    }

    *body = new uint8_t[total];
    if (!*body) {
      ApiResponse::error(req, 500, "oom");
      return false;
    }
  }

  if (!*body || index + len > total) {
    delete[] *body;
    *body = nullptr;
    *used = 0;
    ApiResponse::error(req, 400, "bad upload");
    return false;
  }

  memcpy(*body + index, data, len);
  *used = index + len;
  return *used == total;
}

void ApiServer::setupRoutes() {
  server_.on("/api/current", HTTP_GET, [this](AsyncWebServerRequest* r) { handleCurrent(r); });
  server_.on("/api/health", HTTP_GET, [this](AsyncWebServerRequest* r) { handleHealth(r); });

  server_.on("/api/system", HTTP_GET, [this](AsyncWebServerRequest* r) { handleSystemGet(r); });
  server_.on("/api/system", HTTP_PUT, [](AsyncWebServerRequest*) {}, nullptr,
             [this](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t i, size_t t) { handleSystemPut(r, d, l, i, t); });
  server_.on("/api/system", HTTP_PATCH, [](AsyncWebServerRequest*) {}, nullptr,
             [this](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t i, size_t t) { handleSystemPut(r, d, l, i, t); });

  // Register specific doser-state routes BEFORE generic /api/doser routes.
  // ESPAsyncWebServer may match the first compatible prefix route, so /api/doser
  // must not be registered before /api/doser/state.
  server_.on("/api/doser/state", HTTP_GET, [this](AsyncWebServerRequest* r) { handleDoserStateAllGet(r); });
  server_.on("/api/doser/state/reset", HTTP_POST, [this](AsyncWebServerRequest* r) {
    if (!config_.resetAllDoserLastRuns()) {
      ApiResponse::error(r, 500, "doser state reset failed");
      return;
    }
    handleDoserStateAllGet(r);
  });

  server_.on("/api/doser/state/mark-run", HTTP_POST, [this](AsyncWebServerRequest* r) {
    if (!time_.valid()) {
      ApiResponse::error(r, 503, "rtc unavailable");
      return;
    }

    if (!config_.markAllDoserLastRunsToday(time_.now())) {
      ApiResponse::error(r, 500, "doser state mark failed");
      return;
    }
    handleDoserStateAllGet(r);
  });

  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    char path[20];
    snprintf(path, sizeof(path), "/api/doser/%u", i);
    const uint8_t index = i;

    char statePath[32];
    snprintf(statePath, sizeof(statePath), "/api/doser/%u/state", i);
    server_.on(statePath, HTTP_GET, [this, index](AsyncWebServerRequest* r) { handleDoserStateGet(r, index); });

    char stateResetPath[40];
    snprintf(stateResetPath, sizeof(stateResetPath), "/api/doser/%u/state/reset", i);
    server_.on(stateResetPath, HTTP_POST, [this, index](AsyncWebServerRequest* r) { handleDoserStateReset(r, index); });

    char stateMarkPath[44];
    snprintf(stateMarkPath, sizeof(stateMarkPath), "/api/doser/%u/state/mark-run", i);
    server_.on(stateMarkPath, HTTP_POST, [this, index](AsyncWebServerRequest* r) { handleDoserStateMarkRun(r, index); });

    char lastRunMarkPath[48];
    snprintf(lastRunMarkPath, sizeof(lastRunMarkPath), "/api/doser/%u/last-run/mark", i);
    server_.on(lastRunMarkPath, HTTP_POST, [this, index](AsyncWebServerRequest* r) { handleDoserStateMarkRun(r, index); });

    char lastRunResetPath[44];
    snprintf(lastRunResetPath, sizeof(lastRunResetPath), "/api/doser/%u/last-run/reset", i);
    server_.on(lastRunResetPath, HTTP_POST, [this, index](AsyncWebServerRequest* r) { handleDoserStateReset(r, index); });

    server_.on(path, HTTP_GET, [this, index](AsyncWebServerRequest* r) { handleDoserGet(r, index); });
    server_.on(path, HTTP_PUT, [](AsyncWebServerRequest*) {}, nullptr,
               [this, index](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t i, size_t t) { handleDoserPut(r, index, d, l, i, t); });
    server_.on(path, HTTP_PATCH, [](AsyncWebServerRequest*) {}, nullptr,
               [this, index](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t i, size_t t) { handleDoserPut(r, index, d, l, i, t); });

    char runPath[28];
    snprintf(runPath, sizeof(runPath), "/api/doser/%u/run", i);
    server_.on(runPath, HTTP_POST, [this, index](AsyncWebServerRequest* r) { handleDoserCommand(r, index, CommandType::DoserRun); });

    char stopPath[28];
    snprintf(stopPath, sizeof(stopPath), "/api/doser/%u/stop", i);
    server_.on(stopPath, HTTP_POST, [this, index](AsyncWebServerRequest* r) { handleDoserCommand(r, index, CommandType::DoserStop); });

    char refillPath[32];
    snprintf(refillPath, sizeof(refillPath), "/api/doser/%u/refill", i);
    server_.on(refillPath, HTTP_POST, [this, index](AsyncWebServerRequest* r) { handleDoserCommand(r, index, CommandType::DoserRefill); });
  }

  server_.on("/api/doser", HTTP_GET, [this](AsyncWebServerRequest* r) { handleDoserAllGet(r); });

  const char* relayIds[] = {"co2", "o2", "filter", "light"};
  for (const char* id : relayIds) {
    char path[32];
    snprintf(path, sizeof(path), "/api/relays/%s", id);

    server_.on(path, HTTP_GET, [this, id](AsyncWebServerRequest* r) { handleRelayGet(r, id); });
    server_.on(path, HTTP_PUT, [](AsyncWebServerRequest*) {}, nullptr,
               [this, id](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t i, size_t t) { handleRelayPut(r, id, d, l, i, t); });
    server_.on(path, HTTP_PATCH, [](AsyncWebServerRequest*) {}, nullptr,
               [this, id](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t i, size_t t) { handleRelayPut(r, id, d, l, i, t); });

    char onPath[40];
    snprintf(onPath, sizeof(onPath), "/api/relays/%s/on", id);
    server_.on(onPath, HTTP_POST, [this, id](AsyncWebServerRequest* r) { handleRelayCommand(r, id, CommandType::RelayOn); });

    char offPath[40];
    snprintf(offPath, sizeof(offPath), "/api/relays/%s/off", id);
    server_.on(offPath, HTTP_POST, [this, id](AsyncWebServerRequest* r) { handleRelayCommand(r, id, CommandType::RelayOff); });
  }

  server_.on("/api/relays", HTTP_GET, [this](AsyncWebServerRequest* r) { handleRelayAllGet(r); });

  server_.on("/api/argb", HTTP_GET, [this](AsyncWebServerRequest* r) { handleArgbGet(r); });
  server_.on("/api/argb", HTTP_PUT, [](AsyncWebServerRequest*) {}, nullptr,
             [this](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t i, size_t t) { handleArgbPut(r, d, l, i, t); });
  server_.on("/api/argb", HTTP_PATCH, [](AsyncWebServerRequest*) {}, nullptr,
             [this](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t i, size_t t) { handleArgbPut(r, d, l, i, t); });

  server_.on("/api/temp", HTTP_GET, [this](AsyncWebServerRequest* r) { handleTempGet(r); });
  server_.on("/api/temp", HTTP_PUT, [](AsyncWebServerRequest*) {}, nullptr,
             [this](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t i, size_t t) { handleTempPut(r, d, l, i, t); });
  server_.on("/api/temp", HTTP_PATCH, [](AsyncWebServerRequest*) {}, nullptr,
             [this](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t i, size_t t) { handleTempPut(r, d, l, i, t); });

  // Full-config import/export stays only as service/debug endpoint. Frontend should not use it for normal edits.
  server_.on("/api/config/export", HTTP_GET, [this](AsyncWebServerRequest* r) { handleConfigCompatGet(r); });
  server_.on("/api/config/import", HTTP_POST, [](AsyncWebServerRequest*) {}, nullptr,
             [this](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t i, size_t t) { handleConfigCompatPost(r, d, l, i, t); });

  server_.on("/api/time", HTTP_POST, [](AsyncWebServerRequest*) {}, nullptr,
             [this](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t i, size_t t) { handleTimePost(r, d, l, i, t); });

  server_.on("/api/metrics", HTTP_GET, [this](AsyncWebServerRequest* r) { handleMetrics(r); });
  server_.on("/api/events", HTTP_GET, [this](AsyncWebServerRequest* r) { handleEvents(r); });
  server_.on("/api/logs", HTTP_GET, [this](AsyncWebServerRequest* r) { handleLogs(r); });

  // Both endpoints intentionally do the same thing.
  // /emergency-mode is the semantically correct name: it keeps O2/filter ON,
  // disables dangerous outputs and turns ARGB red. /emergency-off is kept for
  // backwards compatibility with the earlier test scripts.
  server_.on("/api/safety", HTTP_GET, [this](AsyncWebServerRequest* r) { handleSafetyGet(r); });
  // Register longer/specific safety routes first. ESPAsyncWebServer may match
  // prefix-compatible routes, so /api/safety/emergency-mode must come after
  // /api/safety/emergency-mode/clear.
  server_.on("/api/safety/emergency-mode/clear", HTTP_POST, [this](AsyncWebServerRequest* r) { handleEmergencyClear(r); });
  server_.on("/api/safety/clear-emergency-mode", HTTP_POST, [this](AsyncWebServerRequest* r) { handleEmergencyClear(r); });
  server_.on("/api/safety/emergency-override/clear", HTTP_POST, [this](AsyncWebServerRequest* r) { handleEmergencyOverrideClear(r); });
  server_.on("/api/safety/override/clear", HTTP_POST, [this](AsyncWebServerRequest* r) { handleEmergencyOverrideClear(r); });
  server_.on("/api/safety/emergency-mode", HTTP_POST, [this](AsyncWebServerRequest* r) { handleEmergencyOff(r); });
  server_.on("/api/safety/emergency-off", HTTP_POST, [this](AsyncWebServerRequest* r) { handleEmergencyOff(r); });

  server_.on("/api/note", HTTP_GET, [this](AsyncWebServerRequest* r) { handleNoteGet(r); });
  server_.on("/api/note", HTTP_DELETE, [this](AsyncWebServerRequest* r) { handleNoteDelete(r); });
  server_.on("/api/note", HTTP_POST, [](AsyncWebServerRequest*) {}, nullptr,
             [this](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t i, size_t t) { handleNoteUpsert(r, d, l, i, t); });
  server_.on("/api/note", HTTP_PUT, [](AsyncWebServerRequest*) {}, nullptr,
             [this](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t i, size_t t) { handleNoteUpsert(r, d, l, i, t); });
  server_.on("/api/notes/paged", HTTP_GET, [this](AsyncWebServerRequest* r) { handleNotesPaged(r); });
}

void ApiServer::handleCurrent(AsyncWebServerRequest* req) {
  DateTime now = time_.now();
  StaticJsonDocument<1536> doc;

  JsonObject system = doc["system"].to<JsonObject>();
  system["time"]["year"] = now.year();
  system["time"]["month"] = now.month();
  system["time"]["day"] = now.day();
  system["time"]["dayOfWeek"] = now.dayOfTheWeek();
  system["time"]["hour"] = now.hour();
  system["time"]["minute"] = now.minute();
  system["time"]["second"] = now.second();
  system["fan"] = state_.fanRpm;
  system["chipTemp"] = temperatureRead();
  system["outside"]["temp"] = state_.airTemp;
  system["outside"]["hum"] = state_.humidity;
  system["uptime"] = esp_timer_get_time();

  const uint64_t totalSpace = storage_.totalBytes();
  const uint64_t usedSpace = storage_.usedBytes();
  system["totalSpace"] = totalSpace;
  system["usedSpace"] = usedSpace;
  system["freeSpace"] = totalSpace > usedSpace ? totalSpace - usedSpace : 0;
  system["freeHeap"] = ESP.getFreeHeap();
  system["minFreeHeap"] = ESP.getMinFreeHeap();
  system["heapSize"] = ESP.getHeapSize();
  doc["safety"]["emergencyMode"] = state_.emergencyMode;
  doc["safety"]["emergencyOverride"] = state_.emergencyOverride;
  doc["safety"]["restoreAvailable"] = config_.hasPreEmergencySnapshot();
  doc["safety"]["rtcValid"] = state_.rtcValid;

  JsonArray doser = doc["doser"].to<JsonArray>();
  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    JsonObject p = doser.add<JsonObject>();
    p["id"] = i;
    p["status"] = state_.doser[i].status != RelayStatus::Off;
    p["running"] = state_.doser[i].running;
    p["introduced"] = state_.doser[i].progress;
  }

  doc["co2"]["status"] = state_.extra[0] != RelayStatus::Off;
  doc["o2"]["status"] = state_.extra[1] != RelayStatus::Off;
  doc["filter"]["status"] = state_.extra[2] != RelayStatus::Off;
  doc["light"]["status"] = state_.extra[3] != RelayStatus::Off;
  doc["argb"]["status"] = state_.argb != RelayStatus::Off;
  doc["temp"]["status"] = static_cast<uint8_t>(state_.cool == RelayStatus::On ? 1 : (state_.heat == RelayStatus::On ? 2 : 0));
  doc["temp"]["current"] = state_.waterTemp;

  String out;
  serializeJson(doc, out);
  req->send(ApiResponse::withCors(req->beginResponse(200, "application/json", out)));
}

void ApiServer::handleHealth(AsyncWebServerRequest* req) {
  const HealthSnapshot& h = health_.snapshot();
  StaticJsonDocument<512> doc;

  doc["freeHeap"] = h.freeHeap;
  doc["minFreeHeap"] = h.minFreeHeap;
  doc["heapSize"] = h.heapSize;
  doc["largestFreeBlock"] = h.largestFreeBlock;
  doc["uptimeSec"] = h.uptimeSec;

  JsonObject status = doc["status"].to<JsonObject>();
  status["sd"] = h.sdOk;
  status["sdMounted"] = h.sdMounted;
  status["sdFailures"] = h.sdFailureCount;
  status["spiffs"] = h.spiffsOk;
  status["wifi"] = h.wifiOk;
  status["rtc"] = h.rtcOk;
  status["emergencyMode"] = h.emergencyMode;
  status["emergencyOverride"] = h.emergencyOverride;

  String out;
  serializeJson(doc, out);
  req->send(ApiResponse::withCors(req->beginResponse(200, "application/json", out)));
}

void ApiServer::handleSystemGet(AsyncWebServerRequest* req) {
  String out;
  config_.serializeSystem(out);
  req->send(ApiResponse::withCors(req->beginResponse(200, "application/json", out)));
}

void ApiServer::handleSystemPut(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
  static uint8_t* body = nullptr;
  static size_t used = 0;
  if (!collectBody(req, data, len, index, total, &body, &used, 512)) return;

  char error[128] = {0};
  const bool ok = config_.updateSystemFromJsonBytes(body, used, error, sizeof(error));
  delete[] body; body = nullptr; used = 0;

  if (!ok) { ApiResponse::error(req, 400, error); return; }
  handleSystemGet(req);
}

void ApiServer::handleDoserAllGet(AsyncWebServerRequest* req) {
  String out;
  config_.serializeDoserAll(out);
  req->send(ApiResponse::withCors(req->beginResponse(200, "application/json", out)));
}

void ApiServer::handleDoserGet(AsyncWebServerRequest* req, uint8_t doserIndex) {
  String out;
  if (!config_.serializeDoser(doserIndex, out)) {
    ApiResponse::error(req, 404, "doser not found");
    return;
  }
  req->send(ApiResponse::withCors(req->beginResponse(200, "application/json", out)));
}



void ApiServer::handleDoserStateAllGet(AsyncWebServerRequest* req) {
  String out;
  config_.serializeDoserStateAll(out);
  req->send(ApiResponse::withCors(req->beginResponse(200, "application/json", out)));
}

void ApiServer::handleDoserStateGet(AsyncWebServerRequest* req, uint8_t doserIndex) {
  String out;
  if (!config_.serializeDoserState(doserIndex, out)) {
    ApiResponse::error(req, 404, "doser state not found");
    return;
  }
  req->send(ApiResponse::withCors(req->beginResponse(200, "application/json", out)));
}

void ApiServer::handleDoserStateReset(AsyncWebServerRequest* req, uint8_t doserIndex) {
  if (!config_.resetDoserLastRun(doserIndex)) {
    ApiResponse::error(req, 500, "doser state reset failed");
    return;
  }

  handleDoserStateGet(req, doserIndex);
}

void ApiServer::handleDoserStateMarkRun(AsyncWebServerRequest* req, uint8_t doserIndex) {
  if (!time_.valid()) {
    ApiResponse::error(req, 503, "rtc unavailable");
    return;
  }

  if (!config_.markDoserLastRunToday(doserIndex, time_.now())) {
    ApiResponse::error(req, 500, "doser state mark failed");
    return;
  }

  handleDoserStateGet(req, doserIndex);
}

void ApiServer::handleDoserPut(AsyncWebServerRequest* req, uint8_t doserIndex, uint8_t* data, size_t len, size_t index, size_t total) {
  static uint8_t* body[HardwareConfig::DOSER_COUNT] = {nullptr};
  static size_t used[HardwareConfig::DOSER_COUNT] = {0};

  if (doserIndex >= HardwareConfig::DOSER_COUNT) {
    ApiResponse::error(req, 404, "doser not found");
    return;
  }

  if (!collectBody(req, data, len, index, total, &body[doserIndex], &used[doserIndex], 1024)) return;

  char error[128] = {0};
  const bool ok = config_.updateDoserFromJsonBytes(doserIndex, body[doserIndex], used[doserIndex], error, sizeof(error));
  delete[] body[doserIndex]; body[doserIndex] = nullptr; used[doserIndex] = 0;

  if (!ok) { ApiResponse::error(req, 400, error); return; }
  handleDoserGet(req, doserIndex);
}

void ApiServer::handleDoserCommand(AsyncWebServerRequest* req, uint8_t doserIndex, CommandType type) {
  if (doserIndex >= HardwareConfig::DOSER_COUNT) {
    ApiResponse::error(req, 404, "doser not found");
    return;
  }

  Command cmd;
  cmd.type = type;
  cmd.target = doserIndex;

  if (type == CommandType::DoserRefill && req->hasParam("volume")) {
    const float ml = req->getParam("volume")->value().toFloat();
    cmd.value = ml <= 0 ? 0 : static_cast<uint32_t>(ml * 10.0f + 0.5f);
  }

  if (!commands_.send(cmd, pdMS_TO_TICKS(50))) {
    ApiResponse::error(req, 503, "command queue full");
    return;
  }

  ApiResponse::json(req, 202, "{\"status\":\"accepted\"}");
}

void ApiServer::handleRelayAllGet(AsyncWebServerRequest* req) {
  String out;
  if (!config_.serializeRelayAll(out)) {
    ApiResponse::error(req, 500, "relays serialize failed");
    return;
  }
  req->send(ApiResponse::withCors(req->beginResponse(200, "application/json", out)));
}

void ApiServer::handleRelayGet(AsyncWebServerRequest* req, const char* relayId) {
  String out;
  if (!config_.serializeRelay(relayId, out)) {
    ApiResponse::error(req, 404, "relay not found");
    return;
  }
  req->send(ApiResponse::withCors(req->beginResponse(200, "application/json", out)));
}

void ApiServer::handleRelayPut(AsyncWebServerRequest* req, const char* relayId, uint8_t* data, size_t len, size_t index, size_t total) {
  static uint8_t* body = nullptr;
  static size_t used = 0;

  if (!collectBody(req, data, len, index, total, &body, &used, 512)) return;

  char error[128] = {0};
  const bool ok = config_.updateRelayFromJsonBytes(relayId, body, used, error, sizeof(error));
  delete[] body; body = nullptr; used = 0;

  if (!ok) { ApiResponse::error(req, 400, error); return; }
  handleRelayGet(req, relayId);
}

void ApiServer::handleRelayCommand(AsyncWebServerRequest* req, const char* relayId, CommandType type) {
  uint8_t index = 0;
  if (!relayIndexById(relayId, index)) {
    ApiResponse::error(req, 404, "relay not found");
    return;
  }

  Command cmd;
  cmd.type = type;
  cmd.target = index;

  if (!commands_.send(cmd, pdMS_TO_TICKS(50))) {
    ApiResponse::error(req, 503, "command queue full");
    return;
  }

  ApiResponse::json(req, 202, "{\"status\":\"accepted\"}");
}

void ApiServer::handleArgbGet(AsyncWebServerRequest* req) {
  String out;
  config_.serializeArgb(out);
  req->send(ApiResponse::withCors(req->beginResponse(200, "application/json", out)));
}

void ApiServer::handleArgbPut(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
  static uint8_t* body = nullptr;
  static size_t used = 0;

  if (!collectBody(req, data, len, index, total, &body, &used, 4096)) return;

  char error[128] = {0};
  const bool ok = config_.updateArgbFromJsonBytes(body, used, error, sizeof(error));
  delete[] body; body = nullptr; used = 0;

  if (!ok) { ApiResponse::error(req, 400, error); return; }
  handleArgbGet(req);
}

void ApiServer::handleTempGet(AsyncWebServerRequest* req) {
  String out;
  config_.serializeTemp(out);
  req->send(ApiResponse::withCors(req->beginResponse(200, "application/json", out)));
}

void ApiServer::handleTempPut(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
  static uint8_t* body = nullptr;
  static size_t used = 0;

  if (!collectBody(req, data, len, index, total, &body, &used, 512)) return;

  char error[128] = {0};
  const bool ok = config_.updateTempFromJsonBytes(body, used, error, sizeof(error));
  delete[] body; body = nullptr; used = 0;

  if (!ok) { ApiResponse::error(req, 400, error); return; }
  handleTempGet(req);
}

void ApiServer::handleConfigCompatGet(AsyncWebServerRequest* req) {
  String out;
  config_.serializeToJson(out);
  req->send(ApiResponse::withCors(req->beginResponse(200, "application/json", out)));
}

void ApiServer::handleConfigCompatPost(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
  static uint8_t* body = nullptr;
  static size_t used = 0;

  if (!collectBody(req, data, len, index, total, &body, &used, AQUARIUM_MAX_CONFIG_JSON_BYTES)) return;

  char error[128] = {0};
  const bool ok = config_.updateFromJsonBytes(body, used, error, sizeof(error));
  delete[] body; body = nullptr; used = 0;

  if (!ok) { ApiResponse::error(req, 400, error); return; }
  ApiResponse::json(req, 200, "{\"status\":\"imported\"}");
}

void ApiServer::handleTimePost(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
  static uint8_t* body = nullptr;
  static size_t used = 0;

  if (!collectBody(req, data, len, index, total, &body, &used, 256)) return;

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, body, used);
  delete[] body; body = nullptr; used = 0;

  if (err) { ApiResponse::error(req, 400, "bad json"); return; }

  DateTime dt(
    doc["year"] | 2000,
    doc["month"] | 1,
    doc["day"] | 1,
    doc["hour"] | 0,
    doc["minute"] | 0,
    doc["second"] | 0
  );

  if (!time_.adjust(dt)) {
    ApiResponse::error(req, 500, "rtc unavailable");
    return;
  }

  ApiResponse::json(req, 200, "{\"status\":\"ok\"}");
}

void ApiServer::handleMetrics(AsyncWebServerRequest* req) {
  if (!req->hasParam("metric") || !req->hasParam("year") || !req->hasParam("month") || !req->hasParam("day")) {
    ApiResponse::error(req, 400, "missing params");
    return;
  }

  String metric = req->getParam("metric")->value();
  if (!ApiUtils::safeId(metric)) { ApiResponse::error(req, 400, "bad metric"); return; }

  int y = req->getParam("year")->value().toInt();
  int m = req->getParam("month")->value().toInt();
  int d = req->getParam("day")->value().toInt();

  struct CsvContext {
    char path[96];
    size_t offset = 0;
    bool headerSent = false;
    bool done = false;
  };

  auto* ctx = new CsvContext();
  if (!ctx) { ApiResponse::error(req, 500, "oom"); return; }

  if (!MetricStorage::buildPath(metric.c_str(), y, m, d, ctx->path, sizeof(ctx->path))) {
    delete ctx;
    ApiResponse::error(req, 400, "bad date");
    return;
  }

  if (!storage_.exists(ctx->path)) {
    delete ctx;
    ApiResponse::text(req, 404, "not found");
    return;
  }

  AsyncWebServerResponse* response = req->beginChunkedResponse(
    "text/csv",
    [this, ctx](uint8_t* buffer, size_t maxLen, size_t) mutable -> size_t {
      if (ctx->done) {
        delete ctx;
        return 0;
      }

      size_t written = 0;
      if (!ctx->headerSent) {
        const char* header = "ts,value\n";
        const size_t n = strlen(header);
        if (n > maxLen) return 0;
        memcpy(buffer, header, n);
        written += n;
        ctx->headerSent = true;
      }

      SdGuard guard(250);
      if (!guard.locked()) {
        ctx->done = true;
        return written;
      }

      File file = storage_.openReadLocked(ctx->path, guard);
      if (!file) {
        ctx->done = true;
        return written;
      }

      file.seek(ctx->offset);
      MetricRecord rec;
      char line[48];

      while (written + sizeof(line) < maxLen && file.read(reinterpret_cast<uint8_t*>(&rec), sizeof(rec)) == sizeof(rec)) {
        ctx->offset += sizeof(rec);
        const int n = snprintf(line, sizeof(line), "%u,%.2f\n", rec.ts, rec.value / 100.0f);
        if (n <= 0 || written + static_cast<size_t>(n) > maxLen) break;
        memcpy(buffer + written, line, n);
        written += n;
      }

      if (!file.available()) ctx->done = true;
      file.close();
      return written;
    }
  );

  req->send(ApiResponse::withCors(response));
}

void ApiServer::handleEvents(AsyncWebServerRequest* req) {
  if (!req->hasParam("type") || !req->hasParam("year") || !req->hasParam("month")) {
    ApiResponse::error(req, 400, "missing params");
    return;
  }

  String type = req->getParam("type")->value();
  if (!ApiUtils::safeId(type)) { ApiResponse::error(req, 400, "bad type"); return; }

  int y = req->getParam("year")->value().toInt();
  int m = req->getParam("month")->value().toInt();

  struct CsvContext {
    char path[96];
    size_t offset = 0;
    bool headerSent = false;
    bool done = false;
  };

  auto* ctx = new CsvContext();
  if (!ctx) { ApiResponse::error(req, 500, "oom"); return; }

  if (!EventStorage::buildPath(type.c_str(), y, m, ctx->path, sizeof(ctx->path))) {
    delete ctx;
    ApiResponse::error(req, 400, "bad date");
    return;
  }

  if (!storage_.exists(ctx->path)) {
    delete ctx;
    ApiResponse::text(req, 404, "not found");
    return;
  }

  AsyncWebServerResponse* response = req->beginChunkedResponse(
    "text/csv",
    [this, ctx](uint8_t* buffer, size_t maxLen, size_t) mutable -> size_t {
      if (ctx->done) {
        delete ctx;
        return 0;
      }

      size_t written = 0;
      if (!ctx->headerSent) {
        const char* header = "ts,subtype,value\n";
        const size_t n = strlen(header);
        if (n > maxLen) return 0;
        memcpy(buffer, header, n);
        written += n;
        ctx->headerSent = true;
      }

      SdGuard guard(250);
      if (!guard.locked()) {
        ctx->done = true;
        return written;
      }

      File file = storage_.openReadLocked(ctx->path, guard);
      if (!file) {
        ctx->done = true;
        return written;
      }

      file.seek(ctx->offset);
      EventRecord rec;
      char line[64];

      while (written + sizeof(line) < maxLen && file.read(reinterpret_cast<uint8_t*>(&rec), sizeof(rec)) == sizeof(rec)) {
        ctx->offset += sizeof(rec);
        const int n = snprintf(line, sizeof(line), "%u,%u,%.2f\n", rec.ts, rec.subtype, rec.value / 100.0f);
        if (n <= 0 || written + static_cast<size_t>(n) > maxLen) break;
        memcpy(buffer + written, line, n);
        written += n;
      }

      if (!file.available()) ctx->done = true;
      file.close();
      return written;
    }
  );

  req->send(ApiResponse::withCors(response));
}

void ApiServer::handleLogs(AsyncWebServerRequest* req) {
  if (!req->hasParam("type") || !req->hasParam("year") || !req->hasParam("month") || !req->hasParam("day")) {
    ApiResponse::error(req, 400, "missing params");
    return;
  }

  String type = req->getParam("type")->value();
  if (!ApiUtils::safeId(type)) { ApiResponse::error(req, 400, "bad type"); return; }

  int y = req->getParam("year")->value().toInt();
  int m = req->getParam("month")->value().toInt();
  int d = req->getParam("day")->value().toInt();

  char path[96];
  snprintf(path, sizeof(path), "/logs/%04d/%02d/%02d/%s.log", y, m, d, type.c_str());

  AsyncWebServerResponse* response = storage_.beginFileResponse(req, path, "text/plain");
  if (!response) { ApiResponse::text(req, 404, "not found"); return; }
  req->send(ApiResponse::withCors(response));
}

void ApiServer::handleSafetyGet(AsyncWebServerRequest* req) {
  StaticJsonDocument<384> doc;

  doc["emergencyMode"] = state_.emergencyMode;
  doc["emergencyOverride"] = state_.emergencyOverride;
  doc["restoreAvailable"] = config_.hasPreEmergencySnapshot();
  doc["rtcValid"] = state_.rtcValid;

  JsonArray activeReasons = doc["activeReasons"].to<JsonArray>();
  if (!state_.rtcValid) {
    activeReasons.add("rtc invalid");
  }

  String out;
  serializeJson(doc, out);
  req->send(ApiResponse::withCors(req->beginResponse(200, "application/json", out)));
}

void ApiServer::handleEmergencyOff(AsyncWebServerRequest* req) {
  Command cmd;
  cmd.type = CommandType::EmergencyOff;
  if (!commands_.send(cmd, pdMS_TO_TICKS(50))) {
    ApiResponse::error(req, 503, "command queue full");
    return;
  }
  ApiResponse::json(req, 202, "{\"status\":\"accepted\"}");
}

void ApiServer::handleEmergencyClear(AsyncWebServerRequest* req) {
  Command cmd;
  cmd.type = CommandType::EmergencyClear;
  if (!commands_.send(cmd, pdMS_TO_TICKS(50))) {
    ApiResponse::error(req, 503, "command queue full");
    return;
  }
  ApiResponse::json(req, 202, "{\"status\":\"accepted\"}");
}

void ApiServer::handleEmergencyOverrideClear(AsyncWebServerRequest* req) {
  Command cmd;
  cmd.type = CommandType::EmergencyOverrideClear;
  if (!commands_.send(cmd, pdMS_TO_TICKS(50))) {
    ApiResponse::error(req, 503, "command queue full");
    return;
  }
  ApiResponse::json(req, 202, "{\"status\":\"accepted\"}");
}

void ApiServer::handleNoteGet(AsyncWebServerRequest* req) {
  if (!req->hasParam("uid")) { ApiResponse::error(req, 400, "missing uid"); return; }

  String uid = req->getParam("uid")->value();
  if (!ApiUtils::safeId(uid, 48)) { ApiResponse::error(req, 400, "bad uid"); return; }

  char path[96];
  buildNotePath(uid.c_str(), path, sizeof(path));

  AsyncWebServerResponse* response = storage_.beginFileResponse(req, path, "application/json");
  if (!response) { ApiResponse::error(req, 404, "not found"); return; }
  req->send(ApiResponse::withCors(response));
}

void ApiServer::handleNoteDelete(AsyncWebServerRequest* req) {
  if (!req->hasParam("uid")) { ApiResponse::error(req, 400, "missing uid"); return; }

  String uid = req->getParam("uid")->value();
  if (!ApiUtils::safeId(uid, 48)) { ApiResponse::error(req, 400, "bad uid"); return; }

  char path[96];
  buildNotePath(uid.c_str(), path, sizeof(path));

  if (!storage_.exists(path)) { ApiResponse::error(req, 404, "not found"); return; }
  if (!storage_.removeFile(path)) { ApiResponse::error(req, 500, "delete failed"); return; }

  ApiResponse::json(req, 200, "{\"status\":\"deleted\"}");
}

void ApiServer::handleNoteUpsert(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
  static uint8_t* body = nullptr;
  static size_t used = 0;

  if (!collectBody(req, data, len, index, total, &body, &used, AQUARIUM_MAX_NOTE_JSON_BYTES)) return;

  StaticJsonDocument<AQUARIUM_MAX_NOTE_JSON_BYTES> doc;
  DeserializationError err = deserializeJson(doc, body, used);
  delete[] body; body = nullptr; used = 0;

  if (err) { ApiResponse::error(req, 400, "bad json"); return; }

  const char* uid = doc["uid"] | "";
  String uidStr(uid);
  if (!ApiUtils::safeId(uidStr, 48)) { ApiResponse::error(req, 400, "bad uid"); return; }

  char path[96];
  buildNotePath(uid, path, sizeof(path));

  uint8_t buffer[AQUARIUM_MAX_NOTE_JSON_BYTES];
  const size_t written = serializeJson(doc, buffer, sizeof(buffer));
  if (written == 0 || written >= sizeof(buffer)) {
    ApiResponse::error(req, 500, "serialize failed");
    return;
  }

  if (!storage_.writeFileAtomic(path, buffer, written)) {
    ApiResponse::error(req, 500, "write failed");
    return;
  }

  ApiResponse::json(req, 200, "{\"status\":\"ok\"}");
}

void ApiServer::handleNotesPaged(AsyncWebServerRequest* req) {
  int offset = ApiUtils::boundedInt(req, "offset", 0, 0, 100000);
  int limit = ApiUtils::boundedInt(req, "limit", 5, 1, 100);

  if (!storage_.available()) {
    ApiResponse::error(req, 503, "sd unavailable");
    return;
  }

  SdGuard guard(1000);
  if (!guard.locked()) {
    storage_.markFailure("sd busy: notes paged");
    ApiResponse::error(req, 503, "sd busy");
    return;
  }

  if (!storage_.ensureDirRecursive("/notes")) {
    storage_.markFailure("notes dir create failed");
    ApiResponse::error(req, 500, "notes storage failed");
    return;
  }

  File dir = SD.open("/notes");
  if (!dir) {
    storage_.markFailure("notes dir open failed");
    ApiResponse::error(req, 500, "notes storage failed");
    return;
  }

  AsyncResponseStream* res = req->beginResponseStream("application/json");
  res->print("[");

  int i = 0;
  int sent = 0;
  bool first = true;
  File f;

  while ((f = dir.openNextFile())) {
    if (i++ < offset) { f.close(); continue; }
    if (sent >= limit) { f.close(); break; }

    if (!first) res->print(",");

    while (f.available()) {
      uint8_t buf[256];
      size_t n = f.read(buf, sizeof(buf));
      res->write(buf, n);
    }

    f.close();
    sent++;
    first = false;
  }

  dir.close();

  res->print("]");
  req->send(ApiResponse::withCors(res));
}
