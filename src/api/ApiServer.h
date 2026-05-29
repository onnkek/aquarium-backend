#pragma once
#include <ESPAsyncWebServer.h>
#include "config/BuildConfig.h"
#include <SD.h>
#include "core/ConfigService.h"
#include "core/TimeService.h"
#include "core/Logger.h"
#include "storage/StorageManager.h"
#include "core/CommandBus.h"
#include "core/HealthService.h"
#include "models/AppTypes.h"

class ApiServer {
public:
  ApiServer(ConfigService& config, TimeService& time, StorageManager& storage, CommandBus& commands, HealthService& health, Logger& logger, RuntimeState& state)
      : server_(80),
        config_(config), time_(time), storage_(storage), commands_(commands), health_(health), logger_(logger), state_(state) {}
  void begin();
  void tick();

private:
  AsyncWebServer server_;
  ConfigService& config_;
  TimeService& time_;
  StorageManager& storage_;
  CommandBus& commands_;
  HealthService& health_;
  Logger& logger_;
  RuntimeState& state_;

  enum class ConfigTarget : uint8_t {
    System,
    Doser,
    Relay,
    Argb,
    Temp,
    FullConfigCompat,
  };

  void setupRoutes();

  void handleCurrent(AsyncWebServerRequest* req);
  void handleHealth(AsyncWebServerRequest* req);

  void handleSystemGet(AsyncWebServerRequest* req);
  void handleSystemPut(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total);

  void handleDoserAllGet(AsyncWebServerRequest* req);
  void handleDoserGet(AsyncWebServerRequest* req, uint8_t doserIndex);
  void handleDoserStateAllGet(AsyncWebServerRequest* req);
  void handleDoserStateGet(AsyncWebServerRequest* req, uint8_t doserIndex);
  void handleDoserStateReset(AsyncWebServerRequest* req, uint8_t doserIndex);
  void handleDoserStateMarkRun(AsyncWebServerRequest* req, uint8_t doserIndex);
  void handleDoserPut(AsyncWebServerRequest* req, uint8_t doserIndex, uint8_t* data, size_t len, size_t index, size_t total);
  void handleDoserCommand(AsyncWebServerRequest* req, uint8_t doserIndex, CommandType type);

  void handleRelayAllGet(AsyncWebServerRequest* req);
  void handleRelayGet(AsyncWebServerRequest* req, const char* relayId);
  void handleRelayPut(AsyncWebServerRequest* req, const char* relayId, uint8_t* data, size_t len, size_t index, size_t total);
  void handleRelayCommand(AsyncWebServerRequest* req, const char* relayId, CommandType type);

  void handleArgbGet(AsyncWebServerRequest* req);
  void handleArgbPut(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total);

  void handleTempGet(AsyncWebServerRequest* req);
  void handleTempPut(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total);

  void handleConfigCompatGet(AsyncWebServerRequest* req);
  void handleConfigCompatPost(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total);

  void handleTimePost(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total);
  void handleMetrics(AsyncWebServerRequest* req);
  void handleEvents(AsyncWebServerRequest* req);
  void handleLogs(AsyncWebServerRequest* req);
  void handleSafetyGet(AsyncWebServerRequest* req);
  void handleEmergencyOff(AsyncWebServerRequest* req);
  void handleEmergencyClear(AsyncWebServerRequest* req);
  void handleEmergencyOverrideClear(AsyncWebServerRequest* req);

  void handleNoteGet(AsyncWebServerRequest* req);
  void handleNoteDelete(AsyncWebServerRequest* req);
  void handleNoteUpsert(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total);
  void handleNotesPaged(AsyncWebServerRequest* req);

  static bool collectBody(
    AsyncWebServerRequest* req,
    uint8_t* data,
    size_t len,
    size_t index,
    size_t total,
    uint8_t** body,
    size_t* used,
    size_t maxBytes
  );
};
