#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "systemHandlers.h"
#include <ArduinoJson.h>
#include "aquarium-backend.h"
#include "defs.h"

void handleGetConfig(AsyncWebServerRequest *request)
{
  String jsonConfig;
  serializeJson(config, jsonConfig);
  AsyncWebServerResponse *response = request->beginResponse(200, "application/json", jsonConfig);
  addCORS(response);
  request->send(response);
}
void handlePostConfig(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
  static String body;
  if (index == 0)
    body = "";
  body += String((char *)data, len);
  if (index + len == total)
  {

    deserializeJson(config, body);
    saveConfigToSD();
    String jsonString;
    serializeJson(config, jsonString);

    AsyncWebServerResponse *response = request->beginResponse(200, "application/json", jsonString);
    addCORS(response);
    request->send(response);
  }
}

void handleGetCurrent(AsyncWebServerRequest *request)
{
  AsyncWebServerResponse *response = request->beginResponse(200, "application/json", getCurrentInfo());
  addCORS(response);
  request->send(response);
}

void handlePostTime(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
  static String body;
  if (index == 0)
    body = "";
  body += String((char *)data, len);
  if (index + len == total)
  {
    DynamicJsonDocument newTime(64);
    deserializeJson(newTime, body);

    DateTime newDateTime = DateTime(newTime["year"], newTime["month"], newTime["day"], newTime["hour"], newTime["minute"], newTime["second"]);
    safeAdjust(newDateTime);

    String jsonString;
    serializeJson(newTime, jsonString);

    AsyncWebServerResponse *response = request->beginResponse(200, "application/json", jsonString);
    addCORS(response);
    request->send(response);
  }
}
