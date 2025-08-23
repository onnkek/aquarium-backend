#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SD.h>
#include "logsHandlers.h"
#include "aquarium-backend.h"
#include "defs.h"

void handleGetSystemLogs(AsyncWebServerRequest *request)
{
  AsyncWebServerResponse *response = request->beginResponse(200, "application/json", readLOG(LOGS_SYSTEM));
  addCORS(response);
  request->send(response);
}
void handleGetRelayLogs(AsyncWebServerRequest *request)
{
  AsyncWebServerResponse *response = request->beginResponse(200, "application/json", readLOG(LOGS_RELAY));
  addCORS(response);
  request->send(response);
}
void handleGetDoserLogs(AsyncWebServerRequest *request)
{
  AsyncWebServerResponse *response = request->beginResponse(200, "application/json", readLOG(LOGS_DOSER));
  addCORS(response);
  request->send(response);
}

void handleGetClearSystemLogs(AsyncWebServerRequest *request)
{
  SD.remove(LOGS_SYSTEM);
  File file = SD.open(LOGS_SYSTEM, "a");
  file.close();

  AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "System log file has been cleared");
  addCORS(response);
  request->send(response);
}
void handleGetClearRelayLogs(AsyncWebServerRequest *request)
{
  SD.remove(LOGS_RELAY);
  File file = SD.open(LOGS_RELAY, "a");
  file.close();

  AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "Relay log file has been cleared");
  addCORS(response);
  request->send(response);
}
void handleGetClearDoserLogs(AsyncWebServerRequest *request)
{
  SD.remove(LOGS_DOSER);
  File file = SD.open(LOGS_DOSER, "a");
  file.close();

  AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "Doser log file has been cleared");
  addCORS(response);
  request->send(response);
}