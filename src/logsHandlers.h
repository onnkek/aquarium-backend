#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

void handleGetSystemLogs(AsyncWebServerRequest *request);
void handleGetRelayLogs(AsyncWebServerRequest *request);
void handleGetDoserLogs(AsyncWebServerRequest *request);
void handleGetClearSystemLogs(AsyncWebServerRequest *request);
void handleGetClearRelayLogs(AsyncWebServerRequest *request);
void handleGetClearDoserLogs(AsyncWebServerRequest *request);


String readLOG(String path);
