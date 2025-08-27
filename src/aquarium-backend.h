#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "RTClib.h"
#include "systemHandlers.h"
#include "logsHandlers.h"

void parseTime(const char *timeStr, int &hour, int &minute);
void addCORS(AsyncWebServerResponse *response);
String getCurrentInfo();
void loadConfigFromSD();
void saveConfigToSD();
String readLOG(String path);
String getDateString();
String getFullDateNumber(int number);
void checkExtraRelay(int index, DateTime now);
void checkARGB(DateTime now);
void checkPumpSchedule(int index, DateTime now);
void checkTemp();
void safeAdjust(DateTime dt);
DateTime getLastTime();
void vTaskUpdateTime(void *pvParameters);
void vTaskLogger(void *pvParameters);
enum LogType
{
  INFO,
  WARNING,
  ERROR
};
void log(const String &text, LogType type, const String &path);

// void handleGetConfig(AsyncWebServerRequest *request);
// void handlePostConfig(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
// void handleGetCurrent(AsyncWebServerRequest *request);
// void handlePostTime(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);

// void handleGetSystemLogs(AsyncWebServerRequest *request);
// void handleGetRelayLogs(AsyncWebServerRequest *request);
// void handleGetDoserLogs(AsyncWebServerRequest *request);
// void handleGetClearSystemLogs(AsyncWebServerRequest *request);
// void handleGetClearRelayLogs(AsyncWebServerRequest *request);
// void handleGetClearDoserLogs(AsyncWebServerRequest *request);