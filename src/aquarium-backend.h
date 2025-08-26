#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "RTClib.h"
#include "systemHandlers.h"
#include "logsHandlers.h"

unsigned long getTimeFromString(String timeString);
String getStatus(bool status);
void parseTime(const char *timeStr, int &hour, int &minute);
void addCORS(AsyncWebServerResponse *response);
// void webServerTask(void *pvParameters);
String getCurrentInfo();
bool getRelayStatus(int status);
void loadConfigFromSD();
void saveConfigToSD();
String readLOG(String path);
void writeLOG(const String &text, int type, const String &path);
String getDateString();
String getFullDateNumber(int number);
String getContentType(const String &path);
bool stringToBool(String str);
void checkExtraRelay(int index, DateTime now);
void checkARGB(DateTime now);
void checkPumpSchedule(int index, DateTime now);
void checkTemp();
void safeAdjust(DateTime dt);

void handleGetConfig(AsyncWebServerRequest *request);
void handlePostConfig(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void handleGetCurrent(AsyncWebServerRequest *request);
void handlePostTime(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);

void handleGetSystemLogs(AsyncWebServerRequest *request);
void handleGetRelayLogs(AsyncWebServerRequest *request);
void handleGetDoserLogs(AsyncWebServerRequest *request);
void handleGetClearSystemLogs(AsyncWebServerRequest *request);
void handleGetClearRelayLogs(AsyncWebServerRequest *request);
void handleGetClearDoserLogs(AsyncWebServerRequest *request);