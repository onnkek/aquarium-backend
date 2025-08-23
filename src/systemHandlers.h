#pragma once
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "RTClib.h"

void handleGetConfig(AsyncWebServerRequest *request);
void handlePostConfig(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void handleGetCurrent(AsyncWebServerRequest *request);
void handlePostTime(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);

void addCORS(AsyncWebServerResponse *response);

extern DynamicJsonDocument config;