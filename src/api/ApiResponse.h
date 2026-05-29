#pragma once
#include <ESPAsyncWebServer.h>

namespace ApiResponse {
inline void cors(AsyncWebServerResponse* response) {
  response->addHeader("Access-Control-Allow-Origin", "*");
  response->addHeader("Access-Control-Allow-Methods", "GET, POST, PUT, PATCH, DELETE, OPTIONS");
  response->addHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
}
inline AsyncWebServerResponse* withCors(AsyncWebServerResponse* response) {
  cors(response);
  return response;
}
inline void json(AsyncWebServerRequest* req, int code, const char* body) {
  req->send(withCors(req->beginResponse(code, "application/json", body)));
}
inline void text(AsyncWebServerRequest* req, int code, const char* body) {
  req->send(withCors(req->beginResponse(code, "text/plain", body)));
}
inline void error(AsyncWebServerRequest* req, int code, const char* message) {
  char body[160];
  snprintf(body, sizeof(body), "{\"status\":\"error\",\"message\":\"%s\"}", message ? message : "error");
  json(req, code, body);
}
}
