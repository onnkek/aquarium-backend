#pragma once
#include <Arduino.h>

namespace ApiUtils {
inline bool safeId(const String& value, size_t maxLen = 32) {
  if (value.length() == 0 || value.length() > maxLen) return false;
  for (size_t i = 0; i < value.length(); ++i) {
    char c = value[i];
    bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
    if (!ok) return false;
  }
  return true;
}
inline int boundedInt(AsyncWebServerRequest* req, const char* name, int def, int lo, int hi) {
  int v = def;
  if (req->hasParam(name)) v = req->getParam(name)->value().toInt();
  if (v < lo) v = lo;
  if (v > hi) v = hi;
  return v;
}
}
