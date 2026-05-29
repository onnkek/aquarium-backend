#pragma once

#include <Arduino.h>
#include <SD.h>
#include <ESPAsyncWebServer.h>
#include "core/Logger.h"
#include "core/SdGuard.h"

class StorageManager {
public:
  explicit StorageManager(Logger& logger);

  bool begin();

  // Physical SD state from App after SD.begin().
  // mounted() means the card was initialized.
  // available() means it is mounted and not in temporary backoff after IO errors.
  void setMounted(bool mounted);
  void setEnabled(bool enabled); // compatibility alias for setMounted()
  bool mounted() const;
  bool available() const;
  bool writable() const;
  uint32_t failureCount() const;
  void markFailure(const char* reason = nullptr);
  void clearFailure();

  // External users such as FTPServer touch SD directly and cannot use SdGuard.
  // While they are active, internal background writers must not access SD.
  void setExternalBusy(bool busy);
  bool externalBusy() const;

  bool exists(const char* path);
  bool removeFile(const char* path);

  bool appendBinary(const char* path, const uint8_t* data, size_t len);
  bool appendTextLine(const char* path, const char* line);
  bool writeFileAtomic(const char* path, const uint8_t* data, size_t len);

  bool readAll(const char* path, uint8_t* out, size_t maxLen, size_t* actualLen);
  File openReadLocked(const char* path, SdGuard& guard);

  AsyncWebServerResponse* beginFileResponse(
    AsyncWebServerRequest* request,
    const char* path,
    const char* contentType
  );

  uint64_t totalBytes();
  uint64_t usedBytes();

  bool ensureDirRecursive(const char* dir);
  bool ensureParentDirs(const char* path);

private:
  Logger& logger_;
  bool mounted_ = false;
  uint32_t disabledUntilMs_ = 0;
  uint32_t failureCount_ = 0;
  volatile bool externalBusy_ = false;

  bool readyForIo() const;

  void parentPath(const char* path, char* out, size_t outLen);
};
