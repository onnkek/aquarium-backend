#include "storage/StorageManager.h"
#include "config/BuildConfig.h"
#include <string.h>

StorageManager::StorageManager(Logger& logger)
  : logger_(logger) {}

bool StorageManager::begin() {
  return true;
}

void StorageManager::setMounted(bool mounted) {
  mounted_ = mounted;
  if (mounted) {
    disabledUntilMs_ = 0;
  }
}

void StorageManager::setEnabled(bool enabled) {
  setMounted(enabled);
}

bool StorageManager::mounted() const {
  return mounted_;
}

bool StorageManager::readyForIo() const {
  return mounted_ && !externalBusy_ && millis() >= disabledUntilMs_;
}

bool StorageManager::available() const {
  return readyForIo();
}

bool StorageManager::writable() const {
  return readyForIo();
}

uint32_t StorageManager::failureCount() const {
  return failureCount_;
}

void StorageManager::markFailure(const char* reason) {
  failureCount_++;
  disabledUntilMs_ = millis() + AQUARIUM_STORAGE_BACKOFF_MS;

  if (reason && *reason) {
    logger_.warn(LogCategory::Storage, reason);
  }
}

void StorageManager::clearFailure() {
  disabledUntilMs_ = 0;
}

void StorageManager::setExternalBusy(bool busy) {
  externalBusy_ = busy;
}

bool StorageManager::externalBusy() const {
  return externalBusy_;
}

bool StorageManager::exists(const char* path) {
  if (!available()) return false;
  if (!path || !*path) return false;
  SdGuard guard(500);
  if (!guard.locked()) { markFailure("sd busy"); return false; }
  return SD.exists(path);
}

bool StorageManager::removeFile(const char* path) {
  if (!available()) return false;
  if (!path || !*path) return false;
  SdGuard guard(1000);
  if (!guard.locked()) { markFailure("sd busy"); return false; }
  return SD.remove(path);
}

bool StorageManager::appendBinary(const char* path, const uint8_t* data, size_t len) {
  if (!available()) return false;
  if (!path || !*path || !data || len == 0) return false;

  SdGuard guard(1000);
  if (!guard.locked()) {
    markFailure("sd busy: appendBinary");
    return false;
  }

  if (!ensureParentDirs(path)) {
    markFailure("failed to create parent dirs");
    return false;
  }

  File f = SD.open(path, FILE_APPEND);
  if (!f) {
    markFailure("append open failed");
    return false;
  }

  const size_t written = f.write(data, len);
  f.flush();
  f.close();

  if (written != len) { markFailure(); return false; }
  return true;
}

bool StorageManager::appendTextLine(const char* path, const char* line) {
  if (!available()) return false;
  if (!path || !*path || !line) return false;

  SdGuard guard(1000);
  if (!guard.locked()) {
    markFailure("sd busy: appendTextLine");
    return false;
  }

  if (!ensureParentDirs(path)) { markFailure("failed to create parent dirs"); return false; }

  File f = SD.open(path, FILE_APPEND);
  if (!f) {
    markFailure("append text open failed");
    return false;
  }

  f.print(line);
  f.print('\n');
  f.flush();
  f.close();

  return true;
}

bool StorageManager::writeFileAtomic(const char* path, const uint8_t* data, size_t len) {
  if (!available()) return false;
  if (!path || !*path || !data) return false;

  char tmp[128];
  char bak[128];
  snprintf(tmp, sizeof(tmp), "%s.tmp", path);
  snprintf(bak, sizeof(bak), "%s.bak", path);

  SdGuard guard(1500);
  if (!guard.locked()) {
    markFailure("sd busy: writeFileAtomic");
    return false;
  }

  if (!ensureParentDirs(path)) { markFailure("failed to create parent dirs"); return false; }

  File f = SD.open(tmp, FILE_WRITE);
  if (!f) {
    markFailure("atomic tmp open failed");
    return false;
  }

  const size_t written = f.write(data, len);
  f.flush();
  f.close();

  if (written != len) {
    SD.remove(tmp);
    markFailure("atomic tmp write incomplete");
    return false;
  }

  if (SD.exists(bak)) SD.remove(bak);

  if (SD.exists(path)) {
    if (!SD.rename(path, bak)) {
      SD.remove(tmp);
      logger_.error(LogCategory::Storage, "atomic backup rename failed");
      return false;
    }
  }

  if (!SD.rename(tmp, path)) {
    if (SD.exists(bak)) SD.rename(bak, path);
    logger_.error(LogCategory::Storage, "atomic promote failed");
    return false;
  }

  if (SD.exists(bak)) SD.remove(bak);
  return true;
}

bool StorageManager::readAll(const char* path, uint8_t* out, size_t maxLen, size_t* actualLen) {
  if (!available()) return false;
  if (actualLen) *actualLen = 0;
  if (!path || !*path || !out || maxLen == 0) return false;

  SdGuard guard(1000);
  if (!guard.locked()) { markFailure("sd busy"); return false; }

  File f = SD.open(path, FILE_READ);
  if (!f) return false;

  size_t used = 0;
  while (f.available() && used < maxLen) {
    used += f.read(out + used, maxLen - used);
  }
  f.close();

  if (actualLen) *actualLen = used;
  return true;
}

File StorageManager::openReadLocked(const char* path, SdGuard& guard) {
  if (!available() || !path || !*path || !guard.locked()) return File();
  File f = SD.open(path, FILE_READ);
  if (!f) markFailure("open read failed");
  return f;
}

AsyncWebServerResponse* StorageManager::beginFileResponse(
  AsyncWebServerRequest* request,
  const char* path,
  const char* contentType
) {
  if (!request || !path || !*path || !contentType) return nullptr;
  if (!exists(path)) return nullptr;

  // Компромиссный вариант для static-файлов: AsyncWebServer читает SD сам.
  // Для metrics/events ниже оставляем CSV streaming через явный SdGuard.
  return request->beginResponse(SD, path, contentType);
}

uint64_t StorageManager::totalBytes() {
  if (!available()) return 0;
  SdGuard guard(500);
  if (!guard.locked()) return 0;
  return SD.totalBytes();
}

uint64_t StorageManager::usedBytes() {
  if (!available()) return 0;
  SdGuard guard(500);
  if (!guard.locked()) return 0;
  return SD.usedBytes();
}

bool StorageManager::ensureDirRecursive(const char* dir) {
  if (!dir || !*dir) return false;
  if (strcmp(dir, "/") == 0) return true;

  char current[128] = {0};
  size_t pos = 0;

  if (dir[0] == '/') {
    current[pos++] = '/';
    current[pos] = '\0';
  }

  const char* p = dir;
  if (*p == '/') ++p;

  while (*p) {
    const char* slash = strchr(p, '/');
    const size_t partLen = slash ? static_cast<size_t>(slash - p) : strlen(p);

    if (partLen > 0) {
      if (pos > 1 && current[pos - 1] != '/') current[pos++] = '/';
      if (pos + partLen >= sizeof(current)) return false;
      memcpy(current + pos, p, partLen);
      pos += partLen;
      current[pos] = '\0';

      if (!SD.exists(current)) {
        if (!SD.mkdir(current)) return false;
      }
    }

    if (!slash) break;
    p = slash + 1;
  }

  return true;
}

bool StorageManager::ensureParentDirs(const char* path) {
  char parent[128];
  parentPath(path, parent, sizeof(parent));
  return ensureDirRecursive(parent);
}

void StorageManager::parentPath(const char* path, char* out, size_t outLen) {
  if (!out || outLen == 0) return;
  out[0] = '\0';
  if (!path || !*path) return;

  const char* slash = strrchr(path, '/');
  if (!slash) {
    strncpy(out, "/", outLen - 1);
    out[outLen - 1] = '\0';
    return;
  }

  if (slash == path) {
    strncpy(out, "/", outLen - 1);
    out[outLen - 1] = '\0';
    return;
  }

  const size_t len = min<size_t>(static_cast<size_t>(slash - path), outLen - 1);
  memcpy(out, path, len);
  out[len] = '\0';
}
