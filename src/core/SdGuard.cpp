#include "core/SdGuard.h"

static SemaphoreHandle_t gSdMutex = nullptr;

void SdGuard::begin() {
  if (!gSdMutex) gSdMutex = xSemaphoreCreateMutex();
}

SemaphoreHandle_t SdGuard::mutex() { return gSdMutex; }

SdGuard::SdGuard(uint32_t timeoutMs) {
  begin();
  locked_ = xSemaphoreTake(gSdMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

SdGuard::~SdGuard() {
  if (locked_ && gSdMutex) xSemaphoreGive(gSdMutex);
}
