#include "core/WatchdogService.h"
#include <esp_task_wdt.h>
#include <esp_idf_version.h>

void WatchdogService::begin() {
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t cfg{.timeout_ms = 10000, .idle_core_mask = 0, .trigger_panic = true};
  esp_task_wdt_init(&cfg);
#else
  esp_task_wdt_init(10, true);
#endif
  esp_task_wdt_add(nullptr);
}
void WatchdogService::feed() { esp_task_wdt_reset(); }
