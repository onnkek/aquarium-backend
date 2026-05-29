#include "runtime/ControlTask.h"
#include "app/App.h"

bool ControlTask::begin() {
  return xTaskCreatePinnedToCore(
    ControlTask::taskMain,
    "control",
    8192,
    this,
    3,
    &handle_,
    1
  ) == pdPASS;
}

void ControlTask::taskMain(void* arg) {
  auto* self = static_cast<ControlTask*>(arg);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    self->app_.tickControl();
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(50));
  }
}
