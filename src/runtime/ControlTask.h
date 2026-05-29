#pragma once

#include <Arduino.h>

class App;

class ControlTask {
public:
  explicit ControlTask(App& app) : app_(app) {}

  bool begin();

private:
  App& app_;
  TaskHandle_t handle_ = nullptr;

  static void taskMain(void* arg);
};
