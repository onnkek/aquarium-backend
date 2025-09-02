#include <Arduino.h>
#include "aquarium-backend.h"
#include "systemHandlers.h"
#include "logsHandlers.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FTPServer.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <GTimer.h>
#include <Wire.h>
#include <SparkFun_Qwiic_Humidity_AHT20.h>
#include "FastLED.h"
#include <GyverDS18.h>
#include <GyverRelay.h>
#include "RTClib.h"
#include "defs.h"

const int doserPins[DOSER_COUNT] = {DOSER_1, DOSER_2, DOSER_3, DOSER_4};
const char *doserKeys[DOSER_COUNT] = {"0", "1", "2", "3"};

const int extraPins[EXTRA_COUNT] = {RELAY_CO2, RELAY_O2, RELAY_FILTER, RELAY_LIGHT};
const char *extraKeys[EXTRA_COUNT] = {"co2", "o2", "filter", "light"};

AHT20 humiditySensor;
CRGB leds[NUM_LEDS];
byte argb_counter;

RTC_DS3231 rtc;

char daysOfTheWeek[7][12] = {"su", "mo", "tu", "we", "th", "fr", "sa"};
GyverRelay regulatorUp(NORMAL);
GyverRelay regulatorDown(REVERSE);
GyverDS18Single ds(DS18B20);

const char *ssid = "A";
const char *password = "882882882";
int64_t uptime;
DynamicJsonDocument config(16384);

// ================== RELAY STATE ==================
enum RelayStatus
{
  STATUS_OFF,
  STATUS_ON,
  STATUS_RUNNING
};
struct RelayState
{
  bool running = false;          // doser pump
  unsigned long startMillis = 0; // doser pump
  unsigned long durationMs = 0;  // doser pump
  int lastProgressPercent = -1;  // doser pump
  float progress = 0;            // doser pump
  RelayStatus status = STATUS_OFF;
};
RelayState doserStates[DOSER_COUNT];
RelayState extraStates[EXTRA_COUNT];
RelayState argbState;

AsyncWebServer server(80);
FTPServer ftpSrv(SD); // SPIFFS or SD

int statusTemp = 0; // 0 - off, 1 - cool, 2 - heat, 3 - cool+heat

double statusAHT20 = 0;
double statusHumidity = 0;

double inputTemp = 0;

// --- Кэш времени с мьютексом ---
DateTime lastTime;
SemaphoreHandle_t timeMutex;
// --- Очередь для логов ---
struct LogMessage
{
  char path[32];
  char text[128];
};
QueueHandle_t logQueue;

// --- Чтение кэша времени ---
DateTime getLastTime()
{
  DateTime copy(DateTime(2000, 1, 1, 0, 0, 0));
  if (xSemaphoreTake(timeMutex, portMAX_DELAY) == pdTRUE)
  {
    copy = lastTime;
    xSemaphoreGive(timeMutex);
  }
  return copy;
}

// --- Таск: обновление времени из RTC ---
void vTaskUpdateTime(void *pvParameters)
{
  for (;;)
  {
    DateTime now = rtc.now();
    if (xSemaphoreTake(timeMutex, portMAX_DELAY) == pdTRUE)
    {
      lastTime = now;
      if (!lastTime.isValid())
      {
        // writeLOG("!!!!!DATETIME IS INVALID!!!!!", 2, LOGS_SYSTEM);
        Serial.println("!!!!!DATETIME IS INVALID!!!!!");
        Serial.println(String("now.hour(): ") + lastTime.hour());
        Serial.println(String("now.minute(): ") + lastTime.minute());
        Serial.println(String("now.seconds(): ") + lastTime.second());
        Serial.println(String("now.year(): ") + lastTime.year());
        Serial.println(String("now.day(): ") + lastTime.day());
        Serial.println(String("now.month(): ") + lastTime.month());
      }
      xSemaphoreGive(timeMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// --- Таск: логирование в файл ---
void vTaskLogger(void *pvParameters)
{
  LogMessage msg;
  File logFile;

  if (!SD.begin())
  {
    Serial.println("SD init failed!");
    vTaskDelete(NULL);
  }

  for (;;)
  {
    if (xQueueReceive(logQueue, &msg, portMAX_DELAY) == pdPASS)
    {
      logFile = SD.open(msg.path, FILE_APPEND);
      if (logFile)
      {
        logFile.println(msg.text);
        logFile.close();
      }
      else
      {
        Serial.printf("Failed to open file: %s\n", msg.path);
      }
    }
  }
}

void parseTime(const char *timeStr, int &hour, int &minute)
{
  if (!timeStr)
  {
    hour = 0;
    minute = 0;
    return;
  }
  sscanf(timeStr, "%d:%d", &hour, &minute);
}

void addCORS(AsyncWebServerResponse *response)
{
  response->addHeader("Access-Control-Allow-Origin", "*");
  response->addHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
  response->addHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
}

void safeAdjust(DateTime dt)
{
  rtc.adjust(dt);
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED, OUTPUT); // LED

  pinMode(RELAY_COOL, OUTPUT);
  pinMode(RELAY_HEAT, OUTPUT);

  for (int i = 0; i < EXTRA_COUNT; i++)
  {
    pinMode(extraPins[i], OUTPUT);
    digitalWrite(extraPins[i], LOW);
  }

  pinMode(DOSER_1, OUTPUT);
  pinMode(DOSER_2, OUTPUT);
  pinMode(DOSER_3, OUTPUT);
  pinMode(DOSER_4, OUTPUT);

  digitalWrite(DOSER_1, LOW);
  digitalWrite(DOSER_2, LOW);
  digitalWrite(DOSER_3, LOW);
  digitalWrite(DOSER_4, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  if (rtc.begin() == false)
  {
    Serial.println("RTC detected");
    lastTime = rtc.now();
  }

  ds.requestTemp();
  delay(1000);

  FastLED.addLeds<WS2811, ARGB_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  Serial.println("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  btStop();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.printf_P(PSTR("."));
  }
  Serial.println("");
  Serial.println("WiFi connected..!");
  Serial.print("Got IP: ");
  Serial.println(WiFi.localIP());

  timeMutex = xSemaphoreCreateMutex();
  logQueue = xQueueCreate(10, sizeof(LogMessage));
  if (!logQueue)
  {
    Serial.println("Queue create failed!");
    while (1)
      ;
  }

  if (!SPIFFS.begin(true))
  {
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  // bool fsok = LittleFS.begin(true);
  SPI.begin(CLK, MISO, MOSI, SS);
  if (!SD.begin(SS))
  {
    Serial.println("Card Mount Failed");
    return;
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  ftpSrv.begin("admin", "admin");

  // инициализация и проверка подключения
  // если датчик не обнаружен, вводим программу в бесконечный цикл
  if (humiditySensor.begin() == false)
  {
    // writeLOG("AHT20 not detected", 2, LOGS_SYSTEM);
  }
  else
  {
    // writeLOG("AHT20 sensor initialized", 0, LOGS_SYSTEM);
  }

  server.on("/logs/system", HTTP_GET, handleGetSystemLogs);
  server.on("/logs/relay", HTTP_GET, handleGetRelayLogs);
  server.on("/logs/doser", HTTP_GET, handleGetDoserLogs);

  server.on("/logs/clear/system", HTTP_GET, handleGetClearSystemLogs);
  server.on("/logs/clear/relay", HTTP_GET, handleGetClearRelayLogs);
  server.on("/logs/clear/doser", HTTP_GET, handleGetClearDoserLogs);

  server.on("/current", HTTP_GET, handleGetCurrent);
  server.on("/config", HTTP_GET, handleGetConfig);
  server.on(
      "/config", HTTP_POST,
      [](AsyncWebServerRequest *request) {},
      nullptr,
      handlePostConfig);

  server.on(
      "/time", HTTP_POST,
      [](AsyncWebServerRequest *request) {},
      nullptr,
      handlePostTime);

  server.serveStatic("/", SD, "/").setDefaultFile("index.html");
  server.serveStatic("/logs", SD, "/index.html");
  server.serveStatic("/archive", SD, "/index.html");
  server.onNotFound([](AsyncWebServerRequest *request)
                    {
    if (request->method() == HTTP_OPTIONS) {
      AsyncWebServerResponse *response = request->beginResponse(204);
      addCORS(response);
      request->send(response);
    } else {
      request->send(404, "text/html", "Not found");
    } });

  server.begin();

  // Core 1: UpdateTime + Logger
  xTaskCreatePinnedToCore(vTaskUpdateTime, "UpdateTime", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(vTaskLogger, "Logger", 4096, NULL, 1, NULL, 1);
  loadConfigFromSD(); // Read config
  double setting = config["temp"]["setting"];
  double hysteresis = config["temp"]["hysteresis"];
  double k = config["temp"]["k"];
  regulatorUp.setpoint = setting + hysteresis;
  regulatorDown.setpoint = setting - hysteresis;
  regulatorUp.hysteresis = hysteresis;
  regulatorDown.hysteresis = hysteresis;
  regulatorUp.k = k;
  regulatorDown.k = k;
  delay(1000);

  log("ESP32 initialized successfully", INFO, LOGS_SYSTEM);
}

void loop()
{
  ftpSrv.handleFTP();
  EVERY_MS(10)
  {
    for (int i = 0; i < EXTRA_COUNT; i++)
      checkExtraRelay(i, lastTime);

    for (int i = 0; i < DOSER_COUNT; i++)
      checkPumpSchedule(i, lastTime);

    checkARGB(lastTime);
    checkTemp();

    // Reset hasRunToday at midnight
    if (lastTime.hour() == 0 && lastTime.minute() == 0 && lastTime.second() == 0 && config["doser"][String(1)]["hasRunToday"])
    {
      for (int i = 0; i < DOSER_COUNT; i++)
      {
        config["doser"][String(i)]["hasRunToday"] = false;
      }
      saveConfigToSD();
      log("New day -> reset doser flags", WARNING, LOGS_DOSER);
    }
  }

  EVERY_MS(1000)
  {
    // если от датчика получены свежие данные, выводим их
    if (humiditySensor.begin() == true && humiditySensor.available() == true)
    {
      statusAHT20 = humiditySensor.getTemperature(); // получаем температуру
      statusHumidity = humiditySensor.getHumidity(); // получаем относительную влажность
    }
  }
}

String getCurrentInfo()
{
  StaticJsonDocument<512> currentInfo;
  StaticJsonDocument<256> systemInfo;

  StaticJsonDocument<256> doserInfo;
  StaticJsonDocument<64> co2Info;
  StaticJsonDocument<64> o2Info;
  StaticJsonDocument<64> tempInfo;
  StaticJsonDocument<64> lightInfo;
  StaticJsonDocument<64> argbInfo;
  StaticJsonDocument<64> filterInfo;
  StaticJsonDocument<64> aht20Info;

  StaticJsonDocument<64> pump1Info;
  StaticJsonDocument<64> pump2Info;
  StaticJsonDocument<64> pump3Info;
  StaticJsonDocument<64> pump4Info;

  JsonArray doserInfoArray = doserInfo.to<JsonArray>();

  StaticJsonDocument<64> timeInfo;
  timeInfo["year"] = lastTime.year();
  timeInfo["month"] = lastTime.month();
  timeInfo["day"] = lastTime.day();
  timeInfo["dayOfWeek"] = daysOfTheWeek[lastTime.dayOfTheWeek()];
  timeInfo["hour"] = lastTime.hour();
  timeInfo["minute"] = lastTime.minute();
  timeInfo["second"] = lastTime.second();

  aht20Info["temp"] = statusAHT20;
  aht20Info["hum"] = statusHumidity;

  systemInfo["time"] = timeInfo;
  systemInfo["chipTemp"] = temperatureRead();
  systemInfo["outside"] = aht20Info;
  systemInfo["uptime"] = esp_timer_get_time();
  systemInfo["totalSpace"] = SD.totalBytes();
  systemInfo["usedSpace"] = SD.usedBytes();
  systemInfo["freeSpace"] = SD.totalBytes() - SD.usedBytes();
  systemInfo["freeHeap"] = ESP.getFreeHeap();
  systemInfo["heapSize"] = ESP.getHeapSize();
  systemInfo["frequency"] = ESP.getCpuFreqMHz();

  pump1Info["status"] = doserStates[0].status == 0 ? false : true;
  pump1Info["introduced"] = doserStates[0].progress;
  pump2Info["status"] = doserStates[1].status == 0 ? false : true;
  pump2Info["introduced"] = doserStates[1].progress;
  pump3Info["status"] = doserStates[2].status == 0 ? false : true;
  pump3Info["introduced"] = doserStates[2].progress;
  pump4Info["status"] = doserStates[3].status == 0 ? false : true;
  pump4Info["introduced"] = doserStates[3].progress;

  doserInfoArray.add(pump1Info);
  doserInfoArray.add(pump2Info);
  doserInfoArray.add(pump3Info);
  doserInfoArray.add(pump4Info);

  co2Info["status"] = extraStates[0].status == 0 ? false : true; // { "co2", "o2", "filter", "light" };
  o2Info["status"] = extraStates[1].status == 0 ? false : true;
  filterInfo["status"] = extraStates[2].status == 0 ? false : true;
  lightInfo["status"] = extraStates[3].status == 0 ? false : true;

  argbInfo["status"] = argbState.status == 0 ? false : true;
  tempInfo["status"] = statusTemp;
  tempInfo["current"] = inputTemp;

  currentInfo["system"] = systemInfo;
  currentInfo["co2"] = co2Info;
  currentInfo["o2"] = o2Info;
  currentInfo["filter"] = filterInfo;
  currentInfo["light"] = lightInfo;
  currentInfo["temp"] = tempInfo;
  currentInfo["doser"] = doserInfo;
  currentInfo["argb"] = argbInfo;

  String jsonCurrentInfo;
  serializeJson(currentInfo, jsonCurrentInfo);
  return jsonCurrentInfo;
}

void loadConfigFromSD()
{
  File file = SPIFFS.open("/config.json", "r");
  if (!file)
  {
    log("Failed to open config.json file for reading", ERROR, LOGS_SYSTEM);
    return;
  }

  DeserializationError error = deserializeJson(config, file);
  file.close();

  if (error)
  {
    log("deserializeJson() failed: ", ERROR, LOGS_SYSTEM);
    log(error.c_str(), ERROR, LOGS_SYSTEM);
    return;
  }
  log("config.json loaded successfully", INFO, LOGS_SYSTEM);
}

void saveConfigToSD()
{
  File file = SPIFFS.open("/config.json", "w");
  if (!file)
  {
    log("Failed to open config.json file for writing", ERROR, LOGS_SYSTEM);
    return;
  }
  if (serializeJson(config, file) == 0)
  {
    log("Failed to write to config.json", ERROR, LOGS_SYSTEM);
  }
  file.close();
}

String readLOG(String path)
{
  File file = SD.open(path, "r");
  if (!file)
  {
    log("Failed to open " + path + " file for reading", ERROR, LOGS_SYSTEM);
    return "";
  }
  String log;
  int lineCount = 0;
  while (file.available())
  {
    log += file.readStringUntil('\n');
    log += '\n';
    lineCount++;
  }
  return log;
  // log(path + " read successfully", INFO, LOGS_SYSTEM);
}
String getLogType(LogType type)
{
  switch (type)
  {
  case 0:
    return "INFO";
  case 1:
    return "WARNING";
  case 2:
    return "ERROR";
  default:
    break;
  }
}
void log(const String &text, LogType type, const String &path)
{
  LogMessage msg;

  String log = String("[") + getDateString() + "][" + getLogType(type) + "]: " + text;
  path.toCharArray(msg.path, sizeof(msg.path));
  log.toCharArray(msg.text, sizeof(msg.text));
  Serial.println(log);
  xQueueSend(logQueue, &msg, 0);
}

String getDateString()
{
  DateTime now = getLastTime();
  return String(getFullDateNumber(now.month())) + "/" + getFullDateNumber(now.day()) + "/" + now.year() + " " + getFullDateNumber(now.hour()) + ":" + getFullDateNumber(now.minute()) + ":" + getFullDateNumber(now.second());
}

String getFullDateNumber(int number)
{
  String result = String(number);
  if (number < 10)
  {
    result = "0" + String(number);
  }
  return result;
}

String getARGBMode(int mode)
{
  String modeString;
  switch (mode)
  {
  case 1:
    return "STATIC";
  case 2:
    return "CYCLE";
  case 3:
    return "GRADIENT";
  case 4:
    return "CUSTOM";
  default:
    return "OFF";
  }
}

// ================== CHECK ARGB ==================
void checkARGB(DateTime now)
{
  JsonObject argb = config["argb"];
  if (argb.isNull())
    return;

  int mode = argb["mode"] | 0;
  int brightness = argb["brightness"] | 128;
  const char *onStr = argb["on"] | "00:00";
  const char *offStr = argb["off"] | "00:00";

  int onH, onM, offH, offM;
  parseTime(onStr, onH, onM);
  parseTime(offStr, offH, offM);

  int nowM = now.hour() * 60 + now.minute();
  int onMins = onH * 60 + onM;
  int offMins = offH * 60 + offM;

  bool active = (onMins <= offMins) ? (nowM >= onMins && nowM < offMins)
                                    : (nowM >= onMins || nowM < offMins);

  static int lastMode = -1;
  static bool lastActive = false;

  FastLED.setBrightness(brightness);

  // Логируем изменение состояния
  if (active != lastActive || mode != lastMode)
  {
    if (!active || mode == 0)
      log(String("ARGB") + " is OFF (Manual)", INFO, LOGS_RELAY);
    else
      log(String("ARGB") + " is " + mode, INFO, LOGS_RELAY);

    lastActive = active;
    lastMode = mode;
  }

  if (!active || mode == 0)
  {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    argbState.status = STATUS_OFF;
    return;
  }
  JsonObject col = argb["static"];
  CRGB color = CRGB(col["r"] | 0, col["b"] | 0, col["g"] | 0);
  int speed = argb["cycle"]["speed"] | 10;
  static uint8_t hueOffset = 0;
  JsonObject grad = argb["gradient"];
  CRGB startColor = CRGB(grad["start"]["r"] | 0, grad["start"]["b"] | 0, grad["start"]["g"] | 0);
  CRGB endColor = CRGB(grad["end"]["r"] | 0, grad["end"]["b"] | 0, grad["end"]["g"] | 0);
  JsonArray arr = argb["custom"];
  switch (mode)
  {
  case 1:
    // STATIC
    fill_solid(leds, NUM_LEDS, color);
    FastLED.show();
    argbState.status = STATUS_ON;
    break;

  case 2:
    // CYCLE
    EVERY_MS(speed)
    { // скорость движения радуги
      for (int i = 0; i < NUM_LEDS; i++)
      {
        leds[i].setHue(argb_counter + i * 255 / NUM_LEDS);
      }
      argb_counter++;
      FastLED.show();
    }
    argbState.status = STATUS_ON;
    break;

  case 3:
    // GRADIENT
    for (int i = 0; i < NUM_LEDS; i++)
    {
      float t = i / float(NUM_LEDS - 1);
      leds[i] = blend(startColor, endColor, uint8_t(t * 255));
    }
    FastLED.show();
    argbState.status = STATUS_ON;
    break;

  case 4:
    // CUSTOM
    for (int i = 0; i < NUM_LEDS; i++)
    {
      if (i >= arr.size())
        break;
      JsonObject col = arr[i];
      leds[i] = CRGB(col["r"] | 0, col["b"] | 0, col["g"] | 0);
    }
    FastLED.show();
    argbState.status = STATUS_ON;
    break;
  }
}
String toUpper(const char *str)
{
  static String s;
  s = str;
  s.toUpperCase();
  return s.c_str();
}
// ================== CHECK EXTRA RELAY ==================
void checkExtraRelay(int index, DateTime now)
{
  JsonObject relay = config[extraKeys[index]];
  if (relay.isNull())
    return;

  int mode = relay["mode"] | 0;
  const char *onStr = relay["on"] | "00:00";
  const char *offStr = relay["off"] | "00:00";

  int onHour, onMinute, offHour, offMinute;
  parseTime(onStr, onHour, onMinute);
  parseTime(offStr, offHour, offMinute);

  RelayState &state = extraStates[index];

  switch (mode)
  {
  case 0: // OFF
    digitalWrite(extraPins[index], LOW);
    if (state.status != STATUS_OFF)
    {
      log(String("RELAY ") + toUpper(extraKeys[index]) + " is OFF (Manual)", INFO, LOGS_RELAY);
      state.status = STATUS_OFF;
    }
    break;

  case 1: // ON
    digitalWrite(extraPins[index], HIGH);
    if (state.status != STATUS_ON)
    {
      log(String("RELAY ") + toUpper(extraKeys[index]) + " is ON (Manual)", INFO, LOGS_RELAY);
      state.status = STATUS_ON;
    }
    break;

  case 2: // AUTO
    // check if now is between on and off
    int nowMinutes = now.hour() * 60 + now.minute();
    int onMinutes = onHour * 60 + onMinute;
    int offMinutes = offHour * 60 + offMinute;
    bool active = false;

    if (onMinutes <= offMinutes)
      active = (nowMinutes >= onMinutes && nowMinutes < offMinutes);
    else
      active = (nowMinutes >= onMinutes || nowMinutes < offMinutes); // crosses midnight

    if (active && state.status != STATUS_ON)
    {
      digitalWrite(extraPins[index], HIGH);
      log(String("RELAY ") + toUpper(extraKeys[index]) + " is ON (Auto)", INFO, LOGS_RELAY);
      state.status = STATUS_ON;
    }
    else if (!active && state.status != STATUS_OFF)
    {
      digitalWrite(extraPins[index], LOW);
      log(String("RELAY ") + toUpper(extraKeys[index]) + " is OFF (Auto)", INFO, LOGS_RELAY);
      state.status = STATUS_OFF;
    }
    break;
  }
}

// ================== CHECK DOSER PUMP ==================
void checkPumpSchedule(int index, DateTime now)
{
  JsonObject pump = config["doser"][String(index)];
  if (pump.isNull())
    return;

  int mode = pump["mode"] | 0; // 0=OFF,1=MANUAL,2=AUTO
  bool hasRunToday = pump["hasRunToday"] | false;
  const char *timeStr = pump["time"] | "00:00";
  int startHour, startMinute;
  parseTime(timeStr, startHour, startMinute);
  float dosage = pump["dosage"] | 0.0;
  float rate = pump["rate"] | 1.0;
  float volume = pump["currentVolume"] | 200;

  static const char *days[] = {"su", "mo", "tu", "we", "th", "fr", "sa"};
  bool dayEnabled = pump["period"][days[now.dayOfTheWeek()]] | false;

  RelayState &state = doserStates[index];

  switch (mode)
  {
  case 0: // OFF
    if (state.status != STATUS_OFF)
    {
      digitalWrite(doserPins[index], LOW);
      log(String("PUMP ") + String(index + 1) + " is OFF (Manual)", INFO, LOGS_DOSER);
      state.status = STATUS_OFF;
    }
    break;

  case 1: // MANUAL ON
    if (state.status != STATUS_ON)
    {
      digitalWrite(doserPins[index], HIGH);
      log(String("PUMP ") + String(index + 1) + " is ON (Manual)", INFO, LOGS_DOSER);
      state.status = STATUS_ON;
    }
    break;

  case 2: // AUTO
    if (!dayEnabled)
      break;
    if (hasRunToday && state.progress != 100 && !state.running)
    {
      state.progress = 100;
    }
    if (!hasRunToday && state.progress != 0 && !state.running)
    {
      state.progress = 0;
    }
    if (!hasRunToday && now.hour() >= startHour && now.minute() >= startMinute && now.second() >= 0 && !state.running)
    {
      state.durationMs = (unsigned long)((dosage / rate) * 1000);
      state.startMillis = millis();
      state.running = true;
      state.lastProgressPercent = -1;
      digitalWrite(doserPins[index], HIGH);
      log(String("PUMP ") + String(index + 1) + " is ON (Auto) for " + state.durationMs + " ms", INFO, LOGS_DOSER);
      pump["hasRunToday"] = true;
      pump["currentVolume"] = volume - dosage;
      saveConfigToSD();
    }

    // handle running pump
    if (state.running)
    {
      unsigned long elapsed = millis() - state.startMillis;
      int percent = min(100, (int)((elapsed * 100) / state.durationMs));
      if (percent / 1 != state.lastProgressPercent / 1)
      {
        state.progress = percent;
        state.lastProgressPercent = percent;
        if (percent % 10 == 0)
        {
          log(String("PUMP ") + String(index + 1) + " progress: " + percent + "%", INFO, LOGS_DOSER);
        }
      }

      if (elapsed >= state.durationMs)
      {
        digitalWrite(doserPins[index], LOW);
        state.running = false;
        state.status = STATUS_OFF;
        log(String("PUMP ") + String(index + 1) + " is OFF (Auto)", INFO, LOGS_DOSER);
      }
      else
      {
        state.status = STATUS_RUNNING;
      }
    }
    break;
  }
}

void checkTemp()
{
  ds.requestTemp();
  int timeout = config["temp"]["timeout"];
  int mode = config["temp"]["mode"];
  EVERY_MS(1000)
  {
    if (ds.readTemp())
    {
      inputTemp = ds.getTemp();
      // Serial.println(inputTemp);
    }
  }
  switch (mode) // mode: 0 - off, 1 - cool, 2 - heat, 3 - cool+heat, 4 - auto
  {
  case 0:
    if (statusTemp != 0)
    {
      statusTemp = 0;
      digitalWrite(RELAY_COOL, LOW);
      digitalWrite(RELAY_HEAT, LOW);
      log("RELAY COOL is OFF (Manual)", INFO, LOGS_RELAY);
      log("RELAY HEAT is OFF (Manual)", INFO, LOGS_RELAY);
    }
    break;
  case 1:
    if (statusTemp != 1)
    {
      statusTemp = 1;
      digitalWrite(RELAY_COOL, HIGH);
      digitalWrite(RELAY_HEAT, LOW);
      log("RELAY COOL is ON (Manual)", INFO, LOGS_RELAY);
      log("RELAY HEAT is OFF (Manual)", INFO, LOGS_RELAY);
    }
    break;
  case 2:
    if (statusTemp != 2)
    {
      statusTemp = 2;
      digitalWrite(RELAY_COOL, LOW);
      digitalWrite(RELAY_HEAT, HIGH);
      log("RELAY COOL is OFF (Manual)", INFO, LOGS_RELAY);
      log("RELAY HEAT is ON (Manual)", INFO, LOGS_RELAY);
    }
    break;
  case 3:
    if (statusTemp != 3)
    {
      statusTemp = 3;
      digitalWrite(RELAY_COOL, HIGH);
      digitalWrite(RELAY_HEAT, HIGH);
      log("RELAY COOL is ON (Manual)", INFO, LOGS_RELAY);
      log("RELAY HEAT is ON (Manual)", INFO, LOGS_RELAY);
    }
    break;
  case 4:
    EVERY_MS(timeout * 1000)
    {
      regulatorUp.input = inputTemp;
      regulatorDown.input = inputTemp;

      if (regulatorUp.getResult() == 1 && statusTemp != 1) // Cool
      {
        statusTemp = 1;
        digitalWrite(RELAY_COOL, HIGH);
        log("RELAY COOL is ON (Auto)", INFO, LOGS_RELAY);
      }
      if (regulatorDown.getResult() == 1 && statusTemp != 2) // Heat
      {
        statusTemp = 2;
        digitalWrite(RELAY_HEAT, HIGH);
        log("RELAY HEAT is ON (Auto)", INFO, LOGS_RELAY);
      }
      if (regulatorUp.getResult() == 0 && regulatorDown.getResult() == 0 && statusTemp != 0) // Off Cool/Heat
      {
        statusTemp = 0;
        if (regulatorUp.getResult() == 0)
        {
          digitalWrite(RELAY_COOL, LOW);
          log("RELAY COOL is OFF (Auto)", INFO, LOGS_RELAY);
        }
        if (regulatorDown.getResult() == 0)
        {
          digitalWrite(RELAY_HEAT, LOW);
          log("RELAY HEAT is OFF (Auto)", INFO, LOGS_RELAY);
        }
      }
    }
    break;
  }
}
