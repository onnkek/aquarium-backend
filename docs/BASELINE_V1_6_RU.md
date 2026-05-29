# new-arch v1.6 — рабочая базовая точка и следующий порядок тестов

## Зафиксировано из v1.5

На тестовом контроллере подтверждено:

- RTC работает.
- SD монтируется, виден размер карты.
- FTP работает.
- API работает.
- Wi-Fi подключается.
- `/api/health` показывает `sd=true`, `rtc=true`, `sdFailures=0`.

Критичный порядок инициализации теперь считается зафиксированным и не должен меняться без отдельного теста:

```cpp
Serial.begin(...)
SPIFFS.begin(...)
SPI.begin(CLK, MISO, MOSI, CS)
SD.begin(CS)
Wire.begin(SDA, SCL)
rtc.begin()
relays.begin()
sensors.begin()
config.begin()
wifi.begin(...)
scheduler.begin(...)
ftpSrv.begin(...)
api.begin()
controlTask.begin()
```

## Что изменено в v1.6

Временное отключение SD-журнала и SD-событий убрано:

```cpp
#define AQUARIUM_ENABLE_SD_JOURNAL 1
#define AQUARIUM_ENABLE_SD_EVENTS 1
```

Но добавлена защита от конфликта с FTP.

FTP-библиотека работает напрямую с `SD`, поэтому она не может использовать `SdGuard`. Чтобы не было одновременных обращений, внутренние фоновые писатели ставятся на паузу, пока FTP-сессия активна:

```text
FTP active -> StorageManager externalBusy=true -> metrics/events/notes writes skip/defer
FTP active -> Logger storagePaused=true -> журнал в Serial, но не на SD
```

Это сохраняет FTP и возвращает боевые события/журнал без прежней гонки за SD.

## Что тестировать дальше

### 1. Boot

Ожидаемо:

```text
SD Card Size: ...MB
split config loaded
scheduler initialized
sd event writer started
ftp started
api started
water temp invalid: thermostat outputs forced off
connected
heap free=...
```

Не должно быть:

```text
sdCommand(): Card Failed
ff_sd_status(): Check status failed
RTC unavailable
watchdog got triggered
```

### 2. Health

```bash
curl -s $ESP/api/health
```

Ожидаемо:

```json
"sd": true,
"sdMounted": true,
"sdFailures": 0,
"rtc": true
```

### 3. FTP

Проверить:

- `LIST /`
- `MKD /test-v16`
- удалить `/test-v16`
- загрузить маленький файл
- скачать этот файл обратно

Во время активного FTP допустимо, что внутренние логи/ивенты не пишутся на SD. После отключения FTP они снова пишутся.

### 4. Реле

```bash
curl -X POST $ESP/api/safety/emergency-off
curl -X POST $ESP/api/relays/co2/on
curl -X POST $ESP/api/relays/co2/off
curl -X POST $ESP/api/relays/o2/on
curl -X POST $ESP/api/relays/o2/off
curl -X POST $ESP/api/relays/filter/on
curl -X POST $ESP/api/relays/filter/off
curl -X POST $ESP/api/relays/light/on
curl -X POST $ESP/api/relays/light/off
```

### 5. Дозатор

```bash
curl -X PATCH $ESP/api/doser/0 -H 'Content-Type: application/json' -d '{"mode":0,"dosage":1,"rate":0.5,"currentVolume":100}'
curl -X POST $ESP/api/doser/0/run
curl -X POST $ESP/api/doser/0/stop
```

### 6. Термостат без DS18B20

```bash
curl -X PATCH $ESP/api/temp -H 'Content-Type: application/json' -d '{"mode":4,"setting":25,"hysteresis":0.3}'
```

Без валидного датчика воды `heat` и `cool` должны оставаться OFF.

## Следующий этап после v1.6

1. Подключить DS18B20 и проверить thermostat.
2. Подключить AHT20 и проверить метрики воздуха/влажности.
3. Проверить запись `/logs` и `/events` после отключения FTP-клиента.
4. Проверить `/api/metrics` и `/api/events`.
5. Проверить auto-режимы реле по времени.
6. Проверить auto-дозатор на маленькой дозе и безопасном времени.
