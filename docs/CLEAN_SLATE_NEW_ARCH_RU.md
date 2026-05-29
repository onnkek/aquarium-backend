# new-arch: чистая новая архитектура

Эта версия собрана заново без старого монолитного `aquarium-backend.cpp`.

## Главная схема

```text
Frontend / HTTP
    ↓
ApiServer
    ↓
CommandBus — для команд
    ↓
ControlTask
    ↓
SchedulerService / DoserService / RelayService / ThermostatService
    ↓
RelayController / FanController / ArgbController / SensorService
```

Логи идут отдельно:

```text
Logger
  ↓
JournalBus
  ↓
journal task
  ↓
SD /logs/...
```

События идут отдельно:

```text
DoserService / RelayService / ThermostatService
  ↓
EventBus
  ↓
EventWriterTask
  ↓
EventStorage
  ↓
SD /events/...
```

## Почему удалён legacy

Если оставить `aquarium-backend.cpp`, проект снова превращается в два параллельных мира:

1. старая логика в одном огромном файле;
2. новая архитектура с сервисами.

Так нельзя нормально развивать проект. Поэтому legacy-файлы удалены полностью.

## Что сохранено по логике

- 4 помпы дозатора;
- `rate` как мл/сек, как в старом live-проекте;
- расписание дозатора по дням недели;
- `currentVolume` / `maxVolume`;
- реле `co2/o2/filter/light`;
- `heat/cool`;
- ARGB;
- fan PWM;
- метрики в бинарном формате;
- события в бинарном формате;
- notes;
- `/api/current`;
- safe boot LOW=OFF, HIGH=ON.

## Что проверять первым

1. `platformio run`.
2. Boot без нагрузки.
3. Все LED OFF после старта.
4. `/api/health`.
5. `/api/current`.
6. `/api/relays/co2/on` и `/off`.
7. `/api/doser/0/run` и `/stop`.
8. Запись логов на SD.
9. Запись событий на SD.
