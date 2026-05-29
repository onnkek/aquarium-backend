# v2.6 — Emergency snapshot/restore

Теперь emergency mode не просто навсегда переписывает режимы оборудования.

Новая схема:

1. Перед входом в emergency mode прошивка сохраняет snapshot текущих режимов в SPIFFS:

```text
/state/emergency.json
```

2. Затем применяется аварийная политика:

```text
CO2 OFF
Heat/Cool OFF
Doser 1..4 OFF
O2 ON
Filter ON
Light Cooling ON
ARGB static red
```

3. При ручном выходе:

```http
POST /api/safety/emergency-mode/clear
```

прошивка восстанавливает сохранённые режимы реле, дозаторов, temp и ARGB.

## Что восстанавливается

```text
doser[0..3].mode
co2.mode
o2.mode
filter.mode
light.mode
temp.mode
argb.mode
argb.style
argb.brightness
argb.static color
```

## Почему snapshot хранится в SPIFFS

Если ESP перезагрузилась в emergency mode, snapshot не теряется. После ребута можно вызвать clear endpoint, и система восстановит режимы, которые были до аварии.

## Endpoint-ы

Включить emergency mode:

```bash
curl -X POST $ESP/api/safety/emergency-mode
```

Выйти из emergency mode и восстановить прежние режимы:

```bash
curl -X POST $ESP/api/safety/emergency-mode/clear
```

Проверить состояние safety:

```bash
curl -s $ESP/api/safety
```

Пример:

```json
{
  "emergencyMode": true,
  "emergencyOverride": false,
  "restoreAvailable": true,
  "rtcValid": false,
  "activeReasons": ["rtc invalid"]
}
```

## Важно

Если RTC всё ещё невалиден, clear всё равно разрешён. В этом случае:

```text
emergencyMode = false
emergencyOverride = true
```

Это значит: пользователь вручную снял аварийный режим, хотя причина аварии ещё есть.
