# v2.5 — ручной выход из emergency mode

## Зачем

Emergency mode теперь можно снять вручную даже если причина аварии ещё активна, например RTC всё ещё невалиден.

Это нужно для тестов и для ситуаций, когда пользователь осознанно берёт управление на себя.

## Endpoint-ы

### Получить safety state

```bash
curl -s $ESP/api/safety
```

Пример ответа при проблеме RTC и ручном override:

```json
{
  "emergencyMode": false,
  "emergencyOverride": true,
  "rtcValid": false,
  "activeReasons": ["rtc invalid"]
}
```

### Включить emergency mode

```bash
curl -X POST $ESP/api/safety/emergency-mode
```

Alias старого имени:

```bash
curl -X POST $ESP/api/safety/emergency-off
```

### Выйти из emergency mode вручную

```bash
curl -X POST $ESP/api/safety/emergency-mode/clear
```

Alias:

```bash
curl -X POST $ESP/api/safety/clear-emergency-mode
```

## Поведение

Если RTC валиден:

```text
emergencyMode=false
emergencyOverride=false
```

Если RTC невалиден, но пользователь вручную снял emergency:

```text
emergencyMode=false
emergencyOverride=true
rtcValid=false
activeReasons=["rtc invalid"]
```

Пока `emergencyOverride=true`, прошивка не будет автоматически заново включать emergency mode из-за RTC.

Когда RTC снова станет валидным, override автоматически сбросится:

```text
emergencyOverride=false
```

## Важно

Выход из emergency mode не восстанавливает старые режимы оборудования автоматически.

После ручного выхода пользователь сам должен вернуть нужные режимы через API/фронт:

```bash
curl -X PATCH $ESP/api/relays/co2 -H "Content-Type: application/json" -d '{"mode":2}'
curl -X PATCH $ESP/api/temp -H "Content-Type: application/json" -d '{"mode":4}'
curl -X PATCH $ESP/api/doser/0 -H "Content-Type: application/json" -d '{"mode":2}'
```
