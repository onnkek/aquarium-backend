# v1.9 Emergency mode policy

Emergency mode больше не означает «выключить вообще всё». Для аквариума это опасно, потому что фильтрация, аэрация и охлаждение светильников должны продолжать работать.

## Политика emergency mode

При `POST /api/safety/emergency-mode` или старом alias `POST /api/safety/emergency-off` прошивка делает:

| Узел | Состояние | Почему |
|---|---:|---|
| CO2 | OFF | может задушить рыбу |
| Heat | OFF | риск перегрева |
| Cool | OFF | риск неконтролируемого охлаждения/реле |
| Doser 1..4 | OFF | риск передоза удобрений |
| O2 | ON | жизнеобеспечение |
| Filter | ON | жизнеобеспечение |
| Light | ON | это не основной свет, а охлаждение светильников/кулеры |
| ARGB | ON, static red | визуальная индикация аварийного режима |

## Endpoint-ы

```bash
curl -X POST $ESP/api/safety/emergency-mode
```

Старый endpoint оставлен как alias:

```bash
curl -X POST $ESP/api/safety/emergency-off
```

Оба endpoint-а выполняют одну и ту же политику.

## Persistency

Emergency mode сохраняется в split-config:

- `/config/doser.json`: все дозаторы `mode: 0`
- `/config/relays.json`: `co2: 0`, `o2: 1`, `filter: 1`, `light: 1`
- `/config/temp.json`: `mode: 0`
- `/config/argb.json`: `mode: 1`, `style: 1`, `static: red`

Это сделано специально: если просто выключить GPIO, Scheduler через следующий tick может включить узлы обратно из старого config.

## Ожидаемый `/api/current` после emergency mode

```json
{
  "co2": { "status": false },
  "o2": { "status": true },
  "filter": { "status": true },
  "light": { "status": true },
  "argb": { "status": true },
  "temp": { "status": 0 },
  "doser": [
    { "running": false },
    { "running": false },
    { "running": false },
    { "running": false }
  ]
}
```
