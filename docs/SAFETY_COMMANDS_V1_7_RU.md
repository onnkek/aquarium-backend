# v1.7 Safety commands

## Почему реле были `true` после boot

`/api/current` показывает runtime-состояние. Если в split-config у реле стоит `mode: 1`, SchedulerService воспринимает это как Manual ON и включает реле после старта.

Команды вида `POST /api/relays/co2/on` специально меняют `mode` на Manual ON и сохраняют это в `/config/relays.json`, чтобы состояние переживало перезагрузку.

## Что изменено

`POST /api/safety/emergency-off` теперь не просто выключает GPIO, а также переводит опасные подсистемы в config OFF и сохраняет split-config:

- все 4 дозатора: `mode = 0`
- CO2/O2/filter/light: `mode = 0`
- temp: `mode = 0`
- argb: `mode = 0`

Это нужно, чтобы scheduler не включил реле обратно через 100 мс.

## Проверка

```bash
curl -X POST $ESP/api/safety/emergency-off
sleep 1
curl -s $ESP/api/current
curl -s $ESP/api/relays
curl -s $ESP/api/doser
curl -s $ESP/api/temp
curl -s $ESP/api/argb
```

После этого все runtime-status должны быть OFF, а config modes должны быть `0`.
