# Быстрое тестирование без импорта Postman

Подставь IP контроллера:

```bash
ESP=http://192.168.1.50
```

## Smoke

```bash
curl -s $ESP/api/health | jq
curl -s $ESP/api/current | jq
```

## Emergency OFF

```bash
curl -X POST $ESP/api/safety/emergency-off
```

## Реле

```bash
curl -X POST $ESP/api/relays/co2/on
curl -X POST $ESP/api/relays/co2/off
curl -X POST $ESP/api/relays/o2/on
curl -X POST $ESP/api/relays/o2/off
curl -X POST $ESP/api/relays/filter/on
curl -X POST $ESP/api/relays/filter/off
curl -X POST $ESP/api/relays/light/on
curl -X POST $ESP/api/relays/light/off
```

## Дозатор

```bash
curl -X PATCH $ESP/api/doser/0 -H 'Content-Type: application/json' -d '{"mode":0,"dosage":1,"rate":0.5,"currentVolume":100,"maxVolume":450}'
curl -X POST $ESP/api/doser/0/run
sleep 2
curl -X POST $ESP/api/doser/0/stop
```

## Выключить все конфигом на тестовом стенде

```bash
curl -X PATCH $ESP/api/temp -H 'Content-Type: application/json' -d '{"mode":0}'
curl -X PATCH $ESP/api/argb -H 'Content-Type: application/json' -d '{"mode":0}'
curl -X PATCH $ESP/api/relays/co2 -H 'Content-Type: application/json' -d '{"mode":0}'
curl -X PATCH $ESP/api/relays/o2 -H 'Content-Type: application/json' -d '{"mode":0}'
curl -X PATCH $ESP/api/relays/filter -H 'Content-Type: application/json' -d '{"mode":0}'
curl -X PATCH $ESP/api/relays/light -H 'Content-Type: application/json' -d '{"mode":0}'
for i in 0 1 2 3; do curl -X PATCH $ESP/api/doser/$i -H 'Content-Type: application/json' -d '{"mode":0}'; done
```
