# Doser state API v2.3

Persistent state дозатора хранится отдельно от настроек:

```text
/state/doser.json
```

В state хранится только дата последнего auto-run:

```json
{
  "0": { "lastRunYmd": "2026-05-22" },
  "1": { "lastRunYmd": "" },
  "2": { "lastRunYmd": "" },
  "3": { "lastRunYmd": "" }
}
```

## Зачем дата, а не bool

Bool `workedToday=true` требует надежного сброса после полуночи. Если ESP зависла,
ребутнулась или RTC был недоступен в момент сброса, флаг может остаться неверным.

Дата проще и надежнее:

```text
lastRunYmd == today  -> auto-дозирование сегодня заблокировано
lastRunYmd != today  -> auto-дозирование разрешено
lastRunYmd == ""     -> auto-дозирование разрешено
```

Фронт может отображать это как boolean, но в SPIFFS хранится дата.

## Получить state всех помп

```bash
curl -s $ESP/api/doser/state
```

Ответ:

```json
[
  { "id": 0, "lastRunYmd": "2026-05-22" },
  { "id": 1, "lastRunYmd": "" },
  { "id": 2, "lastRunYmd": "" },
  { "id": 3, "lastRunYmd": "" }
]
```

## Получить state одной помпы

```bash
curl -s $ESP/api/doser/0/state
```

## Сбросить отметку одной помпы

Используется после подмены, если в этот же день нужно разрешить повторное auto-внесение.

```bash
curl -X POST $ESP/api/doser/0/state/reset
```

## Сбросить отметку всех помп

```bash
curl -X POST $ESP/api/doser/state/reset
```

## Пометить одну помпу как уже работавшую сегодня

Используется если удобрение внесено вручную, и надо запретить auto-внесение до завтра.

```bash
curl -X POST $ESP/api/doser/0/state/mark-run
```

Alias:

```bash
curl -X POST $ESP/api/doser/0/last-run/mark
```

## Пометить все помпы как уже работавшие сегодня

```bash
curl -X POST $ESP/api/doser/state/mark-run
```

## RTC requirement

`mark-run` требует валидный RTC. Если RTC недоступен, API вернет:

```json
{ "error": "rtc unavailable" }
```

Это сделано специально: нельзя записывать фейковую дату `2000-01-01` как реальный last-run.
