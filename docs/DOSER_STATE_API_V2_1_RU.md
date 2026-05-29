# Doser State API v2.1

`lastRunYmd` больше не является частью `/config/doser.json`. Это persistent-state дозатора, который хранится в SPIFFS отдельно:

```text
/state/doser.json
```

Он нужен, чтобы после ребута ESP не было повторного автодозирования в тот же день.

## Получить state всех помп

```bash
curl -s $ESP/api/doser/state
```

Ответ:

```json
[
  { "id": 0, "lastRunYmd": "2026-05-22", "hasLastRun": true },
  { "id": 1, "lastRunYmd": "", "hasLastRun": false },
  { "id": 2, "lastRunYmd": "", "hasLastRun": false },
  { "id": 3, "lastRunYmd": "", "hasLastRun": false }
]
```

## Получить state одной помпы

```bash
curl -s $ESP/api/doser/0/state
```

Ответ:

```json
{
  "id": 0,
  "lastRunYmd": "2026-05-22",
  "hasLastRun": true
}
```

## Сбросить дату последнего автозапуска одной помпы

Это нужно, если была подмена воды и в тот же день надо внести удобрения повторно.

```bash
curl -X POST $ESP/api/doser/0/state/reset
```

Ответ:

```json
{
  "id": 0,
  "lastRunYmd": "",
  "hasLastRun": false
}
```

После этого auto scheduler снова сможет запустить эту помпу сегодня, если текущее время уже прошло расписание и `mode = 2`.

## Alias для фронта

Оставлен более явный alias:

```bash
curl -X POST $ESP/api/doser/0/last-run/reset
```

Он делает то же самое, что `/api/doser/0/state/reset`.

## Сбросить state всех помп

```bash
curl -X POST $ESP/api/doser/state/reset
```

Использовать аккуратно. Обычно фронту нужна кнопка сброса только на конкретной помпе.


## Исправление v2.2

`hasLastRun` удалён из ответа API, чтобы не путать persistent state с вычисляемым флагом.
Храним только `lastRunYmd`, потому что дата переживает reboot и не требует хрупкого сброса ровно в полночь.

Также порядок регистрации endpoint-ов исправлен: `/api/doser/state` и `/api/doser/{id}/state` регистрируются раньше общих `/api/doser` и `/api/doser/{id}`.
