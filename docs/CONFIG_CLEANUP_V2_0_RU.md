# v2.0 — очистка конфига дозатора

`hasRunToday` и `status` удалены из persistent config.

Активный конфиг дозатора теперь содержит только настройки оборудования:

```text
/config/doser.json
```

Состояние последнего автозапуска хранится отдельно:

```text
/state/doser.json
```

Почему так:

- `status` — runtime, он должен быть в `/api/current`, а не в config.
- `hasRunToday` — старый runtime-флаг, он больше не нужен.
- `lastRunYmd` нужен для защиты от повторного автодозирования после reboot, но это state, а не настройка оборудования.

API `/api/doser` и `/api/config/export` больше не сериализуют `status`/`hasRunToday` в конфиг.
Legacy import по-прежнему понимает старый `hasRunToday`, но мигрирует его в `/state/doser.json`.
