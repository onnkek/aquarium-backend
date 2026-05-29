# SD State Policy v1.2

Цель: SD-карта не должна трогаться из кода, если она не инициализирована или временно переведена в backoff после ошибки.

## Состояния

`StorageManager` теперь является основным gatekeeper для SD:

- `mounted()` — карта успешно прошла `SD.begin()`.
- `available()` — карта mounted и не находится во временном backoff.
- `writable()` — синоним готовности к записи сейчас.
- `failureCount()` — количество IO-ошибок/таймаутов SD с момента старта.

## Правило

Перед любой операцией с SD нужно проверять `storage_.available()`.

Фоновые писатели, API, metrics/events/notes и FTP должны уважать это состояние.

## FTP

FTP остаётся включённым, но `ftp_.handleFTP()` вызывается только если `storage_.available() == true`.
Если SD ушла в backoff, FTP временно ставится на паузу, чтобы не долбить карту.

## Health

`/api/health` теперь показывает:

```json
{
  "status": {
    "sd": true,
    "sdMounted": true,
    "sdFailures": 0
  }
}
```

- `sdMounted=true`, `sd=true` — карта mounted и сейчас доступна.
- `sdMounted=true`, `sd=false` — карта была mounted, но сейчас backoff после ошибки.
- `sdMounted=false`, `sd=false` — SD не поднялась при boot.

## Что это исправляет

Раньше часть кода могла лезть в SD напрямую или при временной ошибке продолжать спамить карту. Теперь основные места идут через `StorageManager` и проверяют state.
