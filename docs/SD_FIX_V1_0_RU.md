# SD fix v1.0

Причина ошибок SD была не в железе: live-проект на той же карте работает.
В `new-arch` SD использовалась одновременно из нескольких мест:

- FTPServer
- `server.serveStatic(..., SD, ...)`
- Journal task
- EventWriter task
- StorageManager / metrics / events

`SdGuard` защищал только наш код, но не FTP и не AsyncWebServer static serving.
Поэтому SD могла получать конкурентные операции и сыпать `sdCommand(): Card Failed`.

## Что сделано

В `BuildConfig.h` добавлены флаги:

```cpp
#define AQUARIUM_ENABLE_FTP 0
#define AQUARIUM_ENABLE_SD_STATIC_WEB 0
#define AQUARIUM_SD_SPI_FREQUENCY_HZ 4000000U
```

FTP и static serving с SD выключены по умолчанию.
SD теперь используется только через контролируемые слои: logger/storage/event writer.

## Как вернуть FTP позже

Только после отдельной ревизии SD ownership. Пока лучше держать:

```cpp
#define AQUARIUM_ENABLE_FTP 0
```

Иначе FTP будет снова ходить в SD мимо `SdGuard`.
