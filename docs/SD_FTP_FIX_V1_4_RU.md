# SD/FTP fix v1.4

Проблема v1.3: FTP был объектом-членом `ApiServer` и создавался при глобальной инициализации `App`, то есть до `setup()` и до `SD.begin()`.

На ESP32 порядок инициализации глобальных объектов из разных translation unit не гарантирован. Из-за этого `FTPServer ftp_(SD)` мог привязаться к объекту SD слишком рано. В результате `SD.begin()` проходил, `/api/health` показывал mounted, но FTP при `CWD /` и `MKD` получал `opendir(/sd/) failed` и `sdCommand(): Card Failed`.

Исправление:

- `FTPServer` больше не хранится как объект-член.
- Он создаётся динамически только внутри `ApiServer::begin()`, то есть после `App::begin()` уже сделал `SPI.begin(...)` и `SD.begin(...)`.
- `ftp_->handleFTP()` по-прежнему вызывается каждый `loop()`, как в live-прошивке.
- FTP не завязан на `StorageManager`, потому что библиотека работает напрямую с `SD`.

Ключевой код:

```cpp
if (!ftp_) {
  ftp_ = new FTPServer(SD);
}
ftp_->begin(FTP_USER, FTP_PASSWORD);
```

Проверка:

1. Прошить v1.4.
2. Подключиться по FTP.
3. `CWD /` должен пройти успешно.
4. `MKD /test` должен создать директорию.
5. В Serial не должно быть `sdCommand(): Card Failed` при FTP-командах.
