# v1.5 FTP/RTC live-init fix

Исправлено:

1. `Wire.begin(SDA, SCL)` вызывается до `rtc.begin()` и `AHT20.begin()`, как в live-прошивке.
2. `SensorService` больше не переинициализирует I2C повторно.
3. FTP вынесен из `ApiServer` на уровень `App.cpp` как `static FTPServer ftpSrv(SD)`, ближе к live-проекту.
4. `ftpSrv.begin()` вызывается после `SD.begin()`, а `ftpSrv.handleFTP()` — каждый `loop()`.

Цель — вернуть порядок и lifetime SD/FTP/RTC максимально близко к рабочей прошивке.
