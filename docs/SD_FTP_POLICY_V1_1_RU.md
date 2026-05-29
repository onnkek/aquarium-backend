# SD / FTP policy v1.1

В live-прошивке SD работает стабильно на том же железе, поэтому v1.1 возвращает SD init к legacy-схеме:

```cpp
SPI.begin(CLK, MISO, MOSI, SS);
SD.begin(SS);
```

FTP нужен проекту, поэтому он включён обратно.

Чтобы не получить конкурентный доступ к SD во время отладки, фоновые SD-писатели по умолчанию выключены:

```cpp
#define AQUARIUM_ENABLE_SD_JOURNAL 0
#define AQUARIUM_ENABLE_SD_EVENTS 0
```

Логи всё равно идут в Serial. Config, notes, metrics и ручные API-операции остаются через StorageManager.

Когда базовая работа SD/FTP подтвердится, следующий шаг — включать journal/event writer по одному и смотреть, какой именно слой конфликтует.
