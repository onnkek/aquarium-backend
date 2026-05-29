# v1.6.1 — compile fix FTPServer::active

Исправлен compile error в локальной FTP-библиотеке.

Причина: метод `FTPServer::active()` был объявлен как `const`, но внутри вызывал `WiFiClient::connected()`, а в Arduino ESP32 этот метод не `const`.

Было:

```cpp
bool FTPServer::active() const;
```

Стало:

```cpp
bool FTPServer::active();
```

Логика работы FTP/SD не менялась.
