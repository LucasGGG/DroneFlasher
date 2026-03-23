# DroneFlasher

ESP32 WiFi drone flash tool — прошивка FC, dump та ELRS через браузер без ПК.

---

## Версії

| Папка | Плата | Підключення до FC |
|-------|-------|-------------------|
| `DroneFlasher/` | ESP32 WROOM-32 | UART (паяєш 4 дроти) |
| `DroneFlasherS3/` | ESP32-S3-WROOM-1-N16R8 | USB Type-C (plug & play) |

---

## DroneFlasher — ESP32 WROOM-32

### Підключення (паяти одноразово)

```
ESP32 GPIO16 (RX2)  ←──  FC TX
ESP32 GPIO17 (TX2)  ──►  FC RX
ESP32 GPIO18        ──►  FC BOOT0   (100 Ом)
ESP32 GPIO19        ──►  FC NRST    (100 Ом)
ESP32 GPIO22        ──►  ELRS RST   (100 Ом)
ESP32 GPIO23        ──►  ELRS GPIO0 (100 Ом)
ESP32 GND           ────  FC GND
```

### Як прошити ESP32

```bash
esptool --chip esp32 --port /dev/cu.usbserial-XXXX --baud 460800 \
  write_flash -z 0x0 DroneFlasher.bin
cu.usbserial-XXXX - ХХХХ - вказати номер порта
```

Або через OTA: `http://10.0.0.1/ota`

### Arduino IDE налаштування

```
Board:            ESP32 Dev Module
Partition Scheme: Default 4MB with spiffs (1.2MB APP / 1.5MB SPIFFS)
```

### Бібліотеки

```
ESP Async WebServer  (mathieucarbou)
Async TCP            (mathieucarbou)
ArduinoJson          (bblanchon, v6)
```

---

## DroneFlasherS3 — ESP32-S3 (USB OTG)

Підключаєш FC через **USB Type-C** прямо в OTG порт ESP32-S3.  
Паяти нічого не потрібно.

```
ESP32-S3 USB OTG ──── FC USB Type-C (будь-який STM32 FC)
```

ESP32-S3 автоматично визначає режим FC:

| Режим FC | Badge | Доступно |
|----------|-------|---------|
| Betaflight запущено | 🟢 CDC | dump + ELRS + перехід в DFU |
| DFU режим (BOOT затиснуто) | 🟡 DFU | hex прошивка |
| Не підключено | 🔴 — | — |

### Arduino IDE налаштування

```
Board:              ESP32S3 Dev Module
Flash Size:         16MB
PSRAM:              OPI PSRAM (8MB)
Partition Scheme:   16M Flash (3MB APP/9.9MB FATFS)
USB CDC On Boot:    Disabled
USB Mode:           Hardware CDC and JTAG
```

---

## Веб-інтерфейс

| URL | Що робить |
|-----|-----------|
| `http://10.0.0.1` | Головна — вибір файлів, прошивка |
| `http://10.0.0.1/ota` | OTA оновлення ESP32 або STM32 hex |
| `http://10.0.0.1/log` | Лог прошивки (live) |

**WiFi:** `CONFIG` / `freeAzov`

---

## Що можна прошити

- **hex** — прошивка польотного контролера (Betaflight, INAV, тощо)
- **dump** — відновлення Betaflight CLI дампу
- **RX** — прошивка ELRS приймача через BF `serialpassthrough`

Кожен пункт незалежний — можна прошити тільки dump, тільки hex, або все разом.

---

## LED індикатор (GPIO2)

| Стан | Моргання |
|------|---------|
| Старт | 3x швидкий блиск |
| FC не підключено | Подвійний блиск-пауза |
| FC підключено / CDC | Повільне (1 сек) |
| Прошивка іде | Швидке (80 мс) |

---

## CLI утиліта (flash_drone.py)

Альтернатива веб-інтерфейсу — пряме використання з терміналу.

```bash
pip install esptool pyserial

# Повний пайплайн
python3 flash_drone.py --hex fc.hex --dump bf_dump.txt --elrs elrs_rx.bin

# Тільки прошивка + dump
python3 flash_drone.py --hex fc.hex --dump bf_dump.txt

# Тільки ELRS
python3 flash_drone.py --elrs elrs_rx.bin
```
