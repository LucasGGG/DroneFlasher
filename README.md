# DroneFlasher

ESP32 WiFi drone flash tool — прошивка FC, dump та ELRS через браузер без ПК.

---

## Версії

| Папка | Плата | Підключення до FC |
|-------|-------|-------------------|
| `DroneFlasher/` | ESP32 WROOM-32 | UART (паяєш 4 дроти) |
| `DroneFlasherS3/` | **ESP32-S3-WROOM-1-N16R8** | USB Type-C (plug & play) ✅ |

---

## DroneFlasherS3 — ESP32-S3 (рекомендовано)

### Підключення

```
LiPo батарея  ──────────────►  FC (живлення)
ESP32-S3 лівий Type-C  ──►  FC USB Type-C (дані D+/D-)
```

> ⚠️ **FC потребує батарею** для живлення — ESP32-S3 не подає 5В на OTG порт.  
> USB кабель передає тільки дані (D+/D-). Батарея обов'язкова.

ESP32-S3 автоматично визначає режим FC:

| Режим FC | Статус | Доступно |
|----------|--------|---------|
| Betaflight запущено | 🟢 CDC | dump + ELRS passthrough |
| DFU режим (BOOT затиснуто при підключенні) | 🟡 DFU | hex прошивка |
| Не підключено | 🔴 none | — |

### Arduino IDE налаштування

```
Board:              ESP32S3 Dev Module
Flash Size:         16MB
PSRAM:              OPI PSRAM (8MB)
Partition Scheme:   16M Flash (3MB APP / 9.9MB FATFS)
USB Mode:           Hardware CDC and JTAG   ← обов'язково
USB CDC On Boot:    Enabled
```

### Бібліотеки

```
ESP Async WebServer  (mathieucarbou)
Async TCP            (mathieucarbou)
ArduinoJson          (bblanchon, v7)
Adafruit NeoPixel
```

---

## DroneFlasher — ESP32 WROOM-32 (старіша версія)

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

## Веб-інтерфейс

**WiFi:** `CONFIG` / `freeAzov` → `http://10.0.0.1`

| URL | Що робить |
|-----|-----------|
| `http://10.0.0.1` | Головна — вибір файлів, прошивка |
| `http://10.0.0.1/ota` | OTA оновлення ESP32 або STM32 hex |
| `http://10.0.0.1/log` | Лог прошивки (live) |
| `http://10.0.0.1/diag` | Діагностика GPIO, USB, RGB LED |

---

## Що можна прошити

- **hex** — прошивка польотного контролера (Betaflight, INAV тощо) через DFU
- **dump** — відновлення Betaflight CLI дампу (FC в звичайному режимі)
- **RX** — прошивка ELRS приймача через BF `serialpassthrough`

Кожен пункт незалежний — можна прошити тільки dump, тільки hex, або все разом.

---

## RGB LED індикатор (WS2812 @ GPIO48)

| Стан | Колір |
|------|-------|
| Очікування FC | 🟡 Жовтий — повільно мигає |
| FC підключено | 🟢 Зелений — горить статично |
| Прошивка іде | 🟢 Зелений — швидко мигає (80 мс) |
| Прошито успішно | 🔵 Синій — 5 секунд |
| Помилка | 🔴 Червоний |

> Якщо LED не реагує — знайди правильний пін через `http://10.0.0.1/diag` (секція RGB LED).

---

## Workflow прошивки

```
1. Підключи LiPo до FC
2. Підключи USB-C: FC → лівий порт ESP32-S3
3. Підключись до WiFi: CONFIG / freeAzov
4. Відкрий http://10.0.0.1
5. Статус стає зеленим → FC визначено
6. Вибери файли (hex / dump / elrs) → ПРОШИТИ
7. Після завершення — синій LED на 5 сек
```

**Для DFU (hex прошивка):** затисни BOOT на FC і підключи USB — FC перейде в DFU режим.

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
