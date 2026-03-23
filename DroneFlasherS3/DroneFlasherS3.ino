/*
 * DroneFlasher S3
 * ESP32-S3-WROOM-1-N16R8
 * WiFi AP: CONFIG / freeAzov  →  http://10.0.0.1
 *
 * ┌──────────────────────────────────────────────────────┐
 * │  Підключення                                         │
 * │                                                      │
 * │  USB OTG Type-C (другий роз'єм, GPIO19/20)           │
 * │      ──────────────────► FC USB Type-C               │
 * │                          (будь-який STM32 FC)        │
 * │                                                      │
 * │  Більше нічого паяти не потрібно!                    │
 * │                                                      │
 * │  USB-UART Type-C (перший роз'єм, CH340)              │
 * │      ──────────────────► ПК (для прошивки ESP32)     │
 * └──────────────────────────────────────────────────────┘
 *
 * Arduino IDE налаштування:
 *   Board:              ESP32S3 Dev Module
 *   Flash Size:         16MB
 *   PSRAM:              OPI PSRAM (8MB)
 *   Partition Scheme:   16M Flash (3MB APP/9.9MB FATFS)
 *   USB CDC On Boot:    Disabled
 *   USB Mode:           Hardware CDC and JTAG
 *   Upload Mode:        UART0 / Hardware CDC
 *
 * Бібліотеки:
 *   ESP Async WebServer  (mathieucarbou, 3.x)
 *   Async TCP            (mathieucarbou, 3.x)
 *   ArduinoJson          (bblanchon, v6)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Update.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/ringbuf.h>

// ESP-IDF USB Host
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"

// ── WiFi AP ──────────────────────────────────────────────────────────────────
static const char* AP_SSID = "CONFIG";
static const char* AP_PASS = "freeAzov";

// ── LED ──────────────────────────────────────────────────────────────────────
#define LED_PIN  2

// ── Web server ───────────────────────────────────────────────────────────────
AsyncWebServer server(80);

// ── Лог ──────────────────────────────────────────────────────────────────────
#define LOG_MAX 10240
static String       flashLog  = "";
static SemaphoreHandle_t logMux = nullptr;

static void addLog(const String& s) {
  if (xSemaphoreTake(logMux, pdMS_TO_TICKS(80)) == pdTRUE) {
    flashLog += s + "\n";
    if (flashLog.length() > LOG_MAX)
      flashLog = flashLog.substring(flashLog.length() - LOG_MAX);
    xSemaphoreGive(logMux);
  }
  Serial.println(s);
}

// ── Стан ─────────────────────────────────────────────────────────────────────
enum UsbMode { USB_NONE, USB_DFU, USB_CDC };

struct State {
  bool    flashing   = false;
  int     progress   = 0;
  String  message    = "Очікування...";
  UsbMode usbMode    = USB_NONE;
  bool    hexReady   = false;
  bool    dumpReady  = false;
  bool    elrsReady  = false;
};
static State g;

static void setMsg(const String& m, int p = -1) {
  g.message = m; if (p >= 0) g.progress = p; addLog(m);
}

// ── USB CDC стан ─────────────────────────────────────────────────────────────
static cdc_acm_dev_hdl_t cdcDev       = nullptr;
static SemaphoreHandle_t cdcRxSem     = nullptr;   // сигнал: прийшли дані
static RingbufHandle_t   cdcRxRing    = nullptr;   // кільцевий буфер RX
static SemaphoreHandle_t cdcMux       = nullptr;

// ── USB DFU константи (STM32 DfuSe) ─────────────────────────────────────────
#define DFU_VID          0x0483
#define DFU_PID          0xDF11
#define DFU_DNLOAD       0x01
#define DFU_GETSTATUS    0x03
#define DFU_CLRSTATUS    0x04
#define DFU_ABORT        0x06
#define DFU_INTF         0

static usb_device_handle_t dfuDevHdl  = nullptr;
static uint8_t             dfuIntfNum = 0;

// ── Flash task ────────────────────────────────────────────────────────────────
static TaskHandle_t flashTask  = nullptr;
static uint8_t      g_elrsUart = 4;
static uint32_t     g_elrsBaud = 420000;

// ── Upload file ───────────────────────────────────────────────────────────────
static File uploadFile;

// ─────────────────────────────────────────────────────────────────────────────
//  CDC callbacks
// ─────────────────────────────────────────────────────────────────────────────
static void cdcRxCb(const uint8_t* data, size_t len, void*) {
  xRingbufferSend(cdcRxRing, data, len, pdMS_TO_TICKS(10));
  xSemaphoreGive(cdcRxSem);
}

static void cdcEventCb(const cdc_acm_host_dev_event_data_t* ev, void*) {
  if (ev->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) {
    addLog("CDC: FC відключено");
    if (xSemaphoreTake(cdcMux, pdMS_TO_TICKS(200)) == pdTRUE) {
      cdcDev = nullptr;
      g.usbMode = USB_NONE;
      xSemaphoreGive(cdcMux);
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  CDC helpers
// ─────────────────────────────────────────────────────────────────────────────
static void cdcFlushRx() {
  size_t sz;
  void* p;
  while ((p = xRingbufferReceive(cdcRxRing, &sz, 0)) != nullptr)
    vRingbufferReturnItem(cdcRxRing, p);
}

static String cdcReadUntil(const String& token, uint32_t timeout = 1500) {
  String buf = "";
  uint32_t t = millis();
  while (millis() - t < timeout) {
    size_t sz;
    uint8_t* p = (uint8_t*)xRingbufferReceive(cdcRxRing, &sz, pdMS_TO_TICKS(50));
    if (p) {
      buf += String((char*)p, sz);
      vRingbufferReturnItem(cdcRxRing, p);
      if (buf.indexOf(token) >= 0) break;
    }
  }
  return buf;
}

static esp_err_t cdcWrite(const uint8_t* d, size_t l) {
  if (!cdcDev) return ESP_FAIL;
  return cdc_acm_host_data_tx_blocking(cdcDev, d, l, 500);
}

static void cdcPrint(const String& s) {
  cdcWrite((const uint8_t*)s.c_str(), s.length());
}

// ─────────────────────────────────────────────────────────────────────────────
//  DFU helpers  (STM32 DfuSe via USB control transfer)
// ─────────────────────────────────────────────────────────────────────────────
static esp_err_t dfuControl(uint8_t req, uint16_t val,
                              uint8_t* data, uint16_t len,
                              bool isIn, uint32_t timeout = 2000) {
  if (!dfuDevHdl) return ESP_ERR_INVALID_STATE;
  usb_transfer_t* t = nullptr;
  esp_err_t r = usb_host_transfer_alloc(8 + len, 0, &t);
  if (r != ESP_OK) return r;

  t->device_handle = dfuDevHdl;
  t->bEndpointAddress = 0;
  t->timeout_ms = timeout;
  t->num_bytes   = 8 + len;

  usb_setup_packet_t* s = (usb_setup_packet_t*)t->data_buffer;
  s->bmRequestType = isIn ? 0xA1 : 0x21;
  s->bRequest      = req;
  s->wValue        = val;
  s->wIndex        = dfuIntfNum;
  s->wLength       = len;
  if (!isIn && data && len) memcpy(t->data_buffer + 8, data, len);

  r = usb_host_transfer_submit_control(nullptr, t);
  // Wait (simple spin – OK for flash task)
  uint32_t deadline = millis() + timeout;
  while (t->status == USB_TRANSFER_STATUS_COMPLETED - 1) {
    if (millis() > deadline) { r = ESP_ERR_TIMEOUT; break; }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  if (r == ESP_OK && isIn && data && len)
    memcpy(data, t->data_buffer + 8, len);
  usb_host_transfer_free(t);
  return r;
}

// Get DFU status (returns state byte at index 4)
static uint8_t dfuGetStatus() {
  uint8_t buf[6] = {};
  dfuControl(DFU_GETSTATUS, 0, buf, 6, true);
  return buf[4]; // bState
}

static void dfuClearStatus() {
  dfuControl(DFU_CLRSTATUS, 0, nullptr, 0, false);
}

// DfuSe: set address pointer
static bool dfuSetAddress(uint32_t addr) {
  uint8_t cmd[5] = {0x21,
    (uint8_t)(addr), (uint8_t)(addr>>8),
    (uint8_t)(addr>>16), (uint8_t)(addr>>24)};
  if (dfuControl(DFU_DNLOAD, 0, cmd, 5, false) != ESP_OK) return false;
  uint32_t t = millis();
  while (millis() - t < 3000) {
    uint8_t st = dfuGetStatus();
    if (st == 5) return true;   // dfuDNLOAD_IDLE
    if (st == 10) { dfuClearStatus(); return false; } // dfuERROR
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return false;
}

// DfuSe: mass erase
static bool dfuMassErase() {
  uint8_t cmd[1] = {0x41};
  if (dfuControl(DFU_DNLOAD, 0, cmd, 1, false) != ESP_OK) return false;
  // mass erase can take up to 20s
  uint32_t t = millis();
  while (millis() - t < 20000) {
    uint8_t st = dfuGetStatus();
    if (st == 5) return true;
    if (st == 10) { dfuClearStatus(); return false; }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  return false;
}

// DfuSe: write block (blockNum >= 2 for real data)
static bool dfuWriteBlock(uint16_t blockNum, uint8_t* data, size_t len) {
  if (dfuControl(DFU_DNLOAD, blockNum, data, (uint16_t)len, false) != ESP_OK)
    return false;
  uint32_t t = millis();
  while (millis() - t < 3000) {
    uint8_t st = dfuGetStatus();
    if (st == 5) return true;
    if (st == 10) { dfuClearStatus(); return false; }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return false;
}

// DfuSe: leave DFU → jump to app
static void dfuLeave(uint32_t appAddr = 0x08000000) {
  dfuSetAddress(appAddr);
  // DFU_DNLOAD with 0 length = manifest
  dfuControl(DFU_DNLOAD, 2, nullptr, 0, false);
  dfuGetStatus();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Intel HEX → DFU flash
// ─────────────────────────────────────────────────────────────────────────────
static bool flashHex() {
  if (!dfuDevHdl) {
    setMsg("FC не в DFU режимі. Підключи FC, затисни BOOT і підключи USB");
    return false;
  }
  File f = SPIFFS.open("/hex.hex", "r");
  if (!f) { setMsg("HEX файл не знайдено"); return false; }

  // Clear any DFU error state
  uint8_t state = dfuGetStatus();
  if (state == 10) dfuClearStatus();

  setMsg("Стирання FC flash...", 3);
  addLog("DFU mass erase...");
  if (!dfuMassErase()) { f.close(); setMsg("Стирання failed"); return false; }
  addLog("Стирання OK");

  // Parse Intel HEX and write
  uint32_t extAddr  = 0;
  uint8_t  pageBuf[2048];
  uint32_t pageBase = 0;
  uint16_t pageLen  = 0;
  bool     first    = true;
  size_t   fileSize = f.size();
  size_t   filePos  = 0;
  uint32_t appStart = 0x08000000;

  const uint16_t BLOCK = 2048;

  auto flushBuf = [&]() -> bool {
    if (!pageLen) return true;
    while (pageLen % 4) pageBuf[pageLen++] = 0xFF;
    // Set address then write in BLOCK-sized chunks
    if (!dfuSetAddress(pageBase)) {
      setMsg("DFU SetAddr 0x" + String(pageBase, HEX) + " failed");
      return false;
    }
    uint16_t off = 0;
    uint16_t blk = 2; // block numbers start at 2
    while (off < pageLen) {
      uint16_t chunk = min((uint16_t)(pageLen - off), (uint16_t)BLOCK);
      if (!dfuWriteBlock(blk, pageBuf + off, chunk)) {
        setMsg("Запис blk " + String(blk) + " @ 0x" + String(pageBase + off, HEX) + " failed");
        return false;
      }
      off += chunk; blk++;
    }
    return true;
  };

  setMsg("Запис прошивки...", 5);

  while (f.available()) {
    String line = f.readStringUntil('\n');
    filePos += line.length() + 1;
    g.progress = 5 + (int)(90.0f * filePos / fileSize);
    line.trim();
    if (line.length() < 11 || line[0] != ':') continue;

    auto hb = [&](int p) -> uint8_t {
      char h[3] = {line[p], line[p+1], 0};
      return (uint8_t)strtol(h, nullptr, 16);
    };
    uint8_t  bc  = hb(1);
    uint16_t off = ((uint16_t)hb(3) << 8) | hb(5);
    uint8_t  rt  = hb(7);

    if (rt == 0x01) break;
    if (rt == 0x04) { extAddr = ((uint32_t)hb(9)<<24)|((uint32_t)hb(11)<<16); continue; }
    if (rt != 0x00) continue;

    uint32_t addr = extAddr | off;
    if (first) { appStart = addr; first = false; }

    for (uint8_t i = 0; i < bc; i++) {
      uint32_t wa = addr + i;
      if (pageLen > 0 && wa != pageBase + pageLen) {
        if (!flushBuf()) { f.close(); return false; }
        pageBase = wa; pageLen = 0;
      }
      if (pageLen == 0) pageBase = wa;
      pageBuf[pageLen++] = hb(9 + i*2);
      if (pageLen == sizeof(pageBuf)) {
        if (!flushBuf()) { f.close(); return false; }
        pageBase += sizeof(pageBuf); pageLen = 0;
      }
    }
    yield();
  }
  if (!flushBuf()) { f.close(); return false; }
  f.close();

  setMsg("Запуск прошивки...", 98);
  dfuLeave(appStart);
  vTaskDelay(pdMS_TO_TICKS(500));

  setMsg("FC прошито успішно!", 100);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Betaflight CLI via USB CDC
// ─────────────────────────────────────────────────────────────────────────────
static bool enterBfCli() {
  if (!cdcDev) return false;
  cdcFlushRx();
  cdcPrint("\r\n#\r\n");
  String r = cdcReadUntil("#", 700);
  return r.indexOf('#') >= 0;
}

static bool restoreDump() {
  if (!cdcDev) { setMsg("FC не підключено — Connect your FC dodik"); return false; }
  File f = SPIFFS.open("/dump.txt", "r");
  if (!f) { setMsg("dump не знайдено"); return false; }

  setMsg("Вхід в BF CLI...", 2);
  if (!enterBfCli()) { f.close(); setMsg("BF CLI не відповідає"); return false; }

  size_t total = 0, done = 0;
  while (f.available()) { f.readStringUntil('\n'); total++; }
  f.seek(0);

  setMsg("Відправка дампу...");
  while (f.available()) {
    String line = f.readStringUntil('\n'); line.trim();
    if (!line.length() || line[0] == '#') { done++; continue; }
    cdcPrint(line + "\r\n");
    // drain response
    size_t sz; void* p;
    uint32_t t = millis();
    while (millis() - t < 40) {
      p = xRingbufferReceive(cdcRxRing, &sz, pdMS_TO_TICKS(10));
      if (p) vRingbufferReturnItem(cdcRxRing, p);
    }
    done++;
    g.progress = (int)(100.0f * done / total);
    if (done % 60 == 0) setMsg("Дамп " + String(done) + "/" + String(total));
    yield();
  }
  f.close();
  cdcPrint("save\r\n");
  vTaskDelay(pdMS_TO_TICKS(2500));
  setMsg("Дамп застосовано", 100);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  esptool SLIP (ELRS via BF serialpassthrough через USB CDC)
// ─────────────────────────────────────────────────────────────────────────────
#define SLIP_END     0xC0
#define SLIP_ESC     0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

static void slipWriteCdc(const uint8_t* data, size_t len) {
  uint8_t buf[len * 2 + 2];
  size_t idx = 0;
  buf[idx++] = SLIP_END;
  for (size_t i = 0; i < len; i++) {
    if      (data[i] == SLIP_END) { buf[idx++] = SLIP_ESC; buf[idx++] = SLIP_ESC_END; }
    else if (data[i] == SLIP_ESC) { buf[idx++] = SLIP_ESC; buf[idx++] = SLIP_ESC_ESC; }
    else buf[idx++] = data[i];
  }
  buf[idx++] = SLIP_END;
  cdcWrite(buf, idx);
}

static bool slipReadCdc(uint8_t* out, size_t* outLen, uint32_t timeout = 2500) {
  uint32_t t = millis();
  bool inPkt = false;
  size_t idx = 0;
  bool esc   = false;
  while (millis() - t < timeout) {
    size_t sz;
    uint8_t* p = (uint8_t*)xRingbufferReceive(cdcRxRing, &sz, pdMS_TO_TICKS(30));
    if (!p) continue;
    for (size_t i = 0; i < sz; i++) {
      uint8_t b = p[i];
      if (b == SLIP_END) {
        if (inPkt && idx > 0) { vRingbufferReturnItem(cdcRxRing, p); *outLen = idx; return true; }
        inPkt = true; idx = 0; esc = false; continue;
      }
      if (!inPkt) continue;
      if (b == SLIP_ESC) { esc = true; continue; }
      if (esc) { b = (b == SLIP_ESC_END) ? SLIP_END : SLIP_ESC; esc = false; }
      if (idx < 512) out[idx++] = b;
    }
    vRingbufferReturnItem(cdcRxRing, p);
  }
  return false;
}

static bool espCmdCdc(uint8_t cmd, const uint8_t* data, uint16_t dLen,
                       uint32_t chk = 0, uint32_t timeout = 2500) {
  size_t pLen = 8 + dLen;
  uint8_t pkt[pLen];
  pkt[0]=0x00; pkt[1]=cmd;
  pkt[2]=dLen&0xFF; pkt[3]=(dLen>>8)&0xFF;
  pkt[4]=chk&0xFF; pkt[5]=(chk>>8)&0xFF;
  pkt[6]=(chk>>16)&0xFF; pkt[7]=(chk>>24)&0xFF;
  if (dLen) memcpy(pkt+8, data, dLen);
  slipWriteCdc(pkt, pLen);
  uint8_t resp[512]; size_t rLen = 0;
  if (!slipReadCdc(resp, &rLen, timeout)) return false;
  if (rLen < 10) return false;
  return resp[1] == cmd && resp[rLen-2] == 0x00;
}

static bool espSyncCdc() {
  uint8_t sd[36] = {0x07,0x07,0x12,0x20};
  memset(sd+4, 0x55, 32);
  for (int i = 0; i < 7; i++) {
    espCmdCdc(0x08, sd, 36, 0, 600);
    uint8_t r[64]; size_t l = 0;
    if (slipReadCdc(r, &l, 400) && l >= 10 && r[1]==0x08 && r[l-2]==0) return true;
    vTaskDelay(pdMS_TO_TICKS(30));
  }
  return false;
}

static uint32_t espChk(const uint8_t* d, size_t l) {
  uint32_t cs = 0xEF;
  for (size_t i = 0; i < l; i++) cs ^= d[i];
  return cs;
}

static bool flashElrs() {
  if (!cdcDev) { setMsg("FC не підключено — Connect your FC dodik"); return false; }
  File f = SPIFFS.open("/elrs.bin", "r");
  if (!f) { setMsg("ELRS bin не знайдено"); return false; }
  size_t fileSize = f.size();

  setMsg("Вхід в BF CLI...", 2);
  if (!enterBfCli()) { f.close(); setMsg("BF CLI не відповідає"); return false; }

  setMsg("BF serialpassthrough UART" + String(g_elrsUart) + "...", 6);
  cdcFlushRx();
  cdcPrint("serialpassthrough " + String(g_elrsUart-1) + " " + String(g_elrsBaud) + " rxtx\r\n");
  vTaskDelay(pdMS_TO_TICKS(500));
  cdcFlushRx();

  setMsg("Синхронізація ELRS...", 10);
  if (!espSyncCdc()) {
    f.close();
    setMsg("ELRS sync failed — перевір RX на FC UART" + String(g_elrsUart));
    return false;
  }
  addLog("ELRS sync OK, " + String(fileSize) + " байт");

  const uint32_t BLOCK_SZ = 0x4000, CHUNK_SZ = 0x0200;
  uint32_t numPkts = (fileSize + CHUNK_SZ-1) / CHUNK_SZ;
  uint32_t eraseSz = ((fileSize + BLOCK_SZ-1) / BLOCK_SZ) * BLOCK_SZ;
  uint32_t flashOff = 0;
  uint8_t beginD[16];
  memcpy(beginD,    &eraseSz,  4);
  memcpy(beginD+4,  &numPkts,  4);
  uint32_t cs = CHUNK_SZ;
  memcpy(beginD+8,  &cs,       4);
  memcpy(beginD+12, &flashOff, 4);
  if (!espCmdCdc(0x02, beginD, 16, 0, 5000)) {
    f.close(); setMsg("FLASH_BEGIN failed"); return false;
  }

  setMsg("Запис ELRS...", 12);
  uint8_t chunk[CHUNK_SZ + 16];
  uint32_t seq = 0; size_t written = 0;
  while (f.available()) {
    memset(chunk+16, 0xFF, CHUNK_SZ);
    f.read(chunk+16, CHUNK_SZ);
    written += CHUNK_SZ;
    uint32_t dsz = CHUNK_SZ, z = 0;
    memcpy(chunk,    &dsz, 4); memcpy(chunk+4, &seq, 4);
    memcpy(chunk+8,  &z,   4); memcpy(chunk+12,&z,   4);
    uint32_t ck = espChk(chunk+16, CHUNK_SZ);
    if (!espCmdCdc(0x03, chunk, CHUNK_SZ+16, ck, 3000)) {
      f.close(); setMsg("FLASH_DATA seq=" + String(seq) + " failed"); return false;
    }
    seq++;
    g.progress = 12 + (int)(85.0f * min(written, fileSize) / fileSize);
    yield();
  }
  f.close();
  uint8_t endD[4] = {};
  espCmdCdc(0x04, endD, 4, 0, 2000);
  setMsg("ELRS прошито успішно", 100);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Flash task
// ─────────────────────────────────────────────────────────────────────────────
struct FlashParams { bool doHex; bool doDump; bool doElrs; };
static FlashParams flashParams;

static void flashTaskFn(void* pv) {
  auto* p = (FlashParams*)pv;
  bool ok = true;
  addLog("=== Початок прошивки ===");

  if (p->doHex && ok) {
    setMsg("Прошивка FC (DFU hex)...", 0);
    ok = flashHex();
    if (ok) { g.progress = 100; vTaskDelay(pdMS_TO_TICKS(800)); }
  }
  if (p->doDump && ok) {
    setMsg("Відновлення дампу...", 0);
    // Wait for CDC to come back after possible reboot
    for (int i = 0; i < 30 && !cdcDev; i++) vTaskDelay(pdMS_TO_TICKS(500));
    ok = restoreDump();
    if (ok) { g.progress = 100; vTaskDelay(pdMS_TO_TICKS(500)); }
  }
  if (p->doElrs && ok) {
    setMsg("Прошивка ELRS...", 0);
    ok = flashElrs();
    if (ok) { g.progress = 100; vTaskDelay(pdMS_TO_TICKS(500)); }
  }

  addLog(ok ? "=== Готово ===" : "=== ПОМИЛКА: " + g.message + " ===");
  if (ok) setMsg("Готово!");
  g.flashing = false;
  flashTask = nullptr;
  vTaskDelete(nullptr);
}

// ─────────────────────────────────────────────────────────────────────────────
//  USB Host task
// ─────────────────────────────────────────────────────────────────────────────
static void usbHostTask(void*) {
  // Install USB host
  usb_host_config_t hcfg = {
    .skip_phy_setup     = false,
    .intr_flags         = ESP_INTR_FLAG_LEVEL1,
  };
  usb_host_install(&hcfg);

  // Install CDC ACM class driver
  cdc_acm_host_driver_config_t cdcCfg = {
    .driver_task_priority = 5,
    .driver_task_stack_size = 4096,
    .xCoreID = 0,
  };
  cdc_acm_host_install(&cdcCfg);

  addLog("USB Host ready");

  while (true) {
    uint32_t evFlags;
    usb_host_lib_handle_events(pdMS_TO_TICKS(200), &evFlags);

    if (evFlags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
      usb_host_device_free_all();
    }

    // Try to open CDC device if not already open
    if (g.usbMode == USB_NONE && !g.flashing) {
      if (xSemaphoreTake(cdcMux, 0) == pdTRUE) {
        cdc_acm_host_device_config_t devCfg = {
          .connection_timeout_ms = 200,
          .out_buffer_size       = 4096,
          .in_buffer_size        = 4096,
          .event_cb              = cdcEventCb,
          .data_cb               = cdcRxCb,
          .user_arg              = nullptr,
        };
        cdc_acm_dev_hdl_t dev = nullptr;
        esp_err_t r = cdc_acm_host_open(0, 0, 0, &devCfg, &dev);
        if (r == ESP_OK && dev) {
          cdcDev    = dev;
          g.usbMode = USB_CDC;
          cdc_acm_host_set_control_line_state(dev, true, true);
          cdc_acm_line_coding_t lc = {115200, 0, 0, 8};
          cdc_acm_host_line_coding_set(dev, &lc);
          addLog("CDC: FC підключено (BF CDC)");
        }
        xSemaphoreGive(cdcMux);
      }
    }

    // Try to detect DFU device
    if (g.usbMode == USB_NONE && !dfuDevHdl && !g.flashing) {
      usb_device_handle_t devHdl = nullptr;
      if (usb_host_device_open(nullptr, 1, &devHdl) == ESP_OK) {
        usb_device_info_t info;
        if (usb_host_get_device_info(devHdl, &info) == ESP_OK) {
          const usb_device_desc_t* desc = info.dev_desc;
          if (desc->idVendor == DFU_VID && desc->idProduct == DFU_PID) {
            usb_host_interface_claim(nullptr, devHdl, DFU_INTF, 0);
            dfuDevHdl  = devHdl;
            dfuIntfNum = DFU_INTF;
            g.usbMode  = USB_DFU;
            addLog("DFU: FC в DFU режимі (VID=0483 PID=DF11)");
          } else {
            usb_host_device_close(nullptr, devHdl);
          }
        } else {
          usb_host_device_close(nullptr, devHdl);
        }
      }
    }

    // DFU device gone?
    if (g.usbMode == USB_DFU && dfuDevHdl) {
      usb_device_info_t info;
      if (usb_host_get_device_info(dfuDevHdl, &info) != ESP_OK) {
        usb_host_interface_release(nullptr, dfuDevHdl, DFU_INTF);
        usb_host_device_close(nullptr, dfuDevHdl);
        dfuDevHdl = nullptr;
        g.usbMode = USB_NONE;
        addLog("DFU: FC відключено");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Upload helper
// ─────────────────────────────────────────────────────────────────────────────
static void handleUpload(const String& path,
                          AsyncWebServerRequest*, const String& fn,
                          size_t index, uint8_t* data, size_t len, bool final) {
  if (!index) { if (uploadFile) uploadFile.close(); uploadFile = SPIFFS.open(path,"w"); }
  if (uploadFile) uploadFile.write(data, len);
  if (final && uploadFile) { uploadFile.close(); addLog("Файл: " + fn + " → " + path); }
}

// ─────────────────────────────────────────────────────────────────────────────
//  HTML
// ─────────────────────────────────────────────────────────────────────────────
static const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html lang="uk"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>DroneFlasher S3</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#111;color:#eee;font-family:'Segoe UI',sans-serif;
     display:flex;flex-direction:column;align-items:center;
     min-height:100vh;padding:24px 12px;gap:14px}
#usb-badge{padding:8px 28px;border-radius:999px;font-size:1rem;
  font-weight:700;letter-spacing:.06em;background:#222;
  transition:color .3s,box-shadow .3s}
#usb-badge.dfu{color:#ffd84b;box-shadow:0 0 12px #ffd84b55}
#usb-badge.cdc{color:#b0ff4b;box-shadow:0 0 12px #b0ff4b55}
#usb-badge.none{color:#ff4b4b;box-shadow:0 0 12px #ff4b4b33}
.card{background:#d0d0d0;border-radius:18px;padding:24px 20px;
      width:100%;max-width:450px}
.ct{font-size:.72rem;font-weight:700;color:#555;
    text-transform:uppercase;letter-spacing:.08em;margin-bottom:12px}
.row{display:flex;align-items:center;gap:10px;margin-bottom:14px}
.lbl{background:#444;color:#fff;border-radius:8px;padding:7px 14px;
     font-size:.95rem;font-weight:600;min-width:80px;text-align:center;flex-shrink:0}
.fw{flex:1;display:flex;flex-direction:column;gap:3px}
.fn{font-size:.74rem;color:#555;min-height:14px;word-break:break-all}
.bc{background:#444;color:#fff;border:none;border-radius:8px;
    padding:7px 12px;cursor:pointer;font-size:.88rem;font-weight:600;width:100%}
.bc:hover{background:#555}.bc.on{background:#3d6b12;color:#b0ff4b}
input[type=file]{display:none}
.sep{height:1px;background:#bbb;margin:6px 0 14px}
.rc{display:flex;align-items:center;gap:8px;margin-bottom:9px}
.cl{font-size:.82rem;color:#444;font-weight:600;min-width:155px}
select{background:#444;color:#fff;border:none;border-radius:8px;
       padding:5px 10px;font-size:.88rem}
#btn-flash{display:block;width:100%;padding:16px;margin-top:8px;
  background:#333;color:#b0ff4b;border:none;border-radius:14px;
  font-size:1.15rem;font-weight:800;letter-spacing:.08em;cursor:pointer}
#btn-flash:hover:not(:disabled){background:#444}
#btn-flash:disabled{opacity:.4;cursor:not-allowed}
#pw{margin-top:12px;display:none}
#bb{background:#bbb;border-radius:8px;height:12px;overflow:hidden}
#bf{height:100%;background:#b0ff4b;border-radius:8px;width:0%;transition:width .3s}
#mg{margin-top:6px;font-size:.82rem;color:#333;text-align:center;
    min-height:17px;word-break:break-all}
.lnk{color:#555;font-size:.76rem;text-decoration:none;margin-top:2px}
.lnk:hover{color:#b0ff4b}
.hint{font-size:.72rem;color:#666;text-align:center;margin-top:4px;line-height:1.4}
</style></head><body>

<div id="usb-badge" class="none">USB: —</div>

<div class="card">
<div class="ct">Прошивка</div>

<div class="row">
  <div class="lbl">hex:</div>
  <div class="fw">
    <button class="bc" id="bh" onclick="pick('hex')">ВИБРАТИ</button>
    <input type="file" id="ih" accept=".hex">
    <div class="fn" id="nh"></div>
  </div>
</div>
<div class="row">
  <div class="lbl">dump:</div>
  <div class="fw">
    <button class="bc" id="bd" onclick="pick('dump')">ВИБРАТИ</button>
    <input type="file" id="id" accept=".txt,.cli,.dump">
    <div class="fn" id="nd"></div>
  </div>
</div>
<div class="row">
  <div class="lbl">RX:</div>
  <div class="fw">
    <button class="bc" id="be" onclick="pick('elrs')">ВИБРАТИ</button>
    <input type="file" id="ie" accept=".bin">
    <div class="fn" id="ne"></div>
  </div>
</div>

<div class="sep"></div>
<div class="ct">ELRS passthrough</div>
<div class="rc">
  <span class="cl">UART FC (BF):</span>
  <select id="su"><option value="1">UART1</option><option value="2">UART2</option>
  <option value="3">UART3</option><option value="4" selected>UART4</option>
  <option value="5">UART5</option><option value="6">UART6</option></select>
</div>
<div class="rc">
  <span class="cl">Бодрейт ELRS:</span>
  <select id="sb"><option value="115200">115200</option>
  <option value="420000" selected>420000</option>
  <option value="460800">460800</option><option value="921600">921600</option></select>
</div>

<button id="btn-flash" disabled onclick="go()">ПРОШИТИ</button>
<div id="pw"><div id="bb"><div id="bf"></div></div><div id="mg"></div></div>
<div class="hint" id="hint"></div>
</div>

<a class="lnk" href="/ota">OTA оновлення ESP32/STM32 &rsaquo;</a>
<a class="lnk" href="/log" target="_blank">Лог прошивки &rsaquo;</a>

<script>
const F={hex:null,dump:null,elrs:null};
const MAP={hex:['bh','ih','nh'],dump:['bd','id','nd'],elrs:['be','ie','ne']};
function pick(k){document.getElementById(MAP[k][1]).click();}
Object.keys(MAP).forEach(k=>{
  document.getElementById(MAP[k][1]).onchange=function(){
    if(this.files[0]){F[k]=this.files[0];
      document.getElementById(MAP[k][2]).textContent=this.files[0].name;
      document.getElementById(MAP[k][0]).classList.add('on');}
    else{F[k]=null;document.getElementById(MAP[k][2]).textContent='';
      document.getElementById(MAP[k][0]).classList.remove('on');}
    upd();
  };
});
function upd(){document.getElementById('btn-flash').disabled=!(F.hex||F.dump||F.elrs);}

async function up(k,f){
  const fd=new FormData();fd.append('file',f,f.name);
  return (await fetch('/upload/'+k,{method:'POST',body:fd})).ok;
}
async function go(){
  const btn=document.getElementById('btn-flash');
  btn.disabled=true;
  document.getElementById('pw').style.display='block';
  sm('Завантаження файлів...');sp(0);
  for(const[k,f] of Object.entries(F)){
    if(f){sm('Завантаження: '+f.name);
      if(!await up(k,f)){sm('Помилка: '+f.name);btn.disabled=false;return;}}
  }
  sm('Запуск...');
  const r=await fetch('/flash',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({hex:!!F.hex,dump:!!F.dump,elrs:!!F.elrs,
      elrsUart:+document.getElementById('su').value,
      elrsBaud:+document.getElementById('sb').value})});
  if(!r.ok){sm('Помилка');btn.disabled=false;return;}
  const iv=setInterval(async()=>{
    const d=await(await fetch('/status')).json();
    sp(d.progress);sm(d.message);
    if(!d.flashing){clearInterval(iv);btn.disabled=false;}
  },600);
}
function sp(p){document.getElementById('bf').style.width=p+'%';}
function sm(t){document.getElementById('mg').textContent=t;}

(function poll(){
  fetch('/status').then(r=>r.json()).then(d=>{
    const el=document.getElementById('usb-badge');
    const hints={
      none:'Підключи FC через USB Type-C (OTG порт ESP32)',
      cdc:'FC підключено (Betaflight CDC) — готово до dump/ELRS/hex→DFU',
      dfu:'FC в DFU режимі — готово до прошивки hex'
    };
    el.textContent='USB: '+d.usbMode.toUpperCase();
    el.className=d.usbMode;
    document.getElementById('hint').textContent=hints[d.usbMode]||'';
    if(!d.flashing&&d.message)sm(d.message);
    if(d.flashing)sp(d.progress);
  }).catch(()=>{});
  setTimeout(poll,1500);
})();
</script></body></html>
)rawhtml";

// ─────────────────────────────────────────────────────────────────────────────
//  OTA сторінка
// ─────────────────────────────────────────────────────────────────────────────
static const char OTA_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html lang="uk"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>OTA</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#111;color:#eee;font-family:'Segoe UI',sans-serif;
     display:flex;flex-direction:column;align-items:center;
     min-height:100vh;padding:32px 12px;gap:16px}
.tabs{display:flex;gap:8px;width:100%;max-width:440px}
.tab{flex:1;padding:9px;border:none;border-radius:10px;font-size:.92rem;
     font-weight:700;cursor:pointer;background:#2a2a2a;color:#888}
.tab.active{background:#333;color:#b0ff4b}
.panel{display:none;width:100%;max-width:440px}
.panel.show{display:flex;flex-direction:column;gap:14px}
.card{background:#d0d0d0;border-radius:18px;padding:26px 22px;
      display:flex;flex-direction:column;gap:12px}
.ct{font-size:.72rem;font-weight:700;color:#555;text-transform:uppercase;letter-spacing:.08em}
.drop{border:2px dashed #888;border-radius:12px;padding:26px 14px;
      text-align:center;cursor:pointer;color:#555;font-size:.9rem;
      transition:border-color .2s,color .2s;user-select:none}
.drop.over,.drop:hover{border-color:#b0ff4b;color:#333}
input[type=file]{display:none}
.fn{font-size:.76rem;color:#444;text-align:center;min-height:15px;word-break:break-all}
.btn{width:100%;padding:14px;background:#333;color:#b0ff4b;border:none;
     border-radius:12px;font-size:1rem;font-weight:800;cursor:pointer}
.btn:hover:not(:disabled){background:#444}.btn:disabled{opacity:.4;cursor:not-allowed}
.bg{background:#bbb;border-radius:8px;height:12px;overflow:hidden;display:none}
.bf{height:100%;background:#b0ff4b;border-radius:8px;width:0%;transition:width .3s}
.st{text-align:center;font-size:.83rem;color:#333;min-height:18px;word-break:break-all}
.note{font-size:.72rem;color:#666;text-align:center;line-height:1.4}
.back{color:#555;font-size:.76rem;text-decoration:none}.back:hover{color:#b0ff4b}
</style></head><body>
<div class="tabs">
  <button class="tab active" onclick="sw('esp')">ESP32</button>
  <button class="tab" onclick="sw('stm')">STM32 (FC)</button>
</div>
<div class="panel show" id="pe">
  <div class="card">
    <div class="ct">Оновлення ESP32</div>
    <div class="drop" id="de" onclick="document.getElementById('ie').click()">
      Перетягни або вибери <b>.bin</b></div>
    <input type="file" id="ie" accept=".bin">
    <div class="fn" id="fe"></div>
    <button class="btn" id="be" disabled onclick="doEsp()">ОНОВИТИ ESP32</button>
    <div class="bg" id="bge"><div class="bf" id="bfe"></div></div>
    <div class="st" id="se"></div>
    <div class="note">Файл: Sketch → Export Compiled Binary → .ino.bin</div>
  </div>
</div>
<div class="panel" id="ps">
  <div class="card">
    <div class="ct">Прошивка STM32 (через DFU)</div>
    <div class="drop" id="ds" onclick="document.getElementById('is').click()">
      Перетягни або вибери <b>.hex</b></div>
    <input type="file" id="is" accept=".hex">
    <div class="fn" id="fs"></div>
    <button class="btn" id="bs" disabled onclick="doStm()">ПРОШИТИ STM32</button>
    <div class="bg" id="bgs"><div class="bf" id="bfs"></div></div>
    <div class="st" id="ss"></div>
    <div class="note">Підключи FC через USB Type-C в DFU режимі (затисни BOOT)</div>
  </div>
</div>
<a class="back" href="/">← Назад</a>
<script>
function sw(t){
  ['esp','stm'].forEach((x,i)=>{
    document.querySelectorAll('.tab')[i].classList.toggle('active',x===t);
    document.getElementById('p'+x).classList.toggle('show',x===t);
  });
}
function mkDrop(did,iid,fid,bid){
  const d=document.getElementById(did),i=document.getElementById(iid);
  let file=null;
  i.onchange=function(){if(this.files[0])set(this.files[0]);};
  d.ondragover=e=>{e.preventDefault();d.classList.add('over');};
  d.ondragleave=()=>d.classList.remove('over');
  d.ondrop=e=>{e.preventDefault();d.classList.remove('over');
    if(e.dataTransfer.files[0])set(e.dataTransfer.files[0]);};
  function set(f){file=f;
    document.getElementById(fid).textContent=f.name+' ('+Math.round(f.size/1024)+' KB)';
    document.getElementById(bid).disabled=false;}
  return()=>file;
}
const ge=mkDrop('de','ie','fe','be');
const gs=mkDrop('ds','is','fs','bs');

function xhr(url,file,onProg,onDone){
  const fd=new FormData();fd.append('file',file,file.name);
  const x=new XMLHttpRequest();x.open('POST',url);
  x.upload.onprogress=e=>{if(e.lengthComputable)onProg(Math.round(e.loaded/e.total*100));};
  x.onload=()=>{try{onDone(JSON.parse(x.responseText));}catch(e){onDone({ok:true});}};
  x.onerror=()=>onDone({ok:false,err:'connection error'});
  x.send(fd);
}
function doEsp(){
  const f=ge();if(!f)return;
  document.getElementById('be').disabled=true;
  document.getElementById('bge').style.display='block';
  st('e','Завантаження...');
  xhr('/ota/upload',f,p=>{
    document.getElementById('bfe').style.width=p+'%';st('e','Завантажено: '+p+'%');
  },d=>{
    d.ok?st('e','Готово! ESP32 перезавантажується...')
        :st('e','Помилка: '+(d.err||'?'));
    if(!d.ok)document.getElementById('be').disabled=false;
  });
}
async function doStm(){
  const f=gs();if(!f)return;
  document.getElementById('bs').disabled=true;
  document.getElementById('bgs').style.display='block';
  st('s','Завантаження hex...');
  const fd=new FormData();fd.append('file',f,f.name);
  let r=await fetch('/upload/hex',{method:'POST',body:fd});
  if(!r.ok){st('s','Помилка завантаження');document.getElementById('bs').disabled=false;return;}
  st('s','Запуск DFU прошивки...');
  r=await fetch('/flash',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({hex:true,dump:false,elrs:false})});
  if(!r.ok){st('s','Помилка');document.getElementById('bs').disabled=false;return;}
  const iv=setInterval(async()=>{
    const d=await(await fetch('/status')).json();
    document.getElementById('bfs').style.width=d.progress+'%';
    st('s',d.message);
    if(!d.flashing){clearInterval(iv);document.getElementById('bs').disabled=false;}
  },700);
}
function st(p,t){document.getElementById('s'+p).textContent=t;}
</script></body></html>
)rawhtml";

// ─────────────────────────────────────────────────────────────────────────────
//  setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  logMux    = xSemaphoreCreateMutex();
  cdcMux    = xSemaphoreCreateMutex();
  cdcRxSem  = xSemaphoreCreateBinary();
  cdcRxRing = xRingbufferCreate(8192, RINGBUF_TYPE_BYTEBUF);

  pinMode(LED_PIN, OUTPUT);
  // Startup blink
  for (int i = 0; i < 6; i++) { digitalWrite(LED_PIN, i%2); delay(100); }

  SPIFFS.begin(true);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(10,0,0,1), IPAddress(10,0,0,1), IPAddress(255,255,255,0));
  WiFi.softAP(AP_SSID, AP_PASS);
  addLog("AP: 10.0.0.1  SSID=" + String(AP_SSID));

  // ── Routes ────────────────────────────────────────────────────────────────
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){
    r->send_P(200,"text/html",INDEX_HTML); });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest* r){
    StaticJsonDocument<128> doc;
    const char* modes[] = {"none","dfu","cdc"};
    doc["usbMode"]  = modes[(int)g.usbMode];
    doc["flashing"] = g.flashing;
    doc["progress"] = g.progress;
    doc["message"]  = g.message;
    String o; serializeJson(doc,o);
    r->send(200,"application/json",o);
  });

  server.on("/log", HTTP_GET, [](AsyncWebServerRequest* r){
    String page = "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<style>*{box-sizing:border-box;margin:0;padding:0}"
      "body{background:#0d0d0d;color:#b0ff4b;font-family:monospace;"
      "padding:14px;font-size:.8rem;line-height:1.6}"
      ".bar{display:flex;gap:10px;margin-bottom:12px;align-items:center}"
      "a{color:#888;text-decoration:none;font-size:.76rem}"
      "a:hover{color:#b0ff4b}"
      "button{background:#222;color:#b0ff4b;border:none;border-radius:6px;"
      "padding:5px 12px;cursor:pointer;font-size:.76rem}"
      "pre{white-space:pre-wrap;word-break:break-all}</style></head><body>"
      "<div class='bar'><a href='/'>← Назад</a>"
      "<button onclick='location.reload()'>Оновити</button></div>"
      "<pre>";
    if (xSemaphoreTake(logMux, pdMS_TO_TICKS(100)) == pdTRUE) {
      page += flashLog; xSemaphoreGive(logMux);
    }
    page += "</pre><script>window.scrollTo(0,document.body.scrollHeight);"
            "setTimeout(()=>location.reload(),2500);</script></body></html>";
    r->send(200,"text/html",page);
  });

  auto mkUpload = [](const String& path, bool& ready){
    return [path,&ready](AsyncWebServerRequest* r, const String& fn,
                          size_t i, uint8_t* d, size_t l, bool f){
      if(g.flashing)return;
      handleUpload(path,r,fn,i,d,l,f);
      if(f)ready=true;
    };
  };
  server.on("/upload/hex",  HTTP_POST,[](AsyncWebServerRequest* r){r->send(200);},
    mkUpload("/hex.hex", g.hexReady));
  server.on("/upload/dump", HTTP_POST,[](AsyncWebServerRequest* r){r->send(200);},
    mkUpload("/dump.txt",g.dumpReady));
  server.on("/upload/elrs", HTTP_POST,[](AsyncWebServerRequest* r){r->send(200);},
    mkUpload("/elrs.bin",g.elrsReady));

  server.on("/flash", HTTP_POST,
    [](AsyncWebServerRequest* r){
      if(g.flashing){r->send(423,"text/plain","busy");return;}
      g.flashing=true; g.progress=0; setMsg("Підготовка...");
      xTaskCreate(flashTaskFn,"flash",10240,&flashParams,1,&flashTask);
      r->send(200,"text/plain","ok");
    },
    nullptr,
    [](AsyncWebServerRequest*, uint8_t* data, size_t len, size_t, size_t){
      StaticJsonDocument<128> doc;
      deserializeJson(doc,data,len);
      flashParams.doHex  = doc["hex"]  | false;
      flashParams.doDump = doc["dump"] | false;
      flashParams.doElrs = doc["elrs"] | false;
      g_elrsUart = doc["elrsUart"] | 4;
      g_elrsBaud = doc["elrsBaud"] | 420000;
    }
  );

  server.on("/ota", HTTP_GET,[](AsyncWebServerRequest* r){
    r->send_P(200,"text/html",OTA_HTML); });

  server.on("/ota/upload", HTTP_POST,
    [](AsyncWebServerRequest* r){
      bool ok=!Update.hasError();
      auto* resp=r->beginResponse(200,"application/json",
        ok?"{\"ok\":true}":"{\"ok\":false,\"err\":\""+String(Update.errorString())+"\"}");
      resp->addHeader("Connection","close");
      r->send(resp);
      if(ok){delay(400);ESP.restart();}
    },
    [](AsyncWebServerRequest*, const String& fn, size_t idx,
       uint8_t* d, size_t l, bool final){
      if(!idx){addLog("OTA: "+fn);
        Update.begin((ESP.getFreeSketchSpace()-0x1000)&0xFFFFF000);}
      if(Update.isRunning())Update.write(d,l);
      if(final){Update.end(true);
        addLog(Update.hasError()?"OTA fail: "+String(Update.errorString()):"OTA OK");}
    }
  );

  server.begin();
  addLog("Server started");

  // USB Host task (Core 0)
  xTaskCreatePinnedToCore(usbHostTask, "usb", 8192, nullptr, 6, nullptr, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
//  loop — LED
// ─────────────────────────────────────────────────────────────────────────────
static uint32_t lastBlink = 0;
static bool     ledSt     = false;

void loop() {
  uint32_t now = millis();
  if (g.flashing) {
    if (now-lastBlink>80){lastBlink=now;ledSt=!ledSt;digitalWrite(LED_PIN,ledSt);}
  } else if (g.usbMode != USB_NONE) {
    if (now-lastBlink>900){lastBlink=now;ledSt=!ledSt;digitalWrite(LED_PIN,ledSt);}
  } else {
    static uint8_t ph=0; static uint32_t bt=0;
    const uint16_t pat[]={80,80,80,700};
    if (now-bt>pat[ph%4]){bt=now;ledSt=(ph%4<2);digitalWrite(LED_PIN,ledSt);ph++;}
  }
  delay(5);
}
