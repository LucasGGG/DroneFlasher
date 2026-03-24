/*
 * DroneFlasher S3
 * ESP32-S3-WROOM-1-N16R8  (16MB flash, 8MB PSRAM)
 * WiFi AP: CONFIG / freeAzov  →  http://10.0.0.1
 *
 * ┌─────────────────────────────────────────────────────────┐
 * │  Підключення                                            │
 * │  USB OTG Type-C (GPIO19/20) ──► FC USB Type-C          │
 * │  Паяти нічого не потрібно!                              │
 * └─────────────────────────────────────────────────────────┘
 *
 * Arduino IDE:
 *   Board:             ESP32S3 Dev Module
 *   USB Mode:          Hardware CDC and JTAG   ← ОБОВ'ЯЗКОВО
 *   Flash Size:        16MB
 *   PSRAM:             OPI PSRAM (8MB)
 *   Partition Scheme:  16M Flash (3MB APP/9.9MB FATFS)
 *   CDC On Boot:       Enabled
 *
 * Бібліотеки:
 *   ESP Async WebServer  (mathieucarbou)
 *   Async TCP            (mathieucarbou)
 *   ArduinoJson          (bblanchon v7)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Update.h>

extern "C" {
#include "usb/usb_host.h"
#include "usb/usb_helpers.h"
}
#include <freertos/ringbuf.h>

// ── WiFi ──────────────────────────────────────────────────────────────────────
static const char* AP_SSID = "CONFIG";
static const char* AP_PASS = "freeAzov";

// ── LED ───────────────────────────────────────────────────────────────────────
#define LED_PIN 2

// ── Web server ────────────────────────────────────────────────────────────────
AsyncWebServer server(80);

// ── Log ───────────────────────────────────────────────────────────────────────
#define LOG_MAX 10240
static String            flashLog;
static SemaphoreHandle_t logMux;

static void addLog(const String& s) {
  if (xSemaphoreTake(logMux, pdMS_TO_TICKS(80)) == pdTRUE) {
    flashLog += s + "\n";
    if (flashLog.length() > LOG_MAX)
      flashLog = flashLog.substring(flashLog.length() - LOG_MAX);
    xSemaphoreGive(logMux);
  }
  Serial.println(s);
}

// ── Global state ──────────────────────────────────────────────────────────────
enum UsbMode : uint8_t { USB_NONE, USB_DFU, USB_CDC };

struct State {
  bool    flashing  = false;
  int     progress  = 0;
  String  message   = "Очікування...";
  UsbMode usbMode   = USB_NONE;
  bool    hexReady  = false;
  bool    dumpReady = false;
  bool    elrsReady = false;
};
static State g;

static void setMsg(const String& m, int p = -1) {
  g.message = m;
  if (p >= 0) g.progress = p;
  addLog(m);
}

// ── ELRS UART config (declared early) ────────────────────────────────────────
static uint8_t  g_elrsUart = 4;
static uint32_t g_elrsBaud = 420000;

// ── USB Host state ────────────────────────────────────────────────────────────
#define DFU_VID      0x0483u
#define DFU_PID      0xDF11u
#define BF_VID       0x0483u
#define BF_PID       0x5740u

static usb_host_client_handle_t usbClient   = nullptr;
static usb_device_handle_t      usbDev      = nullptr;
static uint8_t                  usbDevAddr  = 0;
static SemaphoreHandle_t        usbMux;

// DFU
static uint8_t           dfuIntf     = 0;
static SemaphoreHandle_t dfuXferSem;
static esp_err_t         dfuXferErr;
static usb_transfer_t*   dfuXfer     = nullptr;

// CDC
#define CDC_RX_BUF   8192
#define CDC_TX_MPS   512
#define CDC_RX_MPS   512

static uint8_t           cdcIntf     = 0xFF;
static uint8_t           cdcEpOut    = 0xFF;
static uint8_t           cdcEpIn     = 0xFF;
static usb_transfer_t*   cdcTxXfer   = nullptr;
static usb_transfer_t*   cdcRxXfer   = nullptr;
static SemaphoreHandle_t cdcTxSem;
static RingbufHandle_t   cdcRxBuf;

// ── USB Transfer callbacks ────────────────────────────────────────────────────
static void dfuXferCb(usb_transfer_t* xfer) {
  dfuXferErr = xfer->status;
  xSemaphoreGiveFromISR(dfuXferSem, nullptr);
}

static void cdcTxCb(usb_transfer_t* xfer) {
  (void)xfer;
  xSemaphoreGiveFromISR(cdcTxSem, nullptr);
}

static void cdcRxCb(usb_transfer_t* xfer);  // forward decl

static void resubmitRx() {
  if (!cdcRxXfer || cdcEpIn == 0xFF || !usbDev) return;
  cdcRxXfer->device_handle   = usbDev;
  cdcRxXfer->bEndpointAddress = cdcEpIn;
  cdcRxXfer->num_bytes       = CDC_RX_MPS;
  cdcRxXfer->callback        = cdcRxCb;
  cdcRxXfer->context         = nullptr;
  usb_host_transfer_submit(cdcRxXfer);
}

static void cdcRxCb(usb_transfer_t* xfer) {
  if (xfer->status == USB_TRANSFER_STATUS_COMPLETED && xfer->actual_num_bytes > 0)
    xRingbufferSendFromISR(cdcRxBuf, xfer->data_buffer, xfer->actual_num_bytes, nullptr);
  if (xfer->status != USB_TRANSFER_STATUS_CANCELED)
    resubmitRx();
}

// ── USB device open/close helpers ─────────────────────────────────────────────
static void closeUsbDevice() {
  if (!usbDev) return;
  if (cdcRxXfer) { usb_host_transfer_free(cdcRxXfer); cdcRxXfer = nullptr; }
  if (cdcTxXfer) { usb_host_transfer_free(cdcTxXfer); cdcTxXfer = nullptr; }
  if (dfuXfer)   { usb_host_transfer_free(dfuXfer);   dfuXfer   = nullptr; }
  if (cdcIntf != 0xFF)
    usb_host_interface_release(usbClient, usbDev, cdcIntf);
  if (g.usbMode == USB_DFU)
    usb_host_interface_release(usbClient, usbDev, dfuIntf);
  usb_host_device_close(usbClient, usbDev);
  usbDev = nullptr;
  cdcIntf = cdcEpOut = cdcEpIn = 0xFF;
  g.usbMode = USB_NONE;
  addLog("USB: пристрій відключено");
}

static void openUsbDevice(uint8_t addr) {
  usb_device_handle_t dev = nullptr;
  if (usb_host_device_open(usbClient, addr, &dev) != ESP_OK) return;

  const usb_device_desc_t* desc = nullptr;
  usb_host_get_device_descriptor(dev, &desc);
  uint16_t vid = desc->idVendor, pid = desc->idProduct;
  addLog("USB: VID=" + String(vid, HEX) + " PID=" + String(pid, HEX));

  // ── DFU ──────────────────────────────────────────────────────────────────
  if (vid == DFU_VID && pid == DFU_PID) {
    usb_host_interface_claim(usbClient, dev, 0, 0);
    // Pre-allocate control transfer buffer (max 64B data)
    usb_host_transfer_alloc(8 + 64, 0, &dfuXfer);
    dfuIntf  = 0;
    usbDev   = dev;
    usbDevAddr = addr;
    g.usbMode = USB_DFU;
    addLog("DFU: FC в режимі прошивки");
    return;
  }

  // ── CDC / USB-UART ────────────────────────────────────────────────────────
  const usb_config_desc_t* cfg = nullptr;
  usb_host_get_active_config_descriptor(dev, &cfg);
  if (!cfg) { usb_host_device_close(usbClient, dev); return; }

  // Find CDC data interface (class 0x0A) or CDC ACM comm interface (0x02)
  uint8_t dataIntf = 0xFF, epOut = 0xFF, epIn = 0xFF;
  for (uint8_t i = 0; i < cfg->bNumInterfaces; i++) {
    int offset = 0;
    const usb_intf_desc_t* intf = usb_parse_interface_descriptor(cfg, i, 0, &offset);
    if (!intf) continue;
    if (intf->bInterfaceClass != 0x0A && intf->bInterfaceClass != 0x02) continue;
    // Scan endpoints of this interface for bulk IN and OUT
    for (int ep = 0; ep < intf->bNumEndpoints; ep++) {
      int epOff = offset;
      const usb_ep_desc_t* epd = usb_parse_endpoint_descriptor_by_index(intf, ep,
                                    cfg->wTotalLength, &epOff);
      if (!epd) continue;
      if ((epd->bmAttributes & 0x03) != 0x02) continue;  // bulk only
      if (epd->bEndpointAddress & 0x80) epIn  = epd->bEndpointAddress;
      else                               epOut = epd->bEndpointAddress;
    }
    if (epIn != 0xFF || epOut != 0xFF) { dataIntf = i; break; }
  }

  if (dataIntf == 0xFF) {
    usb_host_device_close(usbClient, dev);
    addLog("USB: немає CDC інтерфейсу");
    return;
  }

  usb_host_interface_claim(usbClient, dev, dataIntf, 0);

  // Set line coding 115200 8N1 via control transfer (EP0)
  uint8_t lc[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};  // 115200, 1stop, noparity, 8data
  usb_transfer_t* initXfer = nullptr;
  usb_host_transfer_alloc(8 + 7, 0, &initXfer);
  if (initXfer) {
    usb_setup_packet_t* s = (usb_setup_packet_t*)initXfer->data_buffer;
    s->bmRequestType = 0x21;  // host→device, class, interface
    s->bRequest      = 0x20;  // SET_LINE_CODING
    s->wValue        = 0;
    s->wIndex        = dataIntf;
    s->wLength       = 7;
    memcpy(initXfer->data_buffer + 8, lc, 7);
    initXfer->device_handle    = dev;
    initXfer->bEndpointAddress = 0;
    initXfer->num_bytes        = 8 + 7;
    initXfer->callback         = [](usb_transfer_t* x){ usb_host_transfer_free(x); };
    initXfer->context          = nullptr;
    usb_host_transfer_submit_control(usbClient, initXfer);
  }

  // Allocate TX and RX transfers
  usb_host_transfer_alloc(CDC_TX_MPS, 0, &cdcTxXfer);
  usb_host_transfer_alloc(CDC_RX_MPS, 0, &cdcRxXfer);

  cdcIntf  = dataIntf;
  cdcEpOut = epOut;
  cdcEpIn  = epIn;
  usbDev   = dev;
  usbDevAddr = addr;
  g.usbMode = USB_CDC;
  addLog("CDC: FC підключено (intf=" + String(dataIntf) +
         " epOut=0x" + String(epOut, HEX) +
         " epIn=0x"  + String(epIn,  HEX) + ")");

  // Start perpetual RX
  resubmitRx();
}

// ── USB Host tasks ────────────────────────────────────────────────────────────
static void usbLibTask(void*) {
  usb_host_config_t cfg = {};
  cfg.skip_phy_setup    = false;
  cfg.intr_flags        = ESP_INTR_FLAG_LEVEL1;
  usb_host_install(&cfg);
  addLog("USB Host: ініціалізовано");

  uint32_t evtFlags = 0;
  while (true) {
    usb_host_lib_handle_events(portMAX_DELAY, &evtFlags);
    if (evtFlags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) break;
  }
  usb_host_uninstall();
  vTaskDelete(nullptr);
}

static void usbClientEventCb(const usb_host_client_event_msg_t* msg, void* arg) {
  if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
    openUsbDevice(msg->new_dev.address);
  } else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
    if (usbDev == msg->dev_gone.dev_hdl) closeUsbDevice();
  }
}

static void usbClientTask(void*) {
  const usb_host_client_config_t cfg = {
    .is_synchronous = false,
    .max_num_event_msg = 5,
    .async = {
      .client_event_callback = usbClientEventCb,
      .callback_arg          = nullptr
    }
  };
  usb_host_client_register(&cfg, &usbClient);
  while (true) usb_host_client_handle_events(usbClient, portMAX_DELAY);
}

// ── CDC helpers ───────────────────────────────────────────────────────────────
static void cdcFlushRx() {
  size_t sz; void* p;
  while ((p = xRingbufferReceive(cdcRxBuf, &sz, 0)) != nullptr)
    vRingbufferReturnItem(cdcRxBuf, p);
}

static String cdcReadUntil(const String& token, uint32_t timeoutMs = 1500) {
  String buf;
  uint32_t t = millis();
  while (millis() - t < timeoutMs) {
    size_t sz;
    uint8_t* p = (uint8_t*)xRingbufferReceive(cdcRxBuf, &sz, pdMS_TO_TICKS(40));
    if (p) {
      buf += String((char*)p, sz);
      vRingbufferReturnItem(cdcRxBuf, p);
      if (buf.indexOf(token) >= 0) break;
    }
  }
  return buf;
}

static bool cdcWrite(const uint8_t* data, size_t len) {
  if (!cdcTxXfer || cdcEpOut == 0xFF || !usbDev) return false;
  size_t off = 0;
  while (off < len) {
    size_t chunk = min(len - off, (size_t)CDC_TX_MPS);
    memcpy(cdcTxXfer->data_buffer, data + off, chunk);
    cdcTxXfer->device_handle    = usbDev;
    cdcTxXfer->bEndpointAddress = cdcEpOut;
    cdcTxXfer->num_bytes        = (int)chunk;
    cdcTxXfer->callback         = cdcTxCb;
    cdcTxXfer->context          = nullptr;
    if (usb_host_transfer_submit(cdcTxXfer) != ESP_OK) return false;
    if (xSemaphoreTake(cdcTxSem, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
    off += chunk;
  }
  return true;
}

static void cdcPrint(const String& s) {
  cdcWrite((const uint8_t*)s.c_str(), s.length());
}

// ── DFU control transfer ──────────────────────────────────────────────────────
static bool dfuCtrl(uint8_t reqType, uint8_t req, uint16_t val,
                    uint8_t* buf, uint16_t len, uint32_t timeoutMs = 2500) {
  if (!dfuXfer || !usbDev) return false;
  usb_setup_packet_t* s = (usb_setup_packet_t*)dfuXfer->data_buffer;
  s->bmRequestType = reqType;
  s->bRequest      = req;
  s->wValue        = val;
  s->wIndex        = dfuIntf;
  s->wLength       = len;
  if (buf && len && (reqType & 0x80) == 0)  // host→device data
    memcpy(dfuXfer->data_buffer + 8, buf, len);
  dfuXfer->device_handle    = usbDev;
  dfuXfer->bEndpointAddress = 0;
  dfuXfer->num_bytes        = 8 + len;
  dfuXfer->callback         = dfuXferCb;
  dfuXfer->context          = nullptr;
  xSemaphoreTake(dfuXferSem, 0);  // clear
  if (usb_host_transfer_submit_control(usbClient, dfuXfer) != ESP_OK) return false;
  if (xSemaphoreTake(dfuXferSem, pdMS_TO_TICKS(timeoutMs)) != pdTRUE) return false;
  if (buf && len && (reqType & 0x80) != 0)  // device→host data
    memcpy(buf, dfuXfer->data_buffer + 8, len);
  return dfuXferErr == (esp_err_t)USB_TRANSFER_STATUS_COMPLETED;
}

static uint8_t dfuGetStatus() {
  uint8_t buf[6] = {};
  dfuCtrl(0xA1, 0x03, 0, buf, 6);
  return buf[4];
}
static void dfuClearStatus() { dfuCtrl(0x21, 0x04, 0, nullptr, 0); }
static void dfuAbort()       { dfuCtrl(0x21, 0x06, 0, nullptr, 0); }

static bool dfuSetAddress(uint32_t addr) {
  uint8_t cmd[5] = {0x21,
    (uint8_t)addr,        (uint8_t)(addr >> 8),
    (uint8_t)(addr >> 16),(uint8_t)(addr >> 24)};
  if (!dfuCtrl(0x21, 0x01, 0, cmd, 5)) return false;
  uint32_t t = millis();
  while (millis() - t < 3000) {
    uint8_t st = dfuGetStatus();
    if (st == 5) return true;
    if (st == 10) { dfuClearStatus(); return false; }
    delay(10);
  }
  return false;
}

static bool dfuMassErase() {
  uint8_t cmd[1] = {0x41};
  if (!dfuCtrl(0x21, 0x01, 0, cmd, 1)) return false;
  uint32_t t = millis();
  while (millis() - t < 30000) {
    uint8_t st = dfuGetStatus();
    if (st == 5) return true;
    if (st == 10) { dfuClearStatus(); return false; }
    delay(200);
  }
  return false;
}

static bool dfuWriteBlock(uint16_t blkNum, uint8_t* data, uint16_t len) {
  // Re-allocate DFU transfer if needed for larger data
  if (!dfuXfer || dfuXfer->data_buffer_size < (size_t)(8 + len)) {
    usb_host_transfer_free(dfuXfer); dfuXfer = nullptr;
    usb_host_transfer_alloc(8 + len, 0, &dfuXfer);
    if (!dfuXfer) return false;
  }
  if (!dfuCtrl(0x21, 0x01, blkNum, data, len)) return false;
  uint32_t t = millis();
  while (millis() - t < 5000) {
    uint8_t st = dfuGetStatus();
    if (st == 5) return true;
    if (st == 10) { dfuClearStatus(); return false; }
    delay(10);
  }
  return false;
}

static void dfuLeave(uint32_t addr = 0x08000000) {
  dfuSetAddress(addr);
  dfuCtrl(0x21, 0x01, 2, nullptr, 0);
  dfuGetStatus();
}

// ── Intel HEX → DFU flash ─────────────────────────────────────────────────────
static bool flashHex() {
  if (g.usbMode != USB_DFU) {
    setMsg("FC не в DFU режимі — затисни BOOT і підключи USB"); return false;
  }
  File f = SPIFFS.open("/hex.hex", "r");
  if (!f) { setMsg("HEX файл не знайдено"); return false; }

  uint8_t st = dfuGetStatus();
  if (st == 10) dfuClearStatus();
  if (st != 2 && st != 5) { dfuAbort(); delay(100); dfuClearStatus(); }

  setMsg("Стирання...", 2);
  if (!dfuMassErase()) { f.close(); setMsg("Стирання не вдалось"); return false; }

  const uint16_t BLKSZ = 2048;
  uint32_t extAddr = 0, pageBase = 0, appStart = 0x08000000;
  uint8_t  pageBuf[BLKSZ];
  uint16_t pageLen = 0;
  bool     first   = true;
  size_t   fSize   = f.size(), fPos = 0;

  auto flush = [&]() -> bool {
    if (!pageLen) return true;
    while (pageLen % 4) pageBuf[pageLen++] = 0xFF;
    if (!dfuSetAddress(pageBase)) {
      setMsg("SetAddr 0x" + String(pageBase, HEX) + " failed"); return false;
    }
    uint16_t off = 0, blk = 2;
    while (off < pageLen) {
      uint16_t chunk = min((uint16_t)(pageLen - off), BLKSZ);
      if (!dfuWriteBlock(blk, pageBuf + off, chunk)) {
        setMsg("WriteBlock " + String(blk) + " failed"); return false;
      }
      off += chunk; blk++;
    }
    return true;
  };

  setMsg("Запис прошивки...", 5);
  while (f.available()) {
    String line = f.readStringUntil('\n');
    fPos += line.length() + 1;
    g.progress = 5 + (int)(90.0f * fPos / fSize);
    line.trim();
    if (line.length() < 11 || line[0] != ':') continue;
    auto hb = [&](int p) -> uint8_t {
      char h[3] = {line[p], line[p+1], 0}; return (uint8_t)strtol(h, nullptr, 16);
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
        if (!flush()) { f.close(); return false; }
        pageBase = wa; pageLen = 0;
      }
      if (!pageLen) pageBase = wa;
      pageBuf[pageLen++] = hb(9 + i * 2);
      if (pageLen == BLKSZ) {
        if (!flush()) { f.close(); return false; }
        pageBase += BLKSZ; pageLen = 0;
      }
    }
    yield();
  }
  if (!flush()) { f.close(); return false; }
  f.close();

  setMsg("Запуск FC...", 98);
  dfuLeave(appStart);
  delay(600);
  setMsg("FC прошито!", 100);
  return true;
}

// ── BF CLI dump ───────────────────────────────────────────────────────────────
static bool enterBfCli() {
  if (g.usbMode != USB_CDC) return false;
  cdcFlushRx();
  cdcPrint("\r\n#\r\n");
  return cdcReadUntil("#", 800).indexOf('#') >= 0;
}

static bool restoreDump() {
  if (g.usbMode != USB_CDC) {
    setMsg("FC не підключено — Connect your FC dodik"); return false;
  }
  File f = SPIFFS.open("/dump.txt", "r");
  if (!f) { setMsg("dump не знайдено"); return false; }

  setMsg("Вхід в CLI...", 2);
  if (!enterBfCli()) { f.close(); setMsg("BF CLI не відповідає"); return false; }

  size_t total = 0, done = 0;
  while (f.available()) { f.readStringUntil('\n'); total++; }
  f.seek(0);
  setMsg("Відправка дампу...");

  while (f.available()) {
    String line = f.readStringUntil('\n'); line.trim();
    if (!line.length() || line[0] == '#') { done++; continue; }
    cdcPrint(line + "\r\n");
    delay(40);
    done++;
    g.progress = (int)(100.0f * done / total);
    if (done % 60 == 0) setMsg("Дамп " + String(done) + "/" + String(total));
    yield();
  }
  f.close();
  cdcPrint("save\r\n");
  delay(2500);
  setMsg("Дамп застосовано", 100);
  return true;
}

// ── SLIP / ELRS passthrough ───────────────────────────────────────────────────
#define SLIP_END     0xC0
#define SLIP_ESC     0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

static void slipWrite(const uint8_t* data, size_t len) {
  if (g.usbMode != USB_CDC) return;
  uint8_t buf[len * 2 + 2]; size_t idx = 0;
  buf[idx++] = SLIP_END;
  for (size_t i = 0; i < len; i++) {
    if      (data[i] == SLIP_END) { buf[idx++]=SLIP_ESC; buf[idx++]=SLIP_ESC_END; }
    else if (data[i] == SLIP_ESC) { buf[idx++]=SLIP_ESC; buf[idx++]=SLIP_ESC_ESC; }
    else buf[idx++] = data[i];
  }
  buf[idx++] = SLIP_END;
  cdcWrite(buf, idx);
}

static bool slipRead(uint8_t* out, size_t* outLen, uint32_t timeoutMs = 2500) {
  uint32_t t = millis(); bool inPkt = false; size_t idx = 0; bool esc = false;
  while (millis() - t < timeoutMs) {
    size_t sz;
    uint8_t* p = (uint8_t*)xRingbufferReceive(cdcRxBuf, &sz, pdMS_TO_TICKS(30));
    if (!p) continue;
    for (size_t i = 0; i < sz; i++) {
      uint8_t b = p[i];
      if (b == SLIP_END) {
        if (inPkt && idx > 0) { vRingbufferReturnItem(cdcRxBuf, p); *outLen = idx; return true; }
        inPkt = true; idx = 0; esc = false; continue;
      }
      if (!inPkt) continue;
      if (b == SLIP_ESC) { esc = true; continue; }
      if (esc) { b = (b == SLIP_ESC_END) ? SLIP_END : SLIP_ESC; esc = false; }
      if (idx < 512) out[idx++] = b;
    }
    vRingbufferReturnItem(cdcRxBuf, p);
  }
  return false;
}

static bool espCmd(uint8_t cmd, const uint8_t* data, uint16_t dLen,
                   uint32_t chk = 0, uint32_t timeoutMs = 2500) {
  size_t pLen = 8 + dLen; uint8_t pkt[pLen];
  pkt[0]=0; pkt[1]=cmd; pkt[2]=dLen&0xFF; pkt[3]=(dLen>>8)&0xFF;
  pkt[4]=chk; pkt[5]=chk>>8; pkt[6]=chk>>16; pkt[7]=chk>>24;
  if (dLen) memcpy(pkt+8, data, dLen);
  slipWrite(pkt, pLen);
  uint8_t resp[512]; size_t rLen = 0;
  if (!slipRead(resp, &rLen, timeoutMs)) return false;
  return rLen >= 10 && resp[1] == cmd && resp[rLen-2] == 0;
}

static bool espSync() {
  uint8_t sd[36] = {0x07,0x07,0x12,0x20}; memset(sd+4, 0x55, 32);
  for (int i = 0; i < 7; i++) {
    espCmd(0x08, sd, 36, 0, 600);
    uint8_t r[64]; size_t l = 0;
    if (slipRead(r, &l, 400) && l >= 10 && r[1] == 0x08 && r[l-2] == 0) return true;
    delay(30);
  }
  return false;
}

static uint32_t espChk(const uint8_t* d, size_t l) {
  uint32_t cs = 0xEF; for (size_t i = 0; i < l; i++) cs ^= d[i]; return cs;
}

static bool flashElrs() {
  if (g.usbMode != USB_CDC) {
    setMsg("FC не підключено — Connect your FC dodik"); return false;
  }
  File f = SPIFFS.open("/elrs.bin", "r");
  if (!f) { setMsg("ELRS bin не знайдено"); return false; }
  size_t fSize = f.size();

  setMsg("BF CLI...", 2);
  if (!enterBfCli()) { f.close(); setMsg("BF CLI не відповідає"); return false; }
  cdcFlushRx();
  cdcPrint("serialpassthrough " + String(g_elrsUart-1) + " " + String(g_elrsBaud) + " rxtx\r\n");
  delay(500); cdcFlushRx();

  setMsg("ELRS sync...", 10);
  if (!espSync()) {
    f.close();
    setMsg("ELRS sync failed — перевір UART" + String(g_elrsUart)); return false;
  }
  addLog("ELRS sync OK, " + String(fSize) + " байт");

  const uint32_t BSZ = 0x4000, CSZ = 0x0200;
  uint32_t nPkts = (fSize+CSZ-1)/CSZ, eSz = ((fSize+BSZ-1)/BSZ)*BSZ, fOff = 0;
  uint8_t bd[16];
  memcpy(bd, &eSz, 4); memcpy(bd+4, &nPkts, 4);
  uint32_t cs = CSZ; memcpy(bd+8, &cs, 4); memcpy(bd+12, &fOff, 4);
  if (!espCmd(0x02, bd, 16, 0, 5000)) { f.close(); setMsg("FLASH_BEGIN failed"); return false; }

  setMsg("Запис ELRS...", 12);
  uint8_t chunk[CSZ+16]; uint32_t seq = 0; size_t written = 0;
  while (f.available()) {
    memset(chunk+16, 0xFF, CSZ); f.read(chunk+16, CSZ); written += CSZ;
    uint32_t dsz = CSZ, z = 0;
    memcpy(chunk,    &dsz, 4); memcpy(chunk+4,  &seq, 4);
    memcpy(chunk+8,  &z,   4); memcpy(chunk+12, &z,   4);
    uint32_t ck = espChk(chunk+16, CSZ);
    if (!espCmd(0x03, chunk, CSZ+16, ck, 3000)) {
      f.close(); setMsg("FLASH_DATA seq=" + String(seq) + " failed"); return false;
    }
    seq++; g.progress = 12 + (int)(85.0f * min(written, fSize) / fSize);
    yield();
  }
  f.close();
  uint8_t ed[4] = {};
  espCmd(0x04, ed, 4, 0, 2000);
  setMsg("ELRS прошито!", 100);
  return true;
}

// ── Flash task ────────────────────────────────────────────────────────────────
struct FlashParams { bool doHex; bool doDump; bool doElrs; };
static FlashParams  flashParams;
static TaskHandle_t flashTask = nullptr;

static void flashTaskFn(void* pv) {
  auto* p = (FlashParams*)pv;
  bool ok = true;
  addLog("=== Початок прошивки ===");

  if (p->doHex && ok) {
    setMsg("Прошивка FC (DFU)...", 0);
    ok = flashHex();
    if (ok) { g.progress = 100; delay(800); }
  }
  if (p->doDump && ok) {
    setMsg("Відновлення дампу...", 0);
    for (int i = 0; i < 30 && g.usbMode != USB_CDC; i++) delay(500);
    ok = restoreDump();
    if (ok) { g.progress = 100; delay(500); }
  }
  if (p->doElrs && ok) {
    setMsg("Прошивка ELRS...", 0);
    ok = flashElrs();
    if (ok) { g.progress = 100; delay(500); }
  }

  addLog(ok ? "=== Готово ===" : "=== ПОМИЛКА: " + g.message + " ===");
  if (ok) setMsg("Готово!");
  g.flashing = false;
  flashTask  = nullptr;
  vTaskDelete(nullptr);
}

// ── Upload helper ─────────────────────────────────────────────────────────────
static File uploadFile;

static void handleUpload(const String& path, AsyncWebServerRequest*,
                          const String& fn, size_t idx,
                          uint8_t* d, size_t l, bool final) {
  if (!idx) { if (uploadFile) uploadFile.close(); uploadFile = SPIFFS.open(path, "w"); }
  if (uploadFile) uploadFile.write(d, l);
  if (final && uploadFile) { uploadFile.close(); addLog("Файл: " + fn + " → " + path); }
}

// ── HTML ──────────────────────────────────────────────────────────────────────
static const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html lang="uk"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>DroneFlasher S3</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#111;color:#eee;font-family:'Segoe UI',sans-serif;
     display:flex;flex-direction:column;align-items:center;
     min-height:100vh;padding:24px 12px;gap:14px}
#ub{padding:8px 28px;border-radius:999px;font-size:1rem;font-weight:700;
    letter-spacing:.06em;background:#222;transition:color .3s,box-shadow .3s}
#ub.dfu{color:#ffd84b;box-shadow:0 0 12px #ffd84b55}
#ub.cdc{color:#b0ff4b;box-shadow:0 0 12px #b0ff4b55}
#ub.none{color:#ff4b4b;box-shadow:0 0 12px #ff4b4b33}
.card{background:#d0d0d0;border-radius:18px;padding:24px 20px;width:100%;max-width:450px}
.ct{font-size:.72rem;font-weight:700;color:#555;text-transform:uppercase;
    letter-spacing:.08em;margin-bottom:12px}
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
select{background:#444;color:#fff;border:none;border-radius:8px;padding:5px 10px;font-size:.88rem}
#bf{display:block;width:100%;padding:16px;margin-top:8px;
    background:#333;color:#b0ff4b;border:none;border-radius:14px;
    font-size:1.15rem;font-weight:800;letter-spacing:.08em;cursor:pointer}
#bf:hover:not(:disabled){background:#444}#bf:disabled{opacity:.4;cursor:not-allowed}
#pw{margin-top:12px;display:none}
#bb{background:#bbb;border-radius:8px;height:12px;overflow:hidden}
#bfill{height:100%;background:#b0ff4b;border-radius:8px;width:0%;transition:width .3s}
#mg{margin-top:6px;font-size:.82rem;color:#333;text-align:center;
    min-height:17px;word-break:break-all}
.hint{font-size:.72rem;color:#666;text-align:center;margin-top:4px;line-height:1.4}
.lnk{color:#555;font-size:.76rem;text-decoration:none;margin-top:2px}
.lnk:hover{color:#b0ff4b}
</style></head><body>
<div id="ub" class="none">USB: —</div>
<div class="card">
<div class="ct">Прошивка</div>
<div class="row"><div class="lbl">hex:</div><div class="fw">
  <button class="bc" id="bh" onclick="pick('h')">ВИБРАТИ</button>
  <input type="file" id="ih" accept=".hex">
  <div class="fn" id="nh"></div></div></div>
<div class="row"><div class="lbl">dump:</div><div class="fw">
  <button class="bc" id="bd" onclick="pick('d')">ВИБРАТИ</button>
  <input type="file" id="id" accept=".txt,.cli,.dump">
  <div class="fn" id="nd"></div></div></div>
<div class="row"><div class="lbl">RX:</div><div class="fw">
  <button class="bc" id="be" onclick="pick('e')">ВИБРАТИ</button>
  <input type="file" id="ie" accept=".bin">
  <div class="fn" id="ne"></div></div></div>
<div class="sep"></div>
<div class="ct">ELRS passthrough</div>
<div class="rc"><span class="cl">UART FC (BF):</span>
  <select id="su"><option value="1">UART1</option><option value="2">UART2</option>
  <option value="3">UART3</option><option value="4" selected>UART4</option>
  <option value="5">UART5</option><option value="6">UART6</option></select></div>
<div class="rc"><span class="cl">Бодрейт ELRS:</span>
  <select id="sb"><option value="115200">115200</option>
  <option value="420000" selected>420000</option>
  <option value="460800">460800</option><option value="921600">921600</option></select></div>
<button id="bf" disabled onclick="go()">ПРОШИТИ</button>
<div id="pw"><div id="bb"><div id="bfill"></div></div><div id="mg"></div></div>
<div class="hint" id="hint"></div>
</div>
<a class="lnk" href="/ota">OTA оновлення &rsaquo;</a>
<a class="lnk" href="/log" target="_blank">Лог прошивки &rsaquo;</a>
<script>
const F={h:null,d:null,e:null};
const K={h:'hex',d:'dump',e:'elrs'};
function pick(k){document.getElementById('i'+k).click();}
'hde'.split('').forEach(k=>{
  document.getElementById('i'+k).onchange=function(){
    if(this.files[0]){F[k]=this.files[0];
      document.getElementById('n'+k).textContent=this.files[0].name;
      document.getElementById('b'+k).classList.add('on');}
    else{F[k]=null;document.getElementById('n'+k).textContent='';
      document.getElementById('b'+k).classList.remove('on');}
    document.getElementById('bf').disabled=!(F.h||F.d||F.e);
  };
});
async function go(){
  const btn=document.getElementById('bf');btn.disabled=true;
  document.getElementById('pw').style.display='block';
  sm('Завантаження...');sp(0);
  for(const k of['h','d','e']){
    if(F[k]){sm('Завантаження: '+F[k].name);
      const fd=new FormData();fd.append('file',F[k],F[k].name);
      if(!(await fetch('/upload/'+K[k],{method:'POST',body:fd})).ok)
        {sm('Помилка: '+F[k].name);btn.disabled=false;return;}}}
  const r=await fetch('/flash',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({hex:!!F.h,dump:!!F.d,elrs:!!F.e,
      elrsUart:+document.getElementById('su').value,
      elrsBaud:+document.getElementById('sb').value})});
  if(!r.ok){sm('Помилка запуску');btn.disabled=false;return;}
  const iv=setInterval(async()=>{
    const d=await(await fetch('/status')).json();
    sp(d.progress);sm(d.message);
    if(!d.flashing){clearInterval(iv);btn.disabled=false;}
  },600);
}
function sp(p){document.getElementById('bfill').style.width=p+'%';}
function sm(t){document.getElementById('mg').textContent=t;}
(function poll(){
  fetch('/status').then(r=>r.json()).then(d=>{
    const el=document.getElementById('ub');
    const h={none:'Підключи FC через USB Type-C (OTG порт)',
             cdc:'FC підключено (BF CDC) — готово',
             dfu:'FC в DFU режимі — готово до hex'};
    el.textContent='USB: '+d.usbMode.toUpperCase();
    el.className=d.usbMode;
    document.getElementById('hint').textContent=h[d.usbMode]||'';
    if(!d.flashing&&d.message)sm(d.message);
    if(d.flashing)sp(d.progress);
  }).catch(()=>{});
  setTimeout(poll,1500);
})();
</script></body></html>
)rawhtml";

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
      transition:border-color .2s;user-select:none}
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
  <button class="tab active" onclick="sw('e')">ESP32</button>
  <button class="tab" onclick="sw('s')">STM32 (FC)</button>
</div>
<div class="panel show" id="pe"><div class="card">
  <div class="ct">Оновлення ESP32-S3</div>
  <div class="drop" id="de" onclick="document.getElementById('ie').click()">
    Перетягни або вибери <b>.bin</b></div>
  <input type="file" id="ie" accept=".bin">
  <div class="fn" id="fe"></div>
  <button class="btn" id="be" disabled onclick="doE()">ОНОВИТИ ESP32</button>
  <div class="bg" id="bge"><div class="bf" id="bfe"></div></div>
  <div class="st" id="se"></div>
  <div class="note">Sketch → Export Compiled Binary → .ino.bin</div>
</div></div>
<div class="panel" id="ps"><div class="card">
  <div class="ct">Прошивка STM32 (DFU)</div>
  <div class="drop" id="ds" onclick="document.getElementById('is').click()">
    Перетягни або вибери <b>.hex</b></div>
  <input type="file" id="is" accept=".hex">
  <div class="fn" id="fs"></div>
  <button class="btn" id="bs" disabled onclick="doS()">ПРОШИТИ STM32</button>
  <div class="bg" id="bgs"><div class="bf" id="bfs"></div></div>
  <div class="st" id="ss"></div>
  <div class="note">Затисни BOOT на FC і підключи USB Type-C</div>
</div></div>
<a class="back" href="/">← Назад</a>
<script>
function sw(t){['e','s'].forEach((x,i)=>{
  document.querySelectorAll('.tab')[i].classList.toggle('active',x===t);
  document.getElementById('p'+x).classList.toggle('show',x===t);});}
function mk(did,iid,fid,bid){
  const d=document.getElementById(did),inp=document.getElementById(iid);
  let f=null;
  inp.onchange=function(){if(this.files[0])set(this.files[0]);};
  d.ondragover=e=>{e.preventDefault();d.classList.add('over');};
  d.ondragleave=()=>d.classList.remove('over');
  d.ondrop=e=>{e.preventDefault();d.classList.remove('over');
    if(e.dataTransfer.files[0])set(e.dataTransfer.files[0]);};
  function set(x){f=x;document.getElementById(fid).textContent=x.name+' ('+Math.round(x.size/1024)+' KB)';
    document.getElementById(bid).disabled=false;}
  return()=>f;
}
const gE=mk('de','ie','fe','be'), gS=mk('ds','is','fs','bs');
function doE(){
  const f=gE();if(!f)return;
  document.getElementById('be').disabled=true;
  document.getElementById('bge').style.display='block';
  st('e','Завантаження...');
  const fd=new FormData();fd.append('file',f,f.name);
  const x=new XMLHttpRequest();x.open('POST','/ota/upload');
  x.upload.onprogress=e=>{if(e.lengthComputable){
    const p=Math.round(e.loaded/e.total*100);
    document.getElementById('bfe').style.width=p+'%';st('e',p+'%');}};
  x.onload=()=>{try{const d=JSON.parse(x.responseText);
    d.ok?st('e','Готово! Перезавантаження...'):st('e','Помилка: '+(d.err||'?'));}
    catch(e){st('e','Перезавантаження...');}};
  x.send(fd);
}
async function doS(){
  const f=gS();if(!f)return;
  document.getElementById('bs').disabled=true;
  document.getElementById('bgs').style.display='block';
  st('s','Завантаження hex...');
  const fd=new FormData();fd.append('file',f,f.name);
  if(!(await fetch('/upload/hex',{method:'POST',body:fd})).ok)
    {st('s','Помилка');document.getElementById('bs').disabled=false;return;}
  st('s','Запуск DFU...');
  await fetch('/flash',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({hex:true,dump:false,elrs:false})});
  const iv=setInterval(async()=>{
    const d=await(await fetch('/status')).json();
    document.getElementById('bfs').style.width=d.progress+'%';st('s',d.message);
    if(!d.flashing){clearInterval(iv);document.getElementById('bs').disabled=false;}
  },700);
}
function st(p,t){document.getElementById('s'+p).textContent=t;}
</script></body></html>
)rawhtml";

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  logMux   = xSemaphoreCreateMutex();
  dfuXferSem = xSemaphoreCreateBinary();
  cdcTxSem   = xSemaphoreCreateBinary();
  cdcRxBuf   = xRingbufferCreate(CDC_RX_BUF, RINGBUF_TYPE_BYTEBUF);
  usbMux     = xSemaphoreCreateMutex();

  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 6; i++) { digitalWrite(LED_PIN, i % 2); delay(100); }

  SPIFFS.begin(true);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(10,0,0,1), IPAddress(10,0,0,1), IPAddress(255,255,255,0));
  WiFi.softAP(AP_SSID, AP_PASS);
  addLog("AP: 10.0.0.1  SSID=" + String(AP_SSID));

  // Start USB Host tasks
  xTaskCreate(usbLibTask,    "usbLib",    4096, nullptr, 5, nullptr);
  delay(200);  // allow lib to init
  xTaskCreate(usbClientTask, "usbClient", 4096, nullptr, 5, nullptr);

  // ── Web routes ────────────────────────────────────────────────────────────
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){
    r->send_P(200, "text/html", INDEX_HTML); });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest* r){
    StaticJsonDocument<128> doc;
    const char* modes[] = {"none","dfu","cdc"};
    doc["usbMode"]  = modes[(int)g.usbMode];
    doc["flashing"] = g.flashing;
    doc["progress"] = g.progress;
    doc["message"]  = g.message;
    String o; serializeJson(doc, o);
    r->send(200, "application/json", o); });

  server.on("/log", HTTP_GET, [](AsyncWebServerRequest* r){
    String pg = "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<style>*{box-sizing:border-box;margin:0;padding:0}"
      "body{background:#0d0d0d;color:#b0ff4b;font-family:monospace;"
      "padding:14px;font-size:.8rem;line-height:1.6}"
      ".bar{display:flex;gap:10px;margin-bottom:12px}"
      "a,button{color:#888;text-decoration:none;font-size:.76rem;"
      "background:#222;border:none;border-radius:6px;padding:5px 12px;cursor:pointer}"
      "a:hover,button:hover{color:#b0ff4b}"
      "pre{white-space:pre-wrap;word-break:break-all}</style></head><body>"
      "<div class='bar'><a href='/'>← Назад</a>"
      "<button onclick='location.reload()'>Оновити</button></div><pre>";
    if (xSemaphoreTake(logMux, pdMS_TO_TICKS(100)) == pdTRUE) {
      pg += flashLog; xSemaphoreGive(logMux); }
    pg += "</pre><script>window.scrollTo(0,document.body.scrollHeight);"
          "setTimeout(()=>location.reload(),2500);</script></body></html>";
    r->send(200, "text/html", pg); });

  auto mkUp = [](const String& path, bool& ready) {
    return [path, &ready](AsyncWebServerRequest* r, const String& fn,
                           size_t i, uint8_t* d, size_t l, bool f) {
      if (g.flashing) return;
      handleUpload(path, r, fn, i, d, l, f);
      if (f) ready = true;
    };
  };

  server.on("/upload/hex",  HTTP_POST, [](AsyncWebServerRequest* r){r->send(200);},
    mkUp("/hex.hex",  g.hexReady));
  server.on("/upload/dump", HTTP_POST, [](AsyncWebServerRequest* r){r->send(200);},
    mkUp("/dump.txt", g.dumpReady));
  server.on("/upload/elrs", HTTP_POST, [](AsyncWebServerRequest* r){r->send(200);},
    mkUp("/elrs.bin", g.elrsReady));

  server.on("/flash", HTTP_POST,
    [](AsyncWebServerRequest* r){
      if (g.flashing) { r->send(423, "text/plain", "busy"); return; }
      g.flashing = true; g.progress = 0; setMsg("Підготовка...");
      xTaskCreate(flashTaskFn, "flash", 10240, &flashParams, 1, &flashTask);
      r->send(200, "text/plain", "ok"); },
    nullptr,
    [](AsyncWebServerRequest*, uint8_t* data, size_t len, size_t, size_t){
      StaticJsonDocument<128> doc; deserializeJson(doc, data, len);
      flashParams.doHex  = doc["hex"]  | false;
      flashParams.doDump = doc["dump"] | false;
      flashParams.doElrs = doc["elrs"] | false;
      g_elrsUart = doc["elrsUart"] | 4;
      g_elrsBaud = doc["elrsBaud"] | 420000; });

  server.on("/ota", HTTP_GET, [](AsyncWebServerRequest* r){
    r->send_P(200, "text/html", OTA_HTML); });

  server.on("/ota/upload", HTTP_POST,
    [](AsyncWebServerRequest* r){
      bool ok = !Update.hasError();
      auto* resp = r->beginResponse(200, "application/json",
        ok ? "{\"ok\":true}" : "{\"ok\":false,\"err\":\"" + String(Update.errorString()) + "\"}");
      resp->addHeader("Connection", "close"); r->send(resp);
      if (ok) { delay(400); ESP.restart(); } },
    [](AsyncWebServerRequest*, const String& fn, size_t idx,
       uint8_t* d, size_t l, bool final){
      if (!idx) { addLog("OTA: " + fn);
        Update.begin((ESP.getFreeSketchSpace()-0x1000) & 0xFFFFF000); }
      if (Update.isRunning()) Update.write(d, l);
      if (final) { Update.end(true);
        addLog(Update.hasError() ? "OTA fail: " + String(Update.errorString()) : "OTA OK"); } });

  server.begin();
  addLog("Ready — http://10.0.0.1");
}

// ── loop ──────────────────────────────────────────────────────────────────────
static uint32_t lastBlink = 0;
static bool     ledSt     = false;

void loop() {
  uint32_t now = millis();
  if (g.flashing) {
    if (now - lastBlink > 80) { lastBlink = now; ledSt = !ledSt; digitalWrite(LED_PIN, ledSt); }
  } else if (g.usbMode != USB_NONE) {
    if (now - lastBlink > 900) { lastBlink = now; ledSt = !ledSt; digitalWrite(LED_PIN, ledSt); }
  } else {
    static uint8_t ph = 0; static uint32_t bt = 0;
    const uint16_t pat[] = {80, 80, 80, 700};
    if (now - bt > pat[ph % 4]) { bt = now; ledSt = (ph % 4 < 2); digitalWrite(LED_PIN, ledSt); ph++; }
  }
  delay(2);
}
