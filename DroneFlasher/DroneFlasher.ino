/*
 * DroneFlasher — ESP32 WROOM-32 (CH340, Type-C)
 * WiFi AP: CONFIG / freeAzov  →  http://10.0.0.1
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │  ПІНИ                                                        │
 * │                                                              │
 * │  FC (польотний контролер — STM32G431):                       │
 * │    ESP32 GPIO16 (RX2)  ←───  FC TX                          │
 * │    ESP32 GPIO17 (TX2)  ────►  FC RX                          │
 * │    ESP32 GPIO18        ────►  FC BOOT0   (100 Ом в ряд)      │
 * │    ESP32 GPIO19        ────►  FC NRST    (100 Ом в ряд)      │
 * │    ESP32 GND           ─────  FC GND                         │
 * │                                                              │
 * │  ELRS RX (паяти до FC):                                      │
 * │    ELRS TX  ───►  FC UART_N RX   (той самий UARTx в BF)      │
 * │    ELRS RX  ◄───  FC UART_N TX                               │
 * │                                                              │
 * │  Керування ресетом ELRS (опційно, але бажано):               │
 * │    ESP32 GPIO22  ────►  ELRS RST    (100 Ом в ряд)           │
 * │    ESP32 GPIO23  ────►  ELRS GPIO0  (100 Ом в ряд)           │
 * │    GND ──── ELRS GND                                         │
 * │                                                              │
 * │  Дані до ELRS ідуть: ESP32 → FC UART2 → BF passthrough       │
 * │  → FC UART_N → ELRS RX (esptool SLIP protocol)               │
 * └──────────────────────────────────────────────────────────────┘
 *
 * Бібліотеки:
 *   ESPAsyncWebServer  — me-no-dev/ESPAsyncWebServer
 *   AsyncTCP           — me-no-dev/AsyncTCP
 *   ArduinoJson        — bblanchon/ArduinoJson  (v6)
 *
 * Partition Scheme: "Default 4MB with spiffs (1.2MB APP / 1.5MB SPIFFS)"
 */

#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Update.h>

// ── WiFi AP ──────────────────────────────────────────────────────────────────
static const char* AP_SSID = "CONFIG";
static const char* AP_PASS = "freeAzov";

// ── Піни ─────────────────────────────────────────────────────────────────────
#define FC_RX_PIN     16
#define FC_TX_PIN     17
#define FC_BOOT0_PIN  18
#define FC_NRST_PIN   19
#define ELRS_RST_PIN  22
#define ELRS_IO0_PIN  23
#define LED_PIN        2   // Вбудований світлодіод DevKit

// ── UART ─────────────────────────────────────────────────────────────────────
HardwareSerial FC_Serial(2);

// ── Web server ───────────────────────────────────────────────────────────────
AsyncWebServer server(80);

// ── Лог прошивки ─────────────────────────────────────────────────────────────
#define LOG_MAX 8192
static String flashLog = "";
static SemaphoreHandle_t logMutex = nullptr;

static void addLog(const String& line) {
  if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    flashLog += line + "\n";
    if (flashLog.length() > LOG_MAX)
      flashLog = flashLog.substring(flashLog.length() - LOG_MAX);
    xSemaphoreGive(logMutex);
  }
  Serial.println(line);
}

// ── Стан ─────────────────────────────────────────────────────────────────────
struct State {
  bool   fcConnected = false;
  bool   flashing    = false;
  int    progress    = 0;
  String message     = "Очікування...";
  bool   hexReady    = false;
  bool   dumpReady   = false;
  bool   elrsReady   = false;
};
static State g;

static void setMsg(const String& m, int progress = -1) {
  g.message = m;
  if (progress >= 0) g.progress = progress;
  addLog(m);
}

static File   uploadFile;
static TaskHandle_t flashTask = nullptr;

// ── ELRS passthrough config (зберігається між сесіями в SPIFFS) ──────────────
static uint8_t  g_elrsUart  = 4;        // BF UART номер (1-6)
static uint32_t g_elrsBaud  = 420000;   // бодрейт ELRS на FC

// ─────────────────────────────────────────────────────────────────────────────
//  HTML / CSS / JS
// ─────────────────────────────────────────────────────────────────────────────
static const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="uk">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>DroneFlasher</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#111;color:#eee;font-family:'Segoe UI',sans-serif;
     display:flex;flex-direction:column;align-items:center;
     min-height:100vh;padding:24px 12px;gap:16px}
#status-badge{
  padding:10px 36px;border-radius:999px;font-size:1.15rem;
  font-weight:700;letter-spacing:.08em;background:#222;
  transition:color .4s,box-shadow .4s}
#status-badge.ok{color:#b0ff4b;box-shadow:0 0 14px #b0ff4b55}
#status-badge.no{color:#ff4b4b;box-shadow:0 0 14px #ff4b4b44}
.card{background:#d0d0d0;border-radius:18px;padding:26px 22px;
      width:100%;max-width:460px}
.card-title{font-size:.78rem;font-weight:700;color:#555;
            letter-spacing:.08em;text-transform:uppercase;margin-bottom:14px}
.row{display:flex;align-items:center;gap:12px;margin-bottom:16px}
.label{background:#444;color:#fff;border-radius:8px;padding:8px 16px;
       font-size:.97rem;font-weight:600;min-width:82px;text-align:center;
       flex-shrink:0}
.file-wrap{flex:1;display:flex;flex-direction:column;gap:4px}
.fname{font-size:.76rem;color:#555;min-height:15px;word-break:break-all}
.btn-ch{background:#444;color:#fff;border:none;border-radius:8px;
        padding:8px 14px;cursor:pointer;font-size:.9rem;font-weight:600;
        width:100%}
.btn-ch:hover{background:#555}
.btn-ch.chosen{background:#3d6b12;color:#b0ff4b}
input[type=file]{display:none}
.sep{height:1px;background:#bbb;margin:8px 0 16px}
.row-cfg{display:flex;align-items:center;gap:10px;margin-bottom:10px}
.cfg-label{font-size:.85rem;color:#444;font-weight:600;min-width:160px}
select,input[type=number]{
  background:#444;color:#fff;border:none;border-radius:8px;
  padding:6px 10px;font-size:.9rem;-webkit-appearance:none}
#btn-flash{
  display:block;width:100%;padding:18px;margin-top:8px;
  background:#333;color:#b0ff4b;border:none;border-radius:14px;
  font-size:1.22rem;font-weight:800;letter-spacing:.1em;
  cursor:pointer;transition:background .2s}
#btn-flash:hover:not(:disabled){background:#444}
#btn-flash:disabled{opacity:.45;cursor:not-allowed}
#progress-wrap{margin-top:14px;display:none}
#bar-bg{background:#bbb;border-radius:8px;height:13px;overflow:hidden}
#bar{height:100%;background:#b0ff4b;border-radius:8px;
     width:0%;transition:width .35s}
#msg{margin-top:7px;font-size:.83rem;color:#333;text-align:center;
     min-height:18px;word-break:break-all}
</style>
</head>
<body>

<div id="status-badge" class="no">DISCONNECTED</div>

<div class="card">
  <div class="card-title">Прошивка</div>

  <div class="row">
    <div class="label">hex:</div>
    <div class="file-wrap">
      <button class="btn-ch" id="btn-hex" onclick="pick('hex')">ВИБРАТИ</button>
      <input type="file" id="inp-hex" accept=".hex">
      <div class="fname" id="name-hex"></div>
    </div>
  </div>

  <div class="row">
    <div class="label">dump:</div>
    <div class="file-wrap">
      <button class="btn-ch" id="btn-dump" onclick="pick('dump')">ВИБРАТИ</button>
      <input type="file" id="inp-dump" accept=".txt,.cli,.dump">
      <div class="fname" id="name-dump"></div>
    </div>
  </div>

  <div class="row">
    <div class="label">RX:</div>
    <div class="file-wrap">
      <button class="btn-ch" id="btn-elrs" onclick="pick('elrs')">ВИБРАТИ</button>
      <input type="file" id="inp-elrs" accept=".bin">
      <div class="fname" id="name-elrs"></div>
    </div>
  </div>

  <div class="sep"></div>
  <div class="card-title">Налаштування ELRS passthrough</div>

  <div class="row-cfg">
    <span class="cfg-label">UART на FC (BF):</span>
    <select id="sel-uart">
      <option value="1">UART1</option>
      <option value="2">UART2</option>
      <option value="3">UART3</option>
      <option value="4" selected>UART4</option>
      <option value="5">UART5</option>
      <option value="6">UART6</option>
    </select>
  </div>
  <div class="row-cfg">
    <span class="cfg-label">Бодрейт ELRS:</span>
    <select id="sel-baud">
      <option value="115200">115200</option>
      <option value="420000" selected>420000</option>
      <option value="460800">460800</option>
      <option value="921600">921600</option>
    </select>
  </div>

  <button id="btn-flash" disabled onclick="startFlash()">ПРОШИТИ</button>

  <div id="progress-wrap">
    <div id="bar-bg"><div id="bar"></div></div>
    <div id="msg">Очікування...</div>
  </div>
</div>

<a style="color:#555;font-size:.78rem;text-decoration:none;margin-top:4px"
   href="/ota">Оновити прошивку ESP32 / STM32 (OTA) &rsaquo;</a>

<a style="color:#555;font-size:.78rem;text-decoration:none;margin-top:2px"
   href="/log" target="_blank">Лог прошивки &rsaquo;</a>

<script>
const files={hex:null,dump:null,elrs:null};

function pick(key){document.getElementById('inp-'+key).click();}

['hex','dump','elrs'].forEach(k=>{
  document.getElementById('inp-'+k).addEventListener('change',function(){
    if(this.files[0]){
      files[k]=this.files[0];
      document.getElementById('name-'+k).textContent=this.files[0].name;
      document.getElementById('btn-'+k).classList.add('chosen');
    }else{
      files[k]=null;
      document.getElementById('name-'+k).textContent='';
      document.getElementById('btn-'+k).classList.remove('chosen');
    }
    document.getElementById('btn-flash').disabled=!(files.hex||files.dump||files.elrs);
  });
});

async function upload(key,file){
  const fd=new FormData();fd.append('file',file,file.name);
  const r=await fetch('/upload/'+key,{method:'POST',body:fd});
  return r.ok;
}

async function startFlash(){
  const btn=document.getElementById('btn-flash');
  btn.disabled=true;
  document.getElementById('progress-wrap').style.display='block';
  setMsg('Завантаження файлів...');setProg(0);

  for(const[k,f] of Object.entries(files)){
    if(f){setMsg('Завантаження: '+f.name);
      if(!await upload(k,f)){setMsg('Помилка: '+f.name);btn.disabled=false;return;}}
  }

  const uart=document.getElementById('sel-uart').value;
  const baud=document.getElementById('sel-baud').value;
  setMsg('Запуск...');
  const r=await fetch('/flash',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({hex:!!files.hex,dump:!!files.dump,elrs:!!files.elrs,
                         elrsUart:parseInt(uart),elrsBaud:parseInt(baud)})});
  if(!r.ok){setMsg('Помилка запуску');btn.disabled=false;return;}
  pollProg(btn);
}

function pollProg(btn){
  const iv=setInterval(async()=>{
    const d=await(await fetch('/status')).json();
    setProg(d.progress);setMsg(d.message);
    if(!d.flashing){clearInterval(iv);btn.disabled=false;}
  },600);
}

function setProg(p){document.getElementById('bar').style.width=p+'%';}
function setMsg(t){document.getElementById('msg').textContent=t;}

(function poll(){
  fetch('/status').then(r=>r.json()).then(d=>{
    const el=document.getElementById('status-badge');
    el.textContent=d.connected?'CONNECTED':'DISCONNECTED';
    el.className=d.connected?'ok':'no';
    if(!d.flashing&&d.message)setMsg(d.message);
    if(d.flashing)setProg(d.progress);
  }).catch(()=>{});
  setTimeout(poll,1800);
})();
</script>
</body>
</html>
)rawhtml";

// ── OTA сторінка (ESP32 self-update + STM32 hex flash) ───────────────────────
static const char OTA_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="uk">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Оновлення — DroneFlasher</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#111;color:#eee;font-family:'Segoe UI',sans-serif;
     display:flex;flex-direction:column;align-items:center;
     min-height:100vh;padding:32px 12px;gap:16px}
.tabs{display:flex;gap:8px;width:100%;max-width:440px}
.tab{flex:1;padding:10px;border:none;border-radius:10px;font-size:.95rem;
     font-weight:700;cursor:pointer;background:#2a2a2a;color:#888;letter-spacing:.04em}
.tab.active{background:#333;color:#b0ff4b}
.panel{display:none;width:100%;max-width:440px}
.panel.show{display:flex;flex-direction:column;gap:14px}
.card{background:#d0d0d0;border-radius:18px;padding:26px 22px;
      display:flex;flex-direction:column;gap:14px}
.card-title{font-size:.75rem;font-weight:700;color:#555;
            text-transform:uppercase;letter-spacing:.08em}
.drop{border:2px dashed #888;border-radius:12px;padding:28px 16px;
      text-align:center;cursor:pointer;color:#555;font-size:.92rem;
      transition:border-color .2s,color .2s;user-select:none}
.drop.over,.drop:hover{border-color:#b0ff4b;color:#333}
input[type=file]{display:none}
.fname{font-size:.78rem;color:#444;text-align:center;min-height:16px;word-break:break-all}
.btn{width:100%;padding:15px;background:#333;color:#b0ff4b;border:none;
     border-radius:12px;font-size:1.05rem;font-weight:800;
     cursor:pointer;letter-spacing:.08em}
.btn:hover:not(:disabled){background:#444}
.btn:disabled{opacity:.4;cursor:not-allowed}
.bar-bg{background:#bbb;border-radius:8px;height:12px;overflow:hidden;display:none}
.bar-fill{height:100%;background:#b0ff4b;border-radius:8px;width:0%;transition:width .3s}
.st{text-align:center;font-size:.85rem;color:#333;min-height:18px;word-break:break-all}
.back{color:#555;font-size:.78rem;text-decoration:none}
.back:hover{color:#b0ff4b}
.note{font-size:.75rem;color:#666;text-align:center;line-height:1.4}
</style>
</head>
<body>

<div class="tabs">
  <button class="tab active" onclick="switchTab('esp')">ESP32</button>
  <button class="tab"        onclick="switchTab('stm')">STM32 (FC)</button>
</div>

<!-- ESP32 панель -->
<div class="panel show" id="panel-esp">
  <div class="card">
    <div class="card-title">Оновлення прошивки ESP32</div>
    <div class="drop" id="drop-esp" onclick="document.getElementById('inp-esp').click()">
      Натисни або перетягни <b>.bin</b> файл сюди
    </div>
    <input type="file" id="inp-esp" accept=".bin">
    <div class="fname" id="fname-esp"></div>
    <button class="btn" id="btn-esp" disabled onclick="doEsp()">ОНОВИТИ ESP32</button>
    <div class="bar-bg" id="bar-bg-esp"><div class="bar-fill" id="bar-esp"></div></div>
    <div class="st" id="st-esp"></div>
    <div class="note">Файл: Sketch → Export Compiled Binary → DroneFlasher.ino.bin</div>
  </div>
</div>

<!-- STM32 панель -->
<div class="panel" id="panel-stm">
  <div class="card">
    <div class="card-title">Прошивка STM32 (польотний контролер)</div>
    <div class="drop" id="drop-stm" onclick="document.getElementById('inp-stm').click()">
      Натисни або перетягни <b>.hex</b> файл сюди
    </div>
    <input type="file" id="inp-stm" accept=".hex">
    <div class="fname" id="fname-stm"></div>
    <button class="btn" id="btn-stm" disabled onclick="doStm()">ПРОШИТИ STM32</button>
    <div class="bar-bg" id="bar-bg-stm"><div class="bar-fill" id="bar-stm"></div></div>
    <div class="st" id="st-stm"></div>
    <div class="note">Переведи FC в DFU-режим: затисни BOOT, підключи живлення</div>
  </div>
</div>

<a class="back" href="/">← Назад на головну</a>

<script>
function switchTab(t){
  document.querySelectorAll('.tab').forEach((b,i)=>b.classList.toggle('active',['esp','stm'][i]===t));
  document.getElementById('panel-esp').classList.toggle('show',t==='esp');
  document.getElementById('panel-stm').classList.toggle('show',t==='stm');
}

function setupDrop(id,inputId,fnameId,btnId){
  const drop=document.getElementById(id);
  const inp=document.getElementById(inputId);
  let file=null;
  inp.onchange=function(){if(this.files[0])set(this.files[0]);};
  drop.ondragover=e=>{e.preventDefault();drop.classList.add('over');};
  drop.ondragleave=()=>drop.classList.remove('over');
  drop.ondrop=e=>{e.preventDefault();drop.classList.remove('over');
    if(e.dataTransfer.files[0])set(e.dataTransfer.files[0]);};
  function set(f){
    file=f;
    document.getElementById(fnameId).textContent=f.name+' ('+Math.round(f.size/1024)+' KB)';
    document.getElementById(btnId).disabled=false;
  }
  return ()=>file;
}

const getEspFile=setupDrop('drop-esp','inp-esp','fname-esp','btn-esp');
const getStmFile=setupDrop('drop-stm','inp-stm','fname-stm','btn-stm');

async function doEsp(){
  const f=getEspFile(); if(!f)return;
  document.getElementById('btn-esp').disabled=true;
  document.getElementById('bar-bg-esp').style.display='block';
  setSt('esp','Завантаження...');
  const fd=new FormData();fd.append('file',f,f.name);
  const xhr=new XMLHttpRequest();
  xhr.open('POST','/ota/upload');
  xhr.upload.onprogress=e=>{
    if(e.lengthComputable){
      const p=Math.round(e.loaded/e.total*100);
      document.getElementById('bar-esp').style.width=p+'%';
      setSt('esp','Завантажено: '+p+'%');
    }
  };
  xhr.onload=()=>{
    try{
      const d=JSON.parse(xhr.responseText);
      d.ok?setSt('esp','Готово! ESP32 перезавантажується...')
          :setSt('esp','Помилка: '+(d.err||'?'));
    }catch(e){setSt('esp','Перезавантаження...');}
    if(!xhr.responseText.includes('"ok":true'))
      document.getElementById('btn-esp').disabled=false;
  };
  xhr.onerror=()=>{setSt('esp','Помилка з\'єднання');
    document.getElementById('btn-esp').disabled=false;};
  xhr.send(fd);
}

async function doStm(){
  const f=getStmFile(); if(!f)return;
  document.getElementById('btn-stm').disabled=true;
  document.getElementById('bar-bg-stm').style.display='block';
  setSt('stm','Завантаження hex...');
  // 1. Upload hex to ESP32 SPIFFS
  const fd=new FormData();fd.append('file',f,f.name);
  let r=await fetch('/upload/hex',{method:'POST',body:fd});
  if(!r.ok){setSt('stm','Помилка завантаження');
    document.getElementById('btn-stm').disabled=false;return;}
  // 2. Start flash (hex only)
  setSt('stm','Запуск прошивки...');
  r=await fetch('/flash',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({hex:true,dump:false,elrs:false})});
  if(!r.ok){setSt('stm','Помилка запуску');
    document.getElementById('btn-stm').disabled=false;return;}
  // 3. Poll progress
  const iv=setInterval(async()=>{
    const d=await(await fetch('/status')).json();
    document.getElementById('bar-stm').style.width=d.progress+'%';
    setSt('stm',d.message);
    if(!d.flashing){
      clearInterval(iv);
      document.getElementById('btn-stm').disabled=false;
    }
  },700);
}

function setSt(panel,t){document.getElementById('st-'+panel).textContent=t;}
</script>
</body>
</html>
)rawhtml";

// ─────────────────────────────────────────────────────────────────────────────
//  FC connection check — шукає реальну відповідь BF CLI
// ─────────────────────────────────────────────────────────────────────────────
static bool checkFcConnected() {
  if (g.flashing) return g.fcConnected;
  FC_Serial.begin(115200, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
  delay(60);
  while (FC_Serial.available()) FC_Serial.read();

  FC_Serial.print("\r\n#\r\n");
  delay(350);

  String resp = "";
  uint32_t t = millis();
  while (millis() - t < 200) {
    if (FC_Serial.available()) resp += (char)FC_Serial.read();
  }
  // Справжній BF CLI завжди повертає "# " або версію
  return resp.indexOf('#') >= 0 || resp.indexOf("Betaflight") >= 0
      || resp.indexOf("INAV") >= 0 || resp.indexOf("Cleanflight") >= 0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  STM32 UART Bootloader
// ─────────────────────────────────────────────────────────────────────────────
static bool stm32WaitAck(uint32_t timeout = 2000) {
  uint32_t t = millis();
  while (millis() - t < timeout) {
    if (FC_Serial.available()) {
      uint8_t b = FC_Serial.read();
      if (b == 0x79) return true;
      if (b == 0x1F) return false;
    }
  }
  return false;
}

static bool stm32Init() {
  digitalWrite(FC_BOOT0_PIN, HIGH);
  delay(10);
  digitalWrite(FC_NRST_PIN, LOW);
  delay(50);
  digitalWrite(FC_NRST_PIN, HIGH);
  delay(150);

  FC_Serial.begin(115200, SERIAL_8E1, FC_RX_PIN, FC_TX_PIN);
  delay(50);
  while (FC_Serial.available()) FC_Serial.read();

  FC_Serial.write(0x7F);
  return stm32WaitAck(3000);
}

static bool stm32ExtendedErase() {
  FC_Serial.write(0x44); FC_Serial.write(0xBB);
  if (!stm32WaitAck()) return false;
  FC_Serial.write(0xFF); FC_Serial.write(0xFF); FC_Serial.write(0x00);
  return stm32WaitAck(15000);
}

static bool stm32WriteMemory(uint32_t addr, const uint8_t* buf, uint8_t len) {
  FC_Serial.write(0x31); FC_Serial.write(0xCE);
  if (!stm32WaitAck()) return false;
  uint8_t cs = ((addr>>24)&0xFF)^((addr>>16)&0xFF)^((addr>>8)&0xFF)^(addr&0xFF);
  FC_Serial.write((addr>>24)&0xFF); FC_Serial.write((addr>>16)&0xFF);
  FC_Serial.write((addr>>8)&0xFF);  FC_Serial.write(addr&0xFF);
  FC_Serial.write(cs);
  if (!stm32WaitAck()) return false;
  uint8_t dcs = (uint8_t)(len - 1);
  FC_Serial.write((uint8_t)(len - 1));
  for (uint8_t i = 0; i < len; i++) { FC_Serial.write(buf[i]); dcs ^= buf[i]; }
  FC_Serial.write(dcs);
  return stm32WaitAck(2000);
}

static bool stm32Go(uint32_t addr = 0x08000000) {
  FC_Serial.write(0x21); FC_Serial.write(0xDE);
  if (!stm32WaitAck()) return false;
  uint8_t cs = ((addr>>24)&0xFF)^((addr>>16)&0xFF)^((addr>>8)&0xFF)^(addr&0xFF);
  FC_Serial.write((addr>>24)&0xFF); FC_Serial.write((addr>>16)&0xFF);
  FC_Serial.write((addr>>8)&0xFF);  FC_Serial.write(addr&0xFF);
  FC_Serial.write(cs);
  return stm32WaitAck(2000);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Intel HEX → STM32 flash
// ─────────────────────────────────────────────────────────────────────────────
static bool flashHex() {
  File f = SPIFFS.open("/hex.hex", "r");
  if (!f) { setMsg("HEX не знайдено"); return false; }

  setMsg("Ініціалізація STM32...", 2);
  addLog("Переводимо FC в bootloader (BOOT0+NRST)...");
  if (!stm32Init())         { f.close(); setMsg("Синхронізація STM32 failed — перевір BOOT0/NRST пайку"); return false; }
  setMsg("Стирання flash...", 5);
  if (!stm32ExtendedErase()){ f.close(); setMsg("Стирання failed");           return false; }

  uint32_t extAddr = 0;
  uint8_t  pageBuf[256];
  uint32_t pageBase  = 0;
  uint8_t  pageLen   = 0;
  bool     firstPage = true;
  size_t   fileSize  = f.size();
  size_t   filePos   = 0;

  auto flushPage = [&]() -> bool {
    if (!pageLen) return true;
    while (pageLen % 4) pageBuf[pageLen++] = 0xFF;
    if (!stm32WriteMemory(pageBase, pageBuf, pageLen)) {
      setMsg("Запис 0x" + String(pageBase, HEX) + " failed");
      return false;
    }
    return true;
  };

  setMsg("Запис прошивки...");

  while (f.available()) {
    String line = f.readStringUntil('\n');
    filePos += line.length() + 1;
    g.progress = 8 + (int)(82.0f * filePos / fileSize);
    line.trim();
    if (line.length() < 11 || line[0] != ':') continue;

    auto hb = [&](int p) -> uint8_t {
      char h[3] = {line[p], line[p+1], 0};
      return (uint8_t)strtol(h, nullptr, 16);
    };
    uint8_t  bc  = hb(1);
    uint16_t off = ((uint16_t)hb(3)<<8) | hb(5);
    uint8_t  rt  = hb(7);

    if (rt == 0x01) break;
    if (rt == 0x04) { extAddr = ((uint32_t)hb(9)<<24)|((uint32_t)hb(11)<<16); continue; }
    if (rt != 0x00) continue;

    uint32_t addr = extAddr | off;
    for (uint8_t i = 0; i < bc; i++) {
      uint8_t  b  = hb(9 + i*2);
      uint32_t wa = addr + i;
      if (firstPage) { pageBase = wa; firstPage = false; }
      if (pageLen == 256 || wa != pageBase + pageLen) {
        if (!flushPage()) { f.close(); return false; }
        pageBase = wa; pageLen = 0;
      }
      pageBuf[pageLen++] = b;
    }
    yield();
  }
  if (!flushPage()) { f.close(); return false; }
  f.close();

  digitalWrite(FC_BOOT0_PIN, LOW);
  stm32Go();
  delay(300);
  FC_Serial.begin(115200, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
  setMsg("STM32 прошито успішно");
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Betaflight CLI dump restore
// ─────────────────────────────────────────────────────────────────────────────
static bool restoreDump() {
  File f = SPIFFS.open("/dump.txt", "r");
  if (!f) { setMsg("dump не знайдено"); return false; }

  setMsg("Вхід в CLI...", 2);
  FC_Serial.begin(115200, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
  delay(300);
  FC_Serial.print("#\r\n");
  delay(400);
  while (FC_Serial.available()) FC_Serial.read();

  size_t total = 0;
  while (f.available()) { f.readStringUntil('\n'); total++; }
  f.seek(0);
  size_t done = 0;

  setMsg("Відправка дампу...");
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line[0] == '#') { done++; continue; }
    FC_Serial.print(line + "\r\n");
    uint32_t t = millis();
    while (millis() - t < 45) { if (FC_Serial.available()) FC_Serial.read(); }
    done++;
    g.progress = (int)(100.0f * done / total);
    if (done % 60 == 0) setMsg("Дамп " + String(done) + "/" + String(total));
    yield();
  }
  f.close();
  FC_Serial.print("save\r\n");
  delay(2500);
  setMsg("Дамп застосовано, збережено");
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  esptool SLIP protocol helpers  (ELRS через BF passthrough)
// ─────────────────────────────────────────────────────────────────────────────
#define SLIP_END     0xC0
#define SLIP_ESC     0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

static void slipWrite(const uint8_t* data, size_t len) {
  FC_Serial.write(SLIP_END);
  for (size_t i = 0; i < len; i++) {
    if      (data[i] == SLIP_END) { FC_Serial.write(SLIP_ESC); FC_Serial.write(SLIP_ESC_END); }
    else if (data[i] == SLIP_ESC) { FC_Serial.write(SLIP_ESC); FC_Serial.write(SLIP_ESC_ESC); }
    else FC_Serial.write(data[i]);
  }
  FC_Serial.write(SLIP_END);
}

static bool slipRead(uint8_t* out, size_t* outLen, uint32_t timeout = 2500) {
  uint32_t t   = millis();
  bool   inPkt = false;
  size_t idx   = 0;
  bool   esc   = false;
  while (millis() - t < timeout) {
    if (!FC_Serial.available()) { delay(1); continue; }
    uint8_t b = FC_Serial.read();
    if (b == SLIP_END) {
      if (inPkt && idx > 0) { *outLen = idx; return true; }
      inPkt = true; idx = 0; esc = false; continue;
    }
    if (!inPkt) continue;
    if (b == SLIP_ESC) { esc = true; continue; }
    if (esc) { b = (b == SLIP_ESC_END) ? SLIP_END : SLIP_ESC; esc = false; }
    if (idx < 512) out[idx++] = b;
  }
  return false;
}

static bool espCmd(uint8_t cmd, const uint8_t* data, uint16_t dLen,
                   uint32_t chk = 0, uint32_t timeout = 2500) {
  size_t pLen = 8 + dLen;
  uint8_t pkt[pLen];
  pkt[0] = 0x00; pkt[1] = cmd;
  pkt[2] = dLen & 0xFF; pkt[3] = (dLen >> 8) & 0xFF;
  pkt[4] = chk & 0xFF; pkt[5] = (chk>>8)&0xFF;
  pkt[6] = (chk>>16)&0xFF; pkt[7] = (chk>>24)&0xFF;
  if (dLen) memcpy(pkt + 8, data, dLen);
  slipWrite(pkt, pLen);

  uint8_t resp[512]; size_t rLen = 0;
  if (!slipRead(resp, &rLen, timeout)) return false;
  if (rLen < 10) return false;
  return resp[1] == cmd && resp[rLen - 2] == 0x00;
}

static bool espSync() {
  uint8_t sd[36] = {0x07,0x07,0x12,0x20};
  memset(sd + 4, 0x55, 32);
  for (int i = 0; i < 7; i++) {
    espCmd(0x08, sd, 36, 0, 600);
    uint8_t r[64]; size_t l = 0;
    if (slipRead(r, &l, 400) && l >= 10 && r[1] == 0x08 && r[l-2] == 0) return true;
    delay(30);
  }
  return false;
}

static uint32_t espChk(const uint8_t* d, size_t l) {
  uint32_t cs = 0xEF;
  for (size_t i = 0; i < l; i++) cs ^= d[i];
  return cs;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Betaflight serialpassthrough  →  esptool flash ELRS
// ─────────────────────────────────────────────────────────────────────────────
static bool enterBfCli() {
  FC_Serial.begin(115200, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
  delay(300);
  FC_Serial.print("#\r\n");
  delay(400);
  String r = "";
  uint32_t t = millis();
  while (millis() - t < 600) { if (FC_Serial.available()) r += (char)FC_Serial.read(); }
  return r.length() > 0;
}

static bool enterPassthrough(uint8_t uartN, uint32_t baud) {
  // BF CLI: serialpassthrough <index> <baud> rxtx
  // BF UART index = uartN - 1
  String cmd = "serialpassthrough " + String(uartN - 1) + " " + String(baud) + " rxtx\r\n";
  FC_Serial.print(cmd);
  delay(500);
  // After this BF stops parsing — it just bridges bytes to the target UART
  while (FC_Serial.available()) FC_Serial.read();

  // Reconfigure baud to match ELRS
  FC_Serial.end();
  delay(50);
  FC_Serial.begin(baud, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
  delay(100);
  return true;
}

static bool elrsResetToBootloader() {
  // Pulse GPIO0 low + RST to enter ESP32 bootloader
  // These pins are wired directly to ELRS pads (GPIO22 / GPIO23 of our ESP32)
  digitalWrite(ELRS_IO0_PIN, LOW);
  delay(10);
  digitalWrite(ELRS_RST_PIN, LOW);
  delay(100);
  digitalWrite(ELRS_RST_PIN, HIGH);
  delay(80);
  return true;
}

static bool flashElrs() {
  File f = SPIFFS.open("/elrs.bin", "r");
  if (!f) { setMsg("ELRS bin не знайдено"); return false; }
  size_t fileSize = f.size();

  setMsg("Вхід в BF CLI...", 2);
  if (!enterBfCli()) {
    f.close(); setMsg("FC не відповідає — Connect your FC dodik"); return false;
  }

  setMsg("Ресет ELRS → bootloader...", 5);
  elrsResetToBootloader();

  setMsg("Passthrough UART" + String(g_elrsUart) + " @ " + String(g_elrsBaud) + "...", 8);
  enterPassthrough(g_elrsUart, g_elrsBaud);

  setMsg("Синхронізація ESP32 (ELRS)...", 10);
  if (!espSync()) {
    f.close();
    FC_Serial.end(); delay(100);
    FC_Serial.begin(115200, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
    digitalWrite(ELRS_IO0_PIN, HIGH);
    setMsg("ELRS sync failed — перевір RX підключення до FC UART" + String(g_elrsUart));
    return false;
  }
  addLog("ELRS sync OK, розмір: " + String(fileSize) + " байт");

  // FLASH_BEGIN
  const uint32_t BLOCK_SZ  = 0x4000;
  const uint32_t CHUNK_SZ  = 0x0200;
  uint32_t numPkts  = (fileSize + CHUNK_SZ - 1) / CHUNK_SZ;
  uint32_t eraseSz  = ((fileSize + BLOCK_SZ - 1) / BLOCK_SZ) * BLOCK_SZ;
  uint32_t flashOff = 0x00000000;
  uint8_t  beginD[16];
  memcpy(beginD,      &eraseSz,  4);
  memcpy(beginD + 4,  &numPkts,  4);
  uint32_t cs = CHUNK_SZ;
  memcpy(beginD + 8,  &cs,       4);
  memcpy(beginD + 12, &flashOff, 4);
  if (!espCmd(0x02, beginD, 16, 0, 5000)) {
    f.close(); setMsg("FLASH_BEGIN failed"); return false;
  }

  // FLASH_DATA
  setMsg("Запис ELRS...", 12);
  uint8_t chunk[CHUNK_SZ + 16];
  uint32_t seq = 0;
  size_t   written = 0;

  while (f.available()) {
    memset(chunk + 16, 0xFF, CHUNK_SZ);
    f.read(chunk + 16, CHUNK_SZ);
    written += CHUNK_SZ;
    if (written > fileSize) written = fileSize;

    uint32_t dsz = CHUNK_SZ, z = 0;
    memcpy(chunk,      &dsz, 4);
    memcpy(chunk + 4,  &seq, 4);
    memcpy(chunk + 8,  &z,   4);
    memcpy(chunk + 12, &z,   4);

    uint32_t dataChk = espChk(chunk + 16, CHUNK_SZ);
    if (!espCmd(0x03, chunk, CHUNK_SZ + 16, dataChk, 3000)) {
      f.close(); setMsg("FLASH_DATA seq=" + String(seq) + " failed"); return false;
    }
    seq++;
    g.progress = 12 + (int)(85.0f * written / fileSize);
    yield();
  }
  f.close();

  // FLASH_END → reboot
  uint8_t endD[4] = {0,0,0,0};
  espCmd(0x04, endD, 4, 0, 2000);

  FC_Serial.end(); delay(200);
  FC_Serial.begin(115200, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
  digitalWrite(ELRS_IO0_PIN, HIGH);

  setMsg("ELRS прошито успішно");
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  FreeRTOS flash task
// ─────────────────────────────────────────────────────────────────────────────
struct FlashParams { bool doHex; bool doDump; bool doElrs; };
static FlashParams flashParams;

static void flashTaskFn(void* pv) {
  auto* p = (FlashParams*)pv;
  bool ok = true;

  addLog("=== Початок прошивки ===");

  if (p->doHex && ok) {
    setMsg("Прошивка FC (hex)...", 0);
    ok = flashHex();
    if (ok) { g.progress = 100; delay(600); }
  }
  if (p->doDump && ok) {
    // Перевірка підключення перед dump
    if (!checkFcConnected()) {
      setMsg("Connect your FC dodik — FC не знайдено для dump");
      ok = false;
    } else {
      setMsg("Відновлення дампу...", 0);
      ok = restoreDump();
      if (ok) { g.progress = 100; delay(600); }
    }
  }
  if (p->doElrs && ok) {
    setMsg("Прошивка ELRS...", 0);
    ok = flashElrs();
    if (ok) { g.progress = 100; delay(600); }
  }

  addLog(ok ? "=== Готово ===" : "=== Завершено з помилкою: " + g.message + " ===");
  if (ok) setMsg("Готово!");
  g.flashing = false;
  FC_Serial.begin(115200, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
  flashTask = nullptr;
  vTaskDelete(nullptr);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Upload handler helper
// ─────────────────────────────────────────────────────────────────────────────
static void handleUpload(const String& path,
                          AsyncWebServerRequest*, const String&,
                          size_t index, uint8_t* data, size_t len, bool final) {
  if (!index) { if (uploadFile) uploadFile.close(); uploadFile = SPIFFS.open(path, "w"); }
  if (uploadFile) uploadFile.write(data, len);
  if (final && uploadFile) uploadFile.close();
}

// ─────────────────────────────────────────────────────────────────────────────
//  setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  logMutex = xSemaphoreCreateMutex();

  pinMode(LED_PIN,      OUTPUT); digitalWrite(LED_PIN,      LOW);
  pinMode(FC_BOOT0_PIN, OUTPUT); digitalWrite(FC_BOOT0_PIN, LOW);
  pinMode(FC_NRST_PIN,  OUTPUT); digitalWrite(FC_NRST_PIN,  HIGH);
  pinMode(ELRS_RST_PIN, OUTPUT); digitalWrite(ELRS_RST_PIN, HIGH);
  pinMode(ELRS_IO0_PIN, OUTPUT); digitalWrite(ELRS_IO0_PIN, HIGH);

  // Startup blink (3x швидко)
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_PIN, i % 2 == 0 ? HIGH : LOW);
    delay(100);
  }

  SPIFFS.begin(true);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(10,0,0,1), IPAddress(10,0,0,1), IPAddress(255,255,255,0));
  WiFi.softAP(AP_SSID, AP_PASS);
  addLog("AP started: 10.0.0.1  SSID=" + String(AP_SSID));

  FC_Serial.begin(115200, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);

  // ── Routes ────────────────────────────────────────────────────────────────
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest* r) {
    StaticJsonDocument<128> doc;
    doc["connected"] = g.fcConnected;
    doc["flashing"]  = g.flashing;
    doc["progress"]  = g.progress;
    doc["message"]   = g.message;
    String out; serializeJson(doc, out);
    r->send(200, "application/json", out);
  });

  // Лог прошивки
  server.on("/log", HTTP_GET, [](AsyncWebServerRequest* r) {
    String page = "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>Flash Log</title>"
      "<style>*{box-sizing:border-box;margin:0;padding:0}"
      "body{background:#0d0d0d;color:#b0ff4b;font-family:monospace;"
      "padding:16px;font-size:.82rem;line-height:1.6}"
      "h2{color:#eee;margin-bottom:12px;font-size:1rem}"
      "pre{white-space:pre-wrap;word-break:break-all}"
      ".bar{display:flex;gap:10px;margin-bottom:14px}"
      "a{color:#888;font-size:.78rem;text-decoration:none}"
      "a:hover{color:#b0ff4b}"
      "button{background:#222;color:#b0ff4b;border:none;border-radius:6px;"
      "padding:6px 14px;cursor:pointer;font-size:.8rem}"
      "button:hover{background:#333}</style></head><body>"
      "<h2>Flash Log</h2>"
      "<div class='bar'>"
      "<a href='/'>← Назад</a>"
      "<button onclick='location.reload()'>Оновити</button>"
      "<button onclick='document.getElementById(\"log\").textContent=\"\"'>Очистити</button>"
      "</div>"
      "<pre id='log'>";
    if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      page += flashLog;
      xSemaphoreGive(logMutex);
    }
    page += "</pre>"
      "<script>window.scrollTo(0,document.body.scrollHeight);"
      "setTimeout(()=>location.reload(),3000);</script>"
      "</body></html>";
    r->send(200, "text/html", page);
  });

  server.on("/log/clear", HTTP_POST, [](AsyncWebServerRequest* r) {
    if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      flashLog = "";
      xSemaphoreGive(logMutex);
    }
    r->send(200, "text/plain", "ok");
  });

  auto makeUpload = [](const String& path, bool& ready) {
    return [path, &ready](AsyncWebServerRequest* r, const String& fn,
                          size_t i, uint8_t* d, size_t l, bool f) {
      if (g.flashing) return;
      handleUpload(path, r, fn, i, d, l, f);
      if (f) { ready = true; addLog("Файл отримано: " + fn + " -> " + path); }
    };
  };

  server.on("/upload/hex",  HTTP_POST, [](AsyncWebServerRequest* r){r->send(200);},
    makeUpload("/hex.hex",  g.hexReady));
  server.on("/upload/dump", HTTP_POST, [](AsyncWebServerRequest* r){r->send(200);},
    makeUpload("/dump.txt", g.dumpReady));
  server.on("/upload/elrs", HTTP_POST, [](AsyncWebServerRequest* r){r->send(200);},
    makeUpload("/elrs.bin", g.elrsReady));

  server.on("/flash", HTTP_POST,
    [](AsyncWebServerRequest* r) {
      if (g.flashing) { r->send(423, "text/plain", "busy"); return; }
      g.flashing = true; g.progress = 0; setMsg("Підготовка...");
      xTaskCreate(flashTaskFn, "flash", 10240, &flashParams, 1, &flashTask);
      r->send(200, "text/plain", "ok");
    },
    nullptr,
    [](AsyncWebServerRequest*, uint8_t* data, size_t len, size_t, size_t) {
      StaticJsonDocument<128> doc;
      deserializeJson(doc, data, len);
      flashParams.doHex  = doc["hex"]  | false;
      flashParams.doDump = doc["dump"] | false;
      flashParams.doElrs = doc["elrs"] | false;
      g_elrsUart = doc["elrsUart"] | 4;
      g_elrsBaud = doc["elrsBaud"] | 420000;
    }
  );

  server.on("/ota", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send_P(200, "text/html", OTA_HTML);
  });

  server.on("/ota/upload", HTTP_POST,
    [](AsyncWebServerRequest* r) {
      bool ok = !Update.hasError();
      AsyncWebServerResponse* resp = r->beginResponse(200, "application/json",
        ok ? "{\"ok\":true}" : "{\"ok\":false,\"err\":\"" + String(Update.errorString()) + "\"}");
      resp->addHeader("Connection", "close");
      r->send(resp);
      if (ok) { delay(500); ESP.restart(); }
    },
    [](AsyncWebServerRequest*, const String& fn, size_t index,
       uint8_t* data, size_t len, bool final) {
      if (!index) {
        addLog("OTA ESP32 старт: " + fn);
        uint32_t freeSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        Update.begin(freeSpace);
      }
      if (Update.isRunning()) Update.write(data, len);
      if (final) {
        Update.end(true);
        addLog(Update.hasError() ? "OTA failed: " + String(Update.errorString()) : "OTA OK");
      }
    }
  );

  server.begin();
  addLog("Web server started");
}

// ─────────────────────────────────────────────────────────────────────────────
//  loop
// ─────────────────────────────────────────────────────────────────────────────
static uint32_t lastCheck = 0;
static uint32_t lastBlink = 0;
static bool     ledState  = false;

void loop() {
  uint32_t now = millis();

  // LED:
  //   Прошивка → швидке мерехтіння (100ms)
  //   FC підключено → повільне мерехтіння (1000ms)
  //   Немає FC → повільне подвійне блимання
  if (g.flashing) {
    if (now - lastBlink > 100) { lastBlink = now; ledState = !ledState; digitalWrite(LED_PIN, ledState); }
  } else if (g.fcConnected) {
    if (now - lastBlink > 1000) { lastBlink = now; ledState = !ledState; digitalWrite(LED_PIN, ledState); }
  } else {
    // Подвійний блиск: on-off-on-off-pause
    static uint8_t blinkPhase = 0;
    static uint32_t blinkT = 0;
    const uint16_t pat[] = {80,80,80,600};
    if (now - blinkT > pat[blinkPhase % 4]) {
      blinkT = now;
      ledState = (blinkPhase % 4 < 2);
      digitalWrite(LED_PIN, ledState);
      blinkPhase++;
    }
  }

  if (!g.flashing && now - lastCheck > 2500) {
    lastCheck = now;
    g.fcConnected = checkFcConnected();
  }
  delay(5);
}
