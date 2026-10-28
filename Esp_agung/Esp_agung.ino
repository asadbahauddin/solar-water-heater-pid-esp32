/*
================================================================================
KODE UTAMA: WebSocket Setpoint + PID + Step Test
Solar Water Heater PID - Agung (Priyagung Dharma Hutomo)
ESP32 DevKit - Netlist v3.1 + revisi heater DC 12V/120W -> MOSFET IRLZ44N
================================================================================

INI MENCAKUP TAHAP 2-4 DARI JADWAL (JADWAL_IMPLEMENTASI_PID_v1.md):
  TAHAP 2 (step test open-loop)   -> mode STEP_TEST di bawah
  TAHAP 3 (hitung Kp/Ki/Kd)       -> BUKAN bagian kode ini. Itu analisis dari
                                     data yang di-export mode STEP_TEST, saya
                                     hitung manual/terpisah setelah kamu kirim
                                     datanya (atau proposal utk metode tuning
                                     sudah di-upload). Kp/Ki/Kd di kode ini
                                     cuma PLACEHOLDER konservatif, HARUS
                                     diganti lewat web dashboard setelah
                                     Tahap 3 selesai.
  TAHAP 4 (closed-loop bench test & fine-tuning) -> mode AUTO_PID di bawah,
                                     Kp/Ki/Kd bisa diubah live dari dashboard.

KEPUTUSAN DESAIN YANG SAYA AMBIL (baca dulu sebelum upload, koreksi kalau
gak sesuai maksud kamu):

1. HEATER OUTPUT: PWM ASLI (analogWrite ke GPIO27), BUKAN time-proportioning.
   Alasan: Q_HEATER sekarang MOSFET (IRLZ44N), bukan SSR/relay -- gak ada
   masalah wear-out dari switching cepat, jadi PWM asli lebih presisi dan
   lebih simpel kodenya dibanding window time-proportioning. Kalau kamu
   justru mau time-proportioning (mis. alasan di proposal/skripsi minta
   itu), kasih tahu, saya ubah.

2. PEMBACAAN DS18B20 DIBUAT NON-BLOCKING. requestTemperatures() versi
   default itu BLOCKING ~750ms per panggilan (buat 12-bit resolution) --
   kalau dipanggil blocking tiap loop sementara ada web server async +
   PID jalan bareng, itu bisa bikin WebSocket lag/jitter. Jadi di kode ini
   dipakai pola request->tunggu->baca terpisah (non-blocking), BUKAN
   delay().

3. MODE SELALU START DI MANUAL SETIAP REBOOT/POWER-ON (demi keamanan) --
   PID/STEP_TEST TIDAK auto-resume walau sebelum mati lampu modenya lagi
   AUTO_PID. Operator harus pilih mode lagi manual dari web dashboard.

4. SAFETY CUTOFF (75C, lihat SAFETY_CUTOFF_C) berlaku di SEMUA mode
   termasuk STEP_TEST -- heater WAJIB mati kalau suhu tangki tembus ini,
   gak peduli mode apa.

5. SETPOINT + Kp/Ki/Kd disimpan permanen (flash/NVS lewat Preferences.h),
   tapi MODE tidak (lihat poin 3).

6. WiFi: ESP32 jadi Access Point sendiri (SoftAP), IP default 192.168.4.1
   (bawaan ESP32, gak perlu di-set manual). SSID/password di bawah --
   GANTI kalau mau, ini cuma placeholder:
     SSID: SWH_PID_Agung
     Password: agung12345

7. LIBRARY WebSocket: ESPAsyncWebServer + AsyncTCP, pakai FORK ESP32Async
   (BUKAN me-no-dev yang sudah deprecated) -- install dari:
     https://github.com/ESP32Async/ESPAsyncWebServer
     https://github.com/ESP32Async/AsyncTCP
   Kalau sebelumnya pernah install versi me-no-dev, HAPUS folder library
   lama itu dulu manual sebelum install yang baru (sering konflik kalau
   dibiarkan dua-duanya).

8. PWM heater dibuat pakai API ledcSetup()+ledcAttachPin()+ledcWrite()
   (LEDC versi CHANNEL-BASED, bukan analogWrite, dan BUKAN ledcAttach()
   pin-based). INI SENGAJA DITULIS UNTUK ESP32 Arduino CORE 2.x (saya cek
   environment kamu pakai core 2.0.17) -- kalau kamu upgrade core ke 3.x
   nanti, API ini masih JALAN (core 3.x tetap menyediakan fungsi lama ini
   untuk backward-compatibility), tapi kalau mau API baru yang pin-based
   (ledcAttach 3-argumen) di core 3.x, bilang saya, saya tulis ulang.
   Channel LEDC yang dipakai: HEATER_PWM_CHANNEL (0) -- JANGAN dipakai
   ulang buat PWM lain di kode ini kalau nanti ditambah (ESP32 core 2.x
   cuma punya 16 channel LEDC, 0-15).

9. DATA STEP TEST: di-buffer di JAVASCRIPT (browser), bisa di-export CSV
   dari tombol "Download CSV" di dashboard. SEKALIAN juga di-print ke
   Serial Monitor format CSV tiap sample (backup kalau browser/koneksi
   putus di tengah test). Gak pakai SPIFFS/LittleFS di ESP32-nya biar
   gak nambah dependency.

CARA PAKAI:
  1. Upload kode ini (pastikan wiring Q_HEATER sudah IRLZ44N sesuai
     Tahap 1 di jadwal, SELESAI dan sudah di-test tanpa PID dulu).
  2. Connect WiFi HP/laptop ke SSID "SWH_PID_Agung".
  3. Buka browser ke http://192.168.4.1/
  4. Dashboard muncul: pilih mode, set parameter, lihat grafik live.
================================================================================
*/

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <BH1750.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>

// ---------------- PIN MAPPING (sesuai netlist v3.1 + revisi terakhir) ------
#define PIN_DS18B20_KOLEKTOR    4   // monitoring saja, bukan PV PID
#define PIN_DS18B20_TANGKI      5   // PV untuk PID (balik lagi ke GPIO5 atas keputusan klien --
                                    // CATATAN: GPIO5 ini yang dulu kebukti bermasalah lewat
                                    // test isolasi swap probe sebelum dipindah ke GPIO33 (yang
                                    // juga kebukti rusak). Test terbaru klien bilang GPIO5 &
                                    // GPIO4 "bisa" buat tangki, tapi metodenya belum saya
                                    // konfirmasi sama seperti test isolasi sebelumnya. Kalau
                                    // suhu tangki balik gak stabil/disconnect, ini kandidat
                                    // pertama yang perlu dicurigai lagi.
#define PIN_PUMP_MOSFET         26  // IRLB8721, on/off saja
#define PIN_HEATER_MOSFET       27  // IRLZ44N, PWM asli (analogWrite)
// PIN_BH1750_SDA1/SCL1 (25/32) SUDAH TIDAK DIPAKAI -- di PCB fisik BH1750
// disolder LANGSUNG ke bus I2C default (GPIO21/22), bukan ke Wire1. GPIO25/32
// bebas dipakai buat keperluan lain kalau perlu nanti.

// I2C BUS (Wire default): SDA=GPIO21, SCL=GPIO22 -- LCD (lewat level shifter)
// DAN BH1750 (kabel langsung, bypass shifter) BERBAGI bus yang sama, karena
// itu kondisi fisik di PCB (bukan pilihan desain) -- sudah divalidasi lewat
// I2C scanner: 0x27 (LCD) dan 0x23 (BH1750) dua-duanya kedetect, gak bentrok
// alamat. Risiko yang belum tervalidasi penuh: keandalan readLightLevel()
// (baca multi-byte) pas LCD juga aktif ditulis di loop yang sama -- pantau
// nilai lux di dashboard, kalau sering -1/aneh pas LCD update, kasih tahu.

// ---------------- KONFIGURASI LAIN ------------------------------------------
#define LCD_I2C_ADDR             0x27   // ASUMSI, cek pakai I2C scanner kalau blank
#define LCD_COLS                 20
#define LCD_ROWS                 4
#define BH1750_LUX_MALAM_AMBANG  10.0

#define WIFI_AP_SSID             "SWH_PID_Agung"
#define WIFI_AP_PASSWORD         "agung12345"

#define SENSOR_CYCLE_MS          2000UL   // 1 siklus baca+kontrol+kirim telemetry
#define DS18B20_CONV_WAIT_MS     800UL    // tunggu konversi 12-bit (~750ms + margin)

#define SAFETY_CUTOFF_C          75.0     // hard limit, independen dari mode apa pun
#define SETPOINT_MIN_C           30.0
#define SETPOINT_MAX_C           70.0

#define HEATER_PWM_FREQ_HZ       1000     // carrier PWM heater, jauh di atas bandwidth termal
#define HEATER_PWM_RES_BITS      8        // 0-255
#define HEATER_PWM_CHANNEL       0        // channel LEDC (API core 2.x, channel-based)

// ---------------- MODE & STATE ----------------------------------------------
enum ControlMode { MODE_MANUAL = 0, MODE_STEP_TEST = 1, MODE_AUTO_PID = 2 };
enum DayNightOverride { DN_AUTO = 0, DN_FORCE_DAY = 1, DN_FORCE_NIGHT = 2 };

ControlMode currentMode = MODE_MANUAL;       // SELALU start manual, lihat catatan #3
DayNightOverride dayNightOverride = DN_AUTO; // reset ke AUTO tiap boot, TIDAK dipersist

bool statusPump = false;
bool manualHeaterOn = false;       // buat bring-up test MOSFET di MODE_MANUAL
bool heaterEmergencyOff = false;   // true kalau safety cutoff pernah kena, perlu clear manual
float heaterDutyPercent = 0.0;     // 0-100, buat display & step test fixed-duty

// ---------------- SENSOR OBJEK -----------------------------------------------
OneWire oneWireKolektor(PIN_DS18B20_KOLEKTOR);
OneWire oneWireTangki(PIN_DS18B20_TANGKI);
DallasTemperature sensorKolektor(&oneWireKolektor);
DallasTemperature sensorTangki(&oneWireTangki);
BH1750 lightMeter;
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

float suhuKolektor = NAN;
float suhuTangki = NAN;
float lux = NAN;
bool isNightRaw = false;   // hasil mentah dari BH1750 (sebelum override)
bool isNightEffective = false; // setelah dipengaruhi dayNightOverride

// ---------------- PID STATE ---------------------------------------------------
// Pola clamped-integral anti-windup + derivative-on-measurement, sama dengan
// pendekatan Arduino PID Library (Brett Beauregard) yang sudah terbukti.
float pidSetpoint = 50.0;
float pidKp = 5.0;    // PLACEHOLDER konservatif -- GANTI setelah Tahap 3!
float pidKi = 0.05;   // PLACEHOLDER konservatif -- GANTI setelah Tahap 3!
float pidKd = 0.0;    // PLACEHOLDER -- mulai dari 0, baru ditambah kalau perlu
float pidOutputSum = 0.0;   // akumulasi integral, sudah termasuk dalam skala output 0-100
float pidLastInput = NAN;
const float PID_OUT_MIN = 0.0;
const float PID_OUT_MAX = 100.0;

// ---------------- PERSISTENCE -------------------------------------------------
Preferences prefs;

// ---------------- WEB SERVER & WEBSOCKET --------------------------------------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ---------------- NON-BLOCKING DS18B20 STATE MACHINE --------------------------
enum DsPhase { DS_IDLE, DS_WAITING_CONVERSION };
DsPhase dsPhase = DS_IDLE;
unsigned long dsRequestTime = 0;
unsigned long lastCycleTime = 0;

// ---------------- FORWARD DECLARATIONS ----------------------------------------
// DITULIS MANUAL (bukan andalkan auto-prototype Arduino IDE) karena file ini
// punya raw string literal besar (INDEX_HTML, R"HTMLPAGE(...)HTMLPAGE") yang
// isinya banyak kurung kurawal {} dari HTML/CSS/JS -- itu bikin ctags/auto-
// prototype generator Arduino IDE salah hitung brace-matching dan GAGAL
// bikin prototype buat semua fungsi yang didefinisikan SETELAH literal itu
// (makanya error "not declared in this scope" muncul buat onWsEvent,
// handleCommand, updateDayNight, dst -- bukan typo, ini bug lama di toolchain
// Arduino, solusinya ya ini: declare manual sebelum dipakai).
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len);
void handleCommand(String msg);
void updateDayNight();
void runControlLogic();
float computePID(float input, float setpoint);
void sendTelemetry();
void updateLCD();
void logSerialCSV(unsigned long t);

// ================================================================================
// HTML DASHBOARD (single file, tanpa dependency CDN -- SoftAP gak ada internet)
// ================================================================================
const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SWH PID - Agung</title>
<style>
  body { font-family: sans-serif; background:#111; color:#eee; margin:0; padding:12px; }
  h1 { font-size:18px; margin:4px 0 12px 0; }
  .card { background:#1c1c1c; border-radius:8px; padding:12px; margin-bottom:12px; }
  .row { display:flex; flex-wrap:wrap; gap:8px; align-items:center; margin-bottom:8px; }
  label { font-size:13px; color:#aaa; min-width:90px; display:inline-block; }
  input[type=number] { width:80px; padding:4px; border-radius:4px; border:1px solid #444; background:#222; color:#eee; }
  button { padding:7px 12px; border-radius:6px; border:none; background:#2a7; color:#fff; font-size:13px; cursor:pointer; }
  button.off { background:#a33; }
  button.sel { background:#27a; }
  button.warn { background:#c70; }
  #chart { width:100%; height:260px; background:#000; border-radius:6px; }
  .statline { font-size:14px; margin:3px 0; }
  .big { font-size:22px; font-weight:bold; }
  .ok { color:#4f8; }
  .bad { color:#f55; }
  .modegroup button { margin-right:4px; }
</style>
</head>
<body>
<h1>Solar Water Heater PID - Dashboard</h1>

<div class="card">
  <div class="statline">Suhu Tangki (PV): <span class="big" id="st">--</span> C</div>
  <div class="statline">Suhu Kolektor: <span id="sk">--</span> C</div>
  <div class="statline">Cahaya: <span id="lux">--</span> lux (<span id="dn">--</span>)</div>
  <div class="statline">Pompa: <span id="pumpstat">--</span> | Heater duty: <span id="heaterstat">--</span>% | Mode: <span id="modestat">--</span></div>
  <div class="statline" id="emergline" style="display:none;" class="bad">** SAFETY CUTOFF AKTIF - HEATER DIPAKSA OFF **</div>
</div>

<div class="card">
  <canvas id="chart"></canvas>
  <div class="row" style="margin-top:8px;">
    <button onclick="clearChart()">Clear Grafik</button>
    <button onclick="downloadCSV()">Download CSV</button>
  </div>
</div>

<div class="card">
  <div class="row modegroup">
    <label>Mode:</label>
    <button id="btnManual" onclick="setMode('MANUAL')">Manual</button>
    <button id="btnStepTest" onclick="setMode('STEP_TEST')">Step Test</button>
    <button id="btnAutoPid" onclick="setMode('AUTO_PID')">Auto PID</button>
  </div>
  <div class="row">
    <label>Pompa:</label>
    <button onclick="cmd('PUMP:ON')">ON</button>
    <button class="off" onclick="cmd('PUMP:OFF')">OFF</button>
  </div>
  <div class="row">
    <label>Heater (manual, mode Manual saja):</label>
    <button onclick="cmd('HEATER:ON')">ON</button>
    <button class="off" onclick="cmd('HEATER:OFF')">OFF</button>
  </div>
  <div class="row">
    <label>Siang/Malam:</label>
    <button id="btnDnAuto" onclick="setDn('AUTO')">Auto (sensor)</button>
    <button id="btnDnDay" onclick="setDn('FORCE_DAY')">Force Siang</button>
    <button id="btnDnNight" onclick="setDn('FORCE_NIGHT')">Force Malam</button>
  </div>
  <div class="row">
    <button class="warn" onclick="cmd('CLEAR_EMERGENCY')">Clear Safety Cutoff (setelah suhu turun aman)</button>
  </div>
</div>

<div class="card">
  <div class="row"><label>Setpoint (C):</label><input type="number" id="inSp" step="0.5"></div>
  <div class="row"><label>Kp:</label><input type="number" id="inKp" step="0.01"></div>
  <div class="row"><label>Ki:</label><input type="number" id="inKi" step="0.001"></div>
  <div class="row"><label>Kd:</label><input type="number" id="inKd" step="0.01"></div>
  <div class="row"><button onclick="saveParams()">Simpan ke ESP32 (persist)</button></div>
</div>

<script>
var ws = new WebSocket('ws://' + location.host + '/ws');
var chartData = []; // {t, tangki, kolektor, sp}
var startTime = null;

ws.onmessage = function(ev) {
  var d = JSON.parse(ev.data);
  document.getElementById('st').textContent = d.tangki.toFixed(2);
  document.getElementById('sk').textContent = d.kolektor.toFixed(2);
  document.getElementById('lux').textContent = d.lux.toFixed(1);
  document.getElementById('dn').textContent = d.night ? 'MALAM' : 'SIANG';
  document.getElementById('pumpstat').textContent = d.pump ? 'ON' : 'OFF';
  document.getElementById('heaterstat').textContent = d.heaterDuty.toFixed(0);
  document.getElementById('modestat').textContent = d.mode;
  document.getElementById('emergline').style.display = d.emergency ? 'block' : 'none';

  highlightMode(d.mode);
  highlightDn(d.dnOverride);

  if (!document.getElementById('inSp').matches(':focus')) document.getElementById('inSp').value = d.sp;
  if (!document.getElementById('inKp').matches(':focus')) document.getElementById('inKp').value = d.kp;
  if (!document.getElementById('inKi').matches(':focus')) document.getElementById('inKi').value = d.ki;
  if (!document.getElementById('inKd').matches(':focus')) document.getElementById('inKd').value = d.kd;

  if (startTime === null) startTime = d.t;
  chartData.push({t: (d.t - startTime)/1000.0, tangki: d.tangki, kolektor: d.kolektor, sp: d.sp});
  if (chartData.length > 3000) chartData.shift();
  drawChart();
};

function cmd(s) { ws.send(s); }
function setMode(m) { cmd('MODE:' + m); }
function setDn(m) { cmd('DN:' + m); }
function saveParams() {
  cmd('SET_SP:' + document.getElementById('inSp').value);
  cmd('SET_KP:' + document.getElementById('inKp').value);
  cmd('SET_KI:' + document.getElementById('inKi').value);
  cmd('SET_KD:' + document.getElementById('inKd').value);
  cmd('SAVE_PARAMS');
}
function highlightMode(m) {
  document.getElementById('btnManual').className = (m=='MANUAL')?'sel':'';
  document.getElementById('btnStepTest').className = (m=='STEP_TEST')?'sel':'';
  document.getElementById('btnAutoPid').className = (m=='AUTO_PID')?'sel':'';
}
function highlightDn(m) {
  document.getElementById('btnDnAuto').className = (m=='AUTO')?'sel':'';
  document.getElementById('btnDnDay').className = (m=='FORCE_DAY')?'sel':'';
  document.getElementById('btnDnNight').className = (m=='FORCE_NIGHT')?'sel':'';
}
function clearChart() { chartData = []; startTime = null; drawChart(); }

function downloadCSV() {
  var csv = 'detik,suhu_tangki,suhu_kolektor,setpoint\n';
  for (var i=0;i<chartData.length;i++) {
    var r = chartData[i];
    csv += r.t.toFixed(1)+','+r.tangki.toFixed(2)+','+r.kolektor.toFixed(2)+','+r.sp.toFixed(2)+'\n';
  }
  var blob = new Blob([csv], {type:'text/csv'});
  var a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'swh_data.csv';
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
}

function drawChart() {
  var c = document.getElementById('chart');
  var ctx = c.getContext('2d');
  c.width = c.clientWidth; c.height = c.clientHeight;
  ctx.clearRect(0,0,c.width,c.height);
  if (chartData.length < 2) return;

  var minT = chartData[0].t, maxT = chartData[chartData.length-1].t;
  var minY = 999, maxY = -999;
  for (var i=0;i<chartData.length;i++) {
    minY = Math.min(minY, chartData[i].tangki, chartData[i].sp);
    maxY = Math.max(maxY, chartData[i].tangki, chartData[i].sp);
  }
  minY -= 2; maxY += 2;
  if (maxY - minY < 1) { maxY += 1; minY -= 1; }

  function px(t) { return 10 + (c.width-20) * (t-minT)/(maxT-minT || 1); }
  function py(y) { return c.height-10 - (c.height-30) * (y-minY)/(maxY-minY || 1); }

  function drawLine(key, color) {
    ctx.beginPath();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    for (var i=0;i<chartData.length;i++) {
      var x = px(chartData[i].t), y = py(chartData[i][key]);
      if (i==0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }
  drawLine('tangki', '#4af');
  drawLine('sp', '#fa4');

  ctx.fillStyle = '#888';
  ctx.font = '11px sans-serif';
  ctx.fillText('suhu tangki (biru) / setpoint (oranye)', 12, 14);
  ctx.fillText(maxY.toFixed(1)+'C', 2, 20);
  ctx.fillText(minY.toFixed(1)+'C', 2, c.height-4);
}

window.onresize = drawChart;
</script>
</body>
</html>
)HTMLPAGE";

// ================================================================================
// SETUP
// ================================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n=== SWH PID WebSocket - boot ==="));

  pinMode(PIN_PUMP_MOSFET, OUTPUT);
  digitalWrite(PIN_PUMP_MOSFET, LOW);

  ledcSetup(HEATER_PWM_CHANNEL, HEATER_PWM_FREQ_HZ, HEATER_PWM_RES_BITS);
  ledcAttachPin(PIN_HEATER_MOSFET, HEATER_PWM_CHANNEL);
  ledcWrite(HEATER_PWM_CHANNEL, 0);   // pastikan OFF sejak awal

  // --- I2C BUS default (GPIO21/22): LCD (lewat level shifter) + BH1750
  // (kabel langsung) BERBAGI bus ini -- kondisi fisik PCB, lihat catatan
  // di bagian PIN MAPPING. ---
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print(F("SWH PID - booting"));

  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire)) {
    Serial.println(F("[WARNING] BH1750 tidak terdeteksi (bus default, GPIO21/22)"));
  }

  // --- DS18B20, NON-BLOCKING: matikan auto-wait ---
  sensorKolektor.begin();
  sensorTangki.begin();
  sensorKolektor.setWaitForConversion(false);
  sensorTangki.setWaitForConversion(false);
  if (sensorKolektor.getDeviceCount() == 0) Serial.println(F("[WARNING] DS18B20 kolektor (GPIO4) tidak terdeteksi"));
  if (sensorTangki.getDeviceCount() == 0)   Serial.println(F("[WARNING] DS18B20 tangki (GPIO33) tidak terdeteksi"));

  // --- Preferences: load setpoint & gains tersimpan, kalau ada ---
  prefs.begin("swhpid", false);
  pidSetpoint = prefs.getFloat("sp", pidSetpoint);
  pidKp       = prefs.getFloat("kp", pidKp);
  pidKi       = prefs.getFloat("ki", pidKi);
  pidKd       = prefs.getFloat("kd", pidKd);
  pidSetpoint = constrain(pidSetpoint, SETPOINT_MIN_C, SETPOINT_MAX_C);

  // --- WiFi SoftAP ---
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
  Serial.print(F("SoftAP IP: "));
  Serial.println(WiFi.softAPIP());   // harusnya 192.168.4.1

  // --- WebSocket ---
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });
  server.begin();

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("AP: SWH_PID_Agung"));
  lcd.setCursor(0,1);
  lcd.print(WiFi.softAPIP());

  lastCycleTime = millis();
  Serial.println(F("Setup selesai. Mode awal: MANUAL (lihat catatan keamanan #3)."));
  Serial.println(F("CSV backup header:"));
  Serial.println(F("ms,suhu_tangki,suhu_kolektor,lux,night,mode,heaterDuty,pump"));
}

// ================================================================================
// WEBSOCKET EVENT & COMMAND HANDLING
// ================================================================================
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    // Cuma handle pesan text single-frame -- cukup buat command pendek kita,
    // gak bangun ulang pesan fragmented (gak perlu buat kasus ini).
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      String msg;
      msg.reserve(len);
      for (size_t i = 0; i < len; i++) msg += (char)data[i];
      handleCommand(msg);
    }
  }
}

void handleCommand(String msg) {
  if (msg == "MODE:MANUAL") {
    currentMode = MODE_MANUAL;
    manualHeaterOn = false;   // selalu mulai OFF tiap masuk mode Manual, demi keamanan
    ledcWrite(HEATER_PWM_CHANNEL, 0);
  } else if (msg == "MODE:STEP_TEST") {
    currentMode = MODE_STEP_TEST;
    pidOutputSum = 0; // reset biar gak nyampur sama sesi PID sebelumnya
  } else if (msg == "MODE:AUTO_PID") {
    currentMode = MODE_AUTO_PID;
    pidOutputSum = 0;
    pidLastInput = NAN;
  } else if (msg == "PUMP:ON") {
    statusPump = true;
  } else if (msg == "PUMP:OFF") {
    statusPump = false;
  } else if (msg == "HEATER:ON") {
    manualHeaterOn = true;
  } else if (msg == "HEATER:OFF") {
    manualHeaterOn = false;
  } else if (msg == "DN:AUTO") {
    dayNightOverride = DN_AUTO;
  } else if (msg == "DN:FORCE_DAY") {
    dayNightOverride = DN_FORCE_DAY;
  } else if (msg == "DN:FORCE_NIGHT") {
    dayNightOverride = DN_FORCE_NIGHT;
  } else if (msg == "CLEAR_EMERGENCY") {
    if (suhuTangki < SAFETY_CUTOFF_C - 2.0) {   // hysteresis kecil, cegah langsung trip lagi
      heaterEmergencyOff = false;
    }
  } else if (msg.startsWith("SET_SP:")) {
    pidSetpoint = constrain(msg.substring(7).toFloat(), SETPOINT_MIN_C, SETPOINT_MAX_C);
  } else if (msg.startsWith("SET_KP:")) {
    pidKp = msg.substring(7).toFloat();
  } else if (msg.startsWith("SET_KI:")) {
    pidKi = msg.substring(7).toFloat();
  } else if (msg.startsWith("SET_KD:")) {
    pidKd = msg.substring(7).toFloat();
  } else if (msg == "SAVE_PARAMS") {
    prefs.putFloat("sp", pidSetpoint);
    prefs.putFloat("kp", pidKp);
    prefs.putFloat("ki", pidKi);
    prefs.putFloat("kd", pidKd);
    Serial.println(F("[INFO] Parameter disimpan ke flash (Preferences)."));
  }
}

// ================================================================================
// LOOP
// ================================================================================
void loop() {
  unsigned long now = millis();

  // --- State machine baca DS18B20 non-blocking ---
  if (dsPhase == DS_IDLE && (now - lastCycleTime >= SENSOR_CYCLE_MS)) {
    lastCycleTime = now;
    sensorKolektor.requestTemperatures();
    sensorTangki.requestTemperatures();
    dsRequestTime = now;
    dsPhase = DS_WAITING_CONVERSION;
  }

  if (dsPhase == DS_WAITING_CONVERSION && (now - dsRequestTime >= DS18B20_CONV_WAIT_MS)) {
    suhuKolektor = sensorKolektor.getTempCByIndex(0);
    suhuTangki   = sensorTangki.getTempCByIndex(0);
    lux = lightMeter.readLightLevel();
    dsPhase = DS_IDLE;

    updateDayNight();
    runControlLogic();
    sendTelemetry();
    updateLCD();
    logSerialCSV(now);
  }

  // WebSocket cleanup (buang client mati) -- ESPAsyncWebServer urus event async,
  // tapi cleanup periodik ini praktik yang disarankan librarynya.
  ws.cleanupClients();
}

void updateDayNight() {
  isNightRaw = (lux >= 0) ? (lux < BH1750_LUX_MALAM_AMBANG) : isNightRaw; // lux<0 = gagal baca, pertahankan state lama
  switch (dayNightOverride) {
    case DN_FORCE_DAY:   isNightEffective = false; break;
    case DN_FORCE_NIGHT: isNightEffective = true;  break;
    default:              isNightEffective = isNightRaw; break;
  }
}

void runControlLogic() {
  // --- Safety cutoff, berlaku SELALU, di atas mode apa pun ---
  if (!isnan(suhuTangki) && suhuTangki >= SAFETY_CUTOFF_C) {
    heaterEmergencyOff = true;
  }

  if (heaterEmergencyOff) {
    heaterDutyPercent = 0;
    ledcWrite(HEATER_PWM_CHANNEL, 0);
    digitalWrite(PIN_PUMP_MOSFET, statusPump ? HIGH : LOW); // pompa tetap boleh manual
    return;
  }

  digitalWrite(PIN_PUMP_MOSFET, statusPump ? HIGH : LOW);

  switch (currentMode) {
    case MODE_MANUAL:
      // Manual ON/OFF heater -- dipakai buat bring-up test fisik MOSFET
      // (lihat command "HEATER:ON"/"HEATER:OFF"), TIDAK ada logic PID di sini.
      heaterDutyPercent = manualHeaterOn ? 100.0 : 0.0;
      ledcWrite(HEATER_PWM_CHANNEL, manualHeaterOn ? 255 : 0);
      break;

    case MODE_STEP_TEST:
      // Heater full ON (100% duty) terus-menerus sampai mode diganti atau
      // safety cutoff kena -- ini JANTUNG Tahap 2 di jadwal.
      heaterDutyPercent = 100.0;
      ledcWrite(HEATER_PWM_CHANNEL, 255);
      break;

    case MODE_AUTO_PID:
      if (!isNightEffective) {
        // Siang -> heater WAJIB off, PID gak jalan, sesuai gating siang/malam.
        heaterDutyPercent = 0;
        ledcWrite(HEATER_PWM_CHANNEL, 0);
        pidOutputSum = 0;
        pidLastInput = NAN;
      } else {
        heaterDutyPercent = computePID(suhuTangki, pidSetpoint);
        ledcWrite(HEATER_PWM_CHANNEL, (int)(heaterDutyPercent * 255.0 / 100.0));
      }
      break;
  }
}

// PID diskrit, clamped-integral anti-windup, derivative-on-measurement.
float computePID(float input, float setpoint) {
  if (isnan(input)) return 0; // sensor gagal baca -> jangan drive heater buta

  float dtSec = SENSOR_CYCLE_MS / 1000.0;
  float error = setpoint - input;

  pidOutputSum += (pidKi * error * dtSec);
  pidOutputSum = constrain(pidOutputSum, PID_OUT_MIN, PID_OUT_MAX);

  float dInput = isnan(pidLastInput) ? 0 : (input - pidLastInput) / dtSec;
  pidLastInput = input;

  float output = pidKp * error + pidOutputSum - pidKd * dInput;
  return constrain(output, PID_OUT_MIN, PID_OUT_MAX);
}

void sendTelemetry() {
  String modeStr = (currentMode==MODE_MANUAL)?"MANUAL":(currentMode==MODE_STEP_TEST)?"STEP_TEST":"AUTO_PID";
  String dnStr = (dayNightOverride==DN_AUTO)?"AUTO":(dayNightOverride==DN_FORCE_DAY)?"FORCE_DAY":"FORCE_NIGHT";

  String json = "{";
  json += "\"t\":" + String(millis()) + ",";
  json += "\"tangki\":" + String(isnan(suhuTangki)?-127:suhuTangki, 2) + ",";
  json += "\"kolektor\":" + String(isnan(suhuKolektor)?-127:suhuKolektor, 2) + ",";
  json += "\"lux\":" + String(lux, 1) + ",";
  json += "\"night\":" + String(isNightEffective ? "true" : "false") + ",";
  json += "\"pump\":" + String(statusPump ? "true" : "false") + ",";
  json += "\"heaterDuty\":" + String(heaterDutyPercent, 1) + ",";
  json += "\"mode\":\"" + modeStr + "\",";
  json += "\"dnOverride\":\"" + dnStr + "\",";
  json += "\"emergency\":" + String(heaterEmergencyOff ? "true" : "false") + ",";
  json += "\"sp\":" + String(pidSetpoint, 2) + ",";
  json += "\"kp\":" + String(pidKp, 3) + ",";
  json += "\"ki\":" + String(pidKi, 3) + ",";
  json += "\"kd\":" + String(pidKd, 3);
  json += "}";

  ws.textAll(json);
}

void updateLCD() {
  lcd.setCursor(0,0);
  lcd.print(F("Kol:"));
  lcd.print(isnan(suhuKolektor) ? "ERR  " : String(suhuKolektor,1));

  lcd.setCursor(10,0);
  lcd.print(F("Tgk:"));
  lcd.print(isnan(suhuTangki) ? "ERR  " : String(suhuTangki,1));

  lcd.setCursor(0,1);
  lcd.print(F("SP:"));
  lcd.print(pidSetpoint,1);
  lcd.print(F(" Dty:"));
  lcd.print((int)heaterDutyPercent);
  lcd.print(F("%  "));

  lcd.setCursor(0,2);
  lcd.print(F("Mode:"));
  lcd.print(currentMode==MODE_MANUAL?"MANUAL  ":currentMode==MODE_STEP_TEST?"STEPTEST":"AUTOPID ");

  lcd.setCursor(0,3);
  lcd.print(heaterEmergencyOff ? F("!! SAFETY CUTOFF !!") : (isNightEffective ? F("MALAM - PID aktif  ") : F("SIANG - heater off  ")));
}

void logSerialCSV(unsigned long t) {
  Serial.print(t); Serial.print(",");
  Serial.print(suhuTangki,2); Serial.print(",");
  Serial.print(suhuKolektor,2); Serial.print(",");
  Serial.print(lux,1); Serial.print(",");
  Serial.print(isNightEffective?1:0); Serial.print(",");
  Serial.print(currentMode); Serial.print(",");
  Serial.print(heaterDutyPercent,1); Serial.print(",");
  Serial.println(statusPump?1:0);
}