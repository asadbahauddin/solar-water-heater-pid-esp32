# Solar Water Heater dengan Kendali Suhu PID Berbasis ESP32

Sistem pemanas air tenaga surya (*solar water heater*, SWH) dengan pemanas listrik cadangan yang dikendalikan secara *closed-loop* menggunakan algoritma PID diskrit pada mikrokontroler ESP32. Pemantauan dan pengaturan parameter dilakukan secara nirkabel melalui *dashboard* web yang di-*hosting* langsung oleh ESP32 (WiFi SoftAP + WebSocket), dan data telemetri direkam ke berkas CSV oleh *logger* Python yang tahan terhadap putus koneksi.

---

## Daftar Isi

1. [Deskripsi Sistem](#1-deskripsi-sistem)
2. [Arsitektur Perangkat Keras](#2-arsitektur-perangkat-keras)
3. [Strategi Kendali](#3-strategi-kendali)
4. [Mekanisme Keselamatan](#4-mekanisme-keselamatan)
5. [Antarmuka Komunikasi (WebSocket)](#5-antarmuka-komunikasi-websocket)
6. [Akuisisi Data (Data Logger)](#6-akuisisi-data-data-logger)
7. [Prosedur Eksperimen](#7-prosedur-eksperimen)
8. [Struktur Repositori](#8-struktur-repositori)
9. [Cara Penggunaan](#9-cara-penggunaan)
10. [Keterbatasan dan Pekerjaan Lanjutan](#10-keterbatasan-dan-pekerjaan-lanjutan)

---

## 1. Deskripsi Sistem

Sistem memakai **sirkulasi paksa (loop tertutup)** antara kolektor surya dan tangki, ditambah pemanas listrik sebagai sumber panas cadangan. Variabel proses (PV) kedua kontroler sama, yaitu **suhu tangki**, tetapi aktuatornya berganti sesuai kondisi penyinaran:

| Kondisi | Sumber panas | Aktuator (variabel manipulasi) | Kontroler |
|---|---|---|---|
| **Siang** | Kolektor surya | Kecepatan pompa sirkulasi (PWM) | PID siang |
| **Malam / mendung** | Elemen pemanas listrik | *Duty cycle* heater (PWM) | PID malam |

Pada siang hari, pompa memindahkan air panas dari kolektor ke tangki. Laju aliran yang lebih tinggi berarti laju perpindahan kalor yang lebih besar, sehingga kecepatan pompa menjadi variabel manipulasi yang efektif. Saat malam atau mendung (penyinaran di bawah ambang), kolektor tidak lagi menjadi sumber panas. Pompa dimatikan agar air tangki tidak bersirkulasi melewati kolektor yang dingin, dan heater mengambil alih.

```
                         ┌──────────────┐  ── air panas ──▶  ┌────────────┐
  Radiasi surya ──────▶  │   Kolektor   │                    │   Tangki   │
                         │ (DS18B20 #1) │  ◀── air dingin ── │(DS18B20 #2)│
                         └──────────────┘        ▲           └─────▲──────┘
                                                 │                 │ panas
                                        Pompa DC (PWM)        Heater (PWM)
                                                 ▲                 ▲
                                            PID siang          PID malam
                                                 ▲                 ▲
  BH1750 (lux) ──▶ gerbang siang/malam ─────────┴────────┬────────┘
  T_kolektor − T_tangki ──▶ gerbang ΔT (pompa) ──────────┘
```

## 2. Arsitektur Perangkat Keras

### 2.1 Komponen Utama

| Fungsi | Komponen |
|---|---|
| Mikrokontroler | ESP32 DevKit (30 pin), logika 3,3 V |
| Sensor suhu | DS18B20 ×2 (kolektor & tangki), resolusi 12-bit, tercelup langsung di air |
| Sensor cahaya | BH1750 (I²C, alamat 0x23) |
| Tampilan lokal | LCD 20×4 I²C (alamat 0x27) melalui *level shifter* BSS138 |
| Driver heater | MOSFET *logic-level* IRLZ44N, PWM 1 kHz / 8-bit |
| Driver pompa | MOSFET *logic-level* IRLB8721PbF, PWM 20 kHz / 8-bit (pompa DC *brushed* 12 V / 5 A) |
| Catu daya | Panel surya + MPPT + baterai 12 V; buck XL4015/XL4016 (12→7 V untuk ESP32, 12→5 V untuk LCD); LDO AMS1117-3.3 untuk sensor |
| Proteksi | Sekring per cabang, dioda Schottky SB1040 (polaritas balik), TVS SMBJ18A |

Daftar komponen lengkap ada di [Komponen agung.txt](Komponen%20agung.txt) dan bill of materials di [NETLIST_SWH_PID_Agung_v1.txt](NETLIST_SWH_PID_Agung_v1.txt) bagian 10.

### 2.2 Pemetaan Pin ESP32

| GPIO | Fungsi | Keterangan |
|---|---|---|
| 4 | DS18B20 kolektor | 1-Wire, *pull-up* 4,7 kΩ ke 3,3 V |
| 5 | DS18B20 tangki (PV) | 1-Wire, *pull-up* 4,7 kΩ ke 3,3 V |
| 21 / 22 | I²C SDA / SCL | Bus bersama LCD (via *level shifter*) dan BH1750 |
| 26 | Gate MOSFET pompa | PWM LEDC kanal 2 (bukan kanal 1, karena kanal 0 dan 1 berbagi *timer*) |
| 27 | Gate MOSFET heater | PWM LEDC kanal 0 |

Pin *strapping* (GPIO 0/2/12/15), pin *flash* (GPIO 6–11), dan pin *input-only* (GPIO 34–39) sengaja tidak dipakai untuk keluaran.

### 2.3 Desain Catu Daya

ESP32 disuplai pada 7 V (bukan 5 V) ke pin VIN. Saat transmisi WiFi, ESP32 menarik arus puncak sesaat sekitar 300–500 mA. Margin tegangan masukan yang lebih besar menjaga regulator *onboard* tetap di atas tegangan *dropout*-nya, sehingga mencegah *brownout*. LCD dan sensor memakai rel terpisah agar tidak terganggu transien tersebut. Rincian perhitungan *sizing* panel, baterai, lebar *trace*, dan topologi *ground* ada di berkas netlist.

> **Catatan revisi:** netlist v3.1 masih mendokumentasikan varian heater AC 500 W dengan SSR (*time-proportional control*) dan aliran kolektor → tangki secara termosifon. Firmware saat ini (v4) memakai heater **DC 12 V / 120 W dengan MOSFET IRLZ44N** dan **sirkulasi paksa dengan pompa ber-PWM**.

## 3. Strategi Kendali

### 3.1 Algoritma PID Diskrit

PID dieksekusi dengan periode cuplik tetap $T_s = 2$ s. Bentuk yang dipakai adalah **PID posisional dengan *derivative-on-measurement*** dan ***clamped-integral anti-windup***, mengikuti pendekatan Arduino PID Library (Beauregard):

$$e[k] = r[k] - y[k]$$

$$I[k] = \mathrm{sat}_{[0,\,100]}\!\left(I[k-1] + K_i\, e[k]\, T_s\right)$$

$$u[k] = \mathrm{sat}_{[0,\,100]}\!\left(K_p\, e[k] + I[k] - K_d\,\frac{y[k]-y[k-1]}{T_s}\right)$$

dengan $r$ = *setpoint* (°C), $y$ = suhu tangki (°C), dan $u$ = keluaran aktuator (%). Keluaran dipetakan ke PWM 8-bit: $\mathrm{duty}_{\mathrm{PWM}} = \lfloor u \cdot 255 / 100 \rfloor$.

Dua instans PID dengan struktur yang sama dijalankan bergantian, masing-masing dengan *gain* sendiri karena dinamika plant-nya berbeda:

| | PID malam (heater) | PID siang (pompa) |
|---|---|---|
| Aktuator | Heater, PWM 1 kHz | Pompa, PWM 20 kHz |
| *Gain* awal (*placeholder*) | $K_p = 5$, $K_i = 0{,}05$, $K_d = 0$ | $K_p = 10$, $K_i = 0{,}1$, $K_d = 0$ |
| Pemetaan keluaran | $u$ langsung | $u < 0{,}5\%$ → 0; selain itu $\max(u,\ u_{\min})$, dengan $u_{\min}$ = *duty* minimum pompa (default 30%) |

Instans yang tidak aktif di-reset (integrator dan memori derivatif dikosongkan), sehingga peralihan siang ↔ malam selalu dimulai tanpa sisa akumulasi integral dari aktuator lain.

Alasan desain:
- ***Derivative-on-measurement*** (bukan *on-error*) menghilangkan *derivative kick* saat *setpoint* diubah mendadak dari *dashboard*.
- ***Clamped integral*** pada rentang keluaran mencegah *integrator windup*, yang signifikan pada plant termal dengan konstanta waktu besar dan aktuator yang sering jenuh di 0% atau 100%.
- **Frekuensi PWM** jauh di atas *bandwidth* termal plant, sehingga dari sisi termal aktuator berperilaku kontinu sebanding dengan $u$. Pompa memakai 20 kHz agar di luar rentang pendengaran (motor *brushed* tidak berdengung).
- ***Duty* minimum pompa** mencegah motor macet (*stall*) pada tegangan rata-rata rendah, kondisi di mana arus tinggi mengalir tanpa ada aliran air.

*Gain* awal hanyalah *placeholder*. Parameter final diperoleh dari identifikasi plant (lihat [§7](#7-prosedur-eksperimen)) dan dapat diubah *live* melalui *dashboard*. *Setpoint* (bersama untuk kedua PID) dibatasi pada 30–70 °C. Semua parameter disimpan permanen di NVS (`Preferences`).

### 3.2 Mode Operasi

| Mode | Siang | Malam | Kegunaan |
|---|---|---|---|
| `MANUAL` | Heater & pompa ON 100% / OFF lewat tombol | (sama) | *Bring-up* dan uji perangkat keras. **Mode default setiap boot.** |
| `STEP_TEST` | Pompa 100%, heater 0% | Heater 100%, pompa 0% | Identifikasi respons *step* masing-masing plant (*open-loop*) |
| `AUTO_PID` | PID siang → pompa (bila gerbang ΔT terbuka), heater 0% | PID malam → heater, pompa 0% | Operasi *closed-loop* |

Tombol pompa dan heater manual hanya berlaku di `MANUAL`. Di mode lain, aktuator sepenuhnya dikendalikan firmware agar data eksperimen tidak tercampur intervensi manual. Mode **tidak** dipersistensi. Setelah *reboot* atau mati listrik, sistem selalu kembali ke `MANUAL` dengan heater dan pompa OFF.

Pada `STEP_TEST` siang, pompa dijalankan tanpa gerbang ΔT. Operator memulai uji saat kolektor sudah lebih panas daripada tangki.

### 3.3 Gerbang Siang/Malam (Histeresis Dua Ambang)

Status siang/malam ditentukan dari BH1750 dengan dua ambang terpisah:

$$\text{malam} \leftarrow \text{lux} < L_{\text{malam}}, \qquad \text{siang} \leftarrow \text{lux} > L_{\text{siang}}, \qquad L_{\text{malam}} \le L_{\text{siang}}$$

Di antara kedua ambang, status sebelumnya dipertahankan. Ini mencegah *chattering* (status berganti bolak-balik tiap siklus) saat senja, fajar, atau ketika awan lewat. **Kondisi mendung diperlakukan sebagai malam** selama intensitasnya turun di bawah $L_{\text{malam}}$. Karena itu, kedua ambang perlu dikalibrasi dari data lapangan. Nilai default (10 / 50 lux) hanya mendeteksi gelap malam, sementara siang mendung umumnya masih ribuan lux. Ambang dapat diubah dari *dashboard* tanpa memprogram ulang firmware.

Jika pembacaan gagal (`lux < 0`), status terakhir dipertahankan. Operator dapat memaksa status lewat *override* `FORCE_DAY` / `FORCE_NIGHT`. *Override* mengabaikan nilai lux sepenuhnya.

### 3.4 Gerbang ΔT Pompa

Pompa hanya bermanfaat bila kolektor lebih panas daripada tangki. Bila sebaliknya (misalnya sore hari atau mendung tipis), sirkulasi justru **membuang** panas tangki ke kolektor. Karena itu, PID siang hanya diizinkan menggerakkan pompa bila gerbang ΔT terbuka:

$$\Delta T = T_{\text{kolektor}} - T_{\text{tangki}}; \qquad \text{buka bila } \Delta T \ge \Delta T_{\text{on}}, \quad \text{tutup bila } \Delta T < \Delta T_{\text{off}}$$

Default $\Delta T_{\text{on}} = 5$ °C dan $\Delta T_{\text{off}} = 2$ °C. Histeresis ini standar pada pengendali *differential* pemanas air surya, untuk mencegah pompa menyala-mati berulang. Selama gerbang tertutup, pompa mati dan PID siang di-reset.

## 4. Mekanisme Keselamatan

1. **Safety cutoff 75 °C.** Berlaku di semua mode. Bila suhu tangki ≥ 75 °C, heater **dan pompa** dipaksa OFF (pompa adalah sumber panas siang hari) dan status `emergency` dikunci (*latched*). Kunci hanya dapat dilepas manual lewat perintah `CLEAR_EMERGENCY` dan hanya bila suhu < 73 °C (histeresis 2 °C).
2. **Deteksi sensor terputus.** DS18B20 mengembalikan −127 °C saat terputus atau gagal CRC. Nilai ini dikonversi menjadi NaN saat dibaca. Bila sensor tangki tidak valid, heater dan pompa dipaksa OFF di semua mode otomatis, dan heater manual juga ditolak. Tanpa ini, PID akan membaca galat sangat besar dan menjenuhkan aktuator ke 100%, sementara *safety cutoff* tidak pernah terpicu.
3. **Start aman.** Mode selalu `MANUAL` setelah boot, heater dan pompa OFF.
4. **Validasi masukan.** *Setpoint*, *gain* (≥ 0), ambang ΔT ($\Delta T_{\text{off}} < \Delta T_{\text{on}}$), dan ambang lux ($L_{\text{malam}} \le L_{\text{siang}}$) divalidasi di firmware, tidak bergantung pada validasi di *browser*.
5. **Pembacaan sensor *non-blocking*.** Konversi DS18B20 12-bit (~750 ms) dijalankan sebagai *state machine* (request → tunggu 800 ms → baca), sehingga *loop* kendali dan WebSocket tidak mengalami *jitter*.

## 5. Antarmuka Komunikasi (WebSocket)

ESP32 beroperasi sebagai *Access Point* (SSID `SWH_PID_Agung`, IP `192.168.4.1`). *Dashboard* tersedia di `http://192.168.4.1/` dan *endpoint* WebSocket di `ws://192.168.4.1/ws`.

### 5.1 Telemetri (ESP32 → klien, *push* tiap ~2 s)

```json
{"t":123456,"tangki":45.12,"kolektor":61.80,"lux":3250.0,"night":false,
 "pump":true,"heaterDuty":0.0,"mode":"AUTO_PID","dnOverride":"AUTO",
 "emergency":false,"sp":50.00,"kp":5.000,"ki":0.050,"kd":0.000,
 "pumpDuty":42.5,"dtOk":true,"pkp":10.000,"pki":0.100,"pkd":0.000,
 "dtOn":5.0,"dtOff":2.0,"pmin":30,"luxM":10,"luxS":50}
```

| Field | Tipe | Keterangan |
|---|---|---|
| `t` | int | `millis()` ESP32 (ms sejak boot, reset tiap reboot) |
| `tangki`, `kolektor` | float | Suhu (°C); `-127` = sensor tidak terbaca |
| `lux` | float | Intensitas cahaya; negatif = pembacaan BH1750 gagal |
| `night` | bool | Status malam efektif (setelah *override*) |
| `pump` | bool | Pompa berjalan (`pumpDuty > 0`) |
| `heaterDuty` | float | Keluaran heater 0–100 % |
| `mode` | string | `MANUAL` \| `STEP_TEST` \| `AUTO_PID` |
| `dnOverride` | string | `AUTO` \| `FORCE_DAY` \| `FORCE_NIGHT` |
| `emergency` | bool | *Safety cutoff* aktif |
| `sp` | float | *Setpoint* (bersama) |
| `kp`, `ki`, `kd` | float | *Gain* PID malam (heater) |
| `pumpDuty` | float | Keluaran pompa 0–100 % |
| `dtOk` | bool | Gerbang ΔT pompa terbuka |
| `pkp`, `pki`, `pkd` | float | *Gain* PID siang (pompa) |
| `dtOn`, `dtOff` | float | Ambang gerbang ΔT (°C) |
| `pmin` | float | *Duty* minimum pompa (%) |
| `luxM`, `luxS` | float | Ambang malam / siang (lux) |

### 5.2 Perintah (klien → ESP32, teks)

`MODE:MANUAL` · `MODE:STEP_TEST` · `MODE:AUTO_PID` · `PUMP:ON` · `PUMP:OFF` · `HEATER:ON` · `HEATER:OFF` · `DN:AUTO` · `DN:FORCE_DAY` · `DN:FORCE_NIGHT` · `CLEAR_EMERGENCY` · `SET_SP:<°C>` · `SET_KP:<v>` · `SET_KI:<v>` · `SET_KD:<v>` · `SET_PKP:<v>` · `SET_PKI:<v>` · `SET_PKD:<v>` · `SET_DTON:<°C>` · `SET_DTOFF:<°C>` · `SET_PMIN:<%>` · `SET_LUXM:<lux>` · `SET_LUXS:<lux>` · `SAVE_PARAMS`

## 6. Akuisisi Data (Data Logger)

Data di *dashboard* hanya tersimpan di memori JavaScript dan hilang saat tab ditutup. Untuk eksperimen berdurasi panjang, dipakai [`swh_logger.py`](swh_logger.py), *logger* asinkron berbasis `websockets` dengan prinsip desain berikut:

| Aspek | Implementasi |
|---|---|
| Durabilitas | Setiap sampel ditulis satu baris lalu langsung `flush()` + `os.fsync()`. Data yang sudah diterima tetap aman walau proses atau koneksi terputus. |
| Atomisitas baris | Baris diformat ke *string* dulu lalu ditulis dengan satu `write()`, sehingga tidak ada baris setengah jadi saat Ctrl+C. |
| Acuan waktu absolut | Kolom `wall_time` (ISO 8601, ms) dari jam laptop ditambahkan, karena `t` dari ESP32 reset ke 0 setiap reboot. |
| Pemulihan koneksi | *Reconnect* otomatis dengan *exponential backoff* 2 → 4 → 8 → … → 30 s, reset ke 2 s setelah berhasil. Data di-*append* ke berkas yang sama. |
| Deteksi koneksi mati | *Watchdog* penerimaan 10 s. Putus WiFi sering tidak menutup soket TCP, sehingga ketiadaan data dipakai sebagai indikator. |
| Deteksi reboot ESP32 | Peringatan dicetak bila nilai `t` menurun. |

Format CSV: `wall_time` diikuti seluruh field telemetri §5.1 dengan urutan yang sama. Nilai boolean ditulis `1`/`0`. *Logger* menolak meng-*append* ke CSV yang kolomnya berbeda (berkas dari versi lama), agar kolom tidak bergeser.

Celah data selama koneksi terputus tidak dapat dipulihkan, karena ESP32 tidak menyimpan riwayat. Celah tersebut terlihat sebagai lompatan pada `wall_time`.

### Visualisasi *live*

[`swh_plot.py`](swh_plot.py) menampilkan grafik yang diperbarui tiap 2 s dari CSV yang sedang ditulis *logger*:

1. Suhu tangki, suhu kolektor, dan *setpoint*.
2. Intensitas cahaya.
3. *Duty* heater dan *duty* pompa.

Grafik berjalan sebagai proses terpisah yang hanya membaca berkas. Dengan begitu, menutup jendela grafik (atau galat pada grafik) tidak memengaruhi perekaman. Hanya baris CSV yang sudah lengkap yang dibaca. Pembacaan sensor error (`-127`, lux negatif) tidak digambar, dan periode koneksi terputus (> 10 s) ditampilkan sebagai garis yang terputus.

## 7. Prosedur Eksperimen

1. **Uji perangkat keras (`MANUAL`).** Verifikasi pembacaan sensor, pompa, dan heater ON/OFF. Pastikan pompa tetap berputar pada *duty* minimum (atur `Duty min` bila tersendat).
2. **Kalibrasi ambang siang/malam.** Rekam lux sepanjang hari, termasuk saat mendung, lalu tetapkan $L_{\text{malam}}$ dan $L_{\text{siang}}$.
3. **Identifikasi plant (`STEP_TEST`), dua kali:**
   - **Heater:** `FORCE_NIGHT`, *step* heater 100% dari kondisi tunak.
   - **Pompa:** `FORCE_DAY`, saat kolektor sudah jauh lebih panas daripada tangki, *step* pompa 100%.

   Dari masing-masing kurva respons suhu tangki diekstrak parameter model, misalnya pendekatan *first-order plus dead time* (gain proses $K$, konstanta waktu $\tau$, waktu mati $\theta$). Perlu dicatat, gain plant pompa bergantung pada ΔT kolektor–tangki, sehingga identifikasi sebaiknya dilakukan pada ΔT yang representatif.
4. **Penalaan (*tuning*).** *Gain* kedua PID dihitung dari parameter model masing-masing dengan metode penalaan yang dipilih, lalu dimasukkan lewat *dashboard*.
5. **Uji *closed-loop* (`AUTO_PID`).** Kinerja dievaluasi dari data terekam, misalnya *rise time*, *overshoot*, *settling time*, dan galat tunak, terpisah untuk siang dan malam, serta perilaku saat peralihan.
6. **Uji lapangan.** Perekaman pagi–sore untuk karakterisasi pemanasan surya (hubungan lux, suhu kolektor, suhu tangki, dan *duty* pompa).

## 8. Struktur Repositori

```
.
├── Esp_agung/
│   └── Esp_agung.ino              # Firmware ESP32 (PID, WebSocket, dashboard, safety)
├── swh_logger.py                  # Data logger telemetri ke CSV
├── swh_plot.py                    # Grafik live dari CSV
├── mulai_logger.bat               # Jalankan logger + grafik dengan klik dua kali (Windows)
├── swh_log_*.csv                  # Contoh data hasil perekaman
├── NETLIST_SWH_PID_Agung_v1.txt   # Netlist, perhitungan daya, catatan desain & BOM
├── SCH_Agung_2026-09-14.pdf       # Skematik
├── PCB_Agung_2026-09-15.pdf       # Layout PCB
├── Gerber_Agung_2026-09-14.zip    # Berkas produksi PCB (Gerber + drill)
└── Komponen agung.txt             # Daftar komponen
```

## 9. Cara Penggunaan

### 9.1 Firmware

- **Toolchain:** Arduino IDE, ESP32 Arduino Core 2.x (diuji pada 2.0.17).
- **Library:** [ESPAsyncWebServer](https://github.com/ESP32Async/ESPAsyncWebServer) dan [AsyncTCP](https://github.com/ESP32Async/AsyncTCP) (*fork* ESP32Async), OneWire, DallasTemperature, BH1750, LiquidCrystal_I2C.
- Unggah `Esp_agung/Esp_agung.ino`, sambungkan perangkat ke WiFi `SWH_PID_Agung`, lalu buka `http://192.168.4.1/`.

### 9.2 Data Logger

```bash
pip install websockets matplotlib
# sambungkan laptop ke WiFi SWH_PID_Agung terlebih dahulu
python swh_logger.py                      # -> swh_log_<YYYYMMDD_HHMMSS>.csv + jendela grafik otomatis
python swh_logger.py -o percobaan_1.csv   # nama berkas kustom (append bila sudah ada)
python swh_logger.py --no-plot            # tanpa grafik
python swh_logger.py --help               # opsi backoff & timeout

python swh_plot.py percobaan_1.csv --window 60   # buka grafik manual: berkas tertentu, 60 menit terakhir
```

*Logger* membuka `swh_plot.py` sebagai proses terpisah bila `matplotlib` tersedia. Bila tidak, perekaman tetap berjalan tanpa grafik. Ctrl+C hanya menghentikan *logger*, sedangkan jendela grafik tetap terbuka sampai ditutup manual. Di Windows dengan Anaconda (environment `swh`), cukup klik dua kali `mulai_logger.bat`.

Untuk perekaman panjang, nonaktifkan mode *sleep* laptop dan cegah Windows berpindah otomatis ke jaringan WiFi lain, karena AP ESP32 tidak menyediakan internet.

## 10. Keterbatasan dan Pekerjaan Lanjutan

- **Ambang lux dan ΔT** masih nilai default dan perlu dikalibrasi dari data lapangan.
- **Mendung hanya dideteksi lewat lux.** Kondisi dengan lux di atas $L_{\text{siang}}$ tetapi kolektor tidak cukup panas (gerbang ΔT tertutup) membuat tangki tidak dipanaskan oleh pompa maupun heater. Bila ini sering terjadi, salah satu pengembangannya adalah memakai heater sebagai cadangan ketika gerbang ΔT tertutup cukup lama.
- **Inisialisasi BH1750 hanya saat boot.** Sensor yang dipasang setelah ESP32 menyala tidak akan terbaca sampai reboot.
- **Perilaku PWM pompa** (kebisingan, *duty* minimum, pemanasan MOSFET pada 20 kHz) perlu diverifikasi pada perangkat keras.
- **Anggaran daya** pada perhitungan baterai belum direvisi untuk WiFi aktif terus-menerus dan pompa yang kini dapat berjalan sepanjang siang (lihat TBD di netlist).
