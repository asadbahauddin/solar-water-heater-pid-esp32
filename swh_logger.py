#!/usr/bin/env python3
"""
SWH PID Logger - logger telemetri ESP32 Solar Water Heater via WebSocket.

Konek ke ws://192.168.4.1/ws (ESP32 SoftAP "SWH_PID_Agung"), setiap pesan JSON
langsung ditulis 1 baris ke CSV lalu flush + fsync, jadi data aman di disk
walaupun script/koneksi putus. Auto-reconnect dengan backoff, append ke file
CSV yang sama.

Dependency: pip install websockets
Pemakaian : python swh_logger.py [-o nama_file.csv] [--url ws://...]
"""

import argparse
import asyncio
import csv
import io
import json
import os
import sys
from datetime import datetime

import websockets

DEFAULT_URL = "ws://192.168.4.1/ws"
SENSOR_ERROR = -127

# Urutan kolom CSV: timestamp wall-clock laptop dulu, lalu field dari ESP32.
ESP_FIELDS = [
    "t", "tangki", "kolektor", "lux", "night", "pump", "heaterDuty",
    "mode", "dnOverride", "emergency", "sp", "kp", "ki", "kd",
]
CSV_FIELDS = ["wall_time"] + ESP_FIELDS


def log(msg):
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}", flush=True)


class CsvSink:
    """File CSV append-only; tiap baris langsung di-flush + fsync ke disk."""

    def __init__(self, path):
        self.path = path
        is_new = not os.path.exists(path) or os.path.getsize(path) == 0
        self.f = open(path, "a", newline="", encoding="utf-8")
        if is_new:
            self._write_line(CSV_FIELDS)

    def _write_line(self, values):
        # Format baris ke string dulu, lalu tulis dengan satu write() supaya
        # baris tidak setengah jadi kalau terinterupsi.
        buf = io.StringIO()
        csv.writer(buf).writerow(values)
        self.f.write(buf.getvalue())
        self.f.flush()
        os.fsync(self.f.fileno())

    def write(self, wall_time, data):
        row = [wall_time]
        for key in ESP_FIELDS:
            v = data.get(key, "")
            if isinstance(v, bool):
                v = int(v)  # 1/0 lebih enak buat plotting/Excel
            row.append(v)
        self._write_line(row)

    def close(self):
        if not self.f.closed:
            self.f.flush()
            os.fsync(self.f.fileno())
            self.f.close()


def fmt_temp(v):
    if v is None:
        return "  n/a "
    try:
        v = float(v)
    except (TypeError, ValueError):
        return "  n/a "
    if v <= SENSOR_ERROR + 0.5:
        return "  ERR "
    return f"{v:5.1f}C"


def status_line(wall, d, n):
    flags = []
    if d.get("pump"):
        flags.append("PUMP")
    if d.get("night"):
        flags.append("NIGHT")
    if d.get("emergency"):
        flags.append("!!EMERGENCY!!")
    duty = d.get("heaterDuty")
    duty_s = f"{float(duty):5.1f}%" if isinstance(duty, (int, float)) else "  n/a"
    return (
        f"{wall.strftime('%H:%M:%S')} #{n:<6} "
        f"Tangki {fmt_temp(d.get('tangki'))} | "
        f"Kolektor {fmt_temp(d.get('kolektor'))} | "
        f"SP {d.get('sp', '?')} | "
        f"{str(d.get('mode', '?')):<9} | Duty {duty_s} "
        f"{' '.join(flags)}"
    )


async def run(args, sink):
    backoff = args.min_backoff
    count = 0
    last_t = None

    while True:
        try:
            log(f"Connecting ke {args.url} ...")
            async with websockets.connect(
                args.url,
                open_timeout=args.timeout,
                ping_interval=None,  # watchdog pakai rx-timeout di bawah
                close_timeout=2,
            ) as ws:
                log("Terhubung. Menunggu data...")
                backoff = args.min_backoff  # reset backoff setelah konek sukses

                while True:
                    # ESP32 push tiap ~2 detik; kalau lama tidak ada data,
                    # anggap koneksi mati (WiFi drop sering tidak menutup TCP).
                    raw = await asyncio.wait_for(ws.recv(), timeout=args.rx_timeout)
                    wall = datetime.now()
                    try:
                        data = json.loads(raw)
                        if not isinstance(data, dict):
                            raise ValueError("bukan objek JSON")
                    except ValueError as e:
                        log(f"Pesan diabaikan (JSON tidak valid: {e}): {str(raw)[:80]!r}")
                        continue

                    sink.write(wall.isoformat(timespec="milliseconds"), data)
                    count += 1

                    t = data.get("t")
                    if isinstance(t, (int, float)):
                        if last_t is not None and t < last_t:
                            log("Catatan: 't' ESP32 mundur -> kemungkinan ESP32 reboot.")
                        last_t = t

                    print(status_line(wall, data, count), flush=True)

        except asyncio.CancelledError:
            raise
        except asyncio.TimeoutError:
            log(f"Timeout (tidak ada data / gagal konek dalam batas waktu).")
        except Exception as e:  # OSError, ConnectionClosed, InvalidHandshake, dll
            log(f"Koneksi putus/gagal: {type(e).__name__}: {e}")

        log(f"Reconnect dalam {backoff:.0f} detik... (total baris tersimpan: {count})")
        await asyncio.sleep(backoff)
        backoff = min(backoff * 2, args.max_backoff)


def main():
    start = datetime.now()
    p = argparse.ArgumentParser(description="Logger telemetri ESP32 SWH PID ke CSV.")
    p.add_argument("-o", "--output",
                   default=f"swh_log_{start.strftime('%Y%m%d_%H%M%S')}.csv",
                   help="file CSV output (default: swh_log_<tanggal_jam>.csv). "
                        "Kalau file sudah ada, data di-append.")
    p.add_argument("--url", default=DEFAULT_URL, help=f"URL WebSocket (default {DEFAULT_URL})")
    p.add_argument("--min-backoff", type=float, default=2.0, help="delay retry awal, detik (default 2)")
    p.add_argument("--max-backoff", type=float, default=30.0, help="delay retry maksimum, detik (default 30)")
    p.add_argument("--timeout", type=float, default=5.0, help="timeout buka koneksi, detik (default 5)")
    p.add_argument("--rx-timeout", type=float, default=10.0,
                   help="anggap putus kalau tidak ada data selama N detik (default 10)")
    args = p.parse_args()

    sink = CsvSink(args.output)
    log(f"Logging ke: {os.path.abspath(args.output)}")
    log("Tekan Ctrl+C untuk berhenti.")
    try:
        asyncio.run(run(args, sink))
    except KeyboardInterrupt:
        pass
    finally:
        sink.close()
        log(f"Berhenti. File CSV ditutup dengan aman: {args.output}")


if __name__ == "__main__":
    sys.exit(main())
