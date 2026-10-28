#!/usr/bin/env python3
"""
SWH PID Live Plot - grafik live dari CSV hasil swh_logger.py.

Berjalan terpisah dari logger: grafik membaca file CSV yang sedang ditulis,
jadi menutup/crash-nya jendela grafik tidak mengganggu perekaman data.

Dependency: pip install matplotlib
Pemakaian : python swh_plot.py                 (file swh_log_*.csv terbaru)
            python swh_plot.py data.csv        (file tertentu)
            python swh_plot.py --window 60     (tampilkan 60 menit terakhir)
"""

import argparse
import csv
import glob
import math
import os
import sys
import time
from datetime import datetime, timedelta

try:
    import matplotlib.pyplot as plt
    import matplotlib.dates as mdates
    from matplotlib.animation import FuncAnimation
except ImportError:
    print("[ERROR] matplotlib belum terinstall. Jalankan: pip install matplotlib")
    input("Tekan Enter untuk menutup...")
    sys.exit(1)

SENSOR_ERROR = -127
GAP_SECONDS = 10  # jeda antar sampel > ini dianggap putus -> garis diputus


def newest_log():
    files = glob.glob("swh_log_*.csv")
    return max(files, key=os.path.getmtime) if files else None


def to_float(s, invalid_below=None):
    try:
        v = float(s)
    except (TypeError, ValueError):
        return math.nan
    if invalid_below is not None and v < invalid_below:
        return math.nan
    return v


class CsvTail:
    """Baca baris baru dari CSV secara inkremental (hanya baris yang sudah lengkap)."""

    def __init__(self, path):
        self.path = path
        self.pos = 0
        self.header = None
        self.pending = ""
        self.last_time = None
        self.data = {k: [] for k in ("time", "tangki", "kolektor", "sp", "lux", "duty", "pump")}

    def poll(self):
        try:
            with open(self.path, "r", encoding="utf-8", newline="") as f:
                f.seek(self.pos)
                chunk = f.read()
                self.pos = f.tell()
        except OSError:
            return False
        if not chunk:
            return False

        text = self.pending + chunk
        lines = text.split("\n")
        self.pending = lines.pop()  # baris terakhir mungkin belum selesai ditulis
        for line in lines:
            line = line.rstrip("\r")
            if not line:
                continue
            values = next(csv.reader([line]))
            if self.header is None:
                self.header = values
                continue
            self._add(dict(zip(self.header, values)))
        return True

    def _add(self, row):
        try:
            t = datetime.fromisoformat(row["wall_time"])
        except (KeyError, ValueError):
            return
        d = self.data
        if self.last_time is not None and (t - self.last_time).total_seconds() > GAP_SECONDS:
            # sisipkan NaN supaya garis tidak menyambung melewati periode putus
            d["time"].append(self.last_time + timedelta(seconds=1))
            for k in ("tangki", "kolektor", "sp", "lux", "duty", "pump"):
                d[k].append(math.nan)
        self.last_time = t
        d["time"].append(t)
        d["tangki"].append(to_float(row.get("tangki"), SENSOR_ERROR + 1))
        d["kolektor"].append(to_float(row.get("kolektor"), SENSOR_ERROR + 1))
        d["sp"].append(to_float(row.get("sp")))
        d["lux"].append(to_float(row.get("lux"), 0))
        d["duty"].append(to_float(row.get("heaterDuty")))
        if row.get("pumpDuty"):
            d["pump"].append(to_float(row["pumpDuty"]))
        else:  # CSV firmware lama: pump cuma 1/0 -> 100/0 biar sejajar skala %
            d["pump"].append(to_float(row.get("pump")) * 100)


def main():
    p = argparse.ArgumentParser(description="Grafik live dari CSV swh_logger.py")
    p.add_argument("csv", nargs="?", help="file CSV (default: swh_log_*.csv terbaru di folder ini)")
    p.add_argument("--window", type=float, default=0,
                   help="hanya tampilkan N menit terakhir (default 0 = semua data)")
    p.add_argument("--interval", type=float, default=2.0, help="refresh grafik tiap N detik (default 2)")
    args = p.parse_args()

    path = args.csv
    if path is None:
        print("Mencari file swh_log_*.csv terbaru...", flush=True)
        while (path := newest_log()) is None:
            time.sleep(1)
    if not os.path.exists(path):
        print(f"Menunggu file {path} dibuat oleh logger...", flush=True)
        while not os.path.exists(path):
            time.sleep(1)
    print(f"Menampilkan grafik dari: {os.path.abspath(path)}", flush=True)

    tail = CsvTail(path)

    fig, (ax_t, ax_lux, ax_out) = plt.subplots(
        3, 1, sharex=True, figsize=(11, 8),
        gridspec_kw={"height_ratios": [3, 1.3, 1.3]})
    fig.canvas.manager.set_window_title(f"SWH PID - {os.path.basename(path)}")

    (l_tangki,) = ax_t.plot([], [], color="tab:blue", lw=1.8, label="Suhu tangki (PV)")
    (l_kol,) = ax_t.plot([], [], color="tab:red", lw=1.2, label="Suhu kolektor")
    (l_sp,) = ax_t.plot([], [], color="tab:orange", lw=1.5, ls="--", label="Setpoint")
    ax_t.set_ylabel("Suhu (°C)")
    ax_t.legend(loc="upper left", fontsize=9)
    ax_t.grid(alpha=0.3)

    (l_lux,) = ax_lux.plot([], [], color="tab:olive", lw=1.2, label="Intensitas cahaya")
    ax_lux.set_ylabel("Lux")
    ax_lux.legend(loc="upper left", fontsize=9)
    ax_lux.grid(alpha=0.3)

    (l_duty,) = ax_out.plot([], [], color="tab:purple", lw=1.2, label="Heater duty")
    (l_pump,) = ax_out.plot([], [], color="tab:green", lw=1.2, label="Pompa duty")
    ax_out.set_ylabel("%")
    ax_out.set_ylim(-5, 105)
    ax_out.legend(loc="upper left", fontsize=9)
    ax_out.grid(alpha=0.3)
    ax_out.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M"))
    ax_out.set_xlabel("Waktu")

    title = fig.suptitle("Menunggu data...", fontsize=11)
    fig.tight_layout()

    def latest(values):
        for v in reversed(values):
            if not math.isnan(v):
                return v
        return math.nan

    def update(_):
        if not tail.poll():
            return
        d = tail.data
        if not d["time"]:
            return
        t = d["time"]
        if args.window > 0:
            start = t[-1] - timedelta(minutes=args.window)
            i = next((i for i, x in enumerate(t) if x >= start), 0)
        else:
            i = 0
        sl = slice(i, None)

        l_tangki.set_data(t[sl], d["tangki"][sl])
        l_kol.set_data(t[sl], d["kolektor"][sl])
        l_sp.set_data(t[sl], d["sp"][sl])
        l_lux.set_data(t[sl], d["lux"][sl])
        l_duty.set_data(t[sl], d["duty"][sl])
        l_pump.set_data(t[sl], d["pump"][sl])

        for ax in (ax_t, ax_lux):
            ax.relim()
            ax.autoscale_view()
        x0, x1 = t[i], t[-1]
        if x0 == x1:
            x1 = x0 + timedelta(seconds=10)
        ax_out.set_xlim(x0, x1)

        def fmt(v, unit):
            return "ERR" if math.isnan(v) else f"{v:.1f}{unit}"

        title.set_text(
            f"Tangki {fmt(latest(d['tangki']), '°C')}  |  Kolektor {fmt(latest(d['kolektor']), '°C')}  |  "
            f"SP {fmt(latest(d['sp']), '°C')}  |  Lux {fmt(latest(d['lux']), '')}  |  "
            f"Update {t[-1].strftime('%H:%M:%S')}")

    anim = FuncAnimation(fig, update, interval=args.interval * 1000, cache_frame_data=False)
    update(None)
    plt.show()
    return anim


if __name__ == "__main__":
    main()
