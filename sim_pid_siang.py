#!/usr/bin/env python3
"""
SIMULASI PID siang (pompa) - BUKAN data pengukuran.

Model fisik sederhana tangki + loop kolektor, dikendalikan algoritma yang SAMA
persis dengan firmware Esp_agung.ino (computePID, gerbang dT, PWM minimum 50%),
siklus kontrol 2 detik. Output CSV berformat sama dengan swh_logger.py
(1 baris per menit) sehingga bisa dibuka dengan swh_plot.py.

Model:
  dTt/dt = k * f(PWM) * (Tc - Tt) - l * (Tt - Tamb)
  f(PWM)  = laju alir relatif pompa DC brushed (0 saat mati, ~0.29 di PWM 127, 1 di 255)
  k       = dikalibrasi otomatis supaya tangki mencapai setpoint pertama kali
            pada --t-sp menit (default 95 = 1 jam 35 menit)
Sensor dimodelkan: DS18B20 resolusi 0.0625 C, BH1750 resolusi 1/1.2 lux.

Gain default Kp=40, Ki=0.0002, Kd=0: pompa penuh (255) saat tangki jauh dari
setpoint, melambat mulai ~2.5 C sebelum setpoint, PWM minimum 127 menjelang
setpoint, mati di setpoint, hidup lagi (127) saat suhu turun. Dengan Kp=5,
Ki=0.05 integral langsung jenuh 100% sehingga pompa baru melambat SESUDAH
melewati setpoint (overshoot) -- coba: --kp 5 --ki 0.05.

Pemakaian: python sim_pid_siang.py [-o sim.csv] [--sp 43] [--kp 40 --ki 0.0002] [--seed 7]
           Juga membuat <nama>.xlsx (data + grafik Excel) dan <nama>.png kalau
           openpyxl / matplotlib terinstall.
"""

import argparse
import csv
import math
import random
from datetime import datetime, timedelta

CYCLE = 2.0  # detik, sama dengan SENSOR_CYCLE_MS firmware


def quant(v, step):
    return round(round(v / step) * step, 2)


class Pid:
    """Salinan computePID() firmware: posisional, clamped-integral, D on measurement."""

    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.reset()

    def reset(self):
        self.sum, self.last = 0.0, None

    def __call__(self, pv, sp):
        e = sp - pv
        self.sum = min(100.0, max(0.0, self.sum + self.ki * e * CYCLE))
        d = 0.0 if self.last is None else (pv - self.last) / CYCLE
        self.last = pv
        return min(100.0, max(0.0, self.kp * e + self.sum - self.kd * d))


def flow(pwm):
    # pompa DC brushed: di bawah ~30% duty praktis belum mengalir
    return 0.0 if pwm == 0 else max(0.0, (pwm / 2.55 - 30.0) / 70.0)


def make_inputs(n, rng, tc_lo, tc_hi, lux_lo, lux_hi):
    """Kolektor & lux: proses acak halus (naik-turun) di dalam rentang, cuaca cerah."""
    tc_mid, tc_amp = (tc_lo + tc_hi) / 2, (tc_hi - tc_lo) / 2
    lux_mid, lux_amp = (lux_lo + lux_hi) / 2, (lux_hi - lux_lo) / 2
    a = b = 0.0
    tcs, luxs = [], []
    for i in range(n):
        frac = i / n
        # Ornstein-Uhlenbeck: acak tapi kontinu (tidak loncat-loncat)
        a += -a * CYCLE / 900 + rng.gauss(0, 0.035)
        b += -b * CYCLE / 300 + rng.gauss(0, 0.05)
        sun = math.sin(math.pi * (0.25 + 0.6 * frac))  # naik menuju siang, landai setelahnya
        tcs.append(tc_mid + 0.92 * tc_amp * math.tanh(0.35 * (sun - 0.85) / 0.15 + a))
        luxs.append(lux_mid + 0.97 * lux_amp * math.tanh(0.5 * (sun - 0.85) / 0.15 + 0.35 * b))
    return tcs, luxs


def simulate(args, k, inputs, rng=None):
    tcs, luxs = inputs
    pid = Pid(args.kp, args.ki, args.kd)
    l = 0.0009 / 60  # rugi panas tangki ke lingkungan (1/detik)
    tt, dt_ok, first, rows = args.t0, False, None, []
    for i, tc_base in enumerate(tcs):
        sec = i * CYCLE
        tamb = 29.0 + 4.0 * sec / (4.25 * 3600)
        f_prev = rows[-1][3] if rows else 0.0
        # kolektor sedikit lebih panas saat air diam (pompa mati), tetap di rentang
        tc_mid = (args.tc_lo + args.tc_hi) / 2
        tc_amp = (args.tc_hi - args.tc_lo) / 2
        tc = tc_mid + tc_amp * math.tanh(math.atanh((tc_base - tc_mid) / tc_amp) + 0.5 * (1 - min(1, f_prev / 0.5)) - 0.25)
        tt_read = quant(tt, 0.0625)
        tc_read = quant(tc, 0.0625)
        d_t = tc_read - tt_read
        if not dt_ok and d_t >= 5.0:
            dt_ok = True
        elif dt_ok and d_t < 2.0:
            dt_ok = False
        if dt_ok:
            u = pid(tt_read, args.sp)
            pwm = 0 if u < 0.5 else int(max(u, 50.0) * 255 / 100)
        else:
            pid.reset()
            pwm = 0
        f = flow(pwm)
        rows.append((sec, tt_read, tc_read, f, pwm, dt_ok, luxs[i]))
        tt += (k * f * (tc - tt) - l * (tt - tamb)) * CYCLE
        if rng:
            tt += rng.gauss(0, 0.0015)  # pencampuran/stratifikasi air
        if first is None and tt_read >= args.sp:
            first = sec / 60
    return rows, first


def export_extra(args):
    """Buat .xlsx (sheet Data, Grafik, Keterangan) dan .png dari CSV hasil simulasi."""
    base = args.output.rsplit(".", 1)[0]
    with open(args.output, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    times = [datetime.fromisoformat(r["wall_time"]) for r in rows]
    col = {k: [float(r[k]) for r in rows] for k in ("tangki", "kolektor", "lux", "pumpPwm", "sp")}

    try:
        from openpyxl import Workbook
        from openpyxl.chart import LineChart, Reference
        from openpyxl.styles import Alignment, Font, PatternFill
    except ImportError:
        print("openpyxl tidak ada -> .xlsx dilewati")
    else:
        wb = Workbook()
        ws = wb.active
        ws.title = "Data"
        ws.append(["No", "Waktu", "Suhu Tangki (C)", "Suhu Kolektor (C)", "Setpoint (C)",
                   "Delta T (C)", "Intensitas Cahaya (lux)", "PWM Pompa (0-255)",
                   "Duty Pompa (%)", "Status Pompa"])
        for c in ws[1]:
            c.font = Font(bold=True, color="FFFFFF")
            c.fill = PatternFill("solid", fgColor="1F4E78")
            c.alignment = Alignment(horizontal="center", vertical="center", wrap_text=True)
        for i, t in enumerate(times):
            pwm = int(col["pumpPwm"][i])
            status = ("Mati" if pwm == 0 else "Cepat" if pwm >= 230
                      else "Pelan" if pwm <= 140 else "Sedang")
            ws.append([i + 1, t.time(), col["tangki"][i], col["kolektor"][i], col["sp"][i],
                       round(col["kolektor"][i] - col["tangki"][i], 2), col["lux"][i], pwm,
                       round(pwm / 255 * 100, 1), status])
        n = len(times) + 1
        for r in ws.iter_rows(min_row=2, max_row=n):
            r[1].number_format = "hh:mm"
            for c in r[2:6]:
                c.number_format = "0.00"
            r[6].number_format = "#,##0.0"
            for c in r:
                c.alignment = Alignment(horizontal="center")
        for letter, w in zip("ABCDEFGHIJ", (6, 9, 14, 15, 12, 11, 16, 15, 13, 12)):
            ws.column_dimensions[letter].width = w
        ws.row_dimensions[1].height = 32
        ws.freeze_panes = "A2"

        g = wb.create_sheet("Grafik")
        cats = Reference(ws, min_col=2, min_row=2, max_row=n)

        def chart(title, ytitle, cols, anchor, colors, ymin, ymax):
            ch = LineChart()
            ch.title, ch.y_axis.title, ch.x_axis.title = title, ytitle, "Waktu"
            ch.height, ch.width = 9, 26
            for c in cols:
                ch.add_data(Reference(ws, min_col=c, min_row=1, max_row=n), titles_from_data=True)
            ch.set_categories(cats)
            ch.x_axis.number_format = "hh:mm"
            ch.x_axis.tickLblSkip = 15
            ch.x_axis.tickMarkSkip = 15
            ch.x_axis.delete = False
            ch.y_axis.delete = False
            ch.y_axis.scaling.min, ch.y_axis.scaling.max = ymin, ymax
            for s, color in zip(ch.series, colors):
                s.smooth = False
                s.marker.symbol = "none"
                s.graphicalProperties.line.solidFill = color
                s.graphicalProperties.line.width = 19050
            ch.legend.position = "b"
            g.add_chart(ch, anchor)

        chart("Suhu Tangki, Kolektor dan Setpoint", "Suhu (C)", (3, 4, 5), "A1",
              ("1F77B4", "D62728", "FF7F0E"), 25, 52)
        chart("PWM Pompa (PID Siang)", "PWM (0-255)", (8,), "A20", ("2CA02C",), 0, 260)
        chart("Intensitas Cahaya", "Lux", (7,), "A39", ("808000",), 7000, 10500)

        k = wb.create_sheet("Keterangan")
        for row in [
            ("SIMULASI - bukan data pengukuran alat", ""),
            ("Mode", "AUTO_PID siang (PID mengatur PWM pompa)"),
            ("Setpoint", f"{args.sp:g} C"),
            ("Kp / Ki / Kd", f"{args.kp:g} / {args.ki:g} / {args.kd:g}"),
            ("Periode kontrol", "2 detik (data dicatat 1 baris per menit)"),
            ("PWM pompa", "8-bit 0-255, minimum 127 (50%) saat menyala"),
            ("Gerbang dT", "pompa boleh nyala jika kolektor - tangki >= 5 C, mati jika < 2 C"),
            ("Resolusi sensor", "DS18B20 0.0625 C, BH1750 0.83 lux"),
            ("Model", "dTt/dt = k*f(PWM)*(Tc-Tt) - l*(Tt-Tamb), k dikalibrasi agar SP "
                      f"tercapai {args.t_sp:g} menit"),
            ("Status pompa", "Cepat >= 230, Sedang 141-229, Pelan 127-140, Mati 0"),
        ]:
            k.append(row)
        k["A1"].font = Font(bold=True, color="C00000", size=12)
        for c in k["A"][1:]:
            c.font = Font(bold=True)
        k.column_dimensions["A"].width = 18
        k.column_dimensions["B"].width = 90
        wb.save(base + ".xlsx")
        print(f"Ditulis: {base}.xlsx")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.dates as mdates
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib tidak ada -> .png dilewati")
        return
    fig, (a, b, c) = plt.subplots(3, 1, sharex=True, figsize=(11, 8.5),
                                  gridspec_kw={"height_ratios": [3, 1.3, 1.6]})
    a.plot(times, col["tangki"], color="tab:blue", lw=1.8, label="Suhu tangki")
    a.plot(times, col["kolektor"], color="tab:red", lw=1.2, label="Suhu kolektor")
    a.plot(times, col["sp"], color="tab:orange", lw=1.5, ls="--", label=f"Setpoint {args.sp:g} °C")
    a.set_ylabel("Suhu (°C)")
    a.legend(loc="lower right", fontsize=9)
    b.plot(times, col["lux"], color="tab:olive", lw=1.2, label="Intensitas cahaya")
    b.set_ylabel("Lux")
    b.legend(loc="lower left", fontsize=9)
    c.step(times, col["pumpPwm"], where="post", color="tab:green", lw=1.3, label="PWM pompa")
    c.axhline(127, color="tab:green", lw=0.8, ls=":", label="PWM minimum 127")
    c.set_ylabel("PWM (0-255)")
    c.set_ylim(-10, 265)
    c.set_yticks([0, 64, 127, 191, 255])
    c.legend(loc="upper right", fontsize=9)
    c.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M"))
    c.set_xlabel("Waktu")
    for ax in (a, b, c):
        ax.grid(alpha=0.3)
    fig.suptitle(f"Simulasi PID Siang (Pompa) - SP {args.sp:g} °C, "
                 f"Kp={args.kp:g} Ki={args.ki:g} Kd={args.kd:g}", fontsize=12)
    fig.tight_layout()
    fig.savefig(base + ".png", dpi=130)
    print(f"Ditulis: {base}.png")


def main():
    p = argparse.ArgumentParser(description="Simulasi PID siang (pompa) SWH")
    p.add_argument("-o", "--output", default="sim_pid_siang_sp43.csv")
    p.add_argument("--sp", type=float, default=43.0)
    p.add_argument("--kp", type=float, default=40.0)
    p.add_argument("--ki", type=float, default=0.0002)
    p.add_argument("--kd", type=float, default=0.0)
    p.add_argument("--t0", type=float, default=30.28, help="suhu awal tangki")
    p.add_argument("--tc-lo", type=float, default=46.34)
    p.add_argument("--tc-hi", type=float, default=49.07)
    p.add_argument("--lux-lo", type=float, default=8000)
    p.add_argument("--lux-hi", type=float, default=10000)
    p.add_argument("--start", default="09:15")
    p.add_argument("--end", default="13:30")
    p.add_argument("--t-sp", type=float, default=95, help="menit sampai setpoint pertama")
    p.add_argument("--seed", type=int, default=7)
    args = p.parse_args()

    day = datetime(2026, 11, 23)
    t_start = datetime.combine(day, datetime.strptime(args.start, "%H:%M").time())
    t_end = datetime.combine(day, datetime.strptime(args.end, "%H:%M").time())
    n = int((t_end - t_start).total_seconds() / CYCLE) + 1
    rng = random.Random(args.seed)
    inputs = make_inputs(n, rng, args.tc_lo, args.tc_hi, args.lux_lo, args.lux_hi)

    # kalibrasi k (bisection) supaya setpoint pertama tercapai di --t-sp menit
    lo, hi = 1e-5, 1e-3
    for _ in range(60):
        k = (lo * hi) ** 0.5
        _, first = simulate(args, k, inputs, random.Random(args.seed + 1))
        if first is None or first > args.t_sp:
            lo = k
        else:
            hi = k
    rows, first = simulate(args, hi, inputs, random.Random(args.seed + 1))

    boot_ms = 312_000 + rng.randint(0, 1999)  # ESP32 menyala ~5 menit sebelum mulai
    fields = ["wall_time", "t", "tangki", "kolektor", "lux", "night", "pumpPwm", "heaterPwm",
              "mode", "dnOverride", "emergency", "sp", "kp", "ki", "kd", "dtOk", "pkp", "pki",
              "pkd", "dtOn", "dtOff", "pmin", "luxM", "luxS"]
    per_min = int(60 / CYCLE)
    with open(args.output, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(fields)
        for i in range(0, len(rows), per_min):
            sec, tt, tc, _, pwm, dt_ok, lux = rows[i]
            wall = t_start + timedelta(seconds=sec, milliseconds=rng.randint(150, 1900))
            w.writerow([wall.isoformat(timespec="milliseconds"), boot_ms + int(sec * 1000),
                        f"{tt:.2f}", f"{tc:.2f}", f"{quant(lux, 1 / 1.2):.1f}", 0, pwm, 0,
                        "AUTO_PID", "AUTO", 0, f"{args.sp:.1f}", 5.0, 0.05, 0.0, int(dt_ok),
                        args.kp, args.ki, args.kd, 5.0, 2.0, 50.0, 10.0, 50.0])

    export_extra(args)
    print(f"Ditulis: {args.output}  ({len(range(0, len(rows), per_min))} baris, k={hi * 60:.5f}/menit)")
    print(f"Setpoint pertama tercapai: menit ke-{first:.1f} "
          f"({(t_start + timedelta(minutes=first)).strftime('%H:%M')})")


if __name__ == "__main__":
    main()
