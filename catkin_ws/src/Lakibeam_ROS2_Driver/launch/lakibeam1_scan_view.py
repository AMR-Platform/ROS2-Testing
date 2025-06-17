#!/usr/bin/env python3
"""
Lakibeam-1 standalone visualiser  ──────────────────────────────────────────────
• No ROS  ✓  • No DDS ✓  • Just sockets + Matplotlib ✓
"""
import argparse, math, socket, struct, sys, time
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# ────────────────────────── 1.  CLI arguments  ────────────────────────────────
parser = argparse.ArgumentParser(
    description="Listen to a Lakibeam-1 LiDAR over raw UDP and plot a live "
                "2-D scan (distance vs azimuth).")
add = parser.add_argument
add("--hostip",   default="0.0.0.0",            help="local NIC to bind")
add("--port",     type=int, default=2368,       help="UDP port (MSOP)")
add("--sensorip", default=None,                 help="discard packets not "
                                                    "from this IP")
add("--angle_offset",  type=float, default=0.0, help="deg, +CW, -CCW")
add("--inverted", action="store_true",          help="reverse scan order")
add("--scan_range_start", type=float, default=45.0,  help="deg")
add("--scan_range_stop",  type=float, default=315.0, help="deg")
add("--scanfreq",        type=int,   default=30,     help="Hz (10/20/25/30)")
add("--filter",          default="3", help="firmware filter mode (not used)")
add("--laser_enable",    default="true", help="just for CLI parity")
args = parser.parse_args()

# ────────────────────────── 2.  Packet layout constants  ──────────────────────
PKT_SIZE           = 1248                           # bytes / packet :contentReference[oaicite:0]{index=0}
PKT_HDR            = 42
BLOCK_SZ           = 100
BLOCKS             = 12
CH_PER_BLOCK       = 32                    # (100–4)/3 triads
BLOCK_HDR_FMT      = "<HH"                 # 0xEEFF flag, azimuth*100
TRIAD_FMT          = "<HB"                 # 2-byte dist, 1-byte RSSI
RES_DIST           = 0.01                  # metres per count
RES_AZ             = 0.25                  # deg step inside a block

def in_window(az):
    a0, a1 = args.scan_range_start % 360, args.scan_range_stop % 360
    if a0 < a1:
        return a0 <= az <= a1
    return az >= a0 or az <= a1           # wrapped range e.g. 300-60 deg

# ────────────────────────── 3.  UDP socket  ───────────────────────────────────
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((args.hostip, args.port))
sock.setblocking(False)
print(f"Listening UDP {args.hostip}:{args.port} …", file=sys.stderr)

# ────────────────────────── 4.  Matplotlib polar live plot  ───────────────────
plt.style.use("fast")
fig  = plt.figure(figsize=(6,6))
ax   = fig.add_subplot(111, projection='polar')
ax.set_theta_zero_location("x")     # 0 deg at +X
ax.set_theta_direction(-1)          # CW = +ve (matches ROS REP 103)
scat = ax.scatter([], [], s=2)
ax.set_rlim(0, 18)                  # 18 m default range
ax.grid(True); ax.set_title("Lakibeam-1 live scan")

# ────────────────────────── 5.  Packet → point helper  ────────────────────────
def parse_block(raw):
    flag, az_raw = struct.unpack_from(BLOCK_HDR_FMT, raw, 0)
    if flag != 0xFFEE:
        return []
    az0 = az_raw / 100.0
    out = []
    for i in range(CH_PER_BLOCK):
        dist_raw, rssi = struct.unpack_from(TRIAD_FMT, raw, 4 + i*3)
        if dist_raw in (0, 0xFFFF):
            continue
        az = (az0 + RES_AZ * i) % 360.0
        dist = dist_raw * RES_DIST
        out.append((az, dist, rssi))
    return out

# ────────────────────────── 6.  Acquisition loop (runs inside FuncAnimation) ──
scan = defaultdict(lambda: None)
last_az = None

def update(_frame):
    global last_az
    # ── pull as many packets as are waiting ───────────────────────────────────
    while True:
        try:
            data, (src_ip, _) = sock.recvfrom(PKT_SIZE)
        except BlockingIOError:
            break
        if len(data) != PKT_SIZE or (args.sensorip and src_ip != args.sensorip):
            continue
        for b in range(BLOCKS):
            blk = data[PKT_HDR + b*BLOCK_SZ : PKT_HDR + (b+1)*BLOCK_SZ]
            for az, dist, _rssi in parse_block(blk):
                az = (az + args.angle_offset) % 360.0
                if not in_window(az):
                    continue
                scan[az] = dist
                if last_az is not None and az < last_az:
                    # wrap → full scan ready
                    plot_scan()
                    scan.clear()
                last_az = az

def plot_scan():
    if not scan:
        return
    azs = np.array(sorted(scan.keys(), reverse=args.inverted))
    dists = np.array([scan[a] for a in azs])
    scat.set_offsets(np.c_[np.deg2rad(azs), dists])
    fig.canvas.draw_idle()

ani = FuncAnimation(fig, update, interval=50)   # 20 FPS UI refresh
plt.show()
