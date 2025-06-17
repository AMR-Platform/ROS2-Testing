#!/usr/bin/env python3
"""
Standalone LakiBeam-1 UDP reader 
--------------------------------------------------
Usage examples
--------------
# use almost-factory defaults
python lakibeam_reader.py

# override a few parameters
python lakibeam_reader.py --sensorip 192.168.8.2 --port 2368 --angle_offset -90
"""
import argparse
import math
import socket
import struct
import sys
from collections import defaultdict


parser = argparse.ArgumentParser(
    description="Listen to a LakiBeam-1 LiDAR over raw UDP and dump one full "
                "scan (angle, distance) to stdout – no ROS needed.")
parser.add_argument("--hostip",           default="0.0.0.0",
                    help="Local interface to bind (use 0.0.0.0 for all)")
parser.add_argument("--port",             type=int, default=2368,
                    help="Port that MSOP packets arrive on")
parser.add_argument("--sensorip",         default=None,
                    help="Optional: discard packets that don’t come from this IP")
parser.add_argument("--angle_offset",     type=float, default=0.0,
                    help="Add this offset (degrees) to every reported azimuth")
parser.add_argument("--scan_range_start", type=float, default=45.0,
                    help="Ignore points before this angle (deg)")
parser.add_argument("--scan_range_stop",  type=float, default=315.0,
                    help="Ignore points after  this angle (deg)")
parser.add_argument("--inverted",         action="store_true",
                    help="Reverse the order of points in each scan")
args = parser.parse_args()


PKT_SIZE             = 1248           # bytes / UDP packet
PKT_HEADER_LEN       = 42             # ethernet + IP + UDP header
DATA_BLOCK_SIZE      = 100            # bytes per “Data Block”
BLOCK_COUNT          = 12             # blocks per packet
BLOCK_HEADER_FMT     = "<HH"          # 0xEEFF flag, 16-bit azimuth
CHANNELS_PER_BLOCK   = 32             # (100-4)/3 = 32 tri-bytes (distL, distH, RSSI)
TRIAD_FMT            = "<HB"          # 2-byte distance, 1-byte RSSI



sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((args.hostip, args.port))
print(f"Listening on {args.hostip}:{args.port} …", file=sys.stderr)


def parse_block(raw: bytes):
    """Parse one 100-byte Data Block -> list[(azimuth_deg, distance_m, rssi)]."""
    flag, az_raw = struct.unpack_from(BLOCK_HEADER_FMT, raw, 0)
    if flag != 0xFFEE:        # invalid / filler block
        return []
    az0 = az_raw / 100.0      # hundredths of a degree → degrees
    # Azimuth increment inside the block is 0.25 deg; index 0 gets az0
    readings = []
    for i in range(CHANNELS_PER_BLOCK):
        dist_raw, rssi = struct.unpack_from(TRIAD_FMT, raw, 4 + i * 3)
        if dist_raw == 0 or dist_raw == 0xFFFF:
            continue          # invalid point
        az = az0 + 0.25 * i
        distance_m = dist_raw / 100.0   # hundredths of a metre → metres
        readings.append((az, distance_m, rssi))
    return readings

def in_range(az):
    """Return True if azimuth is inside the requested scan window."""
    if args.scan_range_start < args.scan_range_stop:
        return args.scan_range_start <= az <= args.scan_range_stop
    # support wrapped ranges like 350->10
    return az >= args.scan_range_start or az <= args.scan_range_stop


scan = defaultdict(lambda: None)  # azimuth → (distance_m, rssi)
last_az = None

while True:
    data, (src_ip, _) = sock.recvfrom(PKT_SIZE)
    if len(data) != PKT_SIZE:
        continue  # skip unexpected sizes
    if args.sensorip and src_ip != args.sensorip:
        continue
    # Iterate over the 12 data blocks
    for blk_idx in range(BLOCK_COUNT):
        off = PKT_HEADER_LEN + blk_idx * DATA_BLOCK_SIZE
        block = data[off:off + DATA_BLOCK_SIZE]
        for az, dist_m, rssi in parse_block(block):
            # Apply offset and wrap to [0,360)
            az = (az + args.angle_offset) % 360.0
            if not in_range(az):
                continue
            scan[az] = (dist_m, rssi)
            # Detect wrap-around → full revolution finished
            if last_az is not None and az < last_az:   # e.g. 359 → 0
                # ===  FULL SCAN READY  ===
                angles = sorted(scan.keys(), reverse=args.inverted)
                print("\n=== NEW SCAN ===")
                for a in angles:
                    d, r = scan[a]
                    print(f"{a:7.2f}°  {d:6.2f} m  RSSI={r}")
                scan.clear()
            last_az = az
