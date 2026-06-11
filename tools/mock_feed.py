#!/usr/bin/env python3
"""
mock_feed.py — pretend to be ride_sim: stream distance/speed over UDP.

Sends JSON-lines packets {"distance_m":..,"speed_mps":..} to the Godot world so
you can exercise LIVE mode before wiring real ride_sim. Matches the contract in
ride-sim/docs/engine_interface.md. Stdlib only.

  python tools/mock_feed.py --route data/route.json --hz 4 --speed 7
"""
import argparse
import json
import socket
import time


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--route", default="data/route.json")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--hz", type=float, default=4.0)
    ap.add_argument("--speed", type=float, default=7.0, help="m/s")
    args = ap.parse_args()

    with open(args.route) as f:
        length = float(json.load(f)["length_m"])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dt = 1.0 / args.hz
    d = 0.0
    print(f"feeding udp://{args.host}:{args.port}  {length/1000:.1f} km @ {args.speed} m/s")
    try:
        while True:
            d = (d + args.speed * dt) % length
            msg = json.dumps({"distance_m": round(d, 2), "speed_mps": args.speed})
            sock.sendto(msg.encode(), (args.host, args.port))
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
