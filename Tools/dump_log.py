#!/usr/bin/env python3
"""Dump the Kidney Guard CSV log from the board over USB serial.

Sends the 'd' command to kidney_guard.ino and saves the lines between
'# BEGIN' and '# END' to a CSV file. Close the Arduino Serial Monitor
first, only one program can open the port.

Usage:
    python3 Tools/dump_log.py                     # auto-detect port
    python3 Tools/dump_log.py -p /dev/cu.usbmodem1101 -o session.csv

Requires pyserial:  python3 -m pip install pyserial
"""
import argparse
import glob
import os
import sys

import serial

# Logs/ in the repo root, independent of the current working directory
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Logs")
DEFAULT_OUTPUT = os.path.normpath(os.path.join(LOG_DIR, "kidney_log.csv"))


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        sys.exit("no /dev/cu.usbmodem* port found, is the board connected?")
    if len(ports) > 1:
        sys.exit(f"several ports found, pick one with -p: {', '.join(ports)}")
    return ports[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("-p", "--port", help="serial port (default: auto-detect)")
    parser.add_argument("-o", "--output", default=DEFAULT_OUTPUT,
                        help="output CSV file (default: Logs/kidney_log.csv)")
    parser.add_argument("-t", "--timeout", type=float, default=5.0,
                        help="seconds of silence before giving up (default: 5)")
    args = parser.parse_args()

    port = args.port or find_port()
    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    rows = 0
    with serial.Serial(port, 115200, timeout=args.timeout) as s, \
            open(args.output, "w") as out:
        s.reset_input_buffer()  # discard live rows already waiting
        s.write(b"d")
        started = False
        while True:
            line = s.readline()
            if not line:
                sys.exit(f"timeout: no '# END' received from {port}")
            line = line.decode(errors="replace").rstrip("\r\n")
            if line.startswith("# BEGIN"):
                started = True
            elif line.startswith("# END"):
                break
            elif started:
                out.write(line + "\n")
                rows += 1

    # rows includes the header line
    print(f"saved {max(rows - 1, 0)} rows from {port} to {args.output}")


if __name__ == "__main__":
    main()
