#!/usr/bin/env python3
"""
control_robstride.py
Gripper 2 motor - cham vat TU DUNG voi bo loc nhieu.

Bo loc nhieu:
  - Moving average 8 mau -> giam nhieu cam bien
  - Phai co CONFIRM_TICKS tick lien tiep vuot threshold moi dung
  - Warmup 20 tick (0.4s) sau khi bat dau

Phim tat:
    Enter        -> Bat dau / Dung
    +/-          -> Tang/giam toc 0.5 rad/s
    r            -> Doi chieu
    0            -> Ve 0
    2.0          -> Set toc do = 2.0 rad/s
    t0.12        -> Doi threshold = 0.12 N·m
    q            -> Thoat

Chay:
    python3 control_robstride.py
    python3 control_robstride.py --id1 7 --id2 6 --speed 1.5 --threshold 0.12
    python3 control_robstride.py --reverse
"""

import argparse
import can
import signal
import struct
import threading
import time
from collections import deque


def _send(bus, arb, data):
    bus.send(can.Message(arbitration_id=arb, data=data, is_extended_id=True))

def enable(bus, mid):
    _send(bus, (3 << 24) | (0xFD << 8) | mid, [0]*8)
    time.sleep(0.1)

def disable(bus, mid):
    _send(bus, (4 << 24) | (0xFD << 8) | mid, [0]*8)

def set_mode(bus, mid, mode):
    payload = struct.pack('<H', 0x7005) + b'\x00\x00' + struct.pack('<I', mode)
    _send(bus, (18 << 24) | (0xFD << 8) | mid, list(payload))
    time.sleep(0.05)

def send_spd(bus, mid, spd):
    payload = struct.pack('<H', 0x700A) + b'\x00\x00' + struct.pack('<f', float(spd))
    _send(bus, (18 << 24) | (0xFD << 8) | mid, list(payload))

def read_both(bus, mid1, mid2):
    _send(bus, (2 << 24) | (0xFD << 8) | mid1, [0]*8)
    _send(bus, (2 << 24) | (0xFD << 8) | mid2, [0]*8)
    results = {}
    deadline = time.time() + 0.030
    while time.time() < deadline:
        if len(results) >= 2:
            break
        r = bus.recv(timeout=0.005)
        if r is None:
            break
        if r.arbitration_id == 0x7FE:
            continue
        resp_mid = (r.arbitration_id >> 8) & 0xFF
        if resp_mid in (mid1, mid2) and resp_mid not in results:
            d = bytes(r.data)
            vel = struct.unpack('>H', d[2:4])[0] / 65535.0 * 88.0 - 44.0
            tor = struct.unpack('>H', d[4:6])[0] / 65535.0 * 11.0 - 5.5
            results[resp_mid] = (vel, tor)
    v1, t1 = results.get(mid1, (None, None))
    v2, t2 = results.get(mid2, (None, None))
    return t1, v1, t2, v2


class MovingAvg:
    def __init__(self, n=8):
        self.buf = deque(maxlen=n)
        self.n   = n

    def update(self, val):
        if val is not None:
            self.buf.append(val)
        return sum(self.buf) / len(self.buf) if self.buf else 0.0

    def ready(self):
        return len(self.buf) >= self.n

    def reset(self):
        self.buf.clear()


def run(args):
    ID1     = args.id1
    ID2     = args.id2
    REV     = args.reverse
    SPD_MAX = 44.0
    DT      = 0.02
    RAMP    = 1.5 * DT

    WARMUP        = 20
    CONFIRM_TICKS = 3
    FILTER_N      = 8

    s = {
        'running':   False,
        'speed':     args.speed,
        'actual':    0.0,
        'threshold': args.threshold,
        'warmup':    0,
        'quit':      False,
    }
    lock = threading.Lock()

    filt1 = MovingAvg(FILTER_N)
    filt2 = MovingAvg(FILTER_N)
    confirm_count = 0

    def show_help():
        print('─' * 60)
        print(f'  Motor {ID1} & {ID2}  |  reverse={REV}')
        print(f'  Speed: {s["speed"]:+.1f} rad/s  |  Threshold: {s["threshold"]} N·m')
        print(f'  Filter: {FILTER_N} mau avg  |  Confirm: {CONFIRM_TICKS} tick lien tiep')
        print('─' * 60)
        print('  [Enter]         -> Bat dau / Dung')
        print('  [+/-] [Enter]   -> Tang/giam toc 0.5 rad/s')
        print('  [r]   [Enter]   -> Doi chieu quay')
        print('  [0]   [Enter]   -> Ve 0 rad/s')
        print('  [2.0] [Enter]   -> Set = 2.0 rad/s')
        print('  [t0.12][Enter]  -> Doi threshold = 0.12 N·m')
        print('  [q]   [Enter]   -> Thoat')
        print('─' * 60)
        print(f'  Cham vat: {CONFIRM_TICKS} tick avg > {s["threshold"]} N·m -> TU DUNG\n')

    def kb():
        show_help()
        while not s['quit']:
            try:
                cmd = input().strip().lower()
            except EOFError:
                break
            with lock:
                if cmd == '':
                    if not s['running']:
                        s['running'] = True
                        s['warmup']  = WARMUP
                        filt1.reset()
                        filt2.reset()
                        arrow = '<<' if s['speed'] < 0 else '>>'
                        print(f'\n  RUN {arrow}  {abs(s["speed"]):.1f} rad/s\n')
                    else:
                        s['running'] = False
                        s['actual']  = 0.0
                        print('\n  STOP  (Enter de chay lai)\n')

                elif cmd == '+':
                    sign = -1 if s['speed'] < 0 else 1
                    s['speed'] = round(sign * min(abs(s['speed']) + 0.5, SPD_MAX), 1)
                    print(f'  Speed -> {s["speed"]:+.1f} rad/s')

                elif cmd == '-':
                    sign = -1 if s['speed'] < 0 else 1
                    s['speed'] = round(sign * max(abs(s['speed']) - 0.5, 0.0), 1)
                    print(f'  Speed -> {s["speed"]:+.1f} rad/s')

                elif cmd == 'r':
                    s['speed'] = -s['speed']
                    arrow = '<<' if s['speed'] < 0 else '>>'
                    print(f'  Doi chieu {arrow}  {s["speed"]:+.1f} rad/s')

                elif cmd == '0':
                    s['speed'] = 0.0
                    print('  Speed -> 0.0')

                elif cmd == 'q':
                    s['quit']    = True
                    s['running'] = False
                    print('\n  Thoat...')

                elif cmd.startswith('t'):
                    try:
                        s['threshold'] = float(cmd[1:])
                        print(f'  Threshold -> {s["threshold"]:.4f} N·m')
                    except ValueError:
                        pass

                else:
                    try:
                        s['speed'] = round(max(-SPD_MAX, min(SPD_MAX, float(cmd))), 2)
                        print(f'  Speed -> {s["speed"]:+.1f} rad/s')
                    except ValueError:
                        show_help()

    threading.Thread(target=kb, daemon=True).start()

    with can.Bus(interface=args.interface, channel=args.channel) as bus:
        print(f'Enable motor {ID1}...')
        set_mode(bus, ID1, 2)
        enable(bus, ID1)
        print(f'Enable motor {ID2}...')
        set_mode(bus, ID2, 2)
        enable(bus, ID2)
        print('Ca 2 motor enabled.\n')

        signal.signal(signal.SIGINT,
                      lambda sg, f: s.update({'quit': True, 'running': False}))

        next_tick     = time.time()
        last_print    = 0.0
        confirm_count = 0

        while not s['quit']:
            with lock:
                running   = s['running']
                target    = s['speed'] if running else 0.0
                threshold = s['threshold']
                warmup    = s['warmup']

            # Ramp
            diff = target - s['actual']
            if abs(diff) <= RAMP:
                s['actual'] = target
            else:
                s['actual'] += RAMP if diff > 0 else -RAMP

            send_spd(bus, ID1, s['actual'])
            send_spd(bus, ID2, -s['actual'] if REV else s['actual'])

            # Doc feedback
            t1_raw, v1, t2_raw, v2 = read_both(bus, ID1, ID2)

            # Moving average (dung gia tri tuyet doi)
            avg1 = filt1.update(abs(t1_raw) if t1_raw is not None else None)
            avg2 = filt2.update(abs(t2_raw) if t2_raw is not None else None)

            # Giam warmup
            if warmup > 0:
                with lock:
                    s['warmup'] = warmup - 1
                confirm_count = 0

            # Contact detection
            contact = False
            if running and warmup == 0 and abs(s['actual']) > 0.05 and filt1.ready():
                if avg1 > threshold or avg2 > threshold:
                    confirm_count += 1
                else:
                    confirm_count = 0

                if confirm_count >= CONFIRM_TICKS:
                    contact = True
                    send_spd(bus, ID1, 0.0)
                    send_spd(bus, ID2, 0.0)
                    with lock:
                        s['running'] = False
                        s['actual']  = 0.0
                    confirm_count = 0
                    print(f'\n  CHAM VAT!'
                          f'  avg1={avg1:.3f}  avg2={avg2:.3f} N·m'
                          f'  (nguong={threshold:.3f})')
                    print(f'  TU DUNG ca 2 motor.')
                    print(f'  [r]+Enter = mo ra  |  [Enter] = kep lai\n')
            else:
                if not running:
                    confirm_count = 0

            # Print 5 Hz
            t_now = time.time()
            if t_now - last_print >= 0.2:
                last_print = t_now
                st  = 'RUN' if running else 'STP'
                r1  = f'{t1_raw:+.3f}' if t1_raw is not None else ' ---'
                r2  = f'{t2_raw:+.3f}' if t2_raw is not None else ' ---'
                wm  = f' (warm {warmup})' if warmup > 0 else ''
                cf  = f' [{confirm_count}/{CONFIRM_TICKS}]' if confirm_count > 0 else ''
                flg = '  CONTACT!' if contact else ''
                print(f'  [{st}] spd={s["actual"]:+5.2f}'
                      f'  raw={r1}/{r2}'
                      f'  avg={avg1:.3f}/{avg2:.3f} N·m'
                      f'{wm}{cf}{flg}')

            next_tick += DT
            time.sleep(max(0, next_tick - time.time()))

        # Shutdown
        print('\nDung motor...')
        for _ in range(20):
            s['actual'] *= 0.7
            send_spd(bus, ID1, s['actual'])
            send_spd(bus, ID2, -s['actual'] if REV else s['actual'])
            time.sleep(DT)
        send_spd(bus, ID1, 0.0)
        send_spd(bus, ID2, 0.0)
        time.sleep(0.2)
        for mid in (ID1, ID2):
            disable(bus, mid)
            set_mode(bus, mid, 0)
        print('Done.')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--id1',       type=int,   default=7)
    ap.add_argument('--id2',       type=int,   default=6)
    ap.add_argument('--speed',     type=float, default=1.5)
    ap.add_argument('--threshold', type=float, default=0.12,
                    help='N·m threshold (default: 0.12)')
    ap.add_argument('--reverse',   action='store_true')
    ap.add_argument('--interface', default='socketcan')
    ap.add_argument('--channel',   default='can0')
    run(ap.parse_args())

if __name__ == '__main__':
    main()
