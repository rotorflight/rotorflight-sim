#!/usr/bin/env python3
"""
Lockstep bridge between the Rotorflight SITL and heli_model.Heli (bridge v0).

    python3 heli_bridge.py <rotorflight_SITL.elf> [--seconds 40] [--log flight.csv]

Starts the SITL in a fresh directory with SITL_LOCKSTEP=1, configures it over the CLI
(ARM on AUX1, angle mode on AUX2), then steps model and firmware together: each 250 us
the model state goes out as a 224-byte FDM packet, the firmware's 68-byte output packet
comes back (motor 1 throttle, servos, stabilized mixer inputs) and drives the model.
RC goes in over MSP RX on UART1 every 20 ms of simulated time, without blocking the
loop. A simple scripted flight: calibrate, arm, spool up, lift off and hover in angle
mode, land, disarm.

Only the Python standard library is used.
"""

import argparse
import math
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from heli_model import Heli, euler   # noqa: E402
from swash import Swash               # noqa: E402

DT = 250e-6
MSP_SET_RAW_RC = 200
MSP_ACC_CALIBRATION = 205


def msp_frame(cmd, payload=b''):
    body = bytes([len(payload), cmd]) + payload
    ck = 0
    for b in body:
        ck ^= b
    return b'$M<' + body + bytes([ck])


def msp_request(sock, cmd, timeout=3.0):
    """Blocking MSP v1 request on a blocking socket; returns the reply payload."""
    sock.sendall(msp_frame(cmd))
    buf, end = b'', time.time() + timeout
    while time.time() < end:
        buf += sock.recv(4096)
        i = buf.find(b'$M>')
        if i >= 0 and len(buf) >= i + 6 + buf[i + 3] and buf[i + 4] == cmd:
            return buf[i + 5:i + 5 + buf[i + 3]]
        if i >= 0 and len(buf) >= i + 6 + buf[i + 3]:
            buf = buf[i + 6 + buf[i + 3]:]
    raise RuntimeError('no MSP reply to %d' % cmd)


def wait_for(path, text, timeout):
    end = time.time() + timeout
    while time.time() < end:
        try:
            if text in open(path, errors='replace').read():
                return True
        except OSError:
            pass
        time.sleep(0.1)
    return False


def cli(port, commands):
    s = socket.create_connection(('127.0.0.1', port), timeout=3)
    s.settimeout(0.3)
    out = b''

    def drain(t):
        nonlocal out
        end = time.time() + t
        while time.time() < end:
            try:
                d = s.recv(4096)
                if not d:
                    return
                out += d
            except socket.timeout:
                pass
    s.sendall(b'#\r\n'); drain(1.0)
    for c in commands:
        s.sendall(c.encode() + b'\r\n'); drain(1.0)
    s.close()
    return out.decode(errors='replace')


def start_sitl(elf, rundir, name, lockstep):
    env = dict(os.environ)
    if lockstep:
        env['SITL_LOCKSTEP'] = '1'
    log = open(os.path.join(rundir, name + '.log'), 'w')
    proc = subprocess.Popen([elf], cwd=rundir, stdout=log, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, env=env)
    return proc, log


def schedule(t, governor=False):
    """Scripted pilot: returns (throttle, collective, aux1 arm, aux2 angle, roll, pitch, yaw) in us.
    Without the governor the throttle stick ramps to 65 % (the model's hover throttle at
    nominal headspeed); with it the stick goes to 100 % and the governor spools up."""
    arm = 1900 if 6.0 <= t < 34.0 else 1000
    angle = 1900
    thr = 1000
    if 8.0 <= t < 34.0:
        thr = 2000 if governor else 1000 + min(1.0, (t - 8.0) / 4.0) * 650
    # collective: ~2.05e-3 per us at the mixer; the model hovers at 0.44 (215 us) at 2100 rpm
    coll = 1500 - 150                                            # low pitch on the ground
    if 18.0 <= t < 26.0:
        coll = 1500 + 230                                        # lift off, climb slowly
        if 24.0 <= t < 25.0:
            coll = 1500 + 330                                    # 1 s punch: loads the governor
    elif 26.0 <= t < 33.0:
        coll = 1500 + 195                                        # descend and land
    roll = 1500 + (150 if 21.0 <= t < 22.0 else 0)              # roll right for 1 s
    pitch = 1500 + (150 if 23.0 <= t < 24.0 else 0)             # pitch stick for 1 s
    return thr, coll, arm, angle, roll, pitch, 1500


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('elf')
    ap.add_argument('--seconds', type=float, default=36.0)
    ap.add_argument('--log', default=None)
    ap.add_argument('--controls', choices=('servos', 'mixer'), default='servos',
                    help='drive the model from the servo outputs through inverse swash mixing '
                         '(default), or from the stabilized mixer inputs')
    ap.add_argument('--cli', action='append', default=[], metavar='COMMAND',
                    help='extra CLI command before save (repeatable), e.g. "set swash_type = CP135"')
    ap.add_argument('--governor', type=int, default=0, metavar='RPM',
                    help='fly with the ELECTRIC governor holding this headspeed')
    a = ap.parse_args()
    elf = os.path.abspath(a.elf)
    rundir = tempfile.mkdtemp(prefix='heli-bridge-')

    # first start writes defaults; then configure over the CLI (save reboots)
    subprocess.run([elf], cwd=rundir, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
    proc, log = start_sitl(elf, rundir, 'config', False)
    wait_for(os.path.join(rundir, 'config.log'), 'bind port 5761', 15)
    commands = ['aux 0 0 0 1700 2100 0 0', 'aux 1 1 1 1700 2100 0 0', 'set motor_poles = 2']
    if a.governor:
        commands += ['feature GOVERNOR', 'set gov_mode = ELECTRIC', 'set main_rotor_gear_ratio = 1,10',
                     'set gov_headspeed = %d' % a.governor]
    out_cli = cli(5761, commands + a.cli + ['save'])
    if 'Invalid' in out_cli or 'ERROR' in out_cli:
        print(out_cli)
    proc.wait(15); log.close()

    # The firmware's stabilized mixer inputs follow Betaflight's sensor frame (X forward,
    # Y left, Z up): positive roll is right wing down, as in the model's FRD, but positive
    # pitch is nose down and positive yaw is nose left - the opposite of FRD q and r.
    heli = Heli(sign_roll=1.0, sign_pitch=-1.0, sign_yaw=-1.0)
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    out.bind(('127.0.0.1', 9002)); out.settimeout(0.05)
    fdm = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    proc, log = start_sitl(elf, rundir, 'fly', True)
    rows = []
    try:
        wait_for(os.path.join(rundir, 'fly.log'), 'bind port 5761', 15)
        msp = socket.create_connection(('127.0.0.1', 5761), timeout=3)
        # before the first FDM packet the firmware still runs on host time and answers
        swash = Swash.from_msp(lambda cmd: msp_request(msp, cmd))
        print(swash.describe())
        msp.setblocking(False)
        decode_err, decode_at, decode_sq, decode_n = [0.0] * 4, [None] * 4, [0.0] * 4, 0
        time.sleep(1.0)
        msp.send(msp_frame(MSP_ACC_CALIBRATION))

        ctrl = (0.0, 0.0, 0.0, 0.0, 0.0)    # throttle, roll, pitch, yaw, collective
        steps = int(a.seconds / DT)
        rc_every = int(0.02 / DT)
        timeouts = 0
        t0 = time.time()
        for n in range(steps):
            t = n * DT
            if n % rc_every == 0:
                thr, coll, arm, angle, r, p, y = schedule(t, bool(a.governor))
                ch = [int(round(v)) for v in (r, p, thr, y, coll, arm, angle, 1000)]
                msp.send(msp_frame(MSP_SET_RAW_RC, struct.pack('<8H', *ch)))
                try:
                    msp.recv(4096)          # drain replies
                except (BlockingIOError, OSError):
                    pass

            heli.step(DT, *ctrl)
            pressure = 101325.0 * (1 - 2.25577e-5 * (-heli.pos[2])) ** 5.25588
            pkt = struct.pack('<28d', t, *heli.rates, *heli.specific_force, *heli.q,
                              *heli.vel, *heli.pos,
                              heli.motor_rpm, 0, 0, 0,
                              pressure, 3.3, 40.0, 8.0, 5.0, heli.vbat, heli.ibat)
            fdm.sendto(pkt, ('127.0.0.1', 9003))
            try:
                d, _ = out.recvfrom(256)
                if len(d) >= 68:
                    motor = struct.unpack_from('<4f', d, 0)
                    mix = struct.unpack_from('<5f', d, 48)
                    if a.controls == 'servos':
                        servo_us = struct.unpack_from('<8f', d, 16)
                        roll_, pitch_, yaw_, coll_ = swash.decode(servo_us)
                        if min(servo_us[:4]) > 0 and t >= 8.0:     # servos live, armed
                            decode_n += 1
                            for i, v in enumerate((roll_, pitch_, yaw_, coll_)):
                                e = abs(v - mix[i])
                                decode_sq[i] += e * e
                                if e > decode_err[i]:
                                    decode_err[i], decode_at[i] = e, (t, v, mix[i])
                        ctrl = (motor[0], roll_, pitch_, yaw_, coll_)
                    else:
                        ctrl = (motor[0], mix[0], mix[1], mix[2], mix[3])
            except socket.timeout:
                timeouts += 1

            if n % int(float(os.environ.get('LOG_EVERY', '0.1')) / DT) == 0:
                roll, pitch, yaw = euler(heli.q)
                rows.append((t, -heli.pos[2], -heli.vel[2], math.degrees(roll), math.degrees(pitch),
                             math.degrees(yaw), heli.rotor_rpm, ctrl[0], ctrl[4], ctrl[1], ctrl[2],
                             heli.vbat, heli.ibat, *heli.rates))
        real = time.time() - t0
        print('%.0f s simulated in %.0f s real, %d steps without reply' % (a.seconds, real, timeouts))
        if a.controls == 'servos':
            print('inverse mixing vs mixer inputs, max |diff| roll %.4f pitch %.4f yaw %.4f coll %.4f'
                  % tuple(decode_err))
            print('  rms roll %.4f pitch %.4f yaw %.4f coll %.4f; worst (t, decoded, mixer): %s'
                  % (*(math.sqrt(q / max(decode_n, 1)) for q in decode_sq),
                     [tuple(round(x, 3) for x in w) for w in decode_at]))
    finally:
        proc.terminate()
        try:
            proc.wait(5)
        except subprocess.TimeoutExpired:
            proc.kill()
        log.close()

    hdr = 't,alt_m,climb_ms,roll_deg,pitch_deg,yaw_deg,rotor_rpm,throttle,mix_coll,mix_roll,mix_pitch,vbat,ibat,p,q,r'
    if a.log:
        with open(a.log, 'w') as f:
            f.write(hdr + '\n')
            for r in rows:
                f.write(','.join('%.4g' % v for v in r) + '\n')
    step = 20 if not os.environ.get('DETAIL') else 2
    for r in rows[::step]:
        if os.environ.get('DETAIL') and not (19.5 <= r[0] <= 27):
            continue
        print('t %5.1f  alt %6.2f m  climb %+5.2f  roll %+6.1f  pitch %+6.1f  rpm %5.0f  thr %.2f  coll %+.2f'
              % (r[0], r[1], r[2], r[3], r[4], r[6], r[7], r[8]))
    shutil.rmtree(rundir, ignore_errors=True)


if __name__ == '__main__':
    main()
