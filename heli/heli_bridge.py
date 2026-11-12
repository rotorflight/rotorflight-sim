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
import json
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
from heli_model import MODELS, euler, body_to_earth, load_model, resolve, trained_models  # noqa: E402
from configs import load_dump         # noqa: E402
import csv                            # noqa: E402
from swash import Swash               # noqa: E402
from esc import Blh32Esc, Hw4Esc      # noqa: E402
from sensors import Sensors, available as sensors_available   # noqa: E402
from wind import Wind                 # noqa: E402
from servos import Servos, TYPES as SERVO_TYPES   # noqa: E402

DT = 250e-6
MSP_SET_RAW_RC = 200
MSP_ACC_CALIBRATION = 205
MSP_MOTOR_TELEMETRY = 139
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_DEBUG = 254       # the firmware's eight debug[] values (int32), of its debug_mode
ARMING_FLAG_NAMES = [
    "NOGYRO", "FAILSAFE", "RXLOSS", "BADRX", "BOXFAILSAFE", "GOVERNOR", "RPM_SIGNAL",
    "THROTTLE", "ANGLE", "BOOTGRACE", "NOPREARM", "LOAD", "CALIB", "CLI", "CMS", "BST",
    "MSP", "PARALYZE", "GPS", "RESCUE_SW", "RPMFILTER", "REBOOT_REQD", "DSHOT_BBANG",
    "NO_ACC_CAL", "MOTOR_PROTO", "OVERRIDE", "ARMSWITCH",
]
MOTOR_POLES = 2
GYRO_COUNTS_PER_DPS = 16.4    # the SITL's fake gyro (GYRO_SCALE in target.c)


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


def cli(port, commands, timeout=3.0):
    """Enter the CLI and run commands one by one, each up to its '# ' prompt."""
    s = socket.create_connection(('127.0.0.1', port), timeout=3)
    s.settimeout(0.05)
    out = b''

    def until_prompt(t):
        nonlocal out
        mark = len(out)
        end = time.time() + t
        while time.time() < end:
            try:
                d = s.recv(65536)
                if not d:
                    return
                out += d
                if b'\n# ' in out[mark:] or out[mark:].endswith(b'# '):
                    return
            except socket.timeout:
                pass
    s.sendall(b'#\r\n'); until_prompt(timeout)
    cli.rejected = []
    for c in commands:
        mark = len(out)
        s.sendall(c.encode() + b'\r\n')
        until_prompt(1.0 if c == 'save' else timeout)
        if b'ERROR' in out[mark:] or b'Invalid' in out[mark:]:
            cli.rejected.append(c)
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


REPLAY_LEAD = 8.0     # s of calibration and arming before the recorded sticks start
REPLAY_TAIL = 3.0     # s disarmed after them: the blackbox closes its log and the flash drains


def load_sticks(path):
    """Recorded sticks: rows t, roll, pitch, yaw, collective, throttle (rcCommand units)."""
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append(tuple(float(r[k]) for k in ('t', 'roll', 'pitch', 'yaw', 'coll', 'thr')))
    return rows


def load_case(folder):
    """A replay case: case.json beside the recorded sticks and the flown tune."""
    with open(os.path.join(folder, 'case.json')) as f:
        return json.load(f)


def replay_rc(sticks, t):
    """RC frame at sim time t: hold neutral, arm at 5 s, then the recording from REPLAY_LEAD on."""
    arm = 1900 if 5.0 <= t and sticks[0][0] + t - REPLAY_LEAD <= sticks[-1][0] else 1000
    tr = sticks[0][0] + t - REPLAY_LEAD
    if tr < sticks[0][0]:
        return 1000, 1500 - 500, arm, 1000, 1500, 1500, 1500
    lo, hi = 0, len(sticks) - 1
    while lo < hi:                         # last row at or before tr
        mid = (lo + hi + 1) // 2
        if sticks[mid][0] <= tr:
            lo = mid
        else:
            hi = mid - 1
    _, r, p, y, c, thr = sticks[lo]
    return 1000 + thr, 1500 + c, arm, 1000, 1500 + r, 1500 + p, 1500 + y


def save_blackbox(flash, out):
    """Copy the used part of the SITL's file-backed flash (erased bytes are 0xFF)."""
    try:
        data = open(flash, 'rb').read()
    except OSError:
        print('no blackbox flash file')
        return
    end = len(data.rstrip(bytes([0xFF])))
    open(out, 'wb').write(data[:end])
    print('blackbox: %d bytes -> %s' % (end, out))


COLL_PER_US = 2.05e-3   # mixer collective per us of stick (default collective rate)


def hover_us(heli):
    """Collective stick above centre at which the model hovers at its nominal headspeed."""
    return heli.coll_hover / COLL_PER_US


def schedule(t, governor=False, acro=False, hover=215.0):
    """Scripted pilot: returns (throttle, collective, aux1 arm, aux2 angle, roll, pitch, yaw) in us.
    Without the governor the throttle stick ramps to 65 % (the model's hover throttle at
    nominal headspeed); with it the stick goes to 100 % and the governor spools up.
    The collective goes in proportion to hover (us above centre, hover_us(); the Blade 360
    215, whose margins the ratios keep)."""
    arm = 1900 if 6.0 <= t < 34.0 else 1000
    angle = 1000 if acro else 1900
    thr = 1000
    if 8.0 <= t < 34.0:
        thr = 2000 if governor else 1000 + min(1.0, (t - 8.0) / 4.0) * 650
    # collective: ~2.05e-3 per us at the mixer; the Blade 360 hovers at 0.44 (215 us) at 2100 rpm
    coll = 1500 - 150                                            # low pitch on the ground
    if 18.0 <= t < 26.0:
        coll = 1500 + hover * 230 / 215                          # lift off, climb slowly
        if 24.0 <= t < 25.0:
            coll = 1500 + hover * 330 / 215                      # 1 s punch: loads the governor
    elif 26.0 <= t < 33.0:
        coll = 1500 + hover * 195 / 215                          # descend and land
    if acro:
        # doublets: 0.5 s one way, hold 1 s, 0.5 s back - the attitude must be held in
        # between and come back level
        def doublet(t0):
            return 150 if t0 <= t < t0 + 0.5 else -150 if t0 + 1.5 <= t < t0 + 2.0 else 0
        roll = 1500 + doublet(20.5)
        pitch = 1500 + doublet(23.0)
    else:
        roll = 1500 + (150 if 21.0 <= t < 22.0 else 0)          # roll right for 1 s
        pitch = 1500 + (150 if 23.0 <= t < 24.0 else 0)         # pitch stick for 1 s
    return thr, coll, arm, angle, roll, pitch, 1500


class RescuePilot:
    """Scripted rescue test: climb in angle mode, roll inverted in acro, let go, rescue.

    Reacts to the model's attitude: the roll stick is held until the heli is inverted
    (whatever the rate profile), the sticks are centred, and the rescue switch (AUX3)
    goes on half a second later and stays on for RESCUE_HOLD s."""
    CLIMB_END = 27.0          # angle-mode climb 18-27 s
    RESCUE_HOLD = 6.0

    def __init__(self, hover=215.0, governor=False):
        self.hover = hover        # hover collective, us above centre (hover_us())
        self.governor = governor  # full throttle stick for the governor, as schedule()
        self.t_inverted = None    # sticks released, heli inverted
        self.t_rescue = None      # rescue switch on
        self.t_end = None         # rescue off, angle mode, descend

    def rc(self, t, heli):
        arm = 1900 if t >= 6.0 and (self.t_end is None or t < self.t_end + 8.0) else 1000
        thr = 1000 + min(1.0, max(0.0, (t - 8.0) / 4.0)) * 650 if t >= 8.0 else 1000
        if self.governor and t >= 8.0:
            thr = 2000
        h = self.hover
        coll, angle, roll, aux3 = 1500 - 150, 1900, 1500, 1000
        if 18.0 <= t < self.CLIMB_END:
            coll = 1500 + h * 260 / 215                            # climb to height
        elif t >= self.CLIMB_END and self.t_inverted is None:
            angle = 1000                                           # acro
            bank = abs(math.degrees(euler(heli.q)[0]))
            coll = 1500 + (h if bank < 90 else -h)                # hover collective, either way up
            if bank < 150:
                roll = 1500 + 500                                  # full roll until inverted
            else:
                self.t_inverted = t
        if self.t_inverted is not None:
            coll, angle = 1500 - h, 1000                           # inverted hover, sticks centred
            if self.t_rescue is None and t >= self.t_inverted + 0.5:
                self.t_rescue = t
            if self.t_rescue is not None and t < self.t_rescue + self.RESCUE_HOLD:
                # rescue on, the pilot lets go: collective centred (rescue adds 1/4 of it)
                coll, aux3 = 1500, 1900
            elif self.t_rescue is not None:
                if self.t_end is None:
                    self.t_end = t
                coll, angle = 1500 + h * 195 / 215, 1900           # angle mode, descend
        return thr, coll, arm, angle, roll, 1500, 1500, aux3


class AutoPilot:
    """Scripted autorotation with the ELECTRIC governor (needs --governor).

    Spool up and climb in angle mode to ~40 m, then the throttle stick goes to 15 % - above
    throttle off, below the governor's 25 % handover - and the governor enters
    AUTOROTATION with the motor at idle. The pilot holds the rotor speed with the
    collective (proportional on headspeed around the model's autorotation collective),
    and flares on the collective near the ground: below FLARE_M the collective follows a
    sink rate that falls with the height, so the rotor's energy cushions the landing.
    With bailout=True the throttle stick goes back to 100 % at BAILOUT_M instead: the
    governor's BAILOUT state spools the motor back up, the pilot climbs away."""
    CLIMB_START, CLIMB_END = 18.0, 34.0
    FLARE_M = 8.0
    BAILOUT_M = 20.0
    PER_US = 2.05e-3          # mixer collective per us of stick (the bridge's mapping)

    def __init__(self, target_rpm, bailout=False):
        self.target = float(target_rpm)
        self.bailout = bailout
        self.t_cut = None
        self.t_flare = None
        self.t_bailout = None
        self.t_land = None

    def rc(self, t, heli):
        arm = 1900 if t >= 6.0 and (self.t_land is None or t < self.t_land + 3.0) else 1000
        thr = 2000 if t >= 8.0 else 1000
        coll = 1500 - 150
        alt = heli.altitude
        climb = -heli.vel[2]
        if self.CLIMB_START <= t < self.CLIMB_END:
            coll = 1500 + 330
        elif t >= self.CLIMB_END and self.t_bailout is None and self.t_land is None:
            if self.t_cut is None:
                self.t_cut = t
            thr = 1150
            # hold the rotor speed: more collective when it runs fast; autorotation collective
            # of the model at its nominal rotor speed as the base
            err = heli.rotor_rpm / self.target - 1.0
            base = 0.35 * heli.coll_hover / self.PER_US
            coll = 1500 + base + 900.0 * err
            if self.bailout and alt < self.BAILOUT_M:
                self.t_bailout = t
            elif not self.bailout and alt < self.FLARE_M:
                if self.t_flare is None:
                    self.t_flare = t
                # flare: sink rate target from 6 m/s at FLARE_M down to 0.3 m/s at the skids
                want = -(0.3 + 5.7 * max(0.0, alt - 0.5) / self.FLARE_M)
                coll = 1500 + base + 160.0 * (want - climb)
            if heli.on_ground and t > self.t_cut + 1.0:
                self.t_land = t
            coll = max(1500 - 200, min(1500 + 450, coll))
        if self.t_bailout is not None:
            thr = 2000
            coll = 1500 + 380                                       # punch out of the descent
        if self.t_land is not None:
            thr, coll = 1150, 1500 - 150
        return thr, coll, arm, 1900, 1500, 1500, 1500, 1000


def check_autorotation(rows, timeouts, steps, pilot, heli):
    """The autorotation test (AutoPilot): fixed bounds after the throttle cut."""
    ALT, CLIMB, ROLL, PITCH, RPM, THR = 1, 2, 3, 4, 6, 7
    if pilot.t_cut is None:
        print('FAIL autorotation: never cut the throttle')
        return False
    tc = pilot.t_cut

    def span(t0, t1, col):
        return [r[col] for r in rows if t0 <= r[0] <= t1]
    t_end = pilot.t_bailout or pilot.t_flare or rows[-1][0]
    rpm = span(tc + 3.0, t_end, RPM)
    sink = span(tc + 4.0, t_end, CLIMB)
    motor = span(tc + 2.5, t_end, THR)        # after the governor's spooldown ramp
    checks = [
        ('height for the test: above 25 m at the cut', span(tc, tc + 0.1, ALT)[0] > 25,
         '%.1f m' % span(tc, tc + 0.1, ALT)[0]),
        # the governor's AUTOROTATION output idles at ~12 % here: below the back-EMF at the
        # rotor speed, so the motor gives no torque
        ('governor AUTOROTATION: motor at idle (below 15 %) from 2.5 s after the cut', max(motor) < 0.15,
         'max %.2f' % max(motor)),
        ('rotor speed held by the airflow: 85-115 %% of %d rpm from 3 s after the cut' % pilot.target,
         min(rpm) > 0.85 * pilot.target and max(rpm) < 1.15 * pilot.target,
         '%.0f-%.0f rpm' % (min(rpm), max(rpm))),
        ('steady descent: sink 3-10 m/s from 4 s after the cut', all(-10 < v < -3 for v in sink),
         '%.1f to %.1f m/s' % (min(sink), max(sink))),
        ('upright: tilt below 15 deg in the descent',
         max(abs(v) for v in span(tc, t_end, ROLL) + span(tc, t_end, PITCH)) < 15,
         '%.1f deg' % max(abs(v) for v in span(tc, t_end, ROLL) + span(tc, t_end, PITCH))),
    ]
    if pilot.bailout:
        tb = pilot.t_bailout
        if tb is None:
            checks.append(('bailout: reached the bailout height', False, 'never'))
        else:
            # the airflow already holds the headspeed: the governor must bring the motor back
            # (BAILOUT) and hold the headspeed under the punch
            back = next((r[0] - tb for r in rows if r[0] >= tb and r[THR] > 0.4), None)
            rpm_b = span(tb, tb + 5.0, RPM)
            low = min(span(tb, tb + 8.0, ALT))
            checks += [
                ('bailout: motor back above 40 % within 2 s', back is not None and back < 2.0,
                 'after %.2f s' % back if back is not None else 'never'),
                ('bailout: headspeed within 5 % of the target for 5 s',
                 max(abs(v - pilot.target) for v in rpm_b) < 0.05 * pilot.target,
                 '%.0f-%.0f rpm' % (min(rpm_b), max(rpm_b))),
                ('bailout: climbing within 5 s', max(span(tb + 2.0, tb + 5.0, CLIMB)) > 0.5,
                 '%.2f m/s' % max(span(tb + 2.0, tb + 5.0, CLIMB))),
                ('bailout: no ground contact', low > 1.0, 'lowest %.1f m' % low),
            ]
    else:
        landed = pilot.t_land is not None
        checks += [
            ('flare and landing: touched down', landed, 'at %.1f s' % pilot.t_land if landed else 'never'),
            ('soft landing: touchdown sink below 2 m/s', landed and heli.touchdown_speed < 2.0,
             '%.2f m/s' % heli.touchdown_speed),
            ('rotor speed at touchdown above 50 %', landed and heli.touchdown_rpm > 0.5 * pilot.target,
             '%.0f rpm' % heli.touchdown_rpm),
        ]
    checks.append(('lockstep: under 1 % of steps without reply', timeouts <= steps // 100,
                   '%d of %d' % (timeouts, steps)))
    ok = True
    for name, passed, detail in checks:
        print('%s %s (%s)' % ('PASS' if passed else 'FAIL', name, detail))
        ok &= passed
    return ok


def check_rescue(rows, timeouts, steps, pilot):
    """The rescue test (RescuePilot): fixed bounds after the rescue switch."""
    ROLL, PITCH, ALT, CLIMB = 3, 4, 1, 2
    if pilot.t_rescue is None:
        print('FAIL rescue: the heli never reached inverted flight')
        return False
    tr, hold = pilot.t_rescue, pilot.RESCUE_HOLD

    def span(t0, t1, col):
        return [r[col] for r in rows if tr + t0 <= r[0] <= tr + t1]

    def tilt(r):
        return max(abs(r[ROLL]), abs(r[PITCH]))
    up = next((r[0] - tr for r in rows if r[0] >= tr and tilt(r) < 30), None)
    alt0, roll0 = span(0, 0.1, ALT)[0], span(0, 0.1, ROLL)[0]
    level = max(tilt(r) for r in rows if tr + 3 <= r[0] <= tr + hold)
    climb = min(span(2.5, hold, CLIMB))
    low = min(span(0, hold, ALT))
    checks = [
        ('inverted at the rescue switch: |roll| above 120 deg', abs(roll0) > 120, '%.0f deg' % roll0),
        ('height for the test: above 8 m at the rescue switch', alt0 > 8.0, '%.1f m' % alt0),
        ('upright: tilt below 30 deg within 2 s', up is not None and up < 2.0,
         'after %.2f s' % up if up is not None else 'never'),
        ('level: tilt below 15 deg from 3 s to the end of rescue', level < 15, '%.1f deg' % level),
        ('climbing: vertical speed above -0.5 m/s from 2.5 s', climb > -0.5, '%.2f m/s' % climb),
        ('no ground contact: altitude above 2 m during rescue', low > 2.0, '%.1f m (from %.1f m)' % (low, alt0)),
        ('lockstep: under 1 % of steps without reply', timeouts <= steps // 100, '%d of %d' % (timeouts, steps)),
    ]
    ok = True
    for name, passed, detail in checks:
        print('%s %s (%s)' % ('PASS' if passed else 'FAIL', name, detail))
        ok &= passed
    return ok


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
    ap.add_argument('--cli-file', metavar='FILE',
                    help='extra CLI commands, one per line, after the dump and the bridge setup')
    ap.add_argument('--check', action='store_true',
                    help='judge the scripted angle-mode flight against fixed bounds; exit 1 on a failure')
    ap.add_argument('--acro', action='store_true', help='fly the stick doublets in acro (no angle mode)')
    ap.add_argument('--rescue', action='store_true',
                    help='rescue test: climb, roll inverted in acro, rescue on AUX3 (rescue_mode CLIMB); '
                         'with --check, judged against fixed bounds')
    ap.add_argument('--autorotation', choices=('land', 'bailout'),
                    help='autorotation test with the governor (needs --governor): climb, throttle stick to '
                         '15 %% (governor AUTOROTATION), hold the headspeed on the collective, then flare and '
                         'land, or bail out at 20 m (governor BAILOUT); with --check, judged against fixed bounds')
    ap.add_argument('--esc', choices=('none', 'hw4', 'blheli32'), default='none',
                    help='hw4: HobbyWing V4, blheli32: BLHeli32 (KISS) telemetry from the model on UART3 (TCP 5763). The '
                         'frequency sensor stays on: the ELECTRIC governor needs a fast rpm '
                         'source and does not take serial ESC telemetry')
    ap.add_argument('--governor', type=int, default=0, metavar='RPM',
                    help='fly with the ELECTRIC governor holding this headspeed')
    ap.add_argument('--model', default='blade360', metavar='PRESET|HELI|FILE',
                    help='a preset (%s), a trained heli (fitting/helis/<heli>/model.json: %s) or a JSON '
                         'parameter file (base preset, constructor args, class constants; see '
                         'heli_model.load_model)' % (', '.join(sorted(MODELS)), ', '.join(trained_models()) or '-'))
    ap.add_argument('--set', action='append', default=[], metavar='NAME=VALUE',
                    help='model constructor argument, e.g. Lb1s=1200 (repeatable)')
    ap.add_argument('--dump', metavar='CLI_DUMP',
                    help='apply a real CLI dump first (board and radio settings filtered, see configs.py)')
    ap.add_argument('--replay', metavar='STICKS_CSV',
                    help='fly recorded sticks (t,roll,pitch,yaw,coll,thr as rcCommand) instead of the script')
    ap.add_argument('--hold-alt', type=float, default=0.0, metavar='M',
                    help='with --replay: a slow virtual pilot (PID on height, ~0.3 Hz, at most +-150 us) '
                         'adds to the recorded collective and holds the model near this height - the model '
                         'lacks translational lift, so open-loop collective drifts in forward flight; '
                         'divided by the upward share of the rotor axis, so it also holds tilted and inverted')
    ap.add_argument('--gyro-chip', metavar='CHIP',
                    help='add gyro/acc chip noise from fitting/sensors/<CHIP>.json (Allan fit of a soak '
                         'at rest), or a path; available: %s' % ', '.join(sensors_available('sensors')))
    ap.add_argument('--vibration', metavar='HELI',
                    help='add rotor-locked gyro vibration from fitting/helis/<HELI>/vibration.json (order '
                         'tracking of logged gyroRAW), or a path; available: %s'
                         % ', '.join(sensors_available('helis')))
    ap.add_argument('--vib-scale', type=float, default=1.0, help='scale the vibration amplitudes')
    ap.add_argument('--sensor-seed', type=int, default=1)
    ap.add_argument('--wind', default='0,0', metavar='SPEED,FROM_DEG',
                    help='steady wind, m/s, blowing from FROM_DEG (0 north, 90 east); the heli faces north')
    ap.add_argument('--slope', default=None, metavar='DEG[,UPHILL_DEG]',
                    help='take off from and land on a slope rising towards UPHILL_DEG (0 north, the '
                         'heli faces north; default 90: across the skids). The heli is set down on it '
                         'at %.0f s, after the accelerometer calibration on level ground; with --check '
                         'the landing is judged too' % 4.0)
    ap.add_argument('--gust', type=float, default=0.0, metavar='SIGMA_W',
                    help='Dryden turbulence, vertical intensity m/s (MIL-F-8785C: 0.1 x the wind at 6 m: '
                         'light ~0.5, moderate ~1.5)')
    ap.add_argument('--servos', choices=sorted(SERVO_TYPES), default='ideal',
                    help='servo dynamics between the pulses and the swash (speed, deadband, lag; '
                         'heli/servos.py); ideal = none')
    ap.add_argument('--tail-servo', choices=sorted(SERVO_TYPES), default=None,
                    help='a servo type of its own for the tail servo (default: as --servos)')
    ap.add_argument('--servo-load', type=float, default=0.0, metavar='FRAC',
                    help='blade feathering load on the servos as a fraction of their stall torque at full '
                         'throw and nominal rpm (x rpm^2); needs --servos')
    ap.add_argument('--tail-servo-load', type=float, default=None, metavar='FRAC',
                    help='the same for the tail servo (default: --servo-load); needs --tail-servo or --servos')
    ap.add_argument('--profile-switch', metavar='T_ON[,T_OFF]',
                    help='switch the PID and rate profile from 1 to 2 at T_ON s (and back at T_OFF) '
                         'on AUX4, through adjustment functions; set up profile 2 with --cli')
    ap.add_argument('--debug-mode', metavar='NAME',
                    help='set the firmware\'s debug_mode (e.g. PROFILE_BLEND, GOVERNOR)')
    ap.add_argument('--debug-log', metavar='CSV',
                    help='log the firmware\'s debug values (MSP_DEBUG) every 20 ms to this file')
    ap.add_argument('--bbl', metavar='OUT',
                    help='save the SITL blackbox (file-backed flash) here after the flight')
    ap.add_argument('--replay-case', metavar='DIR',
                    help='a recorded flight with its reference (heli/replays/<case>: case.json, '
                         'sticks.csv, tune.txt): sets --model, --governor, --hold-alt, --replay and '
                         '--cli-file; with --check, judged against the real flight')
    a = ap.parse_args()
    case = None
    if a.replay_case:
        case = load_case(a.replay_case)
        a.model, a.governor, a.hold_alt = case['model'], case['governor'], case['hold_alt']
        a.replay = os.path.join(a.replay_case, 'sticks.csv')
        a.cli_file = os.path.join(a.replay_case, 'tune.txt')
        if case.get('dump'):
            a.dump = os.path.join(a.replay_case, case['dump'])
    elf = os.path.abspath(a.elf)
    rundir = tempfile.mkdtemp(prefix='heli-bridge-')

    # The firmware's stabilized mixer inputs follow Betaflight's sensor frame (X forward,
    # Y left, Z up): positive roll is right wing down, as in the model's FRD, but positive
    # pitch is nose down and positive yaw is nose left - the opposite of FRD q and r.
    kw = {}
    for item in a.set:
        k, v = item.split('=', 1)
        kw[k] = float(v)
    if resolve(a.model) is None:
        ap.error('--model: no preset, trained heli or file %s (presets: %s; trained: %s)'
                 % (a.model, ', '.join(sorted(MODELS)), ', '.join(trained_models()) or '-'))
    try:
        heli = load_model(a.model, sign_roll=1.0, sign_pitch=-1.0, sign_yaw=-1.0, **kw)
    except (ValueError, TypeError) as e:          # a bad parameter file or --set name
        ap.error('--model: %s' % e)
    sensors = Sensors(a.gyro_chip, a.vibration, seed=a.sensor_seed, vib_scale=a.vib_scale,
                      gear_ratio=heli.gear)
    if a.gyro_chip or a.vibration:
        print('sensors:', sensors.describe())
    if heli.tail_motor and (a.tail_servo or a.tail_servo_load):
        ap.error('--tail-servo / --tail-servo-load: %s has a motorized tail (its lag is the model tail_tau)'
                 % a.model)
    servos = Servos(a.servos, tail=a.tail_servo, load=a.servo_load, tail_load=a.tail_servo_load)
    switch = None
    if a.profile_switch:
        try:
            v = [float(x) for x in a.profile_switch.split(',')]
        except ValueError:
            v = []
        if not 1 <= len(v) <= 2 or (len(v) == 2 and v[1] <= v[0]):
            ap.error('--profile-switch: T_ON[,T_OFF] in s with T_OFF > T_ON, not %r' % a.profile_switch)
        switch = (v[0], v[1] if len(v) > 1 else float('inf'))
    ws, wd = (float(x) for x in a.wind.split(','))
    wind = Wind(ws, wd, a.gust)
    slope = None
    if a.slope:
        try:
            v = [float(x) for x in a.slope.split(',')]
        except ValueError:
            v = []
        if not 1 <= len(v) <= 2 or not 0.0 <= v[0] < 90.0:
            ap.error('--slope: DEG[,UPHILL_DEG] with 0 <= DEG < 90, not %r' % a.slope)
        slope = (v[0], v[1] if len(v) > 1 else 90.0)
        if a.seconds == ap.get_default('seconds'):
            a.seconds = 45.0      # disarm at 34 s, then the rotor runs down on the slope
        if a.replay or a.rescue or a.autorotation:
            ap.error('--slope goes with the scripted flight only')
    if ws or a.gust:
        print(wind.describe(), 'from %.0f deg' % wd)
    sticks = load_sticks(a.replay) if a.replay else None
    seconds = (REPLAY_LEAD + sticks[-1][0] - sticks[0][0] + REPLAY_TAIL) if sticks else a.seconds
    pilot = RescuePilot(hover_us(heli), bool(a.governor)) if a.rescue else None
    if pilot and a.seconds == ap.get_default('seconds'):
        seconds = 46.0
    if a.autorotation:
        if not a.governor:
            ap.error('--autorotation needs --governor RPM')
        pilot = AutoPilot(a.governor, bailout=a.autorotation == 'bailout')
        if a.seconds == ap.get_default('seconds'):
            seconds = 50.0

    # first start writes defaults; then configure over the CLI (save reboots)
    subprocess.run([elf], cwd=rundir, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
    proc, log = start_sitl(elf, rundir, 'config', False)
    wait_for(os.path.join(rundir, 'config.log'), 'bind port 5761', 15)
    commands = load_dump(a.dump) if a.dump else ['set motor_poles = %d' % heli.motor_poles]
    if heli.tail_motor and not a.dump:
        commands += ['set tail_rotor_mode = MOTORIZED']
    # the bridge's radio: MSP RX, channels AETRC123, ARM on AUX1, angle mode on AUX2
    commands += ['feature -RX_SERIAL', 'feature RX_MSP', 'map AETRC123',
                 'aux 0 0 0 1700 2100 0 0', 'aux 1 1 1 1700 2100 0 0']
    if switch:
        # PID and rate profile on AUX4 (adjustment channel 3), continuous adjustments:
        # 1000 us -> profile 1, 2000 us -> profile 2. Before --cli, which sets the profiles up
        commands += ['adjfunc 0 2 255 900 2100 3 1000 2000 900 900 0 1 2',
                     'adjfunc 1 1 255 900 2100 3 1000 2000 900 900 0 1 2']
    if a.debug_mode:
        commands += ['set debug_mode = %s' % a.debug_mode]
    if a.rescue:
        # 53: RESCUE's permanent id. Climb and hover collective per heli: the Blade 360 model
        # hovers at 0.44 (the defaults 450/350 are below it): 600 / 450
        hov = round(1000 * heli.coll_hover)
        commands += ['aux 2 53 2 1700 2100 0 0', 'set rescue_mode = CLIMB',
                     'set rescue_climb_collective = %d' % round(hov * 600 / 440),
                     'set rescue_hover_collective = %d' % round(hov * 450 / 440),
                     'set rescue_pull_up_time = 3', 'set rescue_climb_time = 10']
    if a.bbl:
        # each run starts on fresh flash: log from its start, no ring-buffer erasing
        commands += ['set blackbox_mode = ARMED', 'set blackbox_rolling_erase = OFF', 'set blackbox_initial_erase_kb = 0']
    if not a.dump and (a.governor or a.vibration):
        # the governor and the RPM filter take the rotor speed from the motor eRPM through the
        # gear ratios: pinion/main gear, and (servo tail) main gear/tail as rotor turns
        commands += ['set main_rotor_gear_ratio = 1000,%d' % round(heli.gear * 1000)]
        if sensors.tail_ratio:
            commands += ['set tail_rotor_gear_ratio = 1000,%d' % round(sensors.tail_ratio * 1000)]
    if a.vibration and not a.dump:
        commands += ['feature RPM_FILTER']
    if a.governor:
        commands += ['feature GOVERNOR', 'set gov_mode = ELECTRIC',
                     'set gov_headspeed = %d' % a.governor]
    if a.esc != 'none':
        commands += ['feature ESC_SENSOR', 'serial 2 1024 115200 57600 0 115200',
                     'set esc_sensor_protocol = %s' % {'hw4': 'HOBBYWINGV4', 'blheli32': 'BLHELI32'}[a.esc]]
    if a.cli_file:
        a.cli = [ln.strip() for ln in open(a.cli_file) if ln.strip() and not ln.startswith('#')] + a.cli
    t_cli = time.time()
    out_cli = cli(5761, commands + a.cli + ['save'])
    bad = cli.rejected
    print('CLI: %d commands in %.0f s%s' % (len(commands) + len(a.cli) + 1, time.time() - t_cli,
                                            (', %d rejected:' % len(bad)) if bad else ''))
    for ln in bad[:20]:
        print('   ', ln.strip())
    proc.wait(15); log.close()

    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    out.bind(('127.0.0.1', 9002)); out.settimeout(0.05)
    fdm = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    proc, log = start_sitl(elf, rundir, 'fly', True)
    rows = []
    timeouts, steps = 0, 1
    try:
        wait_for(os.path.join(rundir, 'fly.log'), 'bind port 5761', 15)
        msp = socket.create_connection(('127.0.0.1', 5761), timeout=3)
        # before the first FDM packet the firmware still runs on host time and answers
        swash = Swash.from_msp(lambda cmd: msp_request(msp, cmd))
        print(swash.describe())
        msp.setblocking(False)
        esc = None
        if a.esc != 'none':
            esc_sock = socket.create_connection(('127.0.0.1', 5763), timeout=3)
            esc_sock.setblocking(False)
            esc = {'hw4': Hw4Esc, 'blheli32': Blh32Esc}[a.esc](esc_sock)
        esc_every = int(0.02 / DT)
        msp_rx = b''
        at30 = (0.0, 0.0, 0.0)
        imu_probe = {int(float(x) / DT) for x in os.environ.get('IMU_AT', '').split(',') if x}
        imu_model = []
        # filtered gyro over MSP during the calm descent of the scripted flight (26.5-32 s),
        # every 20 ms: what is left of the vibration after the firmware's filters
        vib_probe = set()
        if (a.gyro_chip or a.vibration) and not sticks and not pilot:
            vib_probe = set(range(int(26.5 / DT), int(32.0 / DT), int(0.02 / DT)))
        imu_probe |= vib_probe
        imu_fw = []
        debug_t, debug_v = [], []      # MSP_DEBUG requests (sim time) and their replies
        wag = []                       # (t, yaw rate deg/s) at 1 kHz after a profile switch
        coll_sw = []                   # (t, collective mixer) around a profile switch
        decode_err, decode_at, decode_sq, decode_n = [0.0] * 4, [None] * 4, [0.0] * 4, 0
        time.sleep(1.0)
        msp.send(msp_frame(MSP_ACC_CALIBRATION))

        ctrl = (0.0, 0.0, 0.0, 0.0, 0.0)    # throttle, roll, pitch, yaw, collective
        last_ctrl, ctrl_since, frozen = None, 0.0, 0.0   # a stalled SITL keeps replying stale outputs
        hold_i = 0.0
        replay_resets = 0
        steps = int(seconds / DT)
        rc_every = int(0.02 / DT)
        timeouts = 0
        t0 = time.time()
        for n in range(steps):
            t = n * DT
            if n % rc_every == 0:
                aux3 = 1000
                if sticks:
                    thr, coll, arm, angle, r, p, y = replay_rc(sticks, t)
                    if a.hold_alt and t >= REPLAY_LEAD:
                        # us of collective per m/s^2 ~ 1 / (dn/dc * g * mixer per us); gains for
                        # ~2 rad/s, damped. The integral takes the steady thrust offset.
                        err = a.hold_alt - heli.altitude
                        hold_i = min(80.0, max(-80.0, hold_i + 2.0 * err * rc_every * DT))
                        corr = 27.0 * err - 19.0 * (-heli.vel[2]) + hold_i
                        # positive collective lifts along the rotor axis: a vertical force
                        # takes 1/up of it (up = the axis' upward share, -1 inverted), faded
                        # to 0 through knife edge where the rotor cannot lift
                        up = -body_to_earth(heli.q, (0.0, 0.0, -1.0))[2]
                        k = 1.0 / up if abs(up) >= 0.5 else up / 0.25
                        coll += max(-150.0, min(150.0, corr * k))
                elif pilot:
                    thr, coll, arm, angle, r, p, y, aux3 = pilot.rc(t, heli)
                else:
                    thr, coll, arm, angle, r, p, y = schedule(t, bool(a.governor), a.acro, hover_us(heli))
                ch = [int(round(v)) for v in (r, p, thr, y, coll, arm, angle, aux3)]
                if switch:
                    ch.append(2000 if switch[0] <= t < switch[1] else 1000)       # AUX4
                msp.send(msp_frame(MSP_SET_RAW_RC, struct.pack('<%dH' % len(ch), *ch)))
                if a.debug_log:
                    msp.send(msp_frame(MSP_DEBUG))      # replies come in order, parsed after the flight
                    debug_t.append(t)
                if n == int(30.0 / DT):
                    msp.send(msp_frame(MSP_MOTOR_TELEMETRY))
                    at30 = (heli.motor_rpm, heli.vbat, heli.ibat)
                if n in (int(5.0 / DT), int(7.0 / DT)):
                    msp.send(msp_frame(MSP_STATUS))
                if n in imu_probe:
                    msp.send(msp_frame(MSP_RAW_IMU))
                    imu_model.append((t, *(math.degrees(x) for x in heli.rates)))
                try:
                    msp_rx += msp.recv(4096)      # replies, parsed after the flight
                except (BlockingIOError, OSError):
                    pass

            if slope and n == int(SLOPE_AT / DT):
                heli.set_ground_slope(*slope)
                heli.place_on_ground()
            if ws or a.gust:
                rel = [heli.vel[i] - heli.wind_e[i] for i in range(3)]
                heli.wind_e = wind.step(DT, max(heli.altitude, 0.0), math.sqrt(sum(x * x for x in rel)))
            if sticks and heli.on_ground:
                # A replay starts mid-flight and flies open loop: the recorded cyclic belongs
                # to the air, and on the skids it would tip the heli over. On the ground the
                # model gets no cyclic - the pilot keeps it flat.
                heli.step(DT, ctrl[0], 0.0, 0.0, ctrl[3], ctrl[4])
                roll_, pitch_, yaw_ = euler(heli.q)
                if max(abs(roll_), abs(pitch_)) > math.radians(20):
                    # An open-loop replay drifts (the pilot corrected by eye) and now and then
                    # comes down: set it back upright on its skids and fly on - what the old
                    # ground model did on every touchdown. Counted, so a score can say so.
                    heli.q = (math.cos(yaw_ / 2), 0.0, 0.0, math.sin(yaw_ / 2))
                    heli.rates = [0.0, 0.0, 0.0]
                    heli.vel = [0.0, 0.0, 0.0]
                    heli.tpp = [0.0, 0.0]
                    heli.set_altitude(0.0)
                    replay_resets += 1
            else:
                heli.step(DT, *ctrl)
            if esc and n % esc_every == 0:
                esc.send(esc_every * DT, ctrl[0], heli.motor_rpm * heli.motor_poles / 2, heli.vbat, heli.ibat)
            pressure = 101325.0 * (1 - 2.25577e-5 * heli.altitude) ** 5.25588
            gyro, force = sensors.step(DT, heli.rates, heli.specific_force, heli.w)
            pkt = struct.pack('<28d', t, *gyro, *force, *heli.q,
                              *heli.vel, heli.pos[0], heli.pos[1], -heli.altitude,
                              heli.motor_rpm * heli.motor_poles / 2,
                              (heli.tail_rpm * 6 if heli.tail_motor else 0), 0, 0,
                              pressure, 3.3, 40.0, 8.0, 5.0, heli.vbat, heli.ibat)
            fdm.sendto(pkt, ('127.0.0.1', 9003))
            try:
                d, _ = out.recvfrom(256)
                if len(d) >= 68:
                    motor = struct.unpack_from('<4f', d, 0)
                    mix = struct.unpack_from('<5f', d, 48)
                    if a.controls == 'servos':
                        servo_us = servos.step(DT, struct.unpack_from('<8f', d, 16), (heli.w / heli.nominal_w) ** 2)
                        roll_, pitch_, yaw_, coll_ = swash.decode(servo_us)
                        if heli.tail_motor:
                            yaw_ = motor[1]       # motorized tail: the tail motor is the yaw input
                        if min(servo_us[:4]) > 0 and t >= 8.0:     # servos live, armed
                            decode_n += 1
                            for i, v in enumerate((roll_, pitch_, mix[2] if heli.tail_motor else yaw_, coll_)):
                                e = abs(v - mix[i])
                                decode_sq[i] += e * e
                                if e > decode_err[i]:
                                    decode_err[i], decode_at[i] = e, (t, v, mix[i])
                        ctrl = (motor[0], roll_, pitch_, yaw_, coll_)
                    else:
                        ctrl = (motor[0], mix[0], mix[1], motor[1] if heli.tail_motor else mix[2], mix[3])
                    heli.tail_throttle = motor[1]
            except socket.timeout:
                timeouts += 1
            if ctrl != last_ctrl or ctrl[0] <= 0.0:
                last_ctrl, ctrl_since = ctrl, t
            else:
                frozen = max(frozen, t - ctrl_since)

            if switch and switch[0] <= t < switch[0] + WAG_SPAN and n % int(0.001 / DT) == 0:
                wag.append((t, math.degrees(heli.rates[2])))
            if switch and abs(t - switch[0]) <= COLL_SPAN:
                coll_sw.append((t - switch[0], ctrl[4]))
            if n % int(float(os.environ.get('LOG_EVERY', '0.1')) / DT) == 0:
                roll, pitch, yaw = euler(heli.q)
                rows.append((t, heli.altitude, -heli.vel[2], math.degrees(roll), math.degrees(pitch),
                             math.degrees(yaw), heli.rotor_rpm, ctrl[0], ctrl[4], ctrl[1], ctrl[2],
                             heli.vbat, heli.ibat, *heli.rates, ctrl[3]))
        real = time.time() - t0
        print('%.0f s simulated in %.0f s real, %d steps without reply' % (seconds, real, timeouts))
        if sticks:
            print('replay: set upright after %d tilted touchdowns' % replay_resets)
        if frozen >= 5.0:
            print('WARNING: FC outputs unchanged for %.1f s with the motor running - the SITL stalled?' % frozen)
        i = -1
        while True:
            i = msp_rx.find(b'$M>', i + 1)
            if i < 0 or len(msp_rx) < i + 5:
                break
            size, cmd = msp_rx[i + 3], msp_rx[i + 4]
            if cmd == MSP_STATUS and len(msp_rx) >= i + 5 + size:
                flags = struct.unpack_from('<I', msp_rx, i + 5 + 17)[0]
                armed = struct.unpack_from('<I', msp_rx, i + 5 + 6)[0] & 1
                print('MSP_STATUS: armed %d, blockers %s' % (armed, [ARMING_FLAG_NAMES[b] if b < len(
                    ARMING_FLAG_NAMES) else str(b) for b in range(32) if flags >> b & 1]))
            if cmd == MSP_RAW_IMU and len(msp_rx) >= i + 5 + size and imu_model:
                # gyroRateDps() is in sensor counts despite its name: gyroADCf / scale, and the
                # SITL's fake gyro scale is 16.4 counts per deg/s
                gx, gy, gz = (v / GYRO_COUNTS_PER_DPS for v in struct.unpack_from('<3h', msp_rx, i + 5 + 6))
                tm, mp, mq, mr = imu_model.pop(0)
                imu_fw.append((tm, gx, gy, gz, mp, mq, mr))
                if int(round(tm / DT)) not in vib_probe:
                    print('MSP_RAW_IMU at %.1f s: firmware gyro %+.1f %+.1f %+.1f deg/s, model p q r %+.1f %+.1f %+.1f'
                          % (tm, gx, gy, gz, mp, mq, mr))
            if cmd == MSP_DEBUG and size == 32 and len(msp_rx) >= i + 5 + size:
                debug_v.append(struct.unpack_from('<8i', msp_rx, i + 5))
            if cmd == MSP_MOTOR_TELEMETRY and len(msp_rx) >= i + 5 + size:
                rpm, _err, volt, curr, _cons, temp, _t2 = struct.unpack_from('<IHHHHHH', msp_rx, i + 6)
                print('MSP_MOTOR_TELEMETRY at 30 s: motor rpm %d, %.2f V, %.2f A, %.1f C (model %.0f rpm, %.2f V, %.2f A)'
                      % (rpm, volt / 1000, curr / 1000, temp / 10, *at30))
        if a.controls == 'servos':
            print('inverse mixing vs mixer inputs, max |diff| roll %.4f pitch %.4f yaw %.4f coll %.4f'
                  % tuple(decode_err))
            print('  rms roll %.4f pitch %.4f yaw %.4f coll %.4f; worst (t, decoded, mixer): %s'
                  % (*(math.sqrt(q / max(decode_n, 1)) for q in decode_sq),
                     [tuple(round(x, 3) for x in w) if w else None for w in decode_at]))
    finally:
        proc.terminate()
        try:
            proc.wait(5)
        except subprocess.TimeoutExpired:
            proc.kill()
        log.close()

    if a.bbl:
        save_blackbox(os.path.join(rundir, 'blackbox_flash.bin'), a.bbl)
    if a.debug_log:
        # the n-th reply answers the n-th request; the last few may still be in flight
        with open(a.debug_log, 'w') as f:
            f.write('t,' + ','.join('debug%d' % i for i in range(8)) + '\n')
            for t, v in zip(debug_t, debug_v):
                f.write('%.2f,%s\n' % (t, ','.join(str(x) for x in v)))
        print('debug values (%s): %d of %d frames -> %s'
              % (a.debug_mode or 'debug_mode as configured', len(debug_v), len(debug_t), a.debug_log))
    hdr ='t,alt_m,climb_ms,roll_deg,pitch_deg,yaw_deg,rotor_rpm,throttle,mix_coll,mix_roll,mix_pitch,vbat,ibat,p,q,r,mix_yaw'
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
    if os.environ.get('KEEP'):
        print('run directory kept:', rundir)
    else:
        shutil.rmtree(rundir, ignore_errors=True)
    if isinstance(pilot, AutoPilot):
        ev = [('cut', pilot.t_cut), ('flare', pilot.t_flare), ('bailout', pilot.t_bailout), ('landed', pilot.t_land)]
        print('autorotation: ' + ', '.join('%s at %.2f s' % (k, v) for k, v in ev if v is not None))
        t0 = (pilot.t_cut or 0) - 1.0
        for r in rows:
            if t0 <= r[0] and int(round(r[0] * 10)) % 10 == 0:
                print('  t %5.1f  alt %6.2f m  climb %+6.2f  rpm %5.0f  thr %.2f  coll %+.3f'
                      % (r[0], r[1], r[2], r[6], r[7], r[8]))
    elif pilot:
        print('rescue: inverted at %s s, switch on at %s s' % (
            '%.2f' % pilot.t_inverted if pilot.t_inverted else '-',
            '%.2f' % pilot.t_rescue if pilot.t_rescue else '-'))
        for r in rows:
            if pilot.t_rescue and pilot.t_rescue - 0.5 <= r[0] <= pilot.t_rescue + 6.0 \
                    and int(round(r[0] * 10)) % 5 == 0:
                print('  t %5.1f  alt %6.2f m  climb %+5.2f  roll %+7.1f  pitch %+6.1f  coll %+.2f'
                      % (r[0], r[1], r[2], r[3], r[4], r[8]))
    vib_rms = None
    if vib_probe:
        fw = [r for r in imu_fw if int(round(r[0] / DT)) in vib_probe]
        vib_rms = []
        if fw:
            # firmware gyro (sensor frame: pitch and yaw opposite to FRD) against the model's
            # true rates; the RMS of the difference is the vibration and noise that got through
            err = [[r[1] - r[4], -r[2] - r[5], -r[3] - r[6]] for r in fw]
            vib_rms = [math.sqrt(sum(e[i] ** 2 for e in err) / len(err)) for i in range(3)]
            print('filtered gyro minus true rate, RMS 26.5-32 s (%d samples): roll %.2f pitch %.2f yaw %.2f deg/s'
                  % (len(err), *vib_rms))
            print('  model true rate RMS %.2f %.2f %.2f deg/s, firmware gyro RMS %.2f %.2f %.2f deg/s'
                  % (*(math.sqrt(sum(r[4 + i] ** 2 for r in fw) / len(fw)) for i in range(3)),
                     *(math.sqrt(sum(r[1 + i] ** 2 for r in fw) / len(fw)) for i in range(3))))
    if a.check:
        if isinstance(pilot, AutoPilot):
            sys.exit(0 if check_autorotation(rows, timeouts, steps, pilot, heli) else 1)
        if pilot:
            sys.exit(0 if check_rescue(rows, timeouts, steps, pilot) else 1)
        if sticks:
            if not case:
                sys.exit('--check with a replay needs --replay-case (the reference to judge against)')
            sys.exit(0 if check_replay(rows, timeouts, steps, replay_resets, case, heli.tail_motor) else 1)
        sys.exit(0 if check_flight(rows, timeouts, steps, a.governor, vib_rms,
                                   slope and (slope[0], heli), switch and (switch[0], wag, coll_sw)) else 1)


def check_replay(rows, timeouts, steps, resets, case, tail_motor=False):
    """A recorded flight replayed open loop (with the height-hold pilot): the model must
    keep flying it, and the trim it needs - throttle under the governor, tail pitch against
    the torque - must stay near the real flight's. Medians over the scored window, at the
    instants the sim flies above 1 m (the real ones from the log over its whole window).
    With the real pack voltage in the case ("vbat"), motor commands compare as motor volts:
    the sim's throttle (and a tail motor) scaled by its own pack voltage over the real one.
    A quantity with a known model gap can carry a sim baseline ("baseline"): it is judged
    against that, with the real value printed beside it, so the case still catches changes."""
    t0, t1 = case['window']
    span = [r for r in rows if t0 <= r[0] <= t1]
    up = [r for r in span if r[1] > 1.0]
    share = len(up) / max(1, len(span))

    real, tol = case['real'], case['tolerance']
    base = case.get('baseline', {})
    ref = dict(real, **base)
    VBAT = 11

    def against(k):
        return ('the sim baseline %.1f %% (real %.1f %%, a known gap)' % (base[k], real[k]) if k in base
                else 'the real %.1f %%' % real[k])

    def median(col, volts=False):
        k = (lambda r: r[VBAT] / real['vbat']) if volts and real.get('vbat') else (lambda r: 1.0)
        v = sorted(r[col] * k(r) for r in up)
        return 100 * v[len(v) // 2] if v else float('nan')

    thr, yaw = median(7, True), median(16, tail_motor)
    at = ' at %.1f V' % real['vbat'] if real.get('vbat') else ''
    checks = [
        ('airborne: above 1 m for %.0f %% of %g-%g s' % (100 * tol['airborne'], t0, t1),
         share >= tol['airborne'], '%.0f %%, lowest %.1f m' % (100 * share, min(r[1] for r in span))),
        ('upright resets after tilted touchdowns: at most %d' % tol['resets'], resets <= tol['resets'],
         '%d' % resets),
        ('throttle: median within %g points of %s%s' % (tol['throttle'], against('throttle'), at),
         abs(thr - ref['throttle']) <= tol['throttle'], '%.1f %%' % thr),
        ('tail trim: median yaw mix within %g points of %s%s'
         % (tol['yaw_mix'], against('yaw_mix'), at if tail_motor else ''),
         abs(yaw - ref['yaw_mix']) <= tol['yaw_mix'], '%.1f %%' % yaw),
        ('lockstep: under 1 % of steps without reply', timeouts <= steps // 100, '%d of %d' % (timeouts, steps)),
    ]
    ok = True
    for name, passed, detail in checks:
        print('%s %s (%s)' % ('PASS' if passed else 'FAIL', name, detail))
        ok &= passed
    return ok


VIB_RMS_MAX = 3.0     # deg/s: filtered gyro error allowed with --gyro-chip / --vibration


SLOPE_AT = 4.0        # s: the heli is set down on the slope after the level ACC calibration


WAG_SPAN = 6.0        # s after a profile switch: the yaw rate is sampled at 1 kHz
WAG_MAX = 20.0        # deg/s: oscillation above ~5 Hz allowed in the yaw rate after it
COLL_SPAN = 0.2       # s around a profile switch: the collective (stick held) must not step
COLL_STEP_MAX = 0.008  # collective change, median 0.05-0.2 s after less 0.05-0.2 s before (mixer)


def tail_wag(samples):
    """Largest yaw-rate oscillation above ~5 Hz (deg/s, the rate less its centred 0.1 s
    mean - which keeps the stick and governor responses out) and its frequency (Hz, from
    the zero crossings around the largest swing)."""
    if len(samples) < 200:
        return 0.0, 0.0
    t = [s[0] for s in samples]
    r = [s[1] for s in samples]
    h = 50
    acc = [0.0]
    for v in r:
        acc.append(acc[-1] + v)
    hp = [r[i] - (acc[i + h + 1] - acc[i - h]) / (2 * h + 1) for i in range(h, len(r) - h)]
    i_max = max(range(len(hp)), key=lambda i: abs(hp[i]))
    lo, hi = max(0, i_max - 250), min(len(hp) - 1, i_max + 250)
    zc = [i for i in range(lo, hi) if hp[i] * hp[i + 1] < 0]
    f = (len(zc) - 1) / 2 / (t[h + zc[-1]] - t[h + zc[0]]) if len(zc) > 2 else 0.0
    return abs(hp[i_max]), f


def check_flight(rows, timeouts, steps, governor, vib_rms=None, slope=None, switch=None):
    """The scripted angle-mode flight (schedule()): lift off at 18 s, roll stick 21-22 s,
    pitch stick 23-24 s, collective punch 24-25 s, descend from 26 s. Fixed bounds.
    switch: (t, yaw-rate samples, collective samples) of --profile-switch - the governor
    moves the headspeed to the new profile's, the tail must not wag (rotorflight-firmware#586)
    and, with the stick held, the collective must not step where the rate profiles' curves
    differ (gov_profile_blend_collective of #591)."""
    def at(t):
        return min(rows, key=lambda r: abs(r[0] - t))

    def span(t0, t1, col):
        return [r[col] for r in rows if t0 <= r[0] <= t1]

    ROLL, PITCH, ALT, RPM, R = 3, 4, 1, 6, 15
    checks = [
        ('lift-off: altitude above 1 m at 24 s', at(24)[ALT] > 1.0, '%.1f m' % at(24)[ALT]),
        ('roll stick: bank 8-30 deg', 8 <= max(span(21, 22.5, ROLL)) <= 30, '%.1f deg' % max(span(21, 22.5, ROLL))),
        ('roll stick released: back within 3 deg at 23 s', abs(at(23)[ROLL]) < 3, '%.1f deg' % at(23)[ROLL]),
        ('pitch stick: 8-30 deg', 8 <= max(abs(v) for v in span(23, 24.5, PITCH)) <= 30,
         '%.1f deg' % max(abs(v) for v in span(23, 24.5, PITCH))),
        ('pitch stick released: back within 3 deg at 25.5 s', abs(at(25.5)[PITCH]) < 3, '%.1f deg' % at(25.5)[PITCH]),
        ('no attitude excursion beyond 35 deg in flight',
         max(abs(v) for v in span(18, 33, ROLL) + span(18, 33, PITCH)) < 35,
         '%.1f deg' % max(abs(v) for v in span(18, 33, ROLL) + span(18, 33, PITCH))),
        ('yaw held: |r| below 1 rad/s', max(abs(v) for v in span(18, 33, R)) < 1.0,
         '%.2f rad/s' % max(abs(v) for v in span(18, 33, R))),
        ('lockstep: under 1 % of steps without reply', timeouts <= steps // 100, '%d of %d' % (timeouts, steps)),
    ]
    if switch:
        amp, f = tail_wag(switch[1])
        checks.append(('profile switch: no tail wag, yaw rate oscillation above 5 Hz below %.0f deg/s '
                       'for %.0f s after the switch at %.1f s' % (WAG_MAX, WAG_SPAN, switch[0]), amp < WAG_MAX,
                       '%.1f deg/s at %.1f Hz' % (amp, f)))
        def med(lo, hi):
            v = sorted(c for dt_, c in switch[2] if lo <= dt_ <= hi)
            return v[len(v) // 2] if v else 0.0
        step = abs(med(0.05, COLL_SPAN) - med(-COLL_SPAN, -0.05))
        checks.append(('profile switch: no collective step, below %.3f across the switch (stick held)'
                       % COLL_STEP_MAX, step < COLL_STEP_MAX, '%.4f' % step))
    elif governor:
        dev = max(abs(v - governor) for v in span(20, 33, RPM)) / governor
        checks.append(('governor: headspeed within 3 %% of %d rpm, 20-33 s' % governor, dev < 0.03,
                       '%.1f %%' % (100 * dev)))
    if slope:
        # landed back on the slope (the flight descends from 26 s, disarms at 34 s): at rest,
        # not rolled over, not sliding. The rotor still turns at the end (it runs down for
        # ~20 s) with the cyclic left at disarm, so it holds the heli flatter than the ground,
        # as a pilot's cyclic into the slope does; the soft skids add a few degrees the other way
        deg, heli = slope
        last = rows[-1]
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(math.radians(last[ROLL]))
                                                     * math.cos(math.radians(last[PITCH]))))))
        td = heli.touchdown_pos
        moved = math.hypot(heli.pos[0] - td[0], heli.pos[1] - td[1]) if td else float('inf')
        checks += [
            ('slope: landed, at rest on the ground at the end', heli.on_ground and abs(last[2]) < 0.05,
             'on ground %s, climb %.2f m/s' % (heli.on_ground, last[2])),
            ('slope: not rolled over, tilt at most %.0f deg on the %.0f deg ground' % (deg + 6, deg),
             tilt <= deg + 6, '%.1f deg' % tilt),
            ('slope: no slide after touchdown, below 0.5 m', moved < 0.5, '%.2f m' % moved),
        ]
    if vib_rms == []:
        # the gyro was to be sampled but no MSP_RAW_IMU reply came back: no evidence, a failure
        checks.append(('vibration filtered: filtered gyro sampled over MSP, 26.5-32 s', False, 'no samples'))
    elif vib_rms is not None:
        checks.append(('vibration filtered: gyro error RMS below %.0f deg/s per axis, 26.5-32 s' % VIB_RMS_MAX,
                       max(vib_rms) < VIB_RMS_MAX, '%.2f / %.2f / %.2f deg/s' % tuple(vib_rms)))
    ok = True
    for name, passed, detail in checks:
        print('%s %s (%s)' % ('PASS' if passed else 'FAIL', name, detail))
        ok &= passed
    return ok


if __name__ == '__main__':
    main()
