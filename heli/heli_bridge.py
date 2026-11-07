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
from heli_model import MODELS, euler, body_to_earth  # noqa: E402
from configs import load_dump         # noqa: E402
import csv                            # noqa: E402
from swash import Swash               # noqa: E402
from esc import Hw4Esc                # noqa: E402
from sensors import Sensors, available as sensors_available   # noqa: E402

DT = 250e-6
MSP_SET_RAW_RC = 200
MSP_ACC_CALIBRATION = 205
MSP_MOTOR_TELEMETRY = 139
MSP_STATUS = 101
MSP_RAW_IMU = 102
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


def schedule(t, governor=False, acro=False):
    """Scripted pilot: returns (throttle, collective, aux1 arm, aux2 angle, roll, pitch, yaw) in us.
    Without the governor the throttle stick ramps to 65 % (the model's hover throttle at
    nominal headspeed); with it the stick goes to 100 % and the governor spools up."""
    arm = 1900 if 6.0 <= t < 34.0 else 1000
    angle = 1000 if acro else 1900
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

    def __init__(self):
        self.t_inverted = None    # sticks released, heli inverted
        self.t_rescue = None      # rescue switch on
        self.t_end = None         # rescue off, angle mode, descend

    def rc(self, t, heli):
        arm = 1900 if t >= 6.0 and (self.t_end is None or t < self.t_end + 8.0) else 1000
        thr = 1000 + min(1.0, max(0.0, (t - 8.0) / 4.0)) * 650 if t >= 8.0 else 1000
        coll, angle, roll, aux3 = 1500 - 150, 1900, 1500, 1000
        if 18.0 <= t < self.CLIMB_END:
            coll = 1500 + 260                                      # climb to height
        elif t >= self.CLIMB_END and self.t_inverted is None:
            angle = 1000                                           # acro
            bank = abs(math.degrees(euler(heli.q)[0]))
            coll = 1500 + (215 if bank < 90 else -215)            # hover collective, either way up
            if bank < 150:
                roll = 1500 + 500                                  # full roll until inverted
            else:
                self.t_inverted = t
        if self.t_inverted is not None:
            coll, angle = 1500 - 215, 1000                         # inverted hover, sticks centred
            if self.t_rescue is None and t >= self.t_inverted + 0.5:
                self.t_rescue = t
            if self.t_rescue is not None and t < self.t_rescue + self.RESCUE_HOLD:
                # rescue on, the pilot lets go: collective centred (rescue adds 1/4 of it)
                coll, aux3 = 1500, 1900
            elif self.t_rescue is not None:
                if self.t_end is None:
                    self.t_end = t
                coll, angle = 1500 + 195, 1900                     # angle mode, descend
        return thr, coll, arm, angle, roll, 1500, 1500, aux3


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
    ap.add_argument('--esc', choices=('none', 'hw4'), default='none',
                    help='hw4: HobbyWing V4 telemetry from the model on UART3 (TCP 5763). The '
                         'frequency sensor stays on: the ELECTRIC governor needs a fast rpm '
                         'source and does not take serial ESC telemetry')
    ap.add_argument('--governor', type=int, default=0, metavar='RPM',
                    help='fly with the ELECTRIC governor holding this headspeed')
    ap.add_argument('--model', choices=sorted(MODELS), default='blade360')
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
    ap.add_argument('--bbl', metavar='OUT',
                    help='save the SITL blackbox (file-backed flash) here after the flight')
    a = ap.parse_args()
    elf = os.path.abspath(a.elf)
    rundir = tempfile.mkdtemp(prefix='heli-bridge-')

    # The firmware's stabilized mixer inputs follow Betaflight's sensor frame (X forward,
    # Y left, Z up): positive roll is right wing down, as in the model's FRD, but positive
    # pitch is nose down and positive yaw is nose left - the opposite of FRD q and r.
    kw = {}
    for item in a.set:
        k, v = item.split('=', 1)
        kw[k] = float(v)
    heli = MODELS[a.model](sign_roll=1.0, sign_pitch=-1.0, sign_yaw=-1.0, **kw)
    sensors = Sensors(a.gyro_chip, a.vibration, seed=a.sensor_seed, vib_scale=a.vib_scale,
                      gear_ratio=heli.gear)
    if a.gyro_chip or a.vibration:
        print('sensors:', sensors.describe())
    sticks = load_sticks(a.replay) if a.replay else None
    seconds = (REPLAY_LEAD + sticks[-1][0] - sticks[0][0] + REPLAY_TAIL) if sticks else a.seconds
    pilot = RescuePilot() if a.rescue else None
    if pilot and a.seconds == ap.get_default('seconds'):
        seconds = 46.0

    # first start writes defaults; then configure over the CLI (save reboots)
    subprocess.run([elf], cwd=rundir, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
    proc, log = start_sitl(elf, rundir, 'config', False)
    wait_for(os.path.join(rundir, 'config.log'), 'bind port 5761', 15)
    commands = load_dump(a.dump) if a.dump else ['set motor_poles = %d' % heli.motor_poles]
    # the bridge's radio: MSP RX, channels AETRC123, ARM on AUX1, angle mode on AUX2
    commands += ['feature -RX_SERIAL', 'feature RX_MSP', 'map AETRC123',
                 'aux 0 0 0 1700 2100 0 0', 'aux 1 1 1 1700 2100 0 0']
    if a.rescue:
        # 53: RESCUE's permanent id. Climb and hover collective per heli: the Blade 360 model
        # hovers at 0.44 (the defaults 450/350 are below it)
        commands += ['aux 2 53 2 1700 2100 0 0', 'set rescue_mode = CLIMB',
                     'set rescue_climb_collective = 600', 'set rescue_hover_collective = 450']
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
    if a.esc == 'hw4':
        commands += ['feature ESC_SENSOR', 'serial 2 1024 115200 57600 0 115200',
                     'set esc_sensor_protocol = HOBBYWINGV4']
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
        if a.esc == 'hw4':
            esc_sock = socket.create_connection(('127.0.0.1', 5763), timeout=3)
            esc_sock.setblocking(False)
            esc = Hw4Esc(esc_sock)
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
        decode_err, decode_at, decode_sq, decode_n = [0.0] * 4, [None] * 4, [0.0] * 4, 0
        time.sleep(1.0)
        msp.send(msp_frame(MSP_ACC_CALIBRATION))

        ctrl = (0.0, 0.0, 0.0, 0.0, 0.0)    # throttle, roll, pitch, yaw, collective
        last_ctrl, ctrl_since, frozen = None, 0.0, 0.0   # a stalled SITL keeps replying stale outputs
        hold_i = 0.0
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
                        err = a.hold_alt - (-heli.pos[2])
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
                    thr, coll, arm, angle, r, p, y = schedule(t, bool(a.governor), a.acro)
                ch = [int(round(v)) for v in (r, p, thr, y, coll, arm, angle, aux3)]
                msp.send(msp_frame(MSP_SET_RAW_RC, struct.pack('<8H', *ch)))
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

            heli.step(DT, *ctrl)
            if (sticks and heli.ground and heli.w > 0.95 * heli.nominal_w
                    and ctrl[4] > 0.9 * heli.coll_hover):
                # The replayed collective is open loop: the pilot held height on the real
                # heli, here thrust only roughly matches the weight. Release the heli from the
                # ground once it is light on its skids; heave damping keeps the drift small.
                heli.ground = False
                heli.on_ground = False
            if esc and n % esc_every == 0:
                esc.send(esc_every * DT, ctrl[0], heli.motor_rpm * heli.motor_poles / 2, heli.vbat, heli.ibat)
            pressure = 101325.0 * (1 - 2.25577e-5 * (-heli.pos[2])) ** 5.25588
            gyro, force = sensors.step(DT, heli.rates, heli.specific_force, heli.w)
            pkt = struct.pack('<28d', t, *gyro, *force, *heli.q,
                              *heli.vel, *heli.pos,
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
                        servo_us = struct.unpack_from('<8f', d, 16)
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

            if n % int(float(os.environ.get('LOG_EVERY', '0.1')) / DT) == 0:
                roll, pitch, yaw = euler(heli.q)
                rows.append((t, -heli.pos[2], -heli.vel[2], math.degrees(roll), math.degrees(pitch),
                             math.degrees(yaw), heli.rotor_rpm, ctrl[0], ctrl[4], ctrl[1], ctrl[2],
                             heli.vbat, heli.ibat, *heli.rates))
        real = time.time() - t0
        print('%.0f s simulated in %.0f s real, %d steps without reply' % (seconds, real, timeouts))
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
    if os.environ.get('KEEP'):
        print('run directory kept:', rundir)
    else:
        shutil.rmtree(rundir, ignore_errors=True)
    if pilot:
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
        if pilot:
            sys.exit(0 if check_rescue(rows, timeouts, steps, pilot) else 1)
        sys.exit(0 if check_flight(rows, timeouts, steps, a.governor, vib_rms) else 1)


VIB_RMS_MAX = 3.0     # deg/s: filtered gyro error allowed with --gyro-chip / --vibration


def check_flight(rows, timeouts, steps, governor, vib_rms=None):
    """The scripted angle-mode flight (schedule()): lift off at 18 s, roll stick 21-22 s,
    pitch stick 23-24 s, collective punch 24-25 s, descend from 26 s. Fixed bounds."""
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
    if governor:
        dev = max(abs(v - governor) for v in span(20, 33, RPM)) / governor
        checks.append(('governor: headspeed within 3 %% of %d rpm, 20-33 s' % governor, dev < 0.03,
                       '%.1f %%' % (100 * dev)))
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
