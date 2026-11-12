"""
Servo dynamics between the firmware's servo pulses and the swash decode: what the blades
actually get. Per servo, in pulse microseconds:

  deadband   the servo ignores command changes smaller than this (its own deadband)
  lag        first-order response of the servo's position loop (time constant)
  speed      slew-rate limit, from the datasheet's s/60 deg at ~10 us per degree
             (a 1500 +- 500 us servo over +-50 deg)

Types (`--servos`): ideal (no dynamics, the default), digital (a typical digital cyclic
servo, 0.06 s/60 deg, 1 us deadband, 10 ms), fast (a high-voltage brushless, 0.04 s/60
deg, 1 us, 6 ms), analog (0.12 s/60 deg, 4 us, 20 ms). `--tail-servo` gives the tail
servo a type of its own.

Load (`--servo-load`, `--tail-servo-load`): the blades' feathering moment pulls them back
to zero pitch and grows with rpm^2 and the deflection. As a fraction L of the servo's
stall torque (`load` at full deflection, +-500 us, and nominal rpm: L = load (w/w_n)^2
|deflection| / 500 us), a servo's speed falls linearly with the torque against it (a DC
motor): (1 - L) of its rated speed moving away from centre, up to (1 + L) moving back. At
L > 1 it cannot hold and is pushed back towards centre. Off (0) by default.

Only the Python standard library.
"""

import math

US_PER_DEG = 10.0

# type: (seconds per 60 deg, deadband us, time constant s)
TYPES = {
    'ideal': None,
    'digital': (0.06, 1.0, 0.010),
    'fast': (0.04, 1.0, 0.006),
    'analog': (0.12, 4.0, 0.020),
}


CENTER_US, HALF_RANGE_US = 1500.0, 500.0


class Servo:
    def __init__(self, sec_per_60, deadband_us, tau, load=0.0):
        self.rate = 60.0 * US_PER_DEG / sec_per_60      # us/s
        self.db = deadband_us
        self.tau = tau
        self.load = load                                 # stall-torque fraction at full throw
        self.held = None                                 # command the servo acts on
        self.lagged = None
        self.pos = None

    def step(self, dt, cmd, rr=1.0):
        """cmd: pulse, us; rr: (rotor speed / nominal)^2, which the load scales with."""
        if self.pos is None:
            self.held = self.lagged = self.pos = cmd
            return cmd
        if abs(cmd - self.held) > self.db:
            self.held = cmd
        self.lagged += (self.held - self.lagged) * min(1.0, dt / self.tau)
        d = self.lagged - self.pos
        lim = self.rate * dt
        off = self.pos - CENTER_US
        L = self.load * rr * abs(off) / HALF_RANGE_US if self.load > 0 else 0.0
        if L > 0:
            if d * off > 0:                      # moving away from centre, against the load
                lim *= max(0.0, 1.0 - L)
            else:                                # back towards it, the load helps
                lim *= min(2.0, 1.0 + L)
        self.pos += max(-lim, min(lim, d))
        if L > 1.0:                              # stalled: the blades push it back
            back = min(abs(self.pos - CENTER_US), self.rate * dt * min(1.0, L - 1.0))
            self.pos -= math.copysign(back, self.pos - CENTER_US)
        return self.pos


TAIL_SERVO = 3      # the yaw servo's output on a variable-pitch tail (swash.decode)


class Servos:
    """All servo outputs of the SITL packet; pulses of 0 (unused outputs) pass through.
    tail: a type of its own for the tail servo (output TAIL_SERVO), else it is `kind` too -
    tail servos are often slower and carry more load than the cyclic ones."""

    def __init__(self, kind='ideal', count=8, tail=None, load=0.0, tail_load=None):
        for k in (kind, tail):
            if k is not None and k not in TYPES:
                raise SystemExit('servo type %r: one of %s' % (k, ', '.join(TYPES)))
        specs = [TYPES[kind]] * count
        if tail is not None and count > TAIL_SERVO:
            specs[TAIL_SERVO] = TYPES[tail]
        loads = [load] * count
        if tail_load is not None and count > TAIL_SERVO:
            loads[TAIL_SERVO] = tail_load
        if any(loads[i] > 0 and specs[i] is None for i in range(count)):
            raise SystemExit('servo load needs a servo type (--servos / --tail-servo): an ideal servo has no motor')
        self.servos = [Servo(*s, load=ld) if s else None for s, ld in zip(specs, loads)]
        if not any(self.servos):
            self.servos = None

    def step(self, dt, pulses, rr=1.0):
        if not self.servos:
            return pulses
        return tuple(s.step(dt, p, rr) if s and p > 0 else p for s, p in zip(self.servos, pulses))
