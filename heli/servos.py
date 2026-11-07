"""
Servo dynamics between the firmware's servo pulses and the swash decode: what the blades
actually get. Per servo, in pulse microseconds:

  deadband   the servo ignores command changes smaller than this (its own deadband)
  lag        first-order response of the servo's position loop (time constant)
  speed      slew-rate limit, from the datasheet's s/60 deg at ~10 us per degree
             (a 1500 +- 500 us servo over +-50 deg)

Types (`--servos`): ideal (no dynamics, the default), digital (a typical digital cyclic
servo, 0.06 s/60 deg, 1 us deadband, 10 ms), fast (a high-voltage brushless, 0.04 s/60
deg, 1 us, 6 ms), analog (0.12 s/60 deg, 4 us, 20 ms). Load (stall torque against the
blade's feathering moment) is not modelled.

Only the Python standard library.
"""

US_PER_DEG = 10.0

# type: (seconds per 60 deg, deadband us, time constant s)
TYPES = {
    'ideal': None,
    'digital': (0.06, 1.0, 0.010),
    'fast': (0.04, 1.0, 0.006),
    'analog': (0.12, 4.0, 0.020),
}


class Servo:
    def __init__(self, sec_per_60, deadband_us, tau):
        self.rate = 60.0 * US_PER_DEG / sec_per_60      # us/s
        self.db = deadband_us
        self.tau = tau
        self.held = None                                 # command the servo acts on
        self.lagged = None
        self.pos = None

    def step(self, dt, cmd):
        if self.pos is None:
            self.held = self.lagged = self.pos = cmd
            return cmd
        if abs(cmd - self.held) > self.db:
            self.held = cmd
        self.lagged += (self.held - self.lagged) * min(1.0, dt / self.tau)
        d = self.lagged - self.pos
        lim = self.rate * dt
        self.pos += max(-lim, min(lim, d))
        return self.pos


class Servos:
    """All servo outputs of the SITL packet; pulses of 0 (unused outputs) pass through."""

    def __init__(self, kind='ideal', count=8):
        if kind not in TYPES:
            raise SystemExit('servo type %r: one of %s' % (kind, ', '.join(TYPES)))
        spec = TYPES[kind]
        self.servos = [Servo(*spec) for _ in range(count)] if spec else None

    def step(self, dt, pulses):
        if not self.servos:
            return pulses
        return tuple(s.step(dt, p) if p > 0 else p for s, p in zip(self.servos, pulses))
