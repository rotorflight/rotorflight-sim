"""
Inverse swash mixing for the Rotorflight SITL: servo pulses (us) back to the
stabilized mixer inputs (roll, pitch, yaw, collective) that the flight model takes.

Reads the configuration over MSP once (MSP_SERVO_CONFIGURATIONS, MSP_MIXER_CONFIG,
MSP_MIXER_INPUTS) and undoes, in reverse order, what flight/servos.c servoUpdate()
and flight/mixer.c mixerUpdateSwash() do:

  servo us = mid + scale * (+-pos)          scale = rpos / rneg, sign = reverse flag
  pos      = swash matrix (SR, SP, SC) + tail servo SY + center trim
  SR,SP,SC = input * rate / 1000 + trims, SC with the collective tilt correction

Servo travel limits, servo speed limits and the cyclic ring act on the way out and are
not undone: the model sees what the servos really do. Rules from the rule-based mixer
are not handled (a warning is printed if any are set).

Only the Python standard library is used.
"""

import math
import struct

MSP_MIXER_CONFIG = 42
MSP_SERVO_CONFIGURATIONS = 120
MSP_MIXER_INPUTS = 170
MSP_MIXER_RULES = 171

SWASH_NONE, SWASH_THRU, SWASH_120, SWASH_135, SWASH_140, SWASH_90L, SWASH_90V = range(7)
SERVO_FLAG_REVERSED = 1
TAIL_MODE_VARIABLE = 0

# servo outputs per (SR, SP, SC), as in mixerUpdateSwash()
SWASH_MATRIX = {
    SWASH_THRU: [(0, 1, 0), (1, 0, 0), (0, 0, 1)],
    SWASH_120:  [(0, -1, 0.5), (0.86602540, 0.5, 0.5), (-0.86602540, 0.5, 0.5)],
    SWASH_135:  [(0, -1, 0.5), (1, 1, 0.5), (-1, 1, 0.5)],
    SWASH_140:  [(0, -1, 0.5), (0.866025, 1, 0.5), (-0.866025, 1, 0.5)],
    SWASH_90L:  [(0, 1, 0), (1, 0, 0)],
    SWASH_90V:  [(0.70710678, 0.70710678, 0), (-0.70710678, 0.70710678, 0)],
}


def solve(a, b):
    """Least squares a x = b for a small dense system (normal equations, Gauss)."""
    n = len(a[0])
    ata = [[sum(a[k][i] * a[k][j] for k in range(len(a))) for j in range(n)] for i in range(n)]
    atb = [sum(a[k][i] * b[k] for k in range(len(a))) for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(ata[r][c]))
        ata[c], ata[p] = ata[p], ata[c]
        atb[c], atb[p] = atb[p], atb[c]
        for r in range(n):
            if r != c and ata[c][c]:
                f = ata[r][c] / ata[c][c]
                ata[r] = [x - f * y for x, y in zip(ata[r], ata[c])]
                atb[r] -= f * atb[c]
    return [atb[i] / ata[i][i] if ata[i][i] else 0.0 for i in range(n)]


class Swash:
    def __init__(self, servos, mixer_config, input_rates, rules_active=0):
        self.servos = servos                      # list of dicts: mid, rneg, rpos, flags
        (self.main_dir, self.tail_mode, _idle, tail_trim, self.swash_type, _ring, _phase,
         _limit, t0, t1, t2, _tta, _geo, self.tilt_pos, self.tilt_neg) = mixer_config
        self.tail_trim = tail_trim / 1000.0
        self.trim = (t0 / 1000.0, t1 / 1000.0, t2 / 1000.0)
        self.rates = input_rates                  # roll, pitch, yaw, collective: rate / 1000
        self.rules_active = rules_active
        if self.swash_type not in SWASH_MATRIX:
            raise ValueError('swash type %d not supported' % self.swash_type)
        self.matrix = SWASH_MATRIX[self.swash_type]
        # 90L/90V have no collective servo: two unknowns only
        self.cols = 3 if self.swash_type in (SWASH_THRU, SWASH_120, SWASH_135, SWASH_140) else 2

    @classmethod
    def from_msp(cls, request):
        """request(cmd) -> payload bytes (blocking MSP v1 request)."""
        d = request(MSP_SERVO_CONFIGURATIONS)
        servos = []
        for i in range(d[0]):
            mid, mn, mx, rneg, rpos, rate, speed, flags = struct.unpack_from('<8H', d, 1 + 16 * i)
            servos.append(dict(mid=mid, min=struct.unpack('<h', struct.pack('<H', mn))[0], max=mx,
                               rneg=rneg, rpos=rpos, speed=speed, flags=flags))
        mc = struct.unpack_from('<BBBhBBhHhhhBBbb', request(MSP_MIXER_CONFIG), 0)
        mi = request(MSP_MIXER_INPUTS)
        rates = [struct.unpack_from('<h', mi, 6 * idx)[0] / 1000.0 for idx in (1, 2, 3, 4)]
        mr = request(MSP_MIXER_RULES)
        rules = sum(1 for i in range(0, len(mr), 7) if mr[i])   # 7-byte rules, oper != 0
        return cls(servos, mc, rates, rules)

    def describe(self):
        names = {1: 'THRU', 2: '120', 3: '135', 4: '140', 5: '90L', 6: '90V'}
        return ('swash %s, %d servos, input rates %s, trims %s, tail %s%s'
                % (names[self.swash_type], len(self.servos), self.rates, self.trim,
                   'variable pitch' if self.tail_mode == TAIL_MODE_VARIABLE else 'motorized',
                   ', %d mixer rules NOT inverted' % self.rules_active if self.rules_active else ''))

    def servo_pos(self, i, us):
        s = self.servos[i]
        d = us - s['mid']
        scale = s['rpos'] if d > 0 else s['rneg']
        pos = d / scale if scale else 0.0
        if s['flags'] & SERVO_FLAG_REVERSED:
            pos = -pos
        return pos

    def decode(self, servo_us):
        """servo_us: the output packet's servo[] -> (roll, pitch, yaw, collective) mixer inputs."""
        n = len(self.matrix)
        pos = [self.servo_pos(i, servo_us[i]) for i in range(n)]
        a = [row[:self.cols] for row in self.matrix]
        x = solve(a, pos)
        sr, sp = x[0] - self.trim[0], x[1] - self.trim[1]
        sc = (x[2] - self.trim[2]) if self.cols == 3 else 0.0
        # undo mixerCollectiveScale(): SC_out = SC * (1 + beta * (SR^2 + SP^2)); same sign as SC
        beta = self.tilt_pos / 100.0 if sc > 0 else -self.tilt_neg / 100.0
        scale = min(2.0, max(0.0, 1 + beta * (sr * sr + sp * sp)))
        sc = sc / scale if scale > 0 else 0.0
        sy = 0.0
        if self.tail_mode == TAIL_MODE_VARIABLE and len(self.servos) > 3:
            sy = self.servo_pos(3, servo_us[3]) - self.tail_trim
        r = self.rates
        return (sr / r[0] if r[0] else 0.0, sp / r[1] if r[1] else 0.0,
                sy / r[2] if r[2] else 0.0, sc / r[3] if r[3] else 0.0)
