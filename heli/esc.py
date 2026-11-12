"""
ESC telemetry for the Rotorflight SITL, fed from the flight model: HobbyWing V4 and
BLHeli32 (KISS).

The SITL serves UART3 on TCP 5763; with `serial 2 1024 ...` (ESC_SENSOR) and
`esc_sensor_protocol = HOBBYWINGV4` its sensors/esc_sensor.c decodes what this sends:

  info frame (13 bytes): 9B 9B range[2] 01 Vnum Vden Inum Iden Ioff 00 00 B9
                         volts/ADC = Vnum / (Vden * 10), amps/ADC = Inum / Iden
  data frame (19 bytes): 9B counter[3] throttle[2] pwm[2] erpm[3] V I Tfet Tcap
                         (12-bit ADC values, big endian)

The firmware ignores voltage and current until it has seen an info frame after a few
syncs, so a few go out first. Frames are sent on simulated time by the caller.

BLHeli32 (`esc_sensor_protocol = BLHELI32`, Blh32Esc): the KISS telemetry frame, 10 bytes
  temp[1] (C) volts[2] (10 mV) amps[2] (10 mA) mAh[2] erpm/100[2] crc8 (poly 0x07)
On a real heli the firmware asks for each frame with the DShot telemetry bit and reads
the next 10 bytes; the SITL has no DShot, so frames stream at the caller's rate and the
firmware takes whole frames as they come (each send is one TCP write).

Only the Python standard library is used.
"""

import math

V_NUM, V_DEN = 2, 25          # 0.008 V per count: 32.7 V full scale
I_NUM, I_DEN = 1, 20          # 0.05 A per count: 204 A full scale
NTC_GAMMA, NTC_DELTA = 0.00025316455696, 0.00296226896087


def temp_adc(celsius):
    """Inverse of calcTempNTC() in esc_sensor.c."""
    a = 1.0 / (celsius + 273.15)
    r = math.exp((a - NTC_DELTA) / NTC_GAMMA)
    return max(1, min(4095, int(round(4096 * r / (1 + r)))))


def clamp12(x):
    return max(0, min(4095, int(round(x))))


class Hw4Esc:
    def __init__(self, sock):
        self.sock = sock
        self.count = 0
        self.infos = 0
        self.temp_fet = 30.0

    def info_frame(self):
        return bytes([0x9B, 0x9B, 0x03, 0xE8, 0x01, V_NUM, V_DEN, I_NUM, I_DEN, 0, 0, 0, 0xB9])

    def data_frame(self, throttle, erpm, volts, amps):
        thr = int(round(max(0.0, min(1.0, throttle)) * 1000))
        v = clamp12(volts / (V_NUM / (V_DEN * 10.0)))
        i = clamp12(amps / (I_NUM / I_DEN))
        t = temp_adc(self.temp_fet)
        self.count = (self.count + 1) & 0xFFFFFF
        return (bytes([0x9B]) + self.count.to_bytes(3, 'big') + thr.to_bytes(2, 'big')
                + thr.to_bytes(2, 'big') + int(max(0, erpm)).to_bytes(3, 'big')
                + v.to_bytes(2, 'big') + i.to_bytes(2, 'big') + t.to_bytes(2, 'big') + t.to_bytes(2, 'big'))

    def send(self, dt, throttle, erpm, volts, amps):
        """One telemetry period: info frames for the first few, data after."""
        # FET temperature: first order towards 30 C + 0.25 C/A, 60 s time constant
        self.temp_fet += (30.0 + 0.25 * amps - self.temp_fet) * dt / 60.0
        if self.infos < 6:
            self.infos += 1
            frame = self.info_frame()
        else:
            frame = self.data_frame(throttle, erpm, volts, amps)
        try:
            self.sock.send(frame)
        except (BlockingIOError, OSError):
            pass


def crc8_kiss(data):
    """crc8_kiss_update() of the firmware: CRC-8, polynomial 0x07, initial 0."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class Blh32Esc:
    """BLHeli32 / KISS telemetry: one 10-byte frame per send."""

    def __init__(self, sock):
        self.sock = sock
        self.temp = 30.0
        self.mah = 0.0

    def frame(self, erpm, volts, amps):
        u16 = lambda x: max(0, min(0xFFFF, int(round(x)))).to_bytes(2, 'big')
        body = (bytes([max(0, min(255, int(round(self.temp))))]) + u16(volts * 100) + u16(amps * 100)
                + u16(self.mah) + u16(erpm / 100))
        return body + bytes([crc8_kiss(body)])

    def send(self, dt, throttle, erpm, volts, amps):
        self.temp += (30.0 + 0.25 * amps - self.temp) * dt / 60.0
        self.mah += amps * dt / 3.6
        try:
            self.sock.send(self.frame(erpm, volts, amps))
        except (BlockingIOError, OSError):
            pass
