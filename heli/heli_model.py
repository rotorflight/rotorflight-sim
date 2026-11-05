"""
Minimal RC helicopter flight model for the Rotorflight SITL (bridge v0).

Rigid body in NED (earth) / FRD (body), quaternion attitude. Rotor flapping and
body derivatives from ArduPilot's SITL Blade 360 model (libraries/SITL/
SIM_Helicopter.cpp, GPLv3), which took them from Walker & Tischler,
"Identification and Control Design of a Sub-Scale Flybarless Helicopter",
VFS 77th Forum, 2021. Rotor speed is a DC motor (torque falling with speed)
against rotor inertia and a collective-dependent drag, so the firmware's
throttle or governor sets the headspeed.

Inputs (normalised, as the SITL's 68-byte output packet carries them):
  throttle    ESC output, 0..1 (motor_speed[0])
  roll, pitch cyclic, about -1..1 (stabilized mixer inputs)
  yaw         tail, about -1..1
  collective  about -1..1
Outputs: the FDM state (body rates, specific force, attitude, velocity,
position) plus rotor/motor rpm and the pack.

Only the Python standard library is used.
"""

import math

G = 9.80665


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def qnorm(q):
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def body_to_earth(q, v):
    """Rotate a body (FRD) vector into earth (NED) with the body->earth quaternion q."""
    w, x, y, z = q
    vx, vy, vz = v
    # v' = q v q*
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def earth_to_body(q, v):
    w, x, y, z = q
    return body_to_earth((w, -x, -y, -z), v)


def euler(q):
    """roll, pitch, yaw (rad) of a body->NED quaternion, ZYX."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class Heli:
    # Blade 360 derivatives (per unit of the model's control inputs)
    Ma1s, Lb1s = 796.7, 5115.2
    Mu, Mv, Lu, Lv = 2.7501, -2.3039, -28.7796, -5.5376
    Xu, Yv, Yp, Zw = -0.2270, -0.1852, 0.2303, -0.5910
    Nr, Nw, Nv = -2.0131, 5.7574, 1.7258
    Ncol, Nped, Zcol = -32.4616, 63.0040, -22.3239
    # rotor flapping (first order); tf_inv_roll/pitch default to tf_inv
    tf_inv = 1.0 / 0.0353
    tf_inv_roll = tf_inv_pitch = None
    # optional fitted rotor law (A, B, k0, k1, n): dw/dt = A u V - B w - w^2 (k0 + k1 |c|^n)
    motor_fit = None
    # main rotor torque reaction on yaw, rad/s^2 at hover (motorized tails); scales with drag
    torque_yaw = 0.0
    motor_poles = 2
    tail_motor = False
    tail_tau = 0.035
    yaw_lag = 0.0
    Lfa1s, Mfb1s = 1.0477, -1.0057
    Lflt, Lflg, Mflt, Mflg = 0.2375, -0.0286, 0.0344, 0.2292

    def __init__(self, nominal_rpm=2100.0, gear_ratio=10.0, hover_throttle=0.65, load_fraction=0.85,
                 pack_cells=6, pack_ah=2.2, pack_r=0.02, hover_power_w=400.0,
                 cyclic_scale=0.5, yaw_scale=1.45, coll_scale=1.0, coll_offset=0.0,
                 sign_roll=1.0, sign_pitch=1.0, sign_yaw=1.0):
        self.nominal_w = nominal_rpm * 2 * math.pi / 60   # rotor, rad/s
        self.gear = gear_ratio
        self.cyclic_scale, self.yaw_scale = cyclic_scale, yaw_scale
        self.coll_scale, self.coll_offset = coll_scale, coll_offset
        self.sign_roll, self.sign_pitch, self.sign_yaw = sign_roll, sign_pitch, sign_yaw
        # collective (model units) at which thrust carries the weight at nominal rpm
        self.coll_hover = G / -self.Zcol
        # rotor drag torque per inertia: k_d * w^2 * (c0 + |coll|^1.5); motor torque
        # per inertia: A * u - B * w (DC motor). In hover the motor turns at
        # load_fraction of its no-load speed for the hover throttle; near 1 it would be a
        # stiff speed source that no governor needs to work for.
        drag_shape = lambda c: 0.25 + abs(c) ** 1.5
        self.drag_shape = drag_shape
        spool_accel = self.nominal_w / 2.5                 # ~2.5 s spool-up at full torque
        self.A = spool_accel / 0.6
        self.B = self.A * hover_throttle * load_fraction / self.nominal_w
        # equilibrium at hover: A*u_h - B*w_n = k_d * w_n^2 * shape(coll_hover)
        margin = self.A * hover_throttle - self.B * self.nominal_w
        self.k_d = margin / (self.nominal_w ** 2 * drag_shape(self.coll_hover))
        # pack; electrical power scales with shaft power, anchored at the hover point
        self.cells, self.pack_ah, self.pack_r = pack_cells, pack_ah, pack_r
        self.shaft_hover = self.k_d * self.nominal_w ** 3 * drag_shape(self.coll_hover)
        self.hover_power = hover_power_w
        self.used_ah = 0.0
        if self.motor_fit:
            A, B, k0, k1, n = self.motor_fit
            self.drag_shape = lambda c: k0 + k1 * abs(c) ** n
            self.shaft_hover = self.nominal_w ** 3 * self.drag_shape(self.coll_hover)
        if self.tf_inv_roll is None:
            self.tf_inv_roll = self.tf_inv
        if self.tf_inv_pitch is None:
            self.tf_inv_pitch = self.tf_inv
        self.ground = True
        self.reset()

    def reset(self):
        self.pos = [0.0, 0.0, 0.0]        # NED, m (z down; 0 = ground)
        self.vel = [0.0, 0.0, 0.0]        # NED, m/s
        self.q = (1.0, 0.0, 0.0, 0.0)     # body -> NED
        self.rates = [0.0, 0.0, 0.0]      # FRD body, rad/s
        self.tpp = [0.0, 0.0]             # flapping b1s (lateral), a1s (longitudinal)
        self.w = 0.0                      # rotor, rad/s
        self.on_ground = True
        self.specific_force = (0.0, 0.0, -G)
        self.vbat = self.pack_voltage(0.0)
        self.ibat = 0.0

    def pack_voltage(self, current):
        soc = max(0.0, 1.0 - self.used_ah / self.pack_ah)
        v_cell = 3.5 + 0.7 * soc
        return self.cells * v_cell - current * self.pack_r

    @property
    def rotor_rpm(self):
        return self.w * 60 / (2 * math.pi)

    @property
    def motor_rpm(self):
        return self.rotor_rpm * self.gear

    def step(self, dt, throttle, roll, pitch, yaw, collective):
        throttle = max(0.0, min(1.0, throttle))
        ctrl_x = self.sign_roll * self.cyclic_scale * roll
        ctrl_y = self.sign_pitch * self.cyclic_scale * pitch
        yaw_cmd = self.sign_yaw * self.yaw_scale * yaw
        coll = self.coll_scale * collective + self.coll_offset

        # rotor speed
        if self.motor_fit:
            A, B = self.motor_fit[:2]
            q_motor = max(0.0, A * throttle * self.vbat - B * self.w) if throttle > 0.02 else 0.0
            q_drag = self.w * self.w * self.drag_shape(coll)
        else:
            q_motor = max(0.0, self.A * throttle - self.B * self.w) if throttle > 0.02 else 0.0
            q_drag = self.k_d * self.w * self.w * self.drag_shape(coll)
        self.w = max(0.0, self.w + (q_motor - q_drag) * dt)
        rr = (self.w / self.nominal_w) ** 2        # thrust and control power scale with rpm^2

        # pack: motor power from torque * speed (inertia-normalised units -> watts)
        power = self.hover_power * q_motor * self.w / self.shaft_hover + 2.0   # + electronics
        self.ibat = power / max(self.vbat, 1.0)
        self.vbat = self.pack_voltage(self.ibat)
        self.used_ah += self.ibat * dt / 3600

        # air-relative body velocity
        u, v, w_ = earth_to_body(self.q, self.vel)
        p, q_, r = self.rates

        # flapping (tip path plane), driven by cyclic and body rates
        b1s, a1s = self.tpp
        ti_r, ti_p = self.tf_inv_roll, self.tf_inv_pitch
        b1s_dot = -p - ti_r * b1s + ti_r * rr * (self.Lfa1s * a1s + self.Lflt * ctrl_x + self.Lflg * ctrl_y)
        a1s_dot = -q_ - ti_p * a1s + ti_p * rr * (self.Mfb1s * b1s + self.Mflt * ctrl_x + self.Mflg * ctrl_y)
        b1s += b1s_dot * dt
        a1s += a1s_dot * dt
        self.tpp = [b1s, a1s]

        # rotational acceleration, body
        p_dot = b1s * self.Lb1s + self.Lu * u + self.Lv * v
        q_dot = a1s * self.Ma1s + self.Mu * u + self.Mv * v
        if self.tail_motor:
            # a tail motor has its own rpm (no rpm^2 scaling) and spins up with a lag
            self.yaw_lag += (yaw_cmd - self.yaw_lag) * dt / self.tail_tau
            yaw_power, yaw_cmd = self.Nped, self.yaw_lag
        else:
            yaw_power = rr * self.Nped
        r_dot = (self.Nv * v + self.Nr * r + yaw_power * yaw_cmd + self.Nw * w_
                 + rr * self.Ncol * (coll - self.coll_hover))
        if self.torque_yaw:
            # main rotor drag torque reacts on the fuselage; the tail cancels it in hover
            r_dot += self.torque_yaw * q_drag * self.nominal_w / self.shaft_hover

        # specific force (aero), body
        fx = -G * a1s + self.Xu * u
        fy = G * b1s + self.Yv * v + self.Yp * p
        fz = self.Zcol * coll * rr + self.Zw * w_

        # ground: held until the rotor lifts more than the weight
        accel_e = body_to_earth(self.q, (fx, fy, fz))
        accel_e = (accel_e[0], accel_e[1], accel_e[2] + G)
        if self.on_ground:
            if accel_e[2] < -0.05:
                self.on_ground = False
            else:
                self.vel = [0.0, 0.0, 0.0]
                self.rates = [0.0, 0.0, 0.0]
                self.tpp = [0.0, 0.0]
                roll_, pitch_, yaw_ = euler(self.q)
                self.q = qnorm((math.cos(yaw_ / 2), 0.0, 0.0, math.sin(yaw_ / 2)))   # level on its skids
                self.specific_force = earth_to_body(self.q, (0.0, 0.0, -G))
                return

        # integrate
        self.rates = [p + p_dot * dt, q_ + q_dot * dt, r + r_dot * dt]
        self.vel = [self.vel[i] + accel_e[i] * dt for i in range(3)]
        self.pos = [self.pos[i] + self.vel[i] * dt for i in range(3)]
        p, q_, r = self.rates
        dq = (0.0, p * 0.5 * dt, q_ * 0.5 * dt, r * 0.5 * dt)
        self.q = qnorm(tuple(a + b for a, b in zip(self.q, qmul(self.q, dq))))
        self.specific_force = (fx, fy, fz)

        if self.pos[2] > 0 and self.ground:       # touched down
            self.pos[2] = 0.0
            self.on_ground = True
            self.vel = [0.0, 0.0, 0.0]


class Stratos200(Heli):
    """
    SAB Goblin-style 200-size direct-drive heli (the user's Stratos 200 in a Bo105
    fuselage), fitted from its blackbox logs in rf-heli-model-fit/:
      rotor law      fit_rotor.py:  dw/dt = A u V - B w - w^2 (k0 + k1 c^2), R^2 0.994
      rate gains     fit_axis.py:   roll 5.07, pitch 4.74 rad/s per mixer input (0.5-3 Hz)
      yaw            fit_yaw.py + 0.5-8 Hz regression: tail motor up = nose right, 66 rad/s^2
                     per unit, 35 ms motor lag, weak damping; main torque nose left (CW rotor)
      hover          logs:          collective 0.636, throttle 0.47 at 15.2 V, tail 0.215
    Not identifiable from those (gentle) flights: the flap/body stiffness Lb1s/Ma1s and
    tf, the cross coupling, and the speed derivatives (kept from the Blade 360).
    """
    motor_fit = (987.74, 16.843, 7.003e-4, 1.101e-2, 2.0)
    motor_poles = 24
    tail_motor = True
    Zcol = -G / 0.636
    Ncol, Nw = 0.0, 0.0
    # bridge yaw input = tail motor, yaw_cmd = sign_yaw (-1) * motor: Nped < 0 gives nose right
    Nped, Nr = -66.0, -0.5
    Lfa1s = Mfb1s = 0.0
    Lflg = Mflt = 0.0
    # speed-coupled moments (flapback, dihedral, weathervane) cannot be identified without
    # velocity in the logs; the Blade 360 values made the drifting open-loop replay yaw and
    # roll on their own, so they are off. Plain translational drag stays.
    Lu = Lv = Mu = Mv = Nv = 0.0

    def __init__(self, Lb1s=1500.0, Ma1s=1500.0, tf=0.03, **kw):
        self.Lb1s, self.Ma1s = Lb1s, Ma1s
        self.tf_inv_roll = self.tf_inv_pitch = 1.0 / tf
        # steady rate per input = Lflt / tf (flapping model at hover)
        self.Lflt, self.Mflg = 5.07 * tf, 4.74 * tf
        self.torque_yaw = -66.0 * 0.215          # CW main rotor: reaction turns the nose left
        args = dict(nominal_rpm=3600.0, gear_ratio=1.0, hover_throttle=0.47, pack_cells=4,
                    pack_ah=0.85, pack_r=0.05, hover_power_w=50.0, cyclic_scale=1.0, yaw_scale=1.0)
        args.update(kw)
        super().__init__(**args)

    @property
    def tail_rpm(self):
        return 14340.0 * math.sqrt(max(self.tail_throttle, 0.0) / 0.215)

    tail_throttle = 0.0


MODELS = {'blade360': Heli, 'stratos200': Stratos200}
