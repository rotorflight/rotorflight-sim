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

import json
import math
import os
import random

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


# LiPo open-circuit voltage per cell at 0, 5, ..., 100 % state of charge (a typical
# discharge curve: the knee below ~10 %, the long plateau around 3.8 V)
LIPO_OCV = (3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.80, 3.82, 3.84,
            3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.20)


def induced_velocity(vh, vc, vx):
    """Induced velocity of a rotor (m/s, through the disk against the thrust) at hover
    induced velocity vh for its thrust, axial climb speed vc (positive along the thrust)
    and in-plane speed vx, by momentum theory:
      climb / hover        vi = -vc/2 + sqrt(vc^2/4 + vh^2)
      windmill brake       vi = -vc/2 - sqrt(vc^2/4 - vh^2)        (vc < -2 vh)
      vortex ring          the empirical fit vi/vh = 1.15 - 1.125 x - 1.372 x^2 - 1.718 x^3
                           - 0.655 x^4, x = vc/vh in (-2, 0) (Leishman, Principles of
                           Helicopter Aerodynamics), where momentum theory has no solution
      forward flight       Glauert: vi = vh^2 / sqrt(vx^2 + (vc + vi)^2), faded in over
                           vx = 0..vh in the vortex ring range (forward speed blows the
                           ring away)"""
    if vh <= 0.0:
        return 0.0
    x = vc / vh
    if x >= 0.0:
        axial = -vc / 2 + math.sqrt(vc * vc / 4 + vh * vh)
    elif x <= -2.0:
        axial = -vc / 2 - math.sqrt(max(0.0, vc * vc / 4 - vh * vh))
    else:
        axial = vh * (1.15 - 1.125 * x - 1.372 * x ** 2 - 1.718 * x ** 3 - 0.655 * x ** 4)
    if vx < 1e-3:
        return axial
    # Glauert by damped fixed point, from the axial value
    vi = max(axial, 0.1 * vh)
    for _ in range(40):
        nxt = vh * vh / math.sqrt(vx * vx + (vc + vi) ** 2)
        if abs(nxt - vi) < 1e-6 * vh:
            vi = nxt
            break
        vi += 0.5 * (nxt - vi)
    if -2.0 < x < 0.0 and vx < vh:
        k = vx / vh
        return (1 - k) * axial + k * vi
    return vi


def ground_effect(radius, height):
    """Induced velocity factor in ground effect (after Cheeseman & Bennett: the same power
    gives T/T_inf = 1 / (1 - (R / 4z)^2)): vi_ige / vi = 1 - (R / 4z)^2, at most halved
    (z: rotor height above the ground)."""
    height = max(height, 1e-3)
    k = (radius / (4.0 * height)) ** 2
    return 1.0 - min(k, 0.5)


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
    # rotor-speed scaling of the flap/body mode, relative to nominal_rpm:
    # Lb1s, Ma1s x (w/w_n)^stiff_rpm_exp, flap time constant x (w_n/w)^tf_rpm_exp
    # (a Lock-number rotor: tf ~ 1/w). 0 = fixed
    stiff_rpm_exp = 0.0
    tf_rpm_exp = 0.0
    # rotor-speed exponent of the cyclic input to the flapping (2: control power ~ rpm^2,
    # the original model). With stiffness ~ rpm^2 and tf ~ 1/rpm, 0 is the consistent
    # choice: flapping per cyclic does not depend on rpm, the rate per cyclic goes ~ rpm
    ctrl_rpm_exp = 2.0
    # optional blade-element thrust with induced inflow, (K, ki): thrust in g
    #   n = K (w/w_n)^2 (coll/3 - lambda/2),  lambda = ki sqrt(n) / (w/w_n)
    # so the hover collective goes as 3/(K wr^2) + 1.5 ki/wr. None = thrust ~ wr^2 coll (Zcol).
    # Blade 360: its hover collective 0.44 (G / -Zcol) with a 65 % inflow share (the Stratos
    # fit has 72 %, blade-element theory ~60 % for these rotors) - so it autorotates
    thrust_inflow = (3.0 / (0.35 * 0.44), 0.65 * 0.44 / 1.5)
    # optional fitted rotor law (A, B, k0, k1, n): dw/dt = A u V - B w - w^2 (k0 + k1 |c|^n)
    motor_fit = None
    # ESC response: the motor gets the throttle behind a first-order lag of esc_tau s (0 =
    # at once). The M7R's analog ESC: 65 ms, fitted with its rotor law (m7r_fit.py)
    esc_tau = 0.0
    # main rotor torque reaction on yaw, rad/s^2 at hover (motorized tails); scales with drag
    torque_yaw = 0.0
    motor_poles = 2
    tail_motor = False
    tail_tau = 0.035
    yaw_lag = 0.0
    # servo tail dynamics between the tail command and the yaw the gyro sees (0 = off, the
    # first-order yaw of before): tail_lag, the tail thrust's build-up with its inflow (s,
    # first order); yaw_mode_hz / yaw_mode_zeta, a lightly damped mode the tail moment
    # reaches the body through (tail boom, drive and mounts flexing; unit gain at rest).
    # They give the yaw loop the phase lag that lets too much yaw gain wag the tail
    tail_lag = 0.0
    yaw_mode_hz = 0.0
    yaw_mode_zeta = 0.3
    # blade lead-lag (ground resonance): lag_nu, the lag mode's frequency per rotor speed
    # (soft in-plane < 1; 0 = off, the rigid rotor of before), lag_zeta its damping (the
    # dampers in the blade grips), lag_mass the share of the heli's mass that moves with
    # it (the rotor's in-plane centre of mass offset per lag), blades 2 or more. The rotor's
    # centre of mass whirls in the disk plane and pushes the hub; the body rocking on its
    # skids pushes back - with the regressive lag frequency w (1 - lag_nu) near a body
    # mode on the ground and too little lag damping the two feed each other
    lag_nu = 0.0
    lag_zeta = 0.05
    lag_mass = 0.05
    blades = 2
    servo_tau = 0.0
    turb = 0.0            # rad/s^2 std of random roll/pitch moments (0 = calm)
    turb_hz = 20.0        # their bandwidth (first-order)
    servo_xy = None
    Lfa1s, Mfb1s = 1.0477, -1.0057
    Lflt, Lflg, Mflt, Mflg = 0.2375, -0.0286, 0.0344, 0.2292
    # rotor inflow (momentum theory): induced velocity in hover v_hover = sqrt(W / 2 rho A),
    # the induced share of the hover shaft power, and the rotor radius and hub height above
    # the skids for the ground effect. The induced velocity follows climb, descent (vortex
    # ring, windmill brake) and forward speed (Glauert): the rotor torque takes the change of
    # induced power T (v_c + v_i) - T v_h from hover - so a steady descent drives the rotor
    # (autorotation) - and the blade-element thrust (thrust_inflow) the change of inflow,
    # which gives translational lift and more thrust per collective in descent
    v_hover = 3.3          # m/s; Blade 360 ~1.1 kg on a 0.36 m rotor
    ind_share = 0.6
    rotor_radius = 0.36
    hub_height = 0.25
    inflow_tau = None      # s, lag of the induced velocity; None: Pitt-Peters 0.849 R / (4 v_hover)
    # skids: four contact points (front/rear x left/right) below the centre of gravity, each
    # a spring and damper normal to the ground and Coulomb friction along it. Sizes as
    # fractions of the rotor radius (typical RC proportions), the body's radii of gyration
    # likewise (the model has no inertias of its own: moments are per unit inertia).
    # None = from rotor_radius
    skid_x = skid_y = cg_height = None
    gyration = None        # (roll, pitch, yaw) radii of gyration, m
    skid_deflection = 0.01  # m, static deflection under the weight (sets the stiffness)
    skid_zeta = 0.6        # damping ratio of the bounce
    skid_mu = 0.6          # friction coefficient (grass, skid tubes)
    # fuselage drag: equivalent flat-plate area as a fraction of the rotor disk (~0.015 for
    # RC helis). Per unit mass with the mass from v_hover (m = 2 rho A v_h^2 / g), the
    # quadratic drag is k |V| V with k = f g / (4 v_h^2), independent of the size. Its power
    # (D V) reaches the rotor through the disk tilt and the inflow, not as a term of its own
    flat_plate = 0.015

    def __init__(self, nominal_rpm=2100.0, gear_ratio=10.0, hover_throttle=0.65, load_fraction=0.85,
                 pack_cells=6, pack_ah=2.2, pack_r=0.02, hover_power_w=400.0, pack_temp=None,
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
        # pack temperature (pack_temp: the air and the pack's start, deg C; None = 25 C and
        # no thermal model, as before): its resistance and usable capacity follow it, and
        # the pack warms with its I^2 R. A LiPo of ~150 Wh/kg at ~1 kJ/kg/K, cooled to the
        # air with a 15 min time constant
        self.pack_ambient = pack_temp
        self.pack_temp = 25.0 if pack_temp is None else float(pack_temp)
        self.pack_heat_cap = 1000.0 * pack_cells * 3.7 * pack_ah / 150.0      # J/K
        self.pack_cooling = self.pack_heat_cap / 900.0                        # W/K
        if self.motor_fit:
            A, B, k0, k1, n = self.motor_fit
            self.drag_shape = lambda c: k0 + k1 * abs(c) ** n
            self.shaft_hover = self.nominal_w ** 3 * self.drag_shape(self.coll_hover)
        if self.tf_inv_roll is None:
            self.tf_inv_roll = self.tf_inv
        if self.tf_inv_pitch is None:
            self.tf_inv_pitch = self.tf_inv
        R = self.rotor_radius
        self.skid_x = self.skid_x or 0.35 * R
        self.skid_y = self.skid_y or 0.22 * R
        self.cg_height = self.cg_height or 0.30 * R
        self.gyration = self.gyration or (0.22 * R, 0.33 * R, 0.30 * R)
        self.skid_points = [(sx * self.skid_x, sy * self.skid_y, self.cg_height)
                            for sx in (1, -1) for sy in (1, -1)]
        # the rest of the airframe, so a heli that tips over lies on its side: rotor head,
        # nose, tail rotor
        self.skid_points += [(0.0, 0.0, self.cg_height - self.hub_height),
                             (0.35 * R, 0.0, 0.1 * R), (-1.15 * R, 0.0, -0.1 * R)]
        # per contact point and unit mass: the weight spread over four points at the static
        # deflection, damping for skid_zeta of the vertical bounce
        self.skid_k = G / (4 * self.skid_deflection)
        self.skid_c = self.skid_zeta * 2 * math.sqrt(4 * self.skid_k) / 4
        self.k_par = self.flat_plate * G / (4 * self.v_hover ** 2)
        self.wind_e = (0.0, 0.0, 0.0)      # air velocity, earth axes (heli/wind.py)
        self.ground = True
        self.ground_g = (0.0, 0.0, 1.0)    # unit normal into the ground (NED): level
        self.touchdown_speed = 0.0
        self.touchdown_rpm = 0.0
        self.touchdown_pos = None
        self.ground_effect = True
        self.inflow = True
        self.servo_xy = [0.0, 0.0]
        self.turb_state = [0.0, 0.0, 0.0]
        self.rng = random.Random(1)
        self.reset()

    def reset(self):
        # centre of gravity, NED, m (z down; the ground is z = 0): resting on the skids
        self.pos = [0.0, 0.0, -(self.cg_height - self.skid_deflection)]
        self.vel = [0.0, 0.0, 0.0]        # NED, m/s
        self.q = (1.0, 0.0, 0.0, 0.0)     # body -> NED
        self.rates = [0.0, 0.0, 0.0]      # FRD body, rad/s
        self.tpp = [0.0, 0.0]             # flapping b1s (lateral), a1s (longitudinal)
        self.w = 0.0                      # rotor, rad/s
        self.n = 0.0                      # rotor thrust, g (along -z body)
        self.vi = 0.0                     # induced velocity, m/s
        self.tail_state = 0.0             # tail moment behind tail_lag, rad/s^2
        self.esc_u = 0.0                  # throttle behind the ESC's lag (esc_tau)
        self.lag = [0j, 0j]               # lag mode: offset and its rate, rotating frame
        self.lag_psi = 0.0                # rotor azimuth for the lag mode, rad
        self.yaw_mode = [0.0, 0.0]        # tail moment through the yaw mode, and its rate
        self.on_ground = True
        self.air_time = 0.0
        self.anchors = {}
        self.specific_force = (0.0, 0.0, -G)
        self.vbat = self.pack_voltage(0.0)
        self.ibat = 0.0

    def pack_resistance(self, soc):
        """Internal resistance: rises towards empty (x2.5 at 0 %) and in the cold, ~x2 per
        20 C below 25 C (an Arrhenius-like exp(0.035 (25 - T)); 25 C above that, as the
        fitted packs were flown warm)."""
        r = self.pack_r * (1.0 + 1.5 * (1.0 - soc) ** 4)
        return r * math.exp(0.035 * max(0.0, 25.0 - self.pack_temp))

    def pack_capacity(self):
        """Usable capacity, Ah: ~0.6 % less per degree below 25 C (-12 % at 5 C)."""
        return self.pack_ah * max(0.5, 1.0 - 0.006 * max(0.0, 25.0 - self.pack_temp))

    def pack_voltage(self, current):
        """Open-circuit voltage from a LiPo discharge curve (per cell against state of
        charge), less the internal resistance (pack_resistance)."""
        soc = max(0.0, 1.0 - self.used_ah / self.pack_capacity())
        x = soc * (len(LIPO_OCV) - 1)
        i = min(int(x), len(LIPO_OCV) - 2)
        v_cell = LIPO_OCV[i] + (LIPO_OCV[i + 1] - LIPO_OCV[i]) * (x - i)
        return self.cells * v_cell - current * self.pack_resistance(soc)

    @property
    def rotor_rpm(self):
        return self.w * 60 / (2 * math.pi)

    @property
    def motor_rpm(self):
        return self.rotor_rpm * self.gear

    def thrust_g(self, coll, w=None):
        """Blade-element thrust in g at collective coll (thrust_inflow), closed form in sqrt(n)."""
        K, ki = self.thrust_inflow
        wr = (self.w if w is None else w) / self.nominal_w
        A, B = K * wr * wr / 3.0, K * wr * ki / 2.0
        s = (-B + math.sqrt(B * B + 4.0 * A * abs(coll))) / 2.0
        return math.copysign(s * s, coll)

    def thrust_g_inflow(self, coll, inflow):
        """Blade-element thrust in g at collective coll with the total inflow through the
        disk (v_c + v_i, m/s, along body up): n = K wr^2 (coll/3 - lambda/2), lambda =
        ki (v_c + v_i) / (v_hover wr) - in hover v_i = v_hover sqrt(n), the law above;
        inverted, both change sign."""
        K, ki = self.thrust_inflow
        wr = max(self.w / self.nominal_w, 1e-3)
        lam = ki * inflow / (self.v_hover * wr)
        return K * wr * wr * (coll / 3.0 - lam / 2.0)

    @property
    def altitude(self):
        """Height of the centre of gravity above its level resting position, m: the skid
        clearance while the heli is level, not when it is tilted (rolled 90 deg on its side
        it reads slightly below zero). The FDM packet, the baro and the checks use it."""
        return -self.pos[2] - self.cg_height + self.skid_deflection

    def set_altitude(self, alt):
        self.pos[2] = -(alt + self.cg_height - self.skid_deflection)

    def set_ground_slope(self, slope_deg, uphill_deg=0.0):
        """Tilt the ground plane (through the earth origin) by slope_deg, rising towards
        the heading uphill_deg. The skid contact acts along its normal and the friction in
        it; altitude, the baro and the ground effect keep the level reference."""
        s, d = math.radians(slope_deg), math.radians(uphill_deg)
        self.ground_g = (math.sin(s) * math.cos(d), math.sin(s) * math.sin(d), math.cos(s))

    def place_on_ground(self):
        """Set the heli down at rest on its skids where it is, keeping its heading: the
        body z along the ground normal, the skids at their static deflection."""
        g = self.ground_g
        _, _, yaw = euler(self.q)
        q = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        s = math.sqrt(g[0] ** 2 + g[1] ** 2)
        if s > 1e-9:
            # tilt earth z onto g about the horizontal axis z x g
            a = math.atan2(s, g[2]) / 2
            q = qnorm(qmul((math.cos(a), -g[1] / s * math.sin(a), g[0] / s * math.sin(a), 0.0), q))
        self.q = q
        on = sum(self.pos[i] * g[i] for i in range(3))       # depth of the CG below the plane
        h = self.cg_height - self.skid_deflection
        self.pos = [self.pos[i] - (on + h) * g[i] for i in range(3)]
        self.vel = [0.0, 0.0, 0.0]
        self.rates = [0.0, 0.0, 0.0]
        self.tpp = [0.0, 0.0]
        self.anchors = {}
        self.on_ground = True
        self.air_time = 0.0

    def height(self):
        """Rotor hub above the ground, m."""
        return self.altitude + self.hub_height

    def contact(self):
        """Skid contact: returns (force in earth axes per unit mass, moment in body axes per
        unit inertia, touching). Spring and damper on each point's depth below the ground.
        Friction sticks and slips: each touching point is tied to an anchor on the ground by
        a spring and damper along it, capped at mu times the normal force - beyond that the
        anchor slides along. So a steady push (the tail on the ground) is held without
        creeping, and a hard one slides."""
        fe = [0.0, 0.0, 0.0]
        mb = [0.0, 0.0, 0.0]
        g = self.ground_g                        # normal into the ground; level: (0, 0, 1)
        dot = lambda a, b: a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
        if -dot(self.pos, g) > self.cg_height + self.skid_x + self.skid_y + self.hub_height:
            self.anchors = {}
            return fe, mb, False                 # no point can reach the ground
        touching = False
        p, q_, r = self.rates
        anchors = {}
        for k, rb in enumerate(self.skid_points):
            re = body_to_earth(self.q, rb)
            pe = [self.pos[i] + re[i] for i in range(3)]
            depth = dot(pe, g)
            if depth <= 0.0:
                continue
            touching = True
            # point velocity: CG velocity + omega x r (body), in earth axes
            wxr = (q_ * rb[2] - r * rb[1], r * rb[0] - p * rb[2], p * rb[1] - q_ * rb[0])
            we = body_to_earth(self.q, wxr)
            vp = [self.vel[i] + we[i] for i in range(3)]
            vn = dot(vp, g)
            fn = max(0.0, self.skid_k * depth + self.skid_c * vn)
            # friction in the ground plane: the point's offset from its anchor and its
            # velocity, both without their normal parts
            a_ = self.anchors.get(k, pe)
            e = [pe[i] - a_[i] for i in range(3)]
            en = dot(e, g)
            e = [e[i] - en * g[i] for i in range(3)]
            t = [-self.skid_k * e[i] - self.skid_c * (vp[i] - vn * g[i]) for i in range(3)]
            cap = self.skid_mu * fn
            mag = math.sqrt(dot(t, t))
            if mag > cap:
                # slipping: friction at its limit, the anchor follows to where the spring
                # alone would give that limit
                sc = cap / mag
                t = [x * sc for x in t]
                el = math.sqrt(dot(e, e))
                lim = cap / self.skid_k
                if el > lim:
                    a_ = [pe[i] - e[i] / el * lim for i in range(3)]
            anchors[k] = a_
            f = tuple(t[i] - fn * g[i] for i in range(3))
            for i in range(3):
                fe[i] += f[i]
            fb = earth_to_body(self.q, f)
            tau = (rb[1] * fb[2] - rb[2] * fb[1], rb[2] * fb[0] - rb[0] * fb[2], rb[0] * fb[1] - rb[1] * fb[0])
            for i in range(3):
                mb[i] += tau[i] / (self.gyration[i] ** 2)
        self.anchors = anchors
        return fe, mb, touching

    def update_inflow(self, dt, u, v, w_):
        """Induced velocity for the current thrust, climb and forward speed, in and out of
        ground effect, with a first-order lag (Pitt-Peters time constant by default).
        self.vi is signed with the thrust (body up), so it goes to zero with the thrust and
        flips with it when inverted. Returns (inflow along body up, v_c_up + v_i, and the
        inflow along the thrust, which sets the induced power)."""
        sgn = 1.0 if self.n >= 0 else -1.0
        vc_up = -w_                          # body z is down; positive thrust points up the rotor axis
        vx = math.hypot(u, v)
        vh = self.v_hover * math.sqrt(abs(self.n))
        target = sgn * induced_velocity(vh, vc_up * sgn, vx)
        if self.ground_effect and self.ground:
            target *= ground_effect(self.rotor_radius, self.height())
        tau = self.inflow_tau or 0.849 * self.rotor_radius / (4 * self.v_hover)
        a = min(1.0, dt / tau)
        self.vi += (target - self.vi) * a
        return vc_up + self.vi, sgn * (vc_up + self.vi)

    def tail_moment(self, dt, n_cmd):
        """The tail's yaw moment (rad/s^2) on the body for the commanded n_cmd: behind the
        thrust lag and the yaw mode when they are set. The mode's state is advanced
        semi-implicitly (stable for the SITL's steps at these frequencies)."""
        if self.tail_lag > 0:
            self.tail_state += (n_cmd - self.tail_state) * min(1.0, dt / self.tail_lag)
            n_cmd = self.tail_state
        if self.yaw_mode_hz > 0:
            wn = 2 * math.pi * self.yaw_mode_hz
            x, xd = self.yaw_mode
            xd += (wn * wn * (n_cmd - x) - 2 * self.yaw_mode_zeta * wn * xd) * dt
            x += xd * dt
            self.yaw_mode = [x, xd]
            n_cmd = x
        return n_cmd

    def lag_step(self, dt, a_hub):
        """Advance the lag mode by dt under the hub acceleration a_hub (body x + i y, m/s^2);
        returns the in-plane force of the rotor on the hub per unit body mass (x + i y).
        Rotating frame, rotor speed w, lag frequency nu w: the rotor's centre-of-mass offset
        z_r obeys z_r'' + 2 zeta nu w z_r' + (nu w)^2 z_r = -a_hub (rotating); in the fixed
        frame z = z_r e^(i psi), and the hub feels -lag_mass z''. Two blades: one degree of
        freedom, the offset across the blade axis (z_r = i s); three or more: the isotropic
        (Coleman) whirl, z_r complex."""
        W = self.w
        wl = self.lag_nu * W
        rot = complex(math.cos(self.lag_psi), math.sin(self.lag_psi))
        a_r = a_hub / rot
        z, zd = self.lag
        if self.blades >= 3:
            zdd = -2 * self.lag_zeta * wl * zd - wl * wl * z - a_r
        else:
            s, sd = z.imag, zd.imag
            sdd = -2 * self.lag_zeta * wl * sd - wl * wl * s - a_r.imag
            zdd = 1j * sdd
        acc_fixed = (zdd + 2j * W * zd - W * W * z) * rot
        zd += zdd * dt
        z += zd * dt
        if self.blades < 3:
            z, zd = 1j * z.imag, 1j * zd.imag
        self.lag = [z, zd]
        self.lag_psi = (self.lag_psi + W * dt) % (2 * math.pi)
        return -self.lag_mass * acc_fixed

    def step(self, dt, throttle, roll, pitch, yaw, collective):
        throttle = max(0.0, min(1.0, throttle))
        if self.esc_tau > 0:
            self.esc_u += (throttle - self.esc_u) * min(1.0, dt / self.esc_tau)
            throttle = self.esc_u
        ctrl_x = self.sign_roll * self.cyclic_scale * roll
        ctrl_y = self.sign_pitch * self.cyclic_scale * pitch
        if self.servo_tau > 0:
            # cyclic servos follow with a first-order lag (the firmware only applies its speed limit)
            k = dt / (self.servo_tau + dt)
            self.servo_xy[0] += (ctrl_x - self.servo_xy[0]) * k
            self.servo_xy[1] += (ctrl_y - self.servo_xy[1]) * k
            ctrl_x, ctrl_y = self.servo_xy
        yaw_cmd = self.sign_yaw * self.yaw_scale * yaw
        coll = self.coll_scale * collective + self.coll_offset

        # air-relative body velocity, rotor inflow
        u, v, w_ = earth_to_body(self.q, [self.vel[i] - self.wind_e[i] for i in range(3)])
        va = math.sqrt(u * u + v * v + w_ * w_)
        inflow, inflow_t = self.update_inflow(dt, u, v, w_) if self.inflow else (0.0, 0.0)

        # rotor speed
        if self.motor_fit:
            A, B = self.motor_fit[:2]
            q_motor = max(0.0, A * throttle * self.vbat - B * self.w) if throttle > 0.02 else 0.0
            q_drag = self.w * self.w * self.drag_shape(coll)
        else:
            q_motor = max(0.0, self.A * throttle - self.B * self.w) if throttle > 0.02 else 0.0
            q_drag = self.k_d * self.w * self.w * self.drag_shape(coll)
        vc_a = -w_ * (1.0 if self.n >= 0 else -1.0)      # descent along the thrust < 0
        if self.inflow and self.w > 0.0 and vc_a < 0.0:
            # Descent: the induced power against hover at the same thrust, T (v_c + v_i) -
            # T v_h (axial momentum theory, as a torque scaled from the hover shaft power's
            # induced share) - negative in a steep descent, where the airflow drives the
            # rotor (autorotation). Only in descent: the rotor law (fitted from real flights
            # for the presets) already holds the power of powered flight - climbs and
            # forward flight against collective - and counting their induced power again
            # loaded the rotor twice (replays flew on too much throttle and tail)
            vh = self.v_hover * math.sqrt(abs(self.n))
            vi_ax = induced_velocity(vh, vc_a, 0.0)
            dp = self.ind_share * self.shaft_hover * abs(self.n) * (vc_a + vi_ax - vh) / self.v_hover
            q_drag += dp / max(self.w, 0.3 * self.nominal_w)
        self.w = max(0.0, self.w + (q_motor - q_drag) * dt)
        rr = (self.w / self.nominal_w) ** 2        # thrust and control power scale with rpm^2
        wr = max(self.w / self.nominal_w, 0.2)
        ks = wr ** self.stiff_rpm_exp if self.stiff_rpm_exp else 1.0
        kt = wr ** self.tf_rpm_exp if self.tf_rpm_exp else 1.0

        # pack: motor power from torque * speed (inertia-normalised units -> watts)
        power = self.hover_power * q_motor * self.w / self.shaft_hover + 2.0   # + electronics
        self.ibat = power / max(self.vbat, 1.0)
        self.vbat = self.pack_voltage(self.ibat)
        self.used_ah += self.ibat * dt / 3600
        if self.pack_ambient is not None:
            soc = max(0.0, 1.0 - self.used_ah / self.pack_capacity())
            heat = self.ibat ** 2 * self.pack_resistance(soc) - self.pack_cooling * (self.pack_temp - self.pack_ambient)
            self.pack_temp += heat * dt / self.pack_heat_cap

        p, q_, r = self.rates

        # flapping (tip path plane), driven by cyclic and body rates
        b1s, a1s = self.tpp
        ti_r, ti_p = self.tf_inv_roll * kt, self.tf_inv_pitch * kt
        kc = rr if self.ctrl_rpm_exp == 2.0 else wr ** self.ctrl_rpm_exp
        b1s_dot = -p - ti_r * b1s + ti_r * kc * (self.Lfa1s * a1s + self.Lflt * ctrl_x + self.Lflg * ctrl_y)
        a1s_dot = -q_ - ti_p * a1s + ti_p * kc * (self.Mfb1s * b1s + self.Mflt * ctrl_x + self.Mflg * ctrl_y)
        b1s += b1s_dot * dt
        a1s += a1s_dot * dt
        self.tpp = [b1s, a1s]

        # rotational acceleration, body
        p_dot = b1s * self.Lb1s * ks + self.Lu * u + self.Lv * v
        q_dot = a1s * self.Ma1s * ks + self.Mu * u + self.Mv * v
        if self.turb > 0 and not self.on_ground:
            # turbulence: band-limited (first order, 20 Hz) random moments, seeded
            a = dt * 2 * math.pi * self.turb_hz
            scale = self.turb * math.sqrt(2 / a)        # keeps the filtered std at turb
            for i in range(3):
                self.turb_state[i] += (self.rng.gauss(0.0, scale) - self.turb_state[i]) * a
            p_dot += self.turb_state[0]
            q_dot += self.turb_state[1]
        if self.tail_motor:
            # a tail motor has its own rpm (no rpm^2 scaling) and spins up with a lag
            self.yaw_lag += (yaw_cmd - self.yaw_lag) * dt / self.tail_tau
            yaw_power, yaw_cmd = self.Nped, self.yaw_lag
        else:
            yaw_power = rr * self.Nped
        n_tail = self.tail_moment(dt, yaw_power * yaw_cmd)
        r_dot = (self.Nv * v + self.Nr * r + n_tail + self.Nw * w_
                 + rr * self.Ncol * (coll - self.coll_hover))
        if self.torque_yaw:
            # main rotor drag torque reacts on the fuselage; the tail cancels it in hover
            r_dot += self.torque_yaw * q_drag * self.nominal_w / self.shaft_hover

        # specific force (aero), body
        fx = -G * a1s + self.Xu * u - self.k_par * va * u
        fy = G * b1s + self.Yv * v + self.Yp * p - self.k_par * va * v
        if self.thrust_inflow and self.inflow:
            # the inflow carries the heave damping (Zw) and translational lift
            self.n = self.thrust_g_inflow(coll, inflow)
            fz = -G * self.n
        elif self.thrust_inflow:
            self.n = self.thrust_g(coll)
            fz = -G * self.n + self.Zw * w_
        else:
            n = -(self.Zcol * coll * rr) / G
            if self.inflow and self.ground_effect and self.ground:
                n /= ground_effect(self.rotor_radius, self.height())
            self.n = n
            fz = -G * n + self.Zw * w_

        # skids on the ground
        accel_e = body_to_earth(self.q, (fx, fy, fz))
        accel_e = [accel_e[0], accel_e[1], accel_e[2] + G]
        touching = False
        if self.ground:
            fe, mb, touching = self.contact()
            if touching:
                for i in range(3):
                    accel_e[i] += fe[i]
                p_dot += mb[0]
                q_dot += mb[1]
                r_dot += mb[2]
                # the accelerometer feels the ground's push too
                fb = earth_to_body(self.q, fe)
                fx, fy, fz = fx + fb[0], fy + fb[1], fz + fb[2]
        if self.lag_nu > 0 and self.w > 1.0:
            # hub acceleration in the disk plane (body x, y): the centre of gravity's plus
            # the body's angular acceleration about it, the hub h above it
            h = self.hub_height - self.cg_height
            ab = earth_to_body(self.q, accel_e)
            a_hub = complex(ab[0] - h * q_dot, ab[1] + h * p_dot)
            F = self.lag_step(dt, a_hub)
            kx, ky = self.gyration[0], self.gyration[1]
            p_dot += h * F.imag / (kx * kx)
            q_dot += -h * F.real / (ky * ky)
            fe_l = body_to_earth(self.q, (F.real, F.imag, 0.0))
            accel_e = [accel_e[i] + fe_l[i] for i in range(3)]
            fx, fy = fx + F.real, fy + F.imag
        if touching and not self.on_ground and self.air_time >= 0.3:
            # touched down after flying (a bounce off the skids does not count)
            self.touchdown_speed = self.vel[2]
            self.touchdown_rpm = self.rotor_rpm
            self.touchdown_pos = (self.pos[0], self.pos[1])
        self.air_time = 0.0 if touching else self.air_time + dt
        self.on_ground = touching

        # integrate
        self.rates = [p + p_dot * dt, q_ + q_dot * dt, r + r_dot * dt]
        self.vel = [self.vel[i] + accel_e[i] * dt for i in range(3)]
        self.pos = [self.pos[i] + self.vel[i] * dt for i in range(3)]
        p, q_, r = self.rates
        dq = (0.0, p * 0.5 * dt, q_ * 0.5 * dt, r * 0.5 * dt)
        self.q = qnorm(tuple(a + b for a, b in zip(self.q, qmul(self.q, dq))))
        self.specific_force = (fx, fy, fz)


class Fitted(Heli):
    """
    A heli from fitted numbers alone: every constant comes from a parameter file (the
    presets in heli/presets/, or fitting/helis/<heli>/model.json written by
    fitting/scripts/train.py). What the fits identify per axis:

      rate_gain_roll / _pitch   steady rate per unit mixer input, rad/s (fit_heli.py
                                spectral gain or fit_flap.py K), at nominal_rpm
      Lb1s, Ma1s, tf            flap/body stiffness (1/s^2) and flap time constant (s);
                                tf_roll / tf_pitch per axis (fit_flap.py), else tf
      yaw_trim                  tail command that holds the main rotor's torque in hover
                                (|Nped| units): torque_yaw = -|Nped| yaw_trim (CW rotor)
      tail_rpm_hover            motorized tail: its rpm at yaw_trim (the SITL's eRPM)
    Class constants (motor_fit, thrust_inflow, Nped, Nr, v_hover, ...) come in as "attrs".
    The speed derivatives and the flapping cross coupling are off: blackbox logs without
    air data cannot identify them.
    """
    Ncol, Nw = 0.0, 0.0
    Lfa1s = Mfb1s = 0.0
    Lflg = Mflt = 0.0
    Lu = Lv = Mu = Mv = Nv = 0.0
    tail_throttle = 0.0

    def __init__(self, rate_gain_roll=5.0, rate_gain_pitch=5.0, Lb1s=700.0, Ma1s=700.0, tf=0.05,
                 tf_roll=None, tf_pitch=None, stiff_rpm_exp=2.0, tf_rpm_exp=1.0, turb=0.0, turb_hz=5.0,
                 servo_tau=0.0, yaw_trim=0.2, tail_rpm_hover=None, **kw):
        self.stiff_rpm_exp, self.tf_rpm_exp = stiff_rpm_exp, tf_rpm_exp
        self.turb, self.turb_hz, self.servo_tau = turb, turb_hz, servo_tau
        self.Lb1s, self.Ma1s = Lb1s, Ma1s
        tr, tp = tf_roll or tf, tf_pitch or tf
        self.tf_inv_roll, self.tf_inv_pitch = 1.0 / tr, 1.0 / tp
        # steady rate per input = Lflt / tf (flapping model at hover)
        self.Lflt, self.Mflg = rate_gain_roll * tr, rate_gain_pitch * tp
        self.yaw_trim = yaw_trim
        self.torque_yaw = -abs(self.Nped) * yaw_trim
        self.tail_rpm_hover = tail_rpm_hover
        super().__init__(**kw)

    @property
    def tail_rpm(self):
        if not self.tail_rpm_hover:
            return 0.0
        return self.tail_rpm_hover * math.sqrt(max(self.tail_throttle, 0.0) / max(self.yaw_trim, 1e-3))


HERE = os.path.dirname(os.path.abspath(__file__))
PRESET_DIR = os.path.join(HERE, 'presets')
# trained helis: fitting/helis/<name>/model.json (fitting/scripts/train.py)
TRAINED_DIR = os.path.join(HERE, '..', 'fitting', 'helis')
FILE_KEYS = {'base', 'args', 'attrs', 'description', 'notes', 'provenance'}


def read_params(path):
    with open(path) as f:
        p = json.load(f)
    if not isinstance(p, dict):
        raise ValueError('%s: a JSON object expected' % path)
    unknown = set(p) - FILE_KEYS
    if unknown:
        raise ValueError('%s: unknown keys %s' % (path, ', '.join(sorted(unknown))))
    if not isinstance(p.get('attrs', {}), dict) or not isinstance(p.get('args', {}), dict):
        raise ValueError('%s: "args" and "attrs" must be objects' % path)
    return p


def subclass(base, name, p, path):
    """base with a parameter file's class constants ("attrs") and constructor defaults
    ("args"; keyword arguments go on top)."""
    attrs = dict(p.get('attrs', {}))
    for k in attrs:
        if not hasattr(base, k):
            raise ValueError('%s: %s has no constant %s' % (path, base.__name__, k))
    args = dict(p.get('args', {}))
    if not attrs and not args:
        return base

    def __init__(self, **kw):
        base.__init__(self, **{**args, **kw})
    attrs['__init__'] = __init__
    attrs['__doc__'] = '\n'.join([p.get('description', '')] + list(p.get('notes', [])))
    attrs['params'] = p
    return type(name, (base,), attrs)


def preset(name, cls_name):
    path = os.path.join(PRESET_DIR, name + '.json')
    return subclass(Fitted, cls_name, read_params(path), path)


# The fitted presets: their numbers and how they were found live in heli/presets/*.json
Stratos200 = preset('stratos200', 'Stratos200')
M7R = preset('m7r', 'M7R')

MODELS = {'blade360': Heli, 'stratos200': Stratos200, 'm7r': M7R}
BASES = dict(MODELS, fitted=Fitted)


def trained_models():
    """Names of the trained helis (fitting/helis/<name>/model.json)."""
    if not os.path.isdir(TRAINED_DIR):
        return []
    return sorted(d for d in os.listdir(TRAINED_DIR)
                  if os.path.isfile(os.path.join(TRAINED_DIR, d, 'model.json')))


def resolve(spec):
    """A model spec as a preset name, a parameter file, or None if it is neither: preset
    names first, then trained helis by name, then a path."""
    if spec in MODELS:
        return spec
    path = os.path.join(TRAINED_DIR, spec, 'model.json')
    if os.sep not in spec and '/' not in spec and os.path.isfile(path):
        return path
    return spec if os.path.isfile(spec) else None


def load_model(spec, **kw):
    """A heli from a preset name (MODELS), a trained heli's name (fitting/helis/<name>/
    model.json) or a JSON parameter file:

        {"base": "m7r",                      preset it starts from (default blade360;
                                             "fitted": the generic Fitted class)
         "args": {"Lb1s": 900, "pack_cells": 6},   constructor arguments
         "attrs": {"v_hover": 3.0},          class constants (Heli's upper-case derivatives,
                                             v_hover, rotor_radius, ...) set before __init__
         "description": "...", "notes": [...], "provenance": {...}}   documentation only

    Keyword arguments (the bridge's --set and frame signs) go on top of "args"."""
    path = resolve(spec)
    if path is None:
        raise ValueError('no preset, trained heli or file %s (presets: %s; trained: %s)'
                         % (spec, ', '.join(sorted(MODELS)), ', '.join(trained_models()) or '-'))
    if path in MODELS:
        return MODELS[path](**kw)
    return from_params(read_params(path), path, **kw)


def from_params(p, path='<params>', **kw):
    """A heli from a parameter dict (the contents of a parameter file)."""
    name = p.get('base', 'blade360')
    if name not in BASES:
        raise ValueError('%s: unknown base %s (bases: %s)' % (path, name, ', '.join(sorted(BASES))))
    base = BASES[name]
    return subclass(base, base.__name__ + 'File', p, path)(**kw)
