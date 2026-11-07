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
    # main rotor torque reaction on yaw, rad/s^2 at hover (motorized tails); scales with drag
    torque_yaw = 0.0
    motor_poles = 2
    tail_motor = False
    tail_tau = 0.035
    yaw_lag = 0.0
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
        self.on_ground = True
        self.air_time = 0.0
        self.anchors = {}
        self.specific_force = (0.0, 0.0, -G)
        self.vbat = self.pack_voltage(0.0)
        self.ibat = 0.0

    def pack_voltage(self, current):
        """Open-circuit voltage from a LiPo discharge curve (per cell against state of
        charge), less the internal resistance, which rises towards empty (x2.5 at 0 %)."""
        soc = max(0.0, 1.0 - self.used_ah / self.pack_ah)
        x = soc * (len(LIPO_OCV) - 1)
        i = min(int(x), len(LIPO_OCV) - 2)
        v_cell = LIPO_OCV[i] + (LIPO_OCV[i + 1] - LIPO_OCV[i]) * (x - i)
        r = self.pack_r * (1.0 + 1.5 * (1.0 - soc) ** 4)
        return self.cells * v_cell - current * r

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

    def step(self, dt, throttle, roll, pitch, yaw, collective):
        throttle = max(0.0, min(1.0, throttle))
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
        r_dot = (self.Nv * v + self.Nr * r + yaw_power * yaw_cmd + self.Nw * w_
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


class Stratos200(Heli):
    """
    A 200-size direct-drive heli with a motorized tail (a Stratos 200 in a scale
    fuselage), fitted from its blackbox logs (Rotorflight 4.7, 3600 rpm, 4S):
      rotor law      dw/dt = A u V - B w - w^2 (k0 + k1 c^2), from motor volts against
                     headspeed and collective (R^2 0.994)
      rate gains     roll 5.07, pitch 4.74 rad/s per mixer input (0.5-3 Hz, setpoint as
                     instrument for the closed-loop flight data)
      yaw            tail motor up = nose right, 66 rad/s^2 per unit, 35 ms motor lag,
                     weak damping; main rotor torque nose left (CW rotor)
      hover          collective 0.636, throttle 0.47 at 15.2 V, tail 0.215
      thrust         blade element with induced inflow, from the hover collective at
                     3600 rpm (0.64) and 3300 rpm (0.715): a/wr^2 + b/wr, a 0.18, b 0.46
    Closed-loop identification, replaying the flight's sticks through the SITL with the
    heli's own configuration and gains:
      flap/body stiffness Lb1s 1400 (roll mode 5.95 Hz), Ma1s 2000, tf 0.05 s - with
      turbulence 4.3 rad/s^2 at 5 Hz (turb=4.3, turb_hz=5) the replay matches the real
      heli's ~5.9 Hz roll/pitch whirl: 4-8 Hz roll 5.85 vs 5.93, pitch 3.70 vs 3.77 deg/s.
      A sharp pitch input is tracked within 10 deg/s RMS, about 10 % slower to peak.
      Stiffness ~ rpm^2 and tf ~ 1/rpm: at 3300 rpm the replay whirls at 5.1 Hz (real
      4.94; 5.9 without the scaling), with too little amplitude (7 vs 10 deg/s roll).
    Not identifiable from those flights: cross coupling and speed derivatives (off).
    Fly it with the heli's CLI dump (--dump heli/replays/stratos_3300/dump.txt): on the
    bridge's default gains the motorized tail does not hold the yaw.
    """
    motor_fit = (987.74, 16.843, 7.003e-4, 1.101e-2, 2.0)
    motor_poles = 24
    # typical 200 size (not measured): ~0.5 kg in the fuselage on a 0.21 m rotor
    v_hover, rotor_radius, hub_height = 3.8, 0.21, 0.15
    tail_motor = True
    Zcol = -G / 0.636
    # hover collective 0.70-0.73 at 3300 rpm, 0.636-0.646 at 3600: a/wr^2 + b/wr with
    # a 0.180, b 0.460 - induced inflow carries most of the hover pitch
    thrust_inflow = (3.0 / 0.1798, 0.4597 / 1.5)
    Ncol, Nw = 0.0, 0.0
    # bridge yaw input = tail motor, yaw_cmd = sign_yaw (-1) * motor: Nped < 0 gives nose right
    Nped, Nr = -66.0, -0.5
    Lfa1s = Mfb1s = 0.0
    Lflg = Mflt = 0.0
    # speed-coupled moments (flapback, dihedral, weathervane) cannot be identified without
    # velocity in the logs; the Blade 360 values made the drifting open-loop replay yaw and
    # roll on their own, so they are off. Plain translational drag stays.
    Lu = Lv = Mu = Mv = Nv = 0.0

    def __init__(self, Lb1s=1400.0, Ma1s=2000.0, tf=0.05, servo_tau=0.0, turb=0.0, turb_hz=5.0,
                 stiff_rpm_exp=2.0, tf_rpm_exp=1.0, **kw):
        self.stiff_rpm_exp, self.tf_rpm_exp = stiff_rpm_exp, tf_rpm_exp
        self.turb_hz = turb_hz
        self.turb = turb
        self.servo_tau = servo_tau
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


class M7R(Heli):
    """
    A 700-class heli with a servo tail (OMP M7R, 12S, analog ESC under the governor),
    fitted from one 333 s sport flight (Rotorflight 4.6, 1500/1800/2000 rpm profiles):
      rotor law      dw/dt = A u V - B w - w^2 (k0 + k1 c^2) (R^2 0.73 in the volts form;
                     the analog ESC adds a throttle response of its own)
      rate gains     roll 6.8, pitch 5.7 rad/s per mixer input (0.3-4 Hz, coherence
                     0.9+, setpoint as instrument)
                     at 1500/1800/2000 rpm roll 5.1/6.7/7.9 (~rpm^1.5), pitch 5.8/5.9/6.3
                     (~rpm^0.5): ~rpm on average, as flapping theory has it with stiffness
                     ~rpm^2 and tf ~1/rpm (ctrl_rpm_exp 0)
      yaw            servo tail 38 rad/s^2 per unit at 1500 rpm (55 at 1800, x rpm^2; R^2
                     0.68 in the cruise flying, the 3D parts fit poorly), damping -2 /s,
                     hover tail trim -0.26..-0.28 at all headspeeds, against the main
                     rotor torque
      hover          collective 0.346 at 1500 rpm (cruise profile, 39 s upright with calm
                     sticks); the inflow law with the Stratos' inflow share (72 %) gives
                     0.273 at 1800 - the 1800 rpm 3D profile's calm moments read 0.280.
                     Without velocity in the log, forward flight cannot be told from hover
      pack           12S, 50.4 V full, 47 mOhm, ~4.5 Ah (4.9 V over 2.66 Ah)
    Closed-loop identification, replaying the cruise profile (1500 rpm, real 242-318 s)
    through the SITL with the header's gains and rates, collective rate 122 and a slow
    height hold (--hold-alt; the open-loop collective drifts without translational lift):
      flap/body      Lb1s = Ma1s 700, tf 0.05 s: roll / pitch tracking error 4.8 / 3.6
                     deg/s (real 4.7 / 4.1), 1.5-3 Hz 3.0 / 2.3 (2.2 / 1.9). Softer (300)
                     rings at 2.3 Hz, stiffer (1500+) tracks better than the real heli in
                     the sim's calm air; tf 0.1 leaves a slow roll error
      yaw            tracking error 11.0 vs 9.3 deg/s; the torque term matches the yaw
                     mixer's median in that flying (-0.21)
      throttle       47.7 % at 49.2 V vs 51.1 % at 45.5 V: the same motor volts
    Gear ratio 8.75 (140/16): the motor's 1x line in the gyro of later flights sits at
    8.75 rotor orders (fitting/helis/omp_m7r). Not in the log: motor poles (10 here: they
    only set the eRPM the SITL sees, the bridge configures the same), speed derivatives (off).
    """
    motor_fit = (3.05, 0.3667, 4.907e-4, 5.248e-4, 2.0)
    motor_poles = 10
    # typical 700 class (not measured): ~5.2 kg on a 0.79 m rotor
    v_hover, rotor_radius, hub_height = 3.3, 0.79, 0.40
    ctrl_rpm_exp = 0.0
    Zcol = -G / 0.273
    thrust_inflow = (3.0 / 0.0765, 0.197 / 1.5)
    Ncol, Nw = 0.0, 0.0
    Nped, Nr = 55.0, -2.0
    Lfa1s = Mfb1s = 0.0
    Lflg = Mflt = 0.0
    Lu = Lv = Mu = Mv = Nv = 0.0

    def __init__(self, Lb1s=700.0, Ma1s=700.0, tf=0.05, servo_tau=0.0, turb=0.0, turb_hz=2.5,
                 stiff_rpm_exp=2.0, tf_rpm_exp=1.0, tf_roll=None, tf_pitch=None, **kw):
        self.stiff_rpm_exp, self.tf_rpm_exp = stiff_rpm_exp, tf_rpm_exp
        self.turb, self.turb_hz, self.servo_tau = turb, turb_hz, servo_tau
        self.Lb1s, self.Ma1s = Lb1s, Ma1s
        # per-axis flap time constants (fit_flap.py identifies them separately); the steady
        # rate per input stays the fitted K = Lflt / tf on each axis
        tr, tp = tf_roll or tf, tf_pitch or tf
        self.tf_inv_roll, self.tf_inv_pitch = 1.0 / tr, 1.0 / tp
        self.Lflt, self.Mflg = 6.8 * tr, 5.7 * tp
        # main rotor torque against the tail trim: the yaw mixer sits at -0.21 (median of
        # the replayed cruise flying, collective precomp included)
        self.torque_yaw = -55.0 * 0.20
        args = dict(nominal_rpm=1800.0, gear_ratio=8.75, hover_throttle=0.6, pack_cells=12,
                    pack_ah=4.5, pack_r=0.047, hover_power_w=1150.0, cyclic_scale=1.0, yaw_scale=1.0)
        args.update(kw)
        super().__init__(**args)


MODELS = {'blade360': Heli, 'stratos200': Stratos200, 'm7r': M7R}


def load_model(spec, **kw):
    """A heli from a preset name (MODELS) or a JSON parameter file:

        {"base": "m7r",                      preset it starts from (default blade360)
         "args": {"Lb1s": 900, "pack_cells": 6},   constructor arguments
         "attrs": {"v_hover": 3.0}}          class constants (Heli's upper-case derivatives,
                                             v_hover, rotor_radius, ...) set before __init__

    Keyword arguments (the bridge's --set and frame signs) go on top of "args"."""
    if spec in MODELS:
        return MODELS[spec](**kw)
    import json
    with open(spec) as f:
        p = json.load(f)
    if not isinstance(p, dict):
        raise ValueError('%s: a JSON object expected' % spec)
    unknown = set(p) - {'base', 'args', 'attrs', 'description'}
    if unknown:
        raise ValueError('%s: unknown keys %s' % (spec, ', '.join(sorted(unknown))))
    name = p.get('base', 'blade360')
    if name not in MODELS:
        raise ValueError('%s: unknown base %s (presets: %s)' % (spec, name, ', '.join(sorted(MODELS))))
    base = MODELS[name]
    attrs = p.get('attrs', {})
    if not isinstance(attrs, dict) or not isinstance(p.get('args', {}), dict):
        raise ValueError('%s: "args" and "attrs" must be objects' % spec)
    for k in attrs:
        if not hasattr(base, k):
            raise ValueError('%s: %s has no constant %s' % (spec, base.__name__, k))
    cls = type(base.__name__ + 'File', (base,), dict(attrs)) if attrs else base
    return cls(**{**p.get('args', {}), **kw})
