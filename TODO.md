# To do

Where the sim goes next, ordered by what it gives firmware testing. Tick a box when its work
is merged.

Today there are three presets: `Heli` (Blade 360, ArduPilot's derivatives), `Stratos200` and
`M7R`. Each is a separate class of numbers fitted from one heli's logs (`heli/heli_model.py`,
`fitting/README.md`).

## 1. One physical model, size presets

Heli size changes what the firmware sees: pack voltage (the 12S ADC clip,
rotorflight-firmware#596), rotor inertia against motor torque (governor spool-up and droop),
the flap/body mode against the PID loop (~6 Hz on the Stratos, 2-3 Hz on the M7R), and the
tail type. New sizes should come from parameters, not from a fitting campaign.

- [ ] `PhysicalHeli`: derive the model's constants from physical data - mass and inertias,
      rotor radius, chord, blade count and mass, blade inertia (Lock number), hub stiffness,
      headspeed, gear ratio, motor Kv and resistance, cells, tail type (servo / motor), tail
      gear ratio and radius. Flapping time constant ~ 16 / (Lock number x rotor speed),
      stiffness from hub spring plus centrifugal terms, thrust from blade element with
      induced inflow (as `thrust_inflow`), motor from Kv/R, rotor drag from profile and
      induced power
- [ ] The `Stratos200` and `M7R` fits as calibration points: `PhysicalHeli` with their
      geometry must come out near the fitted values. First check: the Stratos' Lb1s 1400 at
      3600 rpm scaled by rpm^2 gives ~350 at 1800 rpm, the M7R fit says 700
- [ ] Rotor radius and hover induced velocity (`rotor_radius`, `v_hover`, used by the
      inflow model) are typical values per size for now, not measured
- [ ] Geometry and inertias of the fitted helis (blade length, chord and mass, mass, roll and
      pitch inertia, hub dampers) in their heli files - without them neither check above can
      run. The OMP M5, M6 and M7R logs (`fitting/helis/*/fit.json`) already show the rate per
      cyclic growing ~rpm on all three sizes, as stiffness ~rpm^2 and tf ~1/rpm predict
- [ ] Size presets as parameter sets: ~200, 380, 550, 700 class (and 800 / 90 mm blades for
      the high-voltage end); the Blade 360 kept as a reference
- [x] `--model` takes a preset name or a parameter file (JSON: base preset, constructor
      args, class constants; `heli_model.load_model`)
- [ ] CI runs hover, governor and rescue checks per size class, not only the Blade 360.
      Done for the M7R (700): the scripted pilots fly relative to the preset's hover
      collective, rescue collectives likewise. The Stratos (200, motorized tail) lifts off
      but yaws at 3.2 rad/s and does not roll inverted on the default gains: it needs its
      own tune (tail gains) without a dump

## 2. Gyro noise and vibration

The model's gyro and accelerometer are perfect: the firmware's filters (RPM filter, dynamic
notches, gyro low-pass, D-term filters) are never exercised.

Measure the noise from two sources, so the model separates sensor noise (per gyro chip,
independent of the heli) from vibration (per heli, scaled with rpm):

- [x] **Allan variance on a real flight controller**: 30 min soaks at rest (MSP_RAW_IMU at
      200 Hz) of ICM-40608, IIM-42652 and LSM6DSK320X -> `fitting/sensors/`. White noise
      0.0035-0.0055 deg/s/rtHz (datasheet level), bias instability 2-10 deg/h
- [ ] More chips (ICM-42688-P, BMI270, ICM-42605, MPU6000), several-hour soaks for the rate
      random walk, and the temperature drift over a warm-up from power-on
- [x] Script `fitting/scripts/allan.py`: overlapping Allan deviation from a log, fit of the
      noise terms, plot; results stored per chip
- [x] **Vibration from flight logs**: `fitting/scripts/vibration.py`, order tracking of the
      logged `gyroRAW` (4.6+, any debug mode) per axis: main rotor 1-4x, tail 1-3x, motor
      1-2x and the broadband floor. Done for the OMP M5, M6 and M7R (`fitting/helis/`)
- [x] Vibration of the Stratos (direct drive, motorized tail): main rotor lines; its tail
      (~4x main, varying with yaw) cannot be told from the main 4x
- [ ] Vibration against collective; the rpm exponent (two headspeeds per heli scatter it,
      the model uses rpm^2); motorized-tail lines in the sensor model once a heli has them
- [x] Model: gyro and accel = true value + chip noise (white, bias walk, from the Allan fit)
      + vibration (harmonics locked to the model's rotor, tail and motor rpm, amplitudes from
      the logs). Seeded (`heli/sensors.py`, `--gyro-chip`, `--vibration`)
- [ ] Resonance of the frame/FC mount; accelerometer vibration (no logs carry it)
- [ ] Quantization and range clipping per chip (the SITL applies 16.4 LSB/deg/s and
      256 LSB/g for all), and the sensor's own low-pass
- [x] Checks: with vibration on, the RPM filter must remove the tracked harmonics (filtered
      gyro error below 3 deg/s RMS over MSP in the calm descent), and the hover check still
      passes; with the RPM filter off it must fail (CI, both)
- [ ] Compare sim and real filtered-gyro spectra in `compare_replay.py`

## 3. Refining parameter sets from more flight logs

One heli, one flight, hand-run scripts does not scale. A pipeline that takes logs of any
heli and gives back a preset with its validation:

- [x] **Heli description file** per heli (`fitting/helis/<name>/heli.json`): geometry and
      hardware as in 1 (rotor, blades, gear, motor, ESC, pack, tail, gyro chip), firmware
      version, CLI dump, and the list of logs with their headspeed and profile. Replaces
      the hard-coded flight lists in `common.py` and `fit_cyclic.py`. Started for the
      Stratos and the OMP M5, M6, M7R; their geometry fields are still empty (see 1)
- [x] **Log intake** (`intake.py`): decode, de-duplicate (by content, across files),
      check what each log can feed (rate, gyroRAW, headspeed, power, baro, gains, hover per
      headspeed, sharp inputs, airborne autorotation, GPS), say what is missing, and add
      the logs to the heli file
- [x] **Segmentation** (`segment.py`): ground, autorotation, inverted, aerobatic, hover and
      cruise per sample (hover from baro vario, attitude around the roll trim and calm
      sticks; in ground effect flagged); `fit_heli.py` takes its hover stage from it with a
      5 s gate per headspeed
- [ ] Spool-up and stick-input (doublet, sweep) segments
- [ ] **Fits as stages**, each with a coherence / R^2 gate: rotor law, rate gains per
      headspeed, yaw, hover collective per headspeed (thrust law), vibration (2), then the
      closed-loop flap/body sweep by replay (`sweep_replay.py`, `m7r_sweep.py` generalised).
      Open-loop stages done in `fit_heli.py` (from the heli file, pooled over its logs, per
      headspeed -> `fit.json`); the hover stage is gated (5 s out of ground effect), the
      other gates and the closed-loop sweep are open
- [ ] **Joint fit across helis**: the parameters of `PhysicalHeli` (1) fitted on all helis
      together, with the geometry as known input, so each new heli refines the scaling laws
      instead of getting its own constants. Report where a heli is off the shared law
- [ ] **Validation report** per heli: replay scores (tracking error, whirl band power,
      throttle, yaw mixer) sim vs real per profile, written to a file and committed with
      the preset
- [ ] **Regression**: a small set of public, short log excerpts and their expected scores,
      replayed in CI (scheduled run, it is slow), so a model change that worsens a fitted
      heli shows up
- [x] **Contributing logs** (`CONTRIBUTING_LOGS.md`): a page on how to log a flight for the sim - blackbox rate and
      debug mode, CLI dump, the heli description, and a manoeuvre card (hover, sharp
      roll/pitch/collective doublets, frequency sweeps, a few seconds of each headspeed,
      piros, an autorotation if the heli does them). Logs stay out of the repository unless
      the owner agrees
- [x] Control power and flap/body dynamics from sharp inputs (`fit_flap.py`): the OMP sport
      flights hold them - per axis and headspeed on three sizes (M5, M6, M7R)
- [x] Per-axis L and tf checked by replay on the M7R: no better than the preset (roll too
      good, pitch worse); `tf_roll` / `tf_pitch` stay as options
- [ ] The joint fit across sizes once the geometry is known
- [x] A replay regression in CI (TODO 3, Regression): the session's model changes broke the
      M7R replay twice without a check noticing. `--replay-case heli/replays/m7r_cruise
      --check` (Linux job): airborne share, upright resets, throttle and tail trim against
      the real flight; fails on the inflow-torque regression of #3
- [ ] Test flights the current fits are missing: collective doublets (collective
      sensitivity, heave damping), hover at two headspeeds, an airborne autorotation

## 4. Forward flight

No translational lift and the speed derivatives are off: replays need `--hold-alt`, and
forward flight cannot be tested.

- [x] Translational lift: the induced velocity follows forward speed (Glauert), and the
      blade-element thrust its inflow
- [x] Fuselage drag (quadratic, flat-plate area 1.5 % of the disk); its power reaches the
      rotor through the disk tilt in the inflow (a separate parasite-power term counted it
      twice)
- [ ] Flapback, dihedral (Lv, Mu), weathervane (Nv), tail drag
- [ ] Fit them from logs with GPS speed (blackbox GPS fields) - add a GPS-logged flight to
      the manoeuvre card
- [ ] Replays without `--hold-alt` hold height in cruise; a forward-flight and a piro check

## 5. Autorotation

The rotor only loses speed to drag; upward airflow never drives it. Rotorflight's
autorotation bailout and the governor's autorotation handling cannot be tested.

- [x] Rotor torque from inflow (descent drives the rotor), so the rotor speed holds in an
      autorotation and decays with collective at the flare: momentum theory, the change of
      induced power T (v_c + v_i) - T v_h. All three presets autorotate at 1 g, ~2 v_h sink
- [x] Vortex ring state: the induced velocity follows Leishman's empirical fit between hover
      and the windmill brake - no power relief from the descent there. Its thrust
      fluctuations and roughness are not modelled
- [x] An autorotation check (`--autorotation land`): cut the throttle at height (governor
      AUTOROTATION), hold the headspeed on the collective, flare, land; and the bailout
      (`--autorotation bailout`, governor BAILOUT)
- [ ] Validate the sink rate and rotor decay against a logged autorotation. The OMP logs
      only hold ground spin-downs, and their headspeed is the motor's: it drops to zero when
      the one-way bearing freewheels. Needs an airborne autorotation with a rotor-side rpm
      sensor (add it to the manoeuvre card)

## 6. Ground

The heli used to snap level on the ground, and touchdown zeroed its velocity.

- [x] Ground effect (thrust gain below about one rotor diameter, Cheeseman & Bennett on
      the induced velocity)
- [x] Skid contact: spring, damping and stick-slip friction per contact point (four skid
      points, plus rotor head, nose and tail so a tipped heli lies on its side), about the
      centre of gravity: tilt on landing, sliding and tip-over happen (unit tests: drop,
      slide, tip-over angle, stiction, yaw friction)
- [x] A slope landing check: the ground plane can be tilted (`set_ground_slope`, unit tests
      hold / slide / land level onto it); `--slope DEG[,UPHILL]` sets the heli down on it
      after the level ACC calibration and judges the landing (at rest, not rolled over, no
      slide after touchdown). CI flies 15 deg across the skids. A rollover case is still
      missing: on 40 deg the heli slides before take-off (mu 0.6)
- [ ] Skid sizes and inertias from real helis (now fractions of
      the rotor radius)
- [ ] Ground resonance (stretch, needs a fuselage/hub mode)

## 7. Wind and turbulence

The current disturbance is a random moment on the body, with `turb_hz` tuned per heli.

- [x] Wind (steady and gusts) as air velocity (`heli/wind.py`, `--wind`, `--gust`), with
      Dryden turbulence (MIL-F-8785C low altitude, first-order shaping) scaled with the
      height; the rotor responds through the inflow (translational lift, vertical gusts),
      the drag and the speed derivatives. A hover check in a 5 m/s crosswind with gusts in CI
- [ ] Turbulence scale lengths from the rotor size, and the rotor's response to gusts
      across the disk (the fitted presets have their speed derivatives off, see 4)
- [ ] Check it against the whirl amplitude the fits undersize (Stratos at 3300 rpm, the D
      gain effect)

## 8. Couplings

- [ ] Roll/pitch cross coupling of the flapping (`Lfa1s`, `Mfb1s`, `Lflg`, `Mflt`, off in
      the fitted presets): fit from sharp-input flights
- [ ] Collective to yaw torque on servo tails (`Ncol`, off), to test the firmware's
      collective precompensation
- [ ] Main rotor gyroscopic and tail rotor gyroscopic terms
- [ ] Tail rotor speed, thrust and inflow (servo tails), tail authority loss at low rpm

## 9. Actuators and power

- [ ] ESC response model (the M7R's analog ESC: R^2 0.73 in the volts form); ESC telemetry
      models beyond HobbyWing V4
- [x] Servos: speed limit, deadband and lag per servo type on each pulse before the swash
      decode (`heli/servos.py`, `--servos ideal|digital|fast|analog`; CI flies digital)
- [ ] Servo load (stall torque against the feathering moment), types from measured servos
- [x] Pack: LiPo discharge curve (open-circuit voltage per cell against state of charge)
      and internal resistance rising towards empty, instead of the straight line
- [ ] Pack temperature (resistance and capacity), and a curve fitted from logged Vbat/Ibat

## 10. Engineering

- [x] Unit tests of the model alone, without the firmware: trim, whirl frequency, scaling
      with rpm, thrust law, rotor law - in CI on every push (`tests/`, about a minute)
- [x] Integrator: the fixed-step Euler at 4 kHz against the same model at 40 kHz (step
      response within 2 %, mode frequency within 1 %) for the Blade 360, Stratos and M7R;
      repeat for each new size class
- [ ] Keep `heli/` on the Python standard library; numpy/scipy stay in `fitting/`
