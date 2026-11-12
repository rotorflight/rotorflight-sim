# To do

Where the sim goes next, ordered by what it gives firmware testing. Tick a box when its work
is merged.

Today there are three presets: `Heli` (Blade 360, ArduPilot's derivatives), `Stratos200` and
`M7R` (parameter files in `heli/presets/`, fitted from one heli's logs each), and the helis
`fitting/scripts/train.py` trained from their logs (`fitting/helis/<heli>/model.json`).

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
- [x] Size presets as parameter sets: the trained helis fly by name - 200 (`stratos200`,
      preset), 380 (`protos380`), 500 (`devil505`), Nimbus, 700 (`omp_m7r`, `stratos700`,
      preset `m7r`); the Blade 360 kept as a reference. CI flies the hover and rescue
      checks of `protos380`, `devil505`, `nimbus` and `omp_m7r` from their committed
      `model.json`
- [ ] The Stratos 700 fails both checks (governor droop 3.3 %, the rescue does not climb):
      its rotor law was refused (it made the held-out yaw worse through the torque
      reaction), so it flies the generic DC motor, which the logged hover throttle leaves
      too little headroom. An 800 / 90 mm class needs logs
- [x] `--model` takes a preset name or a parameter file (JSON: base preset, constructor
      args, class constants; `heli_model.load_model`)
- [x] CI runs hover, governor and rescue checks per size class, not only the Blade 360:
      the M7R preset (700) on the bridge's defaults, the trained 380, 500, Nimbus and 700
      helis, and the Stratos (200, motorized tail) with its CLI dump
      (`heli/replays/stratos_3300/dump.txt`; on the defaults it yaws away). The pilots'
      collective and the rescue collectives go in proportion to each model's hover
      collective (the Blade 360's ratios), full throttle stick under the governor

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
- [x] Vibration against collective and the rpm exponent: `vibration.py` fits both together
      (log amplitude on log rpm and collective, with the exponent's standard error), on
      eight helis now. The well-pinned exponents (se < 0.5) scatter from 0 to 4 (median
      1.7, weighted mean 1.9): lines move through frame resonances, no one law. The sensor
      model takes a harmonic's own exponent where it is pinned (clamped 0..4), else 2.
      Collective barely matters (median -0.14 per unit: 1-2 % per 0.1)
- [x] Tail and motor gear ratios from the vibration (`find_orders.py`): rotor-locked lines
      at non-integer orders, combined over headspeeds; filled in for the Devil 505, Nimbus,
      Protos 380 and Stratos 700. Integer tail ratios (M7R 5.0) sit on main rotor lines
- [ ] Motorized-tail lines in the sensor model once a heli has them
- [x] Model: gyro and accel = true value + chip noise (white, bias walk, from the Allan fit)
      + vibration (harmonics locked to the model's rotor, tail and motor rpm, amplitudes from
      the logs). Seeded (`heli/sensors.py`, `--gyro-chip`, `--vibration`)
- [ ] Resonance of the frame/FC mount; accelerometer vibration (no logs carry it)
- [ ] Quantization and range clipping per chip (the SITL applies 16.4 LSB/deg/s and
      256 LSB/g for all), and the sensor's own low-pass
- [x] Checks: with vibration on, the RPM filter must remove the tracked harmonics (filtered
      gyro error below 3 deg/s RMS over MSP in the calm descent), and the hover check still
      passes; with the RPM filter off it must fail (CI, both)
- [x] Compare sim and real filtered-gyro spectra in `compare_replay.py`: the log-spectral
      distance per axis (RMS dB, 1-30 Hz), also in the replay check's report. The M7R replay
      sits 9-16 dB off: the real heli's filtered gyro carries far more (vibration left after
      the filters, turbulence) than the sim's calm air

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
- [x] Spool-up and stick-input (doublet, sweep) segments: `segment.inputs()` overlays,
      seconds per log in `fit.json`; the rotor law fits every powered sample, spool-ups
      included (what identifies A). No log has a frequency sweep yet (manoeuvre card 6)
- [ ] **Fits as stages**, each with a coherence / R^2 gate: rotor law, rate gains per
      headspeed, yaw, hover collective per headspeed (thrust law), vibration (2), then the
      closed-loop flap/body sweep by replay (`sweep_replay.py`, `m7r_sweep.py` generalised).
      Open-loop stages done in `fit_heli.py` (from the heli file, pooled over its logs, per
      headspeed -> `fit.json`); the hover stage is gated (5 s out of ground effect), the
      other gates and the closed-loop sweep are open
- [ ] **Joint fit across helis**: the parameters of `PhysicalHeli` (1) fitted on all helis
      together, with the geometry as known input, so each new heli refines the scaling laws
      instead of getting its own constants. Report where a heli is off the shared law
- [x] **Validation report** per heli: replay scores (tracking error, throttle, tail,
      spectrum distance) of the held-out flights, prior against trained, in each heli's
      `TRAINING.md` and `replay.json` (`replay_report.py`, or `train.py --elf`, which also
      drops stages that make the closed loop worse). Done for the M7R, Nimbus, Stratos 700,
      Protos 380 and Stratos 200
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

### Training loop: new logs refine a preset

The goal is that a new log makes a preset better without anyone running scripts by hand,
and never makes it worse. Here "training" means refitting the model's physical parameters.
It is not a neural net: closed-loop data is too thin for that, and a net would not
extrapolate to autorotation or 3D.

- [x] **Fits write the preset**: `train.py` writes `helis/<heli>/model.json` (the
      `--model` format, base `fitted`: the generic `Fitted` class) and the sim flies it by
      name (`--model <heli>`). The `Stratos200` and `M7R` constants moved out of
      `heli/heli_model.py` into `heli/presets/*.json` (bit-identical trajectories)
- [ ] The heli-specific scripts (`m7r_*.py`, `sweep_replay.py`, the Stratos flight lists in
      `common.py`) are not folded into the generic stages yet; the closed-loop flap sweep
      by replay (`m7r_sweep.py`) would be the replay check's next stage
- [x] **`train.py`, one command per heli**: setup group -> fit/validate split -> pooled
      fit with uncertainty -> per-stage proposals -> held-out scores -> model.json and a
      report (`train.json`, `TRAINING.md`) with what each stage proposed, the scores before
      and after, and why it was taken or kept
- [x] **Held-out logs**: `role` per log in the heli file, one in four from its hash (stable
      as logs come in, editable by hand). A proposal is taken only if it scores no worse on
      the held-out logs - on its own stage and every other (the rate gain is the flap
      model's K). `--elf` adds a closed-loop check: the held-out sticks replayed through
      the SITL with the old and the new model (`replay_check.py`)
- [x] **Per-parameter confidence**: jackknife over 20 s blocks (`jackknife.py`) for every
      fitted number; each stage's gates (R^2, coherence, VAF, events, relative error)
      decide whether it proposes anything. What the logs do not identify stays at the prior
      (the heli file's preset, else a size prior), and an unvalidated proposal never
      replaces a preset or an earlier fit
- [ ] Weight the stages by excitation beyond the gates (a calm hover moves the hover
      collective, not the flap time constant): today each stage only uses its own segments
- [x] **Setup fingerprint per log** (`fingerprint.py`): craft, firmware, board, tail type,
      collective range, ESC protocol from the header. Logs pool only within one group; a
      changed heli starts a new one. Gains and headspeed do not split (the replays take the
      gains from each log). Blades, gear and dampers need a hand-set `group`
- [x] **Log store** (`logstore.py`): each log entry carries its source file, SHA-256 and
      consent; `FIT_LOG_STORE` finds the files by hash. CI trains on synthetic flights of a
      known heli instead of private logs (`fitting/tests`)
- [ ] Stretch: once the physical model's error on the held-out logs stops shrinking, a small
      learned model of what is left over (a residual on the body rates or moments) on top of
      the physics. It stays off by default and is judged by the same held-out replays

## 4. Forward flight

No translational lift and the speed derivatives are off: replays need `--hold-alt`, and
forward flight cannot be tested.

- [x] Translational lift: the induced velocity follows forward speed (Glauert), and the
      blade-element thrust its inflow
- [x] Fuselage drag (quadratic, flat-plate area 1.5 % of the disk); its power reaches the
      rotor through the disk tilt in the inflow (a separate parasite-power term counted it
      twice)
- [ ] Flapback, dihedral (Lv, Mu), weathervane (Nv), tail drag
- [ ] Tail effectiveness and yaw damping against forward speed (the tail wag of 11 started
      in fast forward flight)
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
      the fitted presets): fit from sharp-input flights. Probed on the Stratos 700, Nimbus,
      M7R and Protos: the off-axis response to a sharp input is small and mostly unexplained
      (1 - VAF 0.2-10 with or without coupling), the fitted cross gains (-0.12..+0.12 of the
      own-axis input) disagree in sign between helis - not identifiable from these flights.
      Needs one-axis inputs held long enough for the off-axis rate to build up
- [x] Collective to yaw torque on servo tails (`Ncol`), to test the firmware's collective
      precompensation: the yaw stage proposes it from the yaw regression's collective term,
      less the model's own main-rotor torque reaction through its drag (which responds to
      collective too); the held-out yaw prediction includes both
- [ ] Main rotor gyroscopic and tail rotor gyroscopic terms
- [ ] Tail rotor speed and inflow (servo tails), tail authority loss at low rpm
- [x] Tail thrust lag and a yaw mode between the tail and the gyro (`tail_lag`,
      `yaw_mode_hz`, `yaw_mode_zeta`; `fitting/scripts/fit_tail.py`, the M7R: 6 ms and
      16.4 Hz at zeta 0.23) - the phase the yaw loop was missing (11). One lumped mode: which
      part flexes (boom, belt or torque tube, pushrods, mounts) the logs cannot tell
- [ ] The same fit for the other servo-tail helis (needs a replay case each)

## 9. Actuators and power

- [ ] ESC response model (the M7R's analog ESC: R^2 0.73 in the volts form); ESC telemetry
      models beyond HobbyWing V4
- [x] Servos: speed limit, deadband and lag per servo type on each pulse before the swash
      decode (`heli/servos.py`, `--servos ideal|digital|fast|analog`; CI flies digital)
- [ ] Stratos tail motor in forward flight at 3300 rpm: the replay case `stratos_3300`
      holds the tail with 19.3 % (motor volts at 14.6 V) where the real heli used 25.6 %.
      The tail was fitted in hover at 3600 rpm; its thrust is linear in throttle and
      independent of the pack voltage (a motor's ~ (throttle x V)^2). The case judges the
      tail against the sim baseline until this is fitted
- [ ] Servo load (stall torque against the feathering moment), types from measured servos.
      A loaded tail servo slows and saturates on fast reversals, where a wag starts (11)
- [x] A tail servo type of its own (`--tail-servo`): `--servos` set every servo alike,
      while tail servos are often slower and carry more load (11)
- [x] Pack: LiPo discharge curve (open-circuit voltage per cell against state of charge)
      and internal resistance rising towards empty, instead of the straight line
- [x] A pack fitted from logged Vbat/Ibat (`fit_pack.py`, the `pack` stage of `train.py`):
      capacity and internal resistance against the LiPo curve, one starting charge per log,
      leave-one-log-out errors; the capacity only where a log drew a quarter of the pack
- [ ] Pack temperature (resistance and capacity)

## 10. Engineering

- [x] Unit tests of the model alone, without the firmware: trim, whirl frequency, scaling
      with rpm, thrust law, rotor law - in CI on every push (`tests/`, about a minute)
- [x] Integrator: the fixed-step Euler at 4 kHz against the same model at 40 kHz (step
      response within 2 %, mode frequency within 1 %) for the Blade 360, Stratos and M7R;
      repeat for each new size class
- [ ] Keep `heli/` on the Python standard library; numpy/scipy stay in `fitting/`

## 11. Tail wag after an in-flight profile switch

rotorflight-firmware#586: a downshift in fast forward flight (PID profile switch to a lower
headspeed) set off a violent, self-excited 12 Hz tail wag. Each profile is tuned for its own
headspeed, and tail authority goes with rpm^2, so the low-headspeed profile carries ~1.8x the
yaw gains. Switched at once, those gains run on the still-fast rotor until the governor has
brought the headspeed down: for a few seconds the yaw loop gain is ~1.8x the tune.
rotorflight-firmware#591 blends the gains along the headspeed instead (`gov_profile_blend`).

The sim shows it now: the OMP M7R preset with its tail dynamics fitted from
OMP_M7R_20261022_155150 (the #586 log), flying that log's two profiles
(`heli/scenarios/m7r_586.txt`), wags at 13 Hz right after the downshift and saturates the
tail; the wag dies out near 1450 rpm. The real heli: 12 Hz, from ~1800 rpm down to ~1520.
With `gov_profile_blend = ON` the yaw rate stays within 4 deg/s.

The yaw axis was first order (`Nr`, `Nped`) behind one servo lag, which never reaches
-180 deg at 12 Hz: no yaw gain made it oscillate. The log's mixer-to-gyro response (the
yaw setpoint as instrument; the plain estimate returns the controller's inverse) has 2x
the first-order gain at 5-10 Hz and ~60 deg more lag at 12 Hz.

- [x] The missing phase lag at 12 Hz: a 6 ms tail thrust lag and a 16.4 Hz yaw mode
      (zeta 0.23), fitted against a replay of the log through the SITL with the real RPM
      filter, whose 1x notch adds most of the firmware's own lag at 12 Hz (`fit_tail.py`)
- [x] Calibrate on that log: the log's header holds another profile's gains (yaw P/D
      110/20); fitted from the logged terms, profile 1 flies 154/48 at 1500 rpm and
      profile 3 87/24 at 2000 rpm (`m7r_profiles.py`, now absolute). Each holds at its own
      headspeed in the sim, profile 1's wags at profile 3's
- [ ] The sim's wag starts at once and grows ~3x faster than the real one, which began
      ~1 s after the switch near 1800 rpm: the real heli was in fast forward flight, with
      more yaw damping and tail inflow (4) - fly it again once forward flight is modelled
- [x] Profile switching in the bridge: `--profile-switch T_ON[,T_OFF]` drives the PID and
      rate profile adjustments on AUX4 (`adjfunc` is still dropped from dumps), and
      `--debug-mode` / `--debug-log` write the firmware's debug values (`MSP_DEBUG`, e.g.
      `PROFILE_BLEND`) every 20 ms
- [x] A profile-switch check: `--profile-switch` with `--check` judges the yaw rate above
      ~5 Hz for 6 s after the switch (below 20 deg/s). CI job `profile-switch` merges
      rotorflight-firmware#591 onto the SITL and flies the M7R downshift in hover: with
      `gov_profile_blend = OFF` it must fail on the wag, with `ON` pass
- [ ] The upshift, and forward flight once 4 has it
