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
- [ ] Size presets as parameter sets: ~200, 380, 550, 700 class (and 800 / 90 mm blades for
      the high-voltage end); the Blade 360 kept as a reference
- [ ] `--model` takes a preset name or a parameter file
- [ ] CI runs hover, governor and rescue checks per size class, not only the Blade 360

## 2. Gyro noise and vibration

The model's gyro and accelerometer are perfect: the firmware's filters (RPM filter, dynamic
notches, gyro low-pass, D-term filters) are never exercised.

Measure the noise from two sources, so the model separates sensor noise (per gyro chip,
independent of the heli) from vibration (per heli, scaled with rpm):

- [ ] **Allan variance on a real flight controller**: a bench FC at rest for several hours,
      raw gyro and accel at the full sample rate (blackbox with `debug_mode` set to the raw
      gyro, or a dedicated logging build). Extract angle random walk, bias instability and
      rate random walk per axis, plus accel velocity random walk. Repeat per gyro chip in
      use (e.g. ICM-42688-P, BMI270, ICM-42605, MPU6000) and note the temperature drift
      over a warm-up
- [ ] Script `fitting/scripts/allan.py`: overlapping Allan deviation from a log, fit of the
      noise terms, plot; results stored per chip
- [ ] **Vibration from flight logs**: unfiltered gyro (debug gyro channels or a high
      blackbox rate) against headspeed, tail speed and motor eRPM. Order tracking: amplitude
      of the 1x, 2x and blade-pass harmonics of the main rotor, the tail and the motor per
      axis, and the broadband floor, as a function of rpm and collective. The Stratos and M7R
      logs first, then every new heli (see 3)
- [ ] Model: gyro and accel = true value + chip noise (white, bias walk, from the Allan fit)
      + vibration (harmonics locked to the model's rotor, tail and motor rpm, amplitudes from
      the logs) + optional resonance of the frame/FC mount. Seeded, so checks stay repeatable
- [ ] Quantization, range clipping and the sensor's own low-pass, per chip
- [ ] Checks: with vibration on, the RPM filter must remove the tracked harmonics (filtered
      gyro spectrum below a bound), and the hover check still passes; with the RPM filter
      off it must fail
- [ ] Compare sim and real filtered-gyro spectra in `compare_replay.py`

## 3. Refining parameter sets from more flight logs

One heli, one flight, hand-run scripts does not scale. A pipeline that takes logs of any
heli and gives back a preset with its validation:

- [ ] **Heli description file** per heli (`fitting/helis/<name>/heli.json`): geometry and
      hardware as in 1 (rotor, blades, gear, motor, ESC, pack, tail, gyro chip), firmware
      version, CLI dump, and the list of logs with their headspeed and profile. Replaces
      the hard-coded flight lists in `common.py` and `fit_cyclic.py`
- [ ] **Log intake**: split, de-duplicate, decode (blackbox-decode skill), check that the
      log has what the fits need (logging rate, debug mode, rpm and voltage fields, header
      gains) and say what is missing
- [ ] **Segmentation**: label ground, spool-up, hover, cruise, 3D, inverted, autorotation
      and stick-input segments automatically, so each fit uses the parts it can identify
- [ ] **Fits as stages**, each with a coherence / R^2 gate: rotor law, rate gains per
      headspeed, yaw, hover collective per headspeed (thrust law), vibration (2), then the
      closed-loop flap/body sweep by replay (`sweep_replay.py`, `m7r_sweep.py` generalised)
- [ ] **Joint fit across helis**: the parameters of `PhysicalHeli` (1) fitted on all helis
      together, with the geometry as known input, so each new heli refines the scaling laws
      instead of getting its own constants. Report where a heli is off the shared law
- [ ] **Validation report** per heli: replay scores (tracking error, whirl band power,
      throttle, yaw mixer) sim vs real per profile, written to a file and committed with
      the preset
- [ ] **Regression**: a small set of public, short log excerpts and their expected scores,
      replayed in CI (scheduled run, it is slow), so a model change that worsens a fitted
      heli shows up
- [ ] **Contributing logs**: a page on how to log a flight for the sim - blackbox rate and
      debug mode, CLI dump, the heli description, and a manoeuvre card (hover, sharp
      roll/pitch/collective doublets, frequency sweeps, a few seconds of each headspeed,
      piros, an autorotation if the heli does them). Logs stay out of the repository unless
      the owner agrees
- [ ] Test flights the current fits are missing: sharp roll/pitch/collective inputs (control
      power, flap time constant, collective sensitivity), on a 380-550 class heli as well,
      so the scaling rests on three sizes

## 4. Forward flight

No translational lift and the speed derivatives are off: replays need `--hold-alt`, and
forward flight cannot be tested.

- [ ] Translational lift (inflow with forward speed), flapback, dihedral (Lv, Mu),
      weathervane (Nv), fuselage and tail drag
- [ ] Fit them from logs with GPS speed (blackbox GPS fields) - add a GPS-logged flight to
      the manoeuvre card
- [ ] Replays without `--hold-alt` hold height in cruise; a forward-flight and a piro check

## 5. Autorotation

The rotor only loses speed to drag; upward airflow never drives it. Rotorflight's
autorotation bailout and the governor's autorotation handling cannot be tested.

- [ ] Rotor torque from inflow (descent drives the rotor), so the rotor speed holds in an
      autorotation and decays with collective at the flare
- [ ] Vortex ring state in steep powered descent (stretch)
- [ ] An autorotation check: cut the motor at height, enter, flare, land; and the bailout
      (governor spools back up within its limits)

## 6. Ground

The heli snaps level on the ground, and touchdown zeroes its velocity.

- [ ] Ground effect (thrust gain below about one rotor diameter)
- [ ] Skid contact: spring, damping and friction per contact point, so tilt on landing,
      sliding, tip-over and a slope landing can happen
- [ ] Ground resonance (stretch, needs a fuselage/hub mode)

## 7. Wind and turbulence

The current disturbance is a random moment on the body, with `turb_hz` tuned per heli.

- [ ] Wind (steady and gusts) as air velocity, with turbulence from a Dryden or von Karman
      spectrum scaled to the rotor size; the rotor responds through the speed derivatives (4)
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
- [ ] Servos: rate limit, deadband, lag and load, per servo type (`servo_tau` is off)
- [ ] Pack: discharge curve, internal resistance with temperature and state of charge,
      instead of the straight line

## 10. Engineering

- [ ] Unit tests of the model alone, without the firmware: trim, whirl frequency, scaling
      with rpm, thrust law, rotor law - fast, in CI on every push
- [ ] Integrator: check the fixed-step Euler at 4 kHz against a higher-order step for the
      stiffer presets and size classes
- [ ] Keep `heli/` on the Python standard library; numpy/scipy stay in `fitting/`
