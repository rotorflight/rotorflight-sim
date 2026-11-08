# Heli flight model

A rigid-body helicopter with first-order rotor flapping, a DC-motor rotor-speed model and
the pack, closed around the SITL in lockstep: every 250 us the model's state goes out as an
FDM packet and the firmware's servo and motor outputs come back. Only the Python standard
library is used.

| File | |
|---|---|
| `heli_model.py` | the model. `Heli`: a Blade 360 (derivatives from ArduPilot's `SIM_Helicopter`, after Walker & Tischler, VFS 2021). `Fitted`: a heli from fitted numbers alone (rate gain, flap stiffness and time constant per axis, rotor and thrust laws, tail), the base of the presets and of trained helis |
| `presets/` | the fitted presets as parameter files on `Fitted`: `stratos200` (200 size, direct drive, motorized tail, from its blackbox logs) and `m7r` (OMP M7R, 700 class, servo tail, 12S, from one sport flight); their `notes` say how each number was found |
| `heli_bridge.py` | starts the SITL, configures it over the CLI, runs the lockstep loop, flies a scripted flight or replays recorded sticks |
| `swash.py` | inverse swash mixing: servo pulses back to roll/pitch/collective, from the servo and mixer configuration read over MSP, so the model feels servo travel, speed limits and the cyclic ring |
| `esc.py` | HobbyWing V4 telemetry from the model on UART3 (`--esc hw4`) |
| `sensors.py` | gyro and accelerometer errors: chip noise (`--gyro-chip`) and rotor-locked vibration (`--vibration`), from the fits in `../fitting/sensors/` and `../fitting/helis/` |
| `wind.py` | steady wind and Dryden turbulence as the air velocity (`--wind`, `--gust`) |
| `servos.py` | servo speed, deadband and lag between the pulses and the swash (`--servos`) |
| `configs.py` | load a real CLI dump (`--dump`), minus board, radio and sensor-alignment settings |

## Run

```sh
python3 heli/heli_bridge.py <rotorflight_SITL.elf>            # scripted angle-mode hover
python3 heli/heli_bridge.py <rotorflight_SITL.elf> --governor 2100 --check
python3 heli/heli_bridge.py <rotorflight_SITL.elf> --acro      # stick doublets in acro
python3 heli/heli_bridge.py <rotorflight_SITL.elf> --rescue --check   # rescue from inverted
python3 heli/heli_bridge.py <rotorflight_SITL.elf> --model stratos200 \
        --dump <cli dump> --cli-file <flown settings> --replay <sticks.csv> --bbl sim.bbl
python3 heli/heli_bridge.py <rotorflight_SITL.elf> --replay-case heli/replays/m7r_cruise --check
```

`--model` takes a preset (`blade360`, `stratos200`, `m7r`), a trained heli by name (its
`../fitting/helis/<heli>/model.json`, written by `../fitting/scripts/train.py` - see
`../fitting/README.md`, Training) or a JSON parameter file that starts from one:

```json
{"base": "m7r",
 "args": {"Lb1s": 900, "pack_cells": 6, "nominal_rpm": 2000},
 "attrs": {"v_hover": 3.0, "rotor_radius": 0.69}}
```

`args` are constructor arguments, `attrs` class constants (the derivatives, `v_hover`,
`rotor_radius`, ...); a misspelt name is refused. `--set NAME=VALUE` goes on top. A
trained heli's file has base `fitted` and all of its numbers, each with its source in
`provenance` (fit, preset, size prior, heli file).

The scripted flight: arm at 6 s, spool up, lift off at 18 s, a roll stick at 21 s and a
pitch stick at 23 s in angle mode, a collective punch at 24 s, descend from 26 s, disarm
at 34 s. `--check` judges it against fixed bounds (lift-off, both stick inputs reach 8-30
deg and come back within 3 deg, no excursion beyond 35 deg, yaw held, lockstep replies,
and with `--governor` headspeed within 3 %) and exits 1 on a failure - the closed-loop
regression gate CI runs. A board-alignment error, for example, fails it.

The scripted pilots fly the collective in proportion to the preset's hover collective
(the Blade 360 215 us above centre, the M7R 133, the Stratos 310), keeping the Blade 360's
ratios, and so do the rescue collectives; under the governor the throttle stick goes to
full. The same flight runs on every preset. A preset with a motorized tail gets
`tail_rotor_mode = MOTORIZED` without a dump, but the Stratos needs its own dump
(`heli/replays/stratos_3300/dump.txt`) to hold the yaw.

`--slope DEG[,UPHILL_DEG]` tilts the ground (rising towards UPHILL_DEG, default 90: across
the skids). The heli is set down on it at 4 s, after the accelerometer calibration on level
ground, flies the scripted flight and lands on it again; the run lasts 45 s so the rotor
runs down. `--check` adds: at rest on the ground at the end, not rolled over (tilt at most
the slope + 6 deg), no slide after touchdown (below 0.5 m). The rotor still turns at the
end with the cyclic left at disarm, so the heli rests flatter than the ground (9 deg on 15).

`--rescue` tests Rotorflight's rescue (`rescue_mode = CLIMB`, flip on, switch on AUX3):
climb to ~25 m in angle mode, roll inverted in acro at full stick and hold inverted hover
collective, centre the sticks, and switch rescue on half a second later. With `--check`
it must be upright (tilt under 30 deg) within 2 s, level (under 15 deg) from 3 s, no
longer sinking (above -0.5 m/s) from 2.5 s, and never below 2 m. Climb and hover
collective are set to the Blade 360 model (600 / 450; it hovers at 0.44). With
`rescue_mode = OFF` the heli falls inverted into the ground and the check fails.

`--gyro-chip CHIP` adds a gyro chip's noise (white noise, bias as a Gauss-Markov process,
rate walk; Allan fit of a soak at rest), `--vibration HELI` a heli's measured vibration:
every main rotor, tail and motor harmonic locked to the model's rotor angle, with its
logged amplitude per axis x (rpm / rpm at the fit)^2, plus the broadband floor. Both are
seeded (`--sensor-seed`); `--vib-scale` scales the vibration. Without a dump, `--vibration`
turns on `feature RPM_FILTER` and sets `main_rotor_gear_ratio` / `tail_rotor_gear_ratio` to
the model's gear and the heli's measured tail ratio, so the filter tracks the same lines.
With `--check`, the scripted flight then also samples the filtered gyro over MSP in the
calm descent (26.5-32 s) and requires its difference to the model's true rates below
3 deg/s RMS per axis. With the M7R's vibration that is 1.2 / 0.3 / 0.4 deg/s with the RPM
filter and 9.2 / 6.1 / 4.0 without it (CI runs both).

**Inflow and autorotation.** The rotor's induced velocity follows momentum theory -
climb, vortex ring (Leishman's fit), windmill brake, forward flight (Glauert, so
translational lift) and ground effect - with the Pitt-Peters time constant. The thrust
takes the inflow through the blade-element law (`thrust_inflow`; the Blade 360 gets one
too, 65 % inflow share of its 0.44 hover collective). In descent the rotor torque takes
the change of induced power from hover, T (v_c + v_i) - T v_h: a steep descent drives the
rotor. Not in powered flight: there the rotor law (fitted from real flights for the
presets) already holds the power, and counting the induced power again loaded the rotor
twice. Rotor
size and hover induced velocity (`rotor_radius`, `v_hover`) are typical values per size.

**Wind.** `--wind SPEED,FROM_DEG` blows a steady wind (the heli faces north), `--gust
SIGMA_W` adds Dryden turbulence (MIL-F-8785C at low altitude: light ~0.5, moderate ~1.5
m/s), seeded (`heli/wind.py`). The model flies relative to the air: inflow and
translational lift, a quadratic fuselage drag (flat-plate area 1.5 % of the disk; its power
reaches the rotor through the disk tilt and the inflow), and the speed derivatives where a preset has them.

**Servos and pack.** `--servos digital|fast|analog` puts servo dynamics between the
firmware's pulses and the swash decode: speed (0.06 / 0.04 / 0.12 s per 60 deg at 10 us
per degree), deadband (1 / 1 / 4 us) and a first-order lag (10 / 6 / 20 ms); `ideal`
(default) passes the pulses through (`heli/servos.py`). The pack follows a LiPo discharge
curve with an internal resistance that rises towards empty.

**Ground.** The heli stands on four skid contact points below its centre of gravity
(plus the rotor head, nose and tail, for a heli that tips over), each a spring and damper
normal to the ground (1 cm static deflection, damping ratio 0.6) and stick-slip friction
along it (mu 0.6: a spring to an anchor on the ground, capped at mu times the normal force,
the anchor sliding beyond). So a landing can bounce, tilt, slide or tip over (beyond
atan(skid half-width / CG height), ~36 deg), the tail's push on the ground is held without
creeping, and the accelerometer feels the ground. Skid sizes, CG height and the radii of
gyration are fractions of the rotor radius. `altitude` is the centre of gravity's height
above its level resting position - the skid clearance while level, not when tilted; the
FDM packet, the baro and the checks use it.

`--governor RPM --autorotation land` tests autorotations: climb to ~65 m, throttle stick
to 15 % (above throttle off, below the governor's 25 % handover - the ELECTRIC governor
enters AUTOROTATION with the motor at idle), hold the headspeed on the collective, flare
from 8 m and land. `--check` requires the motor at idle, the headspeed held at 85-115 %
by the airflow, a steady 3-10 m/s sink, a touchdown below 2 m/s and the rotor above 50 %
at touchdown. `--autorotation bailout` puts the throttle back at 20 m instead and punches
the collective: the governor's BAILOUT state must bring the motor back within 2 s, hold
the headspeed within 5 % and climb away without touching the ground. On the Blade 360 at
2100 rpm: 6.0 m/s sink at 2087-2103 rpm, touchdown at 1.5 m/s; bailout motor back in
0.06 s, climbing at 5.8 m/s, lowest point 18.5 m.

`--hold-alt M` adds a slow virtual pilot (PID on height, ~0.3 Hz, at most +-150 us) to the
replayed collective: the model has no translational lift, so the recorded collective of a
cruising flight lets it sink or climb away. It leaves the attitude loops alone. The
correction is divided by the rotor axis' upward share (a tilted rotor needs more collective
for the same vertical force), fades to 0 through knife edge and flips sign inverted, so 3D
replays hold height too.

The bridge warns when the FC outputs stay exactly unchanged for 5 s with the motor running:
the SITL then stalled and the run should not be scored (a lockstep scheduler stall, fixed in
rotorflight-firmware#596).

Replays fly open loop and drift (the pilot corrected by eye): on the ground the model gets
no cyclic (the pilot keeps it flat), and a heli that comes down tilted more than 20 deg is
set back upright on its skids - the bridge prints how often. The old ground model did this
on every touchdown without saying so.

`--replay` takes a recorded flight's sticks (`t,roll,pitch,yaw,coll,thr` in rcCommand
units) and `--bbl` keeps the SITL's own blackbox, so a replay can be compared with the
real flight log it came from.

### Replay cases

`heli/replays/<case>/` holds a recorded flight with what the real one did: `sticks.csv`,
`tune.txt` (the flown settings) and `case.json` (model, governor, height hold, the scored
window, the real medians and the tolerances). `--replay-case DIR` sets `--model`,
`--governor`, `--hold-alt`, `--replay` and `--cli-file` from it, and with `--check` judges
the flight:

    python3 heli/heli_bridge.py <sitl elf> --replay-case heli/replays/m7r_cruise --check

- airborne: the share of the window the sim flies above 1 m
- upright resets after tilted touchdowns (see above)
- throttle under the governor and the tail trim (yaw mix): medians at the airborne instants
  against the real flight's over the window - the power and torque balance of the preset
- lockstep: steps without a reply

A case can bring the heli's CLI dump (`"dump"`), the real pack voltage (`"vbat"` in `real`:
throttle, and a tail motor, then compare as motor volts - the sim's command scaled by its
pack voltage over the real one) and a sim baseline for a quantity with a known model gap
(`"baseline"`, judged against that with the real value printed beside it).

| case | flight | real | sim |
|---|---|---|---|
| `m7r_cruise` | OMP M7R, 1500 rpm cruise, 100 s | throttle 51.0 % at 45.5 V, tail -21.4 % | 51.9 %, -22.4 % |
| `stratos_3300` | Stratos 200 (Bo105, motorized tail), 3300 rpm, 60 s | throttle 45.4 % at 14.6 V, tail motor 25.6 % | 44.3 %, 19.3 % (baseline) |

CI runs both (about 2 and 1 min on Linux). Model changes broke
the M7R replay twice without another check noticing: double-counted rotor power (too much
throttle and tail) and the skid contact (the heli fell away).

## Frames

The model is FRD body / NED earth. The firmware's stabilized mixer inputs follow
Betaflight's sensor frame: positive roll is right wing down (as FRD), positive pitch nose
down and positive yaw nose left (opposite to FRD q and r); the bridge maps them.
