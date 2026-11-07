# Heli flight model

A rigid-body helicopter with first-order rotor flapping, a DC-motor rotor-speed model and
the pack, closed around the SITL in lockstep: every 250 us the model's state goes out as an
FDM packet and the firmware's servo and motor outputs come back. Only the Python standard
library is used.

| File | |
|---|---|
| `heli_model.py` | the model. `Heli`: a Blade 360 (derivatives from ArduPilot's `SIM_Helicopter`, after Walker & Tischler, VFS 2021). `Stratos200`: a 200-size direct-drive heli with a motorized tail, fitted from its blackbox logs. `M7R`: a 700-class heli with a servo tail (OMP M7R, 12S), fitted from one sport flight. See the class docstrings |
| `heli_bridge.py` | starts the SITL, configures it over the CLI, runs the lockstep loop, flies a scripted flight or replays recorded sticks |
| `swash.py` | inverse swash mixing: servo pulses back to roll/pitch/collective, from the servo and mixer configuration read over MSP, so the model feels servo travel, speed limits and the cyclic ring |
| `esc.py` | HobbyWing V4 telemetry from the model on UART3 (`--esc hw4`) |
| `sensors.py` | gyro and accelerometer errors: chip noise (`--gyro-chip`) and rotor-locked vibration (`--vibration`), from the fits in `../fitting/sensors/` and `../fitting/helis/` |
| `configs.py` | load a real CLI dump (`--dump`), minus board, radio and sensor-alignment settings |

## Run

```sh
python3 heli/heli_bridge.py <rotorflight_SITL.elf>            # scripted angle-mode hover
python3 heli/heli_bridge.py <rotorflight_SITL.elf> --governor 2100 --check
python3 heli/heli_bridge.py <rotorflight_SITL.elf> --acro      # stick doublets in acro
python3 heli/heli_bridge.py <rotorflight_SITL.elf> --rescue --check   # rescue from inverted
python3 heli/heli_bridge.py <rotorflight_SITL.elf> --model stratos200 \
        --dump <cli dump> --cli-file <flown settings> --replay <sticks.csv> --bbl sim.bbl
```

The scripted flight: arm at 6 s, spool up, lift off at 18 s, a roll stick at 21 s and a
pitch stick at 23 s in angle mode, a collective punch at 24 s, descend from 26 s, disarm
at 34 s. `--check` judges it against fixed bounds (lift-off, both stick inputs reach 8-30
deg and come back within 3 deg, no excursion beyond 35 deg, yaw held, lockstep replies,
and with `--governor` headspeed within 3 %) and exits 1 on a failure - the closed-loop
regression gate CI runs. A board-alignment error, for example, fails it.

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
too, 65 % inflow share of its 0.44 hover collective), and the rotor torque the change of
induced power from hover, T (v_c + v_i) - T v_h: a steep descent drives the rotor. Rotor
size and hover induced velocity (`rotor_radius`, `v_hover`) are typical values per size.

**Wind.** `--wind SPEED,FROM_DEG` blows a steady wind (the heli faces north), `--gust
SIGMA_W` adds Dryden turbulence (MIL-F-8785C at low altitude: light ~0.5, moderate ~1.5
m/s), seeded (`heli/wind.py`). The model flies relative to the air: inflow and
translational lift, a quadratic fuselage drag (flat-plate area 1.5 % of the disk) whose
power loads the rotor, and the speed derivatives where a preset has them.

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

`--replay` takes a recorded flight's sticks (`t,roll,pitch,yaw,coll,thr` in rcCommand
units) and `--bbl` keeps the SITL's own blackbox, so a replay can be compared with the
real flight log it came from.

## Frames

The model is FRD body / NED earth. The firmware's stabilized mixer inputs follow
Betaflight's sensor frame: positive roll is right wing down (as FRD), positive pitch nose
down and positive yaw nose left (opposite to FRD q and r); the bridge maps them.
