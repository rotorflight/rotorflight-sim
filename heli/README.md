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

`--hold-alt M` adds a slow virtual pilot (PID on height, ~0.3 Hz, at most +-150 us) to the
replayed collective: the model has no translational lift, so the recorded collective of a
cruising flight lets it sink or climb away. It leaves the attitude loops alone. The
correction is divided by the rotor axis' upward share (a tilted rotor needs more collective
for the same vertical force), fades to 0 through knife edge and flips sign inverted, so 3D
replays hold height too.

The bridge warns when the FC outputs stay exactly unchanged for 5 s with the motor running:
the SITL then stalled (rare; open in the firmware) and the run should not be scored.

`--replay` takes a recorded flight's sticks (`t,roll,pitch,yaw,coll,thr` in rcCommand
units) and `--bbl` keeps the SITL's own blackbox, so a replay can be compared with the
real flight log it came from.

## Frames

The model is FRD body / NED earth. The firmware's stabilized mixer inputs follow
Betaflight's sensor frame: positive roll is right wing down (as FRD), positive pitch nose
down and positive yaw nose left (opposite to FRD q and r); the bridge maps them.
