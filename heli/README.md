# Heli flight model

A rigid-body helicopter with first-order rotor flapping, a DC-motor rotor-speed model and
the pack, closed around the SITL in lockstep: every 250 us the model's state goes out as an
FDM packet and the firmware's servo and motor outputs come back. Only the Python standard
library is used.

| File | |
|---|---|
| `heli_model.py` | the model. `Heli`: a Blade 360 (derivatives from ArduPilot's `SIM_Helicopter`, after Walker & Tischler, VFS 2021). `Stratos200`: a 200-size direct-drive heli with a motorized tail, fitted from its blackbox logs (see the class docstring) |
| `heli_bridge.py` | starts the SITL, configures it over the CLI, runs the lockstep loop, flies a scripted flight or replays recorded sticks |
| `swash.py` | inverse swash mixing: servo pulses back to roll/pitch/collective, from the servo and mixer configuration read over MSP, so the model feels servo travel, speed limits and the cyclic ring |
| `esc.py` | HobbyWing V4 telemetry from the model on UART3 (`--esc hw4`) |
| `configs.py` | load a real CLI dump (`--dump`), minus board, radio and sensor-alignment settings |

## Run

```sh
python3 heli/heli_bridge.py <rotorflight_SITL.elf>            # scripted angle-mode hover
python3 heli/heli_bridge.py <rotorflight_SITL.elf> --governor 2100 --check
python3 heli/heli_bridge.py <rotorflight_SITL.elf> --acro      # stick doublets in acro
python3 heli/heli_bridge.py <rotorflight_SITL.elf> --model stratos200 \
        --dump <cli dump> --cli-file <flown settings> --replay <sticks.csv> --bbl sim.bbl
```

The scripted flight: arm at 6 s, spool up, lift off at 18 s, a roll stick at 21 s and a
pitch stick at 23 s in angle mode, a collective punch at 24 s, descend from 26 s, disarm
at 34 s. `--check` judges it against fixed bounds (lift-off, both stick inputs reach 8-30
deg and come back within 3 deg, no excursion beyond 35 deg, yaw held, lockstep replies,
and with `--governor` headspeed within 3 %) and exits 1 on a failure - the closed-loop
regression gate CI runs. A board-alignment error, for example, fails it.

`--replay` takes a recorded flight's sticks (`t,roll,pitch,yaw,coll,thr` in rcCommand
units) and `--bbl` keeps the SITL's own blackbox, so a replay can be compared with the
real flight log it came from.

## Frames

The model is FRD body / NED earth. The firmware's stabilized mixer inputs follow
Betaflight's sensor frame: positive roll is right wing down (as FRD), positive pitch nose
down and positive yaw nose left (opposite to FRD q and r); the bridge maps them.
