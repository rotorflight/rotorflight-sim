# Rotorflight simulation

Flight models for the [Rotorflight](https://github.com/rotorflight/rotorflight-firmware)
SITL: the firmware runs as a host process, these models close the loop around it - in
lockstep, one physics step per gyro cycle - so a configuration, a tune or a firmware change
can be flown without a heli.

| Folder | |
|---|---|
| [`heli/`](heli/README.md) | rigid-body helicopter with rotor flapping, a DC-motor rotor-speed model and the pack; drives from the firmware's servo outputs through inverse swash mixing; scripted flights, stick replay of real logs, and the hover check (`--check`). Python standard library only |
| [`TODO.md`](TODO.md) | where the sim goes next: size presets from one physical model, gyro noise and vibration, a fitting pipeline for more helis, forward flight, autorotation |
| [`fitting/`](fitting/README.md) | fitting a heli preset from blackbox logs, and validating it by replaying the real flight through the SITL; gyro chip noise (`sensors/`) and per-heli vibration and fits (`helis/`) |
| [`tests/`](tests/test_model.py) | unit tests of the model alone, without the firmware (`python3 -m unittest discover -s tests`) |

## Quick start

Build the SITL in a firmware checkout (see `src/main/target/SITL/README.md` there):

```sh
make TARGET=SITL                     # -> obj/main/rotorflight_SITL.elf
```

then, from this repository:

```sh
python3 heli/heli_bridge.py <firmware>/obj/main/rotorflight_SITL.elf --check
python3 heli/heli_bridge.py <firmware>/obj/main/rotorflight_SITL.elf --governor 2100 --check
```

`--check` flies a scripted angle-mode hover (arm, spool up, lift off, roll and pitch stick
steps, a collective punch, land) and exits non-zero when it leaves fixed bounds.

## Interface

The SITL's UDP interface is defined by the firmware (`src/main/target/SITL/README.md`):
FDM state in on port 9003, servo/motor outputs back on 9002, UARTs on TCP 5760+n. A packet
format change there needs the matching change here.

## CI

- **Firmware PRs**: the firmware's SITL workflow checks out this repository at a pinned
  commit and runs the hover checks - a change here reaches firmware CI when that pin moves.
- **This repository** (`.github/workflows/ci.yml`): on every push and PR, and every two
  days, runs the model's unit tests, builds the firmware SITL (Linux and Windows) and flies
  the hover, governor, rescue and vibration (RPM filter) checks against it, so model changes
  are tested and firmware regressions show up between releases.

## License

GPL-3.0, as the firmware. The Blade 360 derivatives come from ArduPilot's
`SIM_Helicopter` (GPLv3).
