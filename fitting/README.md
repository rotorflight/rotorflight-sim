# Fitting a heli preset from blackbox logs

How the `Stratos200` preset of [`../heli/heli_model.py`](../heli/heli_model.py) was fitted from
the blackbox logs of a 200-size direct-drive heli with a motorized tail (Stratos 200 in a
Bo105 fuselage, flights of 2026-10-08), and validated by replaying the recorded sticks
through the Rotorflight SITL with the real configuration. The scripts are kept so the fit
can be redone, or repeated for another heli: the flight lists at the top of
`scripts/common.py` and `scripts/fit_cyclic.py` name the Stratos flights, replace them for
your own logs. The logs themselves are not part of this repository.

Needs numpy, scipy and matplotlib, and the `blackbox-decode` skill of
[rotorflight-agent-skills](https://github.com/rotorflight/rotorflight-agent-skills) for
decoding (a checkout next to this repository, or `BLACKBOX_DECODE_DIR` pointing at its
`skills/blackbox-decode/scripts`). Run everything from this folder, with `python -I` (the
logs are untrusted input).

## Data

```sh
python -I scripts/split_logs.py data/split <RTFL_BLACKBOX_LOG_...BBL> ...   # one .bbl per log, duplicates skipped
node <skill>/bbl.js decode data/split/20261008_180641_00.bbl -o data/split/20261008_180641_00
```

`scripts/common.py` loads `data/split/<name>.0.npy` (bbl.js) or `<name>.npz` (the skill's
pure-Python fallback decoder); `FIT_DATA_DIR` moves the folder. The flights used:

| log | headspeed | gains (roll P/I/D/F/B) | airborne |
|---|---|---|---|
| 160146_11, 160146_12 | 3300 | 50/120/12/95/15 | ~120 s, ~100 s |
| 172721_00 | 3600 | 50/120/12/95/15 | 50 s |
| 174938_00 | 3600 | 45/100/20/95/15 | 64 s |
| 180641_00 | 3600 | 45/85/100/95/15 | 73 s |
| 182311_01 | 3600 | 45/85/120/95/15 | 101 s |

Mostly gentle hovering in acro. The CLI dump is one day older than the flights: the flown
gains come from each log's header (`scripts/header_overrides.py`).

## What the logs identify

**Main rotor and motor** (`scripts/fit_rotor.py`), direct drive:

    dw/dt = A u V - B w - w^2 (k0 + k1 c^2)
    A 987.7, B 16.84, k0 7.0e-4, k1 1.10e-2          R^2 0.994 on the six flights

Fitted as motor volts `u V = (B w + w^2(...) + dw/dt) / A` - under the governor `dw/dt`
alone is noise. A/B gives Kv ~560 rpm/V; hover at 3600 rpm takes 47 % throttle at
15.2 V with the motor at 90 % of its no-load speed. Ground runs (`SPOOL` in the script)
sit a constant ~1.2 V off and are left out.

**Cyclic rate gain** (`scripts/fit_axis.py`): closed-loop flights, so the plant is
estimated with the setpoint as instrument, G = S_yr / S_ur. Coherent only at low
frequency (roll to ~1 Hz, pitch to ~3 Hz): roll 5.07, pitch 4.74 rad/s per unit mixer
input. Above that the estimate drifts towards -1/C (the inverse controller) and says
nothing about the plant.

**Yaw, motorized tail** (`scripts/fit_yaw.py` and a 0.5-8 Hz regression): more tail motor
turns the nose right (CW main rotor, reaction nose left) - checked by the correlation of
tail motor and later yaw acceleration, -0.64..-0.81 at 40 ms in all flights. 66 rad/s^2
per unit tail motor, ~35 ms motor lag, weak rate damping; hover balance at 21.5-23.5 % tail.

**Not identifiable** from these flights: the flapping/body stiffness (Lb1s, Ma1s) and the
flapping time constant, roll/pitch cross coupling, and the speed derivatives (no velocity
in the logs). The ~6 Hz whirl of this heli lives exactly there - see the replay below.
`scripts/fit_cyclic.py` (equation error) and `scripts/fit_cyclic_freq.py` (2x2 frequency
fit) are kept as the attempts that did not work: noise and the closed loop leave the
coupled 4-state model unidentifiable.

## Replay through the SITL

```sh
python -I scripts/export_sticks.py 20261008_180641_00 1.0 50.0 replay/sticks.csv
python -I scripts/header_overrides.py 20261008_180641_00 replay/flown.txt
python ../heli/heli_bridge.py <rotorflight_SITL.elf> --model stratos200 \
    --dump <cli dump> --cli-file replay/flown.txt --replay replay/sticks.csv --bbl replay/sim.bbl
node <skill>/bbl.js decode replay/sim.bbl -o replay/sim
python -I scripts/compare_replay.py 20261008_180641_00 replay/sim.0.npy
```

The SITL runs the real configuration (dump minus board, radio and alignment settings, see
`../heli/configs.py`) with the gains actually flown, the pilot's recorded sticks as MSP RC,
lockstep with the model, and writes its own blackbox; the comparison aligns both logs on
the sticks and compares the same airborne window. Run one SITL at a time (fixed ports).

Getting there found and fixed:
- `align_board_yaw = 180` in the dump inverted roll and pitch in the SITL (its sensors
  already come in body axes) - alignment settings are filtered now
- an in-flight profile switch (`adjfunc`) picked another profile - adjfuncs are filtered,
  the flown settings come from the log header
- the SITL sent motors as a PWM pulse with the analog `min_throttle` offset, and did not
  build the RPM filter - firmware fixes in rotorflight-firmware#596
- `blackbox_rolling_erase = ON` from the dump - off for sim runs

Calm air, flight 180641, 35 s airborne: rotor, governor, tail and yaw match (yaw tracking
error RMS 1.85 real / 1.90 sim deg/s, tail 23.6 / 22.2 %), but roll and pitch track far
better in the sim (1.4 vs 6.9 deg/s): the real heli's ~5.9 Hz whirl is missing.

## Closed-loop identification of the whirl (`scripts/sweep_replay.py`)

Each case replays the first 49 s of flight 180641 through the SITL and is scored against
the real flight (`FIT_DUMP=<cli dump> python -I scripts/sweep_replay.py ...`).

1. **Calm air, flap stiffness L (= Lb1s = Ma1s) x flap time constant tf**: no case shows
   the whirl, not even a plant mode at 6 Hz with damping 0.17 - the firmware's PIDs damp
   it, and nothing excites it.
2. **Servo lag 15-45 ms**: no whirl either; tracking just gets slightly worse.
3. **With turbulence** (random roll/pitch moments): the closed loop rings at the flap/body
   mode, and the peak follows sqrt(L) - L 800: 4.4-4.6 Hz, 1400: 5.7-6.1 Hz, 2500: 7.9 Hz.
   The real 5.86 Hz pins **L ~ 1400**; the ratio of 4-8 Hz power to tracking error puts
   **tf at ~0.05 s**.
4. **5 Hz turbulence band, pitch stiffness Ma1s 2000** (roll Lb1s stays 1400) bring the
   roll/pitch ratio to the real one. With turbulence 4.3 rad/s^2 (the preset's defaults):

| | real | sim |
|---|---|---|
| roll / pitch gyro peak | 5.86 / 5.86 Hz | ~6.0 / ~5.7 Hz |
| roll / pitch 4-8 Hz (deg/s) | 5.56 / 3.52 | 5.55 / 3.78 |
| roll / pitch tracking error RMS (deg/s) | 6.88 / 4.91 | 6.75 / 5.15 |
| yaw tracking error RMS (deg/s) | 1.85 | 1.96 |

So the whirl reads as the heli's flap/body mode near 6 Hz, lightly damped once the rate
loop closes around it, and kept going by disturbances - not a loop instability. The real
peak is sharper (a less damped mode) and the sim's pitch has too much above 8 Hz.

## Step response (`scripts/step_overlay.py`)

The logs hold one sharp stick input: pitch to ~120 deg/s at 56.4 s in flight 182311.
Replayed without turbulence, the sim peaks 10 % under the setpoint and 40 ms late, the
real heli 7 % over at the setpoint peak; over the 4 s sequence the sim's pitch rate stays
within 10 deg/s RMS of the real one (real signal 35 deg/s RMS). The model is a little
sluggish in pitch - likely the flap time constant, which the whirl damping pins. A test
flight with sharp roll and pitch inputs would settle it.

## Predictive check

Same model and turbulence, each flight's own gains: roll whirl 4-8 Hz real 10.7 (D 12) ->
6.2 (D 120) deg/s, sim 6.4 -> 4.9. Right direction, about half the size (x0.77 vs x0.57):
the sim can rank gain changes for the whirl, not yet size them.

## Rotor speed

With the same gains, the whirl sits at 4.94 Hz at 3300 rpm (flights 160146_11/12) and
6.05 Hz at 3600 - +22 % for +9 % rpm - and the heli hovers on more collective at 3300
(0.70-0.73) than thrust ~ rpm^2 x collective allows (0.757 from 0.636 at 3600).

- **Thrust**: blade element with induced inflow (`thrust_inflow`): the hover collective
  goes as a/wr^2 + b/wr; the two headspeeds give a 0.18, b 0.46, so inflow carries most of
  the hover pitch. Around hover this makes collective ~1.55x more powerful than the linear
  law (2.4 vs 1.6 g per unit). The baro vario cannot confirm it: regressing vertical
  acceleration on collective gives 1.3-3.8 g, R^2 ~0.1.
- **Flap/body mode** (`stiff_rpm_exp`, `tf_rpm_exp`), replaying 160146_11 (3300 rpm, 34 s
  airborne, `turb=4.3`):

| stiffness ~ rpm^k, tf ~ 1/rpm^m | roll centroid | pitch centroid | roll 4-8 Hz |
|---|---|---|---|
| real | 4.94 Hz | 4.88 Hz | 10.4 deg/s |
| k 0, m 0 (fixed) | 5.94 | 6.26 | 6.6 |
| k 0, m 1 | 5.99 | 6.56 | 6.8 |
| k 2, m 0 | 5.09 | 5.85 | 6.8 |
| **k 2, m 1** (preset) | 5.13 | 5.89 | 7.1 |
| k 4, m 0 | 4.58 | 5.45 | 7.0 |

  k = 2 (hub stiffness from centrifugal and thrust terms) puts the roll mode within 4 %;
  tf barely matters. Pitch stays high and the amplitude low at 3300. At 3600 the preset
  is unchanged: flight 180641 replays at 4-8 Hz roll 5.85 / pitch 3.70 deg/s (real 5.93 /
  3.77), tracking error 6.95 / 5.13 (real 7.17 / 5.23).

`scripts/compare_replay.py` aligns sim and real by cross-correlating the stick motion over
any offset, so a replay can start anywhere in the flight - start it before the spool-up:
arming mid-flight with the throttle up leaves the rotor stopped.

## Open

- a less damped mode and lower-band turbulence (the real peak is sharper; the whirl's
  amplitude at 3300 rpm and the effect of D gain are both undersized)
- pitch at other headspeeds (sim 5.9 Hz vs real 4.9 Hz at 3300)
- a test flight with sharp roll/pitch/collective inputs, for control power, the flap time
  constant and the collective sensitivity
