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

## OMP M7R (700 class, servo tail): `scripts/m7r_*.py`

One 333 s sport flight (Rotorflight 4.6, Nexus XR, 12S, analog ESC under the governor,
1500 rpm cruise / 1800 and 2000 rpm 3D / autorotation profiles), no CLI dump. The
rotorflight-presets repository has no M7 preset; the replay uses the log header's own
gains and rates.

```sh
node <skill>/bbl.js decode OMP_M7R_<date>.bbl -o data/m7r
python -I scripts/m7r_fit.py data/m7r.0.npy                  # open-loop fits
python -I scripts/m7r_profiles.py data/m7r.0.npy               # gains of profiles 2 and 3
python -I scripts/m7r_inputs.py data/m7r.0.npy replay/m7r [1|2|3]   # CLI file + sticks per profile
python -I scripts/m7r_sweep.py [-p 1|2|3] <elf> data/m7r.0.npy replay/m7r replay/m7r/sweep L=700,1500 tf=0.05,0.1
```

- **Rotor law**: R^2 0.73 in the volts form (the Stratos 0.994): the analog ESC has a
  throttle response of its own. Through the replay it holds: 47.7 % throttle at 49.2 V in
  the sim, 51.1 % at 45.5 V in the log - the same motor volts.
- **Rate gains** at 1500 / 1800 / 2000 rpm: roll 5.1 / 6.7 / 7.9 (~rpm^1.5), pitch
  5.8 / 5.9 / 6.3 (~rpm^0.5) rad/s per mixer input, coherence 0.9+. On average ~rpm, as
  flapping theory gives with stiffness ~rpm^2 and tf ~1/rpm - the model's former rpm^2 on
  the cyclic input made it ~rpm^3, so `ctrl_rpm_exp` (0 here; the older presets keep 2).
- **Hover**: collective 0.346 at 1500 rpm (cruise). The 3D profiles' calm moments (0.280 at
  1800, 0.179 over 3 s at 2000) mix in forward flight and fit no single thrust law; the
  preset takes the Stratos' inflow share (72 %), which gives 0.273 at 1800.
- **Servo tail**: 38 rad/s^2 per unit at 1500 rpm (R^2 0.68 in the cruise flying; the 3D
  parts fit poorly), damping -2 /s.
- **Pack**: 12S, 50.4 V full, 47 mOhm, ~4.5 Ah.
- **Replay** of the cruise profile (real 242-318 s) with `--hold-alt 10`, scored where the
  sim flies above 1 m - the flap/body sweep:

| Lb1s = Ma1s, tf | roll err | pitch err | roll 1.5-3 Hz | pitch 1.5-3 Hz |
|---|---|---|---|---|
| real | 4.7 | 4.1 | 2.2 | 1.9 |
| 300, 0.05 (no height hold) | 9.1 | 7.8 | 6.5 | 3.4 |
| **700, 0.05** (preset) | 4.8 | 3.6 | 3.0 | 2.3 |
| 700, 0.10 | 12.8 | 3.4 | 2.0 | 1.9 |
| 1500, 0.05 | 2.5 | 3.0 | 1.0 | 1.6 |
| 3000, 0.05 | 2.0 | 2.7 | 0.6 | 1.4 |

  Softer rings at 2.3 Hz; stiffer tracks better than the real heli in calm air (as with
  the Stratos, real disturbances are missing). Yaw tracking 11.1 vs 9.3 deg/s, yaw mixer
  -21.3 vs -21.4 %.

- **3D profiles**: the header holds profile 1 only. `m7r_profiles.py` takes profiles 2
  and 3 from the logged PID terms - each term against its input, as a ratio to profile 1
  (which gives back its header gains within 1-2). Rates and stop gains are profile 1's;
  roll D rises to ~32, yaw P falls to 71 / 61, collective rate 114. The replays (preset,
  `--hold-alt 10`, which divides its correction by the rotor axis' upward share so it
  holds inverted too) fly 3D including inverted hover; scored above 1 m (55 % of them -
  open-loop sticks lose the heli in the fast parts the pilot corrected by eye):

| profile | roll err | roll 1.5-3 Hz | pitch err | pitch 1.5-3 Hz | yaw err | throttle |
|---|---|---|---|---|---|---|
| 2, 1800 rpm sim / real | 8.2 / 7.9 | 4.2 / 4.1 | 5.4 / 9.1 | 2.6 / 4.7 | 5.5 / 6.0 | 61.1 / 60.6 % |
| 3, 2000 rpm sim / real | 6.7 / 7.8 | 4.5 / 4.6 | 12.1 / 14.4 | 9.4 / 9.6 | 4.1 / 5.5 | 70.6 / 73.4 % |

  The throttle at three headspeeds confirms the rotor law. `m7r_sweep.py` runs a replay
  up to three times: some long replays go wrong in the SITL (outputs freeze or the
  blackbox ends early or stays empty - a lockstep scheduler stall, fixed in
  rotorflight-firmware#596).

Found on the way: the SITL read a 12S pack as 36.3 V - its emulated ADC clipped at the
generic scale's full range; fixed in rotorflight-firmware#596 (scale 250, 82.5 V).

## Heli files (`helis/<heli>/heli.json`)

One file per heli: hardware (pack, gear and tail ratios, motor, tail type, gyro chip,
geometry), firmware, and its logs with the headspeeds flown. The scripts below take it
instead of hard-coded flight lists and write their results next to it. Logs are decoded
with `bbl.js decode <file> -o $FIT_DATA_DIR/<stem>` and found by stem. So far: the Stratos
200 and the OMP M5, M6 and M7R (Rotorflight 4.6, Nexus XR, 12S, servo tails). Their
geometry fields are empty - needed for the physical model (TODO 1).

## Log intake (`scripts/intake.py`)

```sh
FIT_DATA_DIR=<decoded logs> python -I scripts/intake.py helis/<heli>/heli.json <log.bbl> ... [--dry-run]
```

Decodes the files, skips logs the heli file already has (by content, also across files),
reports what each log can feed and what is missing, and adds the new logs to the heli
file. How to fly and log for it: [`../CONTRIBUTING_LOGS.md`](../CONTRIBUTING_LOGS.md).

## Training (`scripts/train.py`)

```sh
export FIT_DATA_DIR=<decoded logs> FIT_LOG_STORE=<folders with the .bbl files>
python -I scripts/intake.py helis/<heli>/heli.json <new logs.bbl> ...   # decode, check, add to the manifest
python -I scripts/train.py helis/<heli>/heli.json [--elf <rotorflight_SITL>] [--dry-run]
python ../heli/heli_bridge.py <rotorflight_SITL> --model <heli> ...      # fly the trained heli
```

"Training" here is refitting the model's physical numbers from more logs, never at the
cost of predicting the heli worse. One command per heli, whenever logs come in:

1. **Setup** (`fingerprint.py`): each log's craft name, firmware, board, tail type,
   collective range and ESC protocol from its header form a setup group. Logs pool only
   within a group, so a moved flight controller or a rebuilt heli does not pull the old
   numbers. The newest group with three flights or more is trained (`--group` picks
   another). Gains and headspeed do not split a group. Blades, gear or dampers cannot be
   seen in the header: set `"group"` on the log entries by hand.
2. **Split**: every log entry has a `role`, `fit` or `validate`. New logs get one from
   their hash (one in four validates), so the split stays put as logs are added, and can
   be edited. Validate logs are never fitted.
3. **Fit with uncertainty**: `fit_heli.py`'s stages and `fit_flap.py` on the fit logs, then
   a jackknife (`jackknife.py`): the airborne time is cut into 20 s blocks dealt into 8
   folds, each fold left out once and everything refitted. The spread of the refits is
   the standard error of every number in `fit.json` / `flap.json` (`uncertainty`, `se`);
   a number one fold cannot produce at all is `unstable`.
4. **Proposals** (`model_params.py`): each stage proposes the model numbers it identifies
   if its gates pass - rotor (`motor_fit`; R^2 >= 0.6, A and B within 25 %), thrust
   (`thrust_inflow` from the hover collective; a gated hover, within 10 %), rate
   (`rate_gain_*`, and the rpm law `ctrl_rpm_exp` from two headspeeds; coherence >= 0.6,
   within 20 %), flap (L, tf and K per axis from the sharp inputs; VAF >= 0.85 over 5
   inputs, within 35 %), yaw (`Nped`, `Nr`, `Ncol`, `yaw_trim`, servo tails; R^2 >= 0.3;
   `Ncol` is the collective term of the yaw regression less the model's own main-rotor
   torque reaction, which responds to collective through the drag), power (hover watts),
   pack (`pack_r`, `pack_ah` from `fit_pack.py`: Vbat/Ibat against the LiPo curve, one
   starting charge per log; resistance within 30 %, capacity within 25 % and only where a
   log drew a quarter of the pack). Proposals are made against the model as it stands
   when their stage comes up. They go on top of the current model: `model.json` if it was trained on
   the same setup, else the prior - the preset the heli file names, or a generic heli of
   its size class (the M7R's or the Stratos' dynamics, the size's rotor and pack) with
   what the heli file knows of its hardware.
5. **Held-out check** (`holdout.py`): each proposal predicts what the held-out logs
   measured - throttle for the logged headspeed and its change over every powered sample
   (rotor), the hover collective (thrust), the rate per cyclic (rate), the rate through
   the sharp inputs (flap), the yaw acceleration with the torque reaction and the tail
   trim (yaw), hover watts (power), Vbat for the drawn charge (pack).
   A proposal is taken if it scores no worse on its own stage and no more than 2 % worse
   on every other:
   the rate gain is also the flap model's K, and the M7R's spectral and sharp-input gains
   disagree on pitch. Stages that depend on the one judged are proposed again on top of it
   first (yaw on the rotor law's torque reaction, the rotor law on the thrust law's hover
   point), so a new rotor law is not refused for what the yaw stage will refit after it. A proposal the held-out logs cannot judge replaces only a size
   prior, never a preset or an earlier fit.
6. **Replay check** (`replay_check.py`, `--elf`): the held-out logs' longest upright
   stretch at one headspeed is replayed through the SITL with the old and the new model
   (the log's gains from its header, the heli's CLI dump if the store has it, the height
   hold) and scored against the real flight - tracking error per axis, throttle and tail.
   If the new model scores more than 10 % worse, nothing changes. The open-loop check
   above cannot see what the closed loop makes of the numbers: the open-loop flap fit
   once replayed worse than the M7R preset on pitch (see the flap section below).
7. **Report**: `model.json` (base `fitted`, every number with its source in `provenance`)
   if anything changed, and `train.json` / `TRAINING.md`: what each stage proposed, the
   held-out scores of the prior, the current and the new model, and why each proposal was
   taken or kept.

`logstore.py` keeps the manifest: each log entry has its source file, its SHA-256 and the
owner's consent (`private` unless they agree to publish). `FIT_LOG_STORE` names the
folders the files are found in, by hash (`logstore.py check|backfill|decode`). The logs
stay out of the repository.

**First training** (2026-11-09), all helis from their priors (the presets for the Stratos
200 and the M7R, size priors for the rest). Held-out scores prior -> new where the stage
was taken (lower is better, units in `holdout.py`); "kept": proposed and refused, "-": no
proposal (gates):

| heli | fit / held-out logs | rotor | thrust | rate | flap | yaw | power | pack |
|---|---|---|---|---|---|---|---|---|
| OMP M5 | 1 / 1 | 9.59 -> 1.15 | - | kept | kept | 2.34 -> 2.20 | - | - |
| OMP M6 | 1 / 1 | kept | - | kept | - | - | - | - |
| OMP M7R | 2 / 1 | 6.04 -> 1.74 | - | kept | 0.061 -> 0.060 | 1.00 -> 0.70 | - | - |
| Stratos 200 | 5 / 1 | kept | - | 0.126 -> 0.006 | - | - | - | kept |
| ALZRC Devil 505 | 2 / 0 | - | - | not validated | kept | not validated | - | - |
| MSH Protos 380 | 6 / 2 | - | - | 0.129 -> 0.101 | kept | 0.588 -> 0.584 | - | - |
| Nimbus | 12 / 6 | 15.6 -> 2.29 | 0.014 -> 0.012 | 0.41 -> 0.065 | kept | 0.43 -> 0.28 | not validated | 0.198 -> 0.177 |
| Stratos 700 | 14 / 8 | kept | kept | 0.40 -> 0.066 | 0.46 -> 0.061 | kept | 0.20 -> 0.031 | 0.30 -> 0.15 |

- The helis with many logs (Stratos 700, Nimbus) take most stages; the OMP sport flights
  (one fit log each) mostly the rotor law and the yaw.
- Rate and flap often refuse each other: on real logs the spectral gain and the sharp
  inputs' K disagree, most on pitch (M7R 6.54 vs 5.60) - the closed loop shows in one of
  them. Whichever is taken, the other stage must not get worse.
- The rotor law's throttle error falls from 6-16 % to 1-2 % where it is taken. On the
  Stratos 700 and the M6 it is refused for the yaw: its drag against collective drives
  the model's torque reaction, which then predicts the held-out yaw worse, and the yaw
  stage's own refit does not recover it (the Stratos 700's yaw fit scores worse than its
  prior). The firmware's collective-to-tail precompensation in the logged yaw mixer is a
  likely suspect - the model has none.
- The M7R's rotor law comes out with ~10x the preset's A (a much faster spool-up): its
  analog ESC regulates on its own, so throttle is not motor volts - the held-out
  spool-ups agree with the fit, but treat its rotor dynamics with care.
- Packs: Nimbus 6S 3.1 Ah, 26 mOhm; Stratos 700 12S 26.9 mOhm (its capacity, 6.4 +- 1.3 Ah,
  misses the 25 % gate). The Stratos 200's (0.55 Ah, 105 mOhm) predicts its held-out
  flight worse than the preset's and is kept out.
- The Stratos 200's newest log is another setup (Foxeer F722, 4.6, 5000 rpm) and trains
  separately once it has company; the preset's own setup (Nexus XR, 4.7) was trained.
  The Protos folder holds two setups too: its June 1-3 logs carry the craft name "Devil
  505" on 4.5 with collective range +-1008 (the Devil 505's is +-1250) - the flight
  controller moved over - and the June 22 logs (4.6, +-1083) were trained.
- The rate stage is judged by the same spectral estimator on the held-out logs; where no
  flap fit cross-checks it (Stratos 200), only the replay check is independent.

Replay check of the M7R (`replay_check.py`, its held-out 3D flight replayed from the
spool-up, scored over the cruise profile at 1500 rpm, 253-324 s), real / preset / trained:

| | roll err | pitch err | yaw err | throttle | tail | score |
|---|---|---|---|---|---|---|
| real | 4.62 | 4.08 | 9.59 | 51.2 % | -24.1 % | |
| preset | 2.88 | 2.87 | 6.30 | 50.0 % | -21.2 % | 0.82 |
| trained | 1.80 | 3.47 | 11.23 | 50.4 % | -21.2 % | 0.79 |

Pitch tracking and throttle move towards the real heli; roll tracks better than real (the
sim's calm air, as the open-loop flap values did before); the yaw overshoots by about as
much as the preset undershot. On balance no worse, so the trained model stands - but it
is a mixed result, and one 70 s stretch: the replay check needs longer held-out cruise.

**Tests** (`fitting/tests`, CI job `training`): a heli of known numbers is flown by a
simple autopilot in the model (`tests/synthetic.py`: governor, height hold, angle-mode
hovers, stick noise, doublets) and logged in the blackbox fields. The stages must recover
its numbers (rate gains within 4 %, flap L and tf within 12 %, rotor law within 8 %), the
held-out scoring must prefer them to a perturbed heli, training from a size prior must
reach them, and training on another heli's logs must change nothing.

Found by those tests and fixed on the way:

- The spectral rate gain was the median |G| over 0.3-4 Hz. On helis whose flap/body mode
  sits in that band (2-6 Hz) the resonance lifted it 15-25 % (the synthetic heli: 6.9 /
  6.5 for 6.0 / 5.2). It now fits the mode's magnitude K L / |L - w^2 + j w / tf| to |G|,
  weighted by coherence (`gain_fit`; the median stays as `median_gain`): 6.00 / 5.22.
- `fit_flap.py` simulated its windows with explicit Euler at 200 Hz, which damps a 4-6 Hz
  mode wrongly (tf 0.035 for 0.045), and held each input half a step late. It now steps
  the exact discretisation with the input at mid-step: L and tf within 2-6 %.
- The rotor law in `fit_heli.py` regressed dw/dt over airborne samples. Under the governor
  in the air the rotor speed barely moves, so only A/B (the motor's Kv) was identified -
  the Stratos came out at A 22 against `fit_rotor.py`'s 988. It now fits the motor-volts
  form over every powered sample, spool-ups included, like `fit_rotor.py` (Stratos A 624,
  R^2 0.994).
- Logs without a baro (no altitude field) had every hover flagged as in ground effect, so
  no hover counted. Their hovers now count as of unknown height (`height_unknown_s`).

## Open-loop fits per heli (`scripts/fit_heli.py`)

```sh
FIT_DATA_DIR=<decoded logs> python -I scripts/fit_heli.py helis/omp_m6/heli.json   # -> helis/omp_m6/fit.json
```

The stages of `m7r_fit.py` from the heli file, pooled over its logs and split by
headspeed: rotor law, hover collective, rate gains, servo-tail yaw, whirl peak. The three
OMP helis (same pilot and tune), rate per cyclic in rad/s per unit mixer input (roll /
pitch, coherence 0.6-0.9; the median |G| of the first version, which reads high where the
flap/body mode sits in the band - see Training; `gain` in fit.json is now the fitted K):

| | 1600 rpm | 1900 rpm | 2200 rpm |
|---|---|---|---|
| M5 | 4.95 / 4.39 | 5.36 / 6.66 | 8.22 / 7.44 |
| M6 | 4.47 / 4.85 | 5.78 / 6.66 | 8.12 / 6.35 |
| M7R (1500 / 1800 / 2100) | 4.89 / 5.88 | 6.74 / 7.59 | 7.88 / 8.30 |

The rate grows ~rpm on every size - flapping theory with stiffness ~rpm^2 and tf ~1/rpm,
as the M7R preset already has it (`ctrl_rpm_exp` 0). The closed-loop gyro peak sits at
2.2-2.6 Hz on all three, independent of size and headspeed. The rotor law fits R^2
0.48-0.75 in the volts form (these ESCs add a throttle response of their own). Hover
collective comes from the hover segments only (`scripts/segment.py`: ground,
autorotation, inverted, aerobatic, hover, cruise - hover is calm sticks, level within
5 deg around the roll trim, |baro vario| < 0.5 m/s for 1.5 s), out of ground effect
(1.5 m above the take-off ground; it clearly needs less collective near the ground), with
a 5 s gate per headspeed. These sport flights pass it at one headspeed each: M6 0.340 at
1600 rpm (7 s), M7R 0.351 at 1500 rpm (8 s; the preset's hand-picked 0.346). Every other
headspeed has 0-3 s of hover - the inflow law needs a flight that hovers at two
headspeeds (manoeuvre card).

## Flap/body dynamics from sharp inputs (`scripts/fit_flap.py`)

```sh
FIT_DATA_DIR=<decoded logs> python -I scripts/fit_flap.py helis/omp_m7r/heli.json --plot flap.png   # -> helis/omp_m7r/flap.json
```

The sport flights hold many sharp roll and pitch inputs (setpoint swinging by > 150 deg/s
within 0.3 s). Around each, 0.3 s before to 1.2 s after, the model's first-order flapping
on a rigid body - p' = L b, b' = -p - b/tf + K u(t - d), u the mixer's cyclic - is fitted
by output error to the gyro, per axis and headspeed. The mixer is what reaches the rotor,
so this holds in closed loop too. VAF 0.92-0.99:

| heli | axis, rpm | events | K rad/s | L 1/s^2 | tf s | mode |
|---|---|---|---|---|---|---|
| M7R | roll 1800 | 9 | 6.77 | 1823 | 0.021 | 5.7 Hz |
| M7R | pitch 1800 / 2100 | 33 / 8 | 5.54 / 6.92 | 404 / 571 | 0.070 / 0.047 | 3.0 / 3.4 Hz |
| M6 | roll 2200 | 4 | 8.20 | 1448 | 0.025 | 5.2 Hz |
| M6 | pitch 1900 / 2200 | 3 / 6 | 4.85 / 6.25 | 383 / 521 | 0.054 / 0.041 | 2.7 / 3.1 Hz |
| M5 | roll 2200 | 5 | 8.15 | 1987 | 0.015 | 4.9 Hz |
| M5 | pitch 1900 / 2200 | 21 / 13 | 5.21 / 6.20 | 424 / 332 | 0.069 / 0.058 | 3.1 / 2.6 Hz |

- K matches the spectral rate gains of `fit_heli.py` (M7R roll 6.77 vs 6.74).
- Pitch is ~4x softer than roll on every heli (L ~400 vs 1450-2000): the pitch inertia is
  a few times the roll inertia, the rotor the same.
- Across headspeed, the pitch stiffness follows rpm^2 on the M7R (x1.41 for 1800->2100,
  rpm^2 says x1.36) and the M6 (x1.36 vs x1.34); tf falls a little faster than 1/rpm. The
  M5 does not follow (fewer events at 2200, more 3D).
- Across size, the pitch stiffness is about the same (~400) from the M5 to the M7R.
- The delay comes out at 0: it is absorbed in tf, which is therefore longer than the
  flapping alone.

The M7R preset (L 700 / 700, tf 0.05, from the closed-loop replay sweep) sits between the
roll and pitch values. Replayed (cruise profile, `m7r_sweep.py`, `tf_roll` / `tf_pitch`):

| | roll err | roll 1.5-3 Hz | pitch err | pitch 1.5-3 Hz |
|---|---|---|---|---|
| real | 4.8 | 2.4 | 4.1 | 1.9 |
| preset 700 / 700, tf 0.05 | 5.0 | 3.3 | 5.8 | 2.9 |
| open-loop 1823 / 404, tf 0.021 / 0.070 | 2.8 | 1.9 | 6.1 | 3.8 |

The open-loop values make roll track better than the real heli (calm air, no disturbances)
and pitch worse; the preset stays. The open-loop fit's tf absorbs the delays of the whole
loop (servos, filters), which the replay has on its own.

**The replay itself, checked against the state before the session's model changes**
(same profile, sim/real): the inflow torque in powered flight raised the throttle to 56.7 %
(real 51.2) and the tail to -42 % (-21.7) - it is now only applied in descent (throttle
48.1 / 51.1, yaw mixer -22.3 / -22.0, as before); and the replay needed an explicit
upright reset on tilted touchdowns, which the old ground model did silently (92 % above
1 m with 7 resets, before 95 %).

## Gyro chip noise (`scripts/allan.py`)

```sh
python -I scripts/allan.py <soak.csv> --chip ICM40608 [--lsb-per-dps 16.4] --plot adev.png   # -> sensors/ICM40608.json
```

A flight controller at rest for 30 min, MSP_RAW_IMU polled at ~200 Hz (filtered gyro in
counts, acc in g). Overlapping Allan deviation per axis; at tau >= 1 s the firmware's
filters and the poll rate do not matter, so the white noise, bias instability and rate
walk hold.

| chip | white noise deg/s/rtHz (x / y / z) | bias instability deg/h | acc white ug/rtHz |
|---|---|---|---|
| ICM-40608 | 0.0037 / 0.0051 / 0.0033 | 6.1 / 2.2 / 3.1 | 91 / 114 / 74 |
| IIM-42652 | 0.0045 / 0.0055 / 0.0039 | 10.2 / 5.0 / 7.1 | 97 / - / 150 |
| LSM6DSK320X | 0.0036 / 0.0036 / 0.0036 | 4.6 / 7.6 / 4.9 | 117 / - / 110 |

The white noise is at the datasheet level. The axis that carried 1 g sits on one count
without dither (marked -); the sensor model replaces it with the other axes'.

## Vibration (`scripts/vibration.py`)

```sh
FIT_DATA_DIR=<decoded logs> python -I scripts/vibration.py helis/omp_m6/heli.json --plot m6.png   # -> helis/omp_m6/vibration.json
```

Order tracking of the logged `gyroRAW` (the decimated gyro before the RPM filter, notches
and low-pass; Rotorflight 4.6+ logs it at any debug mode) in steady 4 s airborne windows.
The spectra are rpm-locked throughout; besides the main rotor 1x/2x/4x, the tail sits at
4.75x (M5, M6) and 5.0x (M7R), and the motor at its gear ratio: 8.85x (M5, 115/13), 9.31x
(M6, 121/13), 8.75x (M7R, 140/16 - the M7R preset had 10). Amplitudes at the fit's
reference headspeed, deg/s peak, roll / pitch / yaw:

| | main 1x | main 2x | main 4x | tail 1x | tail 2x | motor 1x | floor deg/s/rtHz (roll) |
|---|---|---|---|---|---|---|---|
| M5 (1900 rpm) | 6.8 / 4.7 / 1.4 | 13.3 / 3.9 / 4.2 | 3.7 / 1.6 / 1.5 | 13.0 / 1.2 / 1.2 | 4.1 / 0.5 / 3.0 | 2.9 / 0.7 / 0.6 | 0.15 |
| M6 (1900 rpm) | 10.5 / 9.4 / 1.9 | 4.8 / 3.1 / 1.0 | 3.6 / 1.6 / 1.6 | 25.2 / 3.1 / 3.0 | 4.6 / 0.3 / 3.1 | 3.0 / 0.9 / 2.4 | 0.15 |
| M7R (1800 rpm) | 3.9 / 5.3 / 3.2 | 9.4 / 4.1 / 2.8 | 4.9 / 1.6 / 1.0 | 2.0 / 0.6 / 0.4 | 3.3 / 0.4 / 2.0 | 1.3 / 0.2 / 1.3 | 0.07 |
| Stratos 200 (3600 rpm) | 17.7 / 5.4 / 30.6 | 8.9 / 3.5 / 9.4 | 3.0 / 4.4 / 1.8 (with the tail) | - | - | - (direct drive) | 0.13 |

The Stratos is direct drive with a motorized tail: no motor line apart from the rotor's,
and its tail turns at 3-4.6x the main rotor (median 4.05), moving by 4-5 % within 4 s as
the yaw control - it never holds still long enough to be tracked on its own and sits in
the main 4x line it crosses (`vibration.py` tracks a motorized tail at its own logged
speed where it does hold). Its main 1x is strong on yaw and roll.

The tail rotor's 1x on the roll axis is the strongest line on the M5 and M6. The rpm
exponents scatter (two headspeeds per heli, different manoeuvres); the sensor model
scales with rpm^2. In the SITL, with the firmware's default filters, the M7R's vibration
leaves 1.2 / 0.3 / 0.4 deg/s RMS in the filtered gyro with the RPM filter and 9.2 / 6.1 /
4.0 without (the M5 and M6, 2.5 deg/s on roll with it: their broadband floor, which no
notch removes).

**End to end.** A Blade 360 hover check with `--vibration omp_m7r` at 2100 rpm, its SITL
blackbox at the full rate (`--cli "set blackbox_rate_denom = 1"`, ~800 Hz, 1 MB for 28 s
of the 2 MB flash), order-tracked like a real log: the firmware's `gyroRAW` carries the
injected lines at their amplitude x (2100/1797)^2 - main 1x 5.27/7.25/4.37 deg/s
(injected 5.32/7.27/4.37), 2x 12.75/5.59/3.75 (12.86/5.64/3.79), 4x 6.55/2.09/1.34
(6.73/2.16/1.36), tail 1x 2.55/0.77/0.44 (2.67/0.80/0.49). `compare_replay.py` compares
the vibration that reached the sensor (gyroRAW) and the PIDs (gyroADC) above 20 Hz between
a replay and its real flight.

## Open

- a less damped mode and lower-band turbulence (the real peak is sharper; the whirl's
  amplitude at 3300 rpm and the effect of D gain are both undersized)
- pitch at other headspeeds (sim 5.9 Hz vs real 4.9 Hz at 3300)
- a test flight with sharp roll/pitch/collective inputs, for control power, the flap time
  constant and the collective sensitivity
- M7R: its yaw fit; gear ratio and motor
  poles (not in a 4.6 log header)
