# Stratos 700 - training report

Trained 2026-11-09 on setup `7180bcda` (craft Stratos700, firmware 4.6, board RDMS NEXUS_XR, tail servo, collective -1042,1042, esc 0/0), from the prior: size prior (700 class, M7R dynamics). Written by `scripts/train.py`.

- fit logs (14): Stratos700_20261008_133322.0, Stratos700_20261008_133504.0, Stratos700_20261008_134002.0, Stratos700_20261008_134448.0, Stratos700_all_20261008_001340.5, Stratos700_all_20261008_001340.6, Stratos700_all_20261008_001340.9, Stratos700_all_20261008_001340.12, Stratos700_all_20261016_182858.0, Stratos700_all_20261016_182858.1, Stratos700_all_20261016_182858.4, Stratos700_all_20261016_182858.5, Stratos700_all_20261016_182858.6, Stratos700_all_20261016_182858.8
- held-out logs (8): Stratos700_20261008_133648.0, Stratos700_20261008_134116.0, Stratos700_all_20261008_001340.7, Stratos700_all_20261008_001340.8, Stratos700_all_20261008_001340.10, Stratos700_all_20261008_001340.11, Stratos700_all_20261016_182858.2, Stratos700_all_20261016_182858.9

## Stages

| stage | decision | held-out before | after |
|---|---|---|---|
| rotor | kept | 12.41 | 3.302 |
| thrust | kept | 0.1173 | 0.02806 |
| rate | taken | 0.3963 | 0.08652 |
| flap | taken | 0.1735 | 0.06084 |
| yaw | kept (worse) | 0.225 | 0.2384 |
| power | taken | 0.2027 | 0.03141 |
| pack | taken | 0.2984 | 0.1515 |

**rotor**: kept: makes yaw 0.225 -> 0.8668 worse
- R^2 0.812, A 11.98 +- 13 %, B 1.937 +- 13 %
- held-out before: RMS throttle error 12.4 % over 92580 samples
- held-out after: RMS throttle error 3.3 % over 92580 samples

**thrust**: kept: makes yaw 0.225 -> 0.4061 worse
- inflow law from 1700, 1800, 1900 rpm: share 1 %
- hover collective at 1900 rpm 0.357
- held-out before: hover collective error -0.145 at 1700 rpm (77 s), -0.137 at 1800 rpm (55 s), -0.083 at 1900 rpm (38 s), -0.104 at 2200 rpm (2 s)
- held-out after: hover collective error -0.016 at 1700 rpm (77 s), -0.032 at 1800 rpm (55 s), +0.000 at 1900 rpm (38 s), -0.064 at 2200 rpm (2 s)

**rate**: taken: held-out 0.3963 -> 0.08652
- rate ~ rpm^1.16 over 1700/1800/1900 rpm -> ctrl_rpm_exp 0.16
- at 1900 rpm: roll 4.11, pitch 4.39
- held-out before: ln(model/measured) pitch 1700 +0.32, pitch 1800 +0.28, roll 1900 +0.53, pitch 1900 +0.31, pitch 2200 +0.54
- held-out after: ln(model/measured) pitch 1700 +0.04, pitch 1800 +0.01, roll 1900 +0.03, pitch 1900 +0.05, pitch 2200 +0.30

**flap**: taken: held-out 0.1735 -> 0.06084
- roll from 23 sharp inputs at 1900 rpm: L 3068 +- 18 %, tf 0.037 +- 20 %, VAF 0.93
- pitch from 66 sharp inputs at 1900 rpm: L 942 +- 4 %, tf 0.067 +- 4 %, VAF 0.93
- held-out before: 1 - VAF 0.173 over 48 sharp inputs
- held-out after: 1 - VAF 0.061 over 48 sharp inputs

**yaw**: kept (worse): held-out 0.225 -> 0.2384
- N_ped 43.6 at 1900 rpm (R^2 0.71) -> Nped 43.6 at 1900 rpm, Nr -0.37, Ncol 21.4
- hover tail trim -0.207 at 1900 rpm
- held-out before: R^2 0.779, tail trim error 0.004
- held-out after: R^2 0.773, tail trim error 0.011

**power**: taken: held-out 0.2027 -> 0.03141
- hover 1488 W at 47.1 V, throttle 0.70 (1900 rpm)
- held-out before: hover power 1150 W vs 1442 W
- held-out after: hover power 1488 W vs 1442 W

**pack**: taken: held-out 0.2984 -> 0.1515
- 26.9 mOhm +- 11 % (12S, RMS 0.18 V over 14 logs)
- 6.35 Ah +- 20 %, the largest draw 44 % of it
- held-out before: RMS Vbat error 0.30 V over 8 logs
- held-out after: RMS Vbat error 0.15 V over 8 logs

## Held-out scores

Lower is better (`scripts/holdout.py`).

| stage | prior | current | new |
|---|---|---|---|
| rotor | 12.41 | 12.41 | 8.652 |
| thrust | 0.1173 | 0.1173 | 0.1173 |
| rate | 0.3963 | 0.3963 | 0.0656 |
| flap | 0.4558 | 0.4558 | 0.06084 |
| yaw | 0.225 | 0.225 | 0.225 |
| power | 0.2027 | 0.2027 | 0.03141 |
| pack | 0.2984 | 0.2984 | 0.1515 |

## Replay check

Held-out flights replayed through the SITL (`replay_check.py`) with the prior and the new model, scored against the real flight (lower is better): 1.955 -> 1.545, verdict **ok**.

`Stratos700_20261008_133648.0`, 8-110 s at 1700 rpm:

| | real | prior | new |
|---|---|---|---|
| roll_err_rms | 13.7 | 5.55 | 3.04 |
| pitch_err_rms | 21.61 | 17.74 | 15.89 |
| yaw_err_rms | 46.2 | 26.38 | 26.95 |
| throttle | 60.5 | 57.5 | 65.2 |
| tail | -21.6 | -35.4 | -29.2 |
| roll_spectrum_db | | 22.0 | 21.5 |
| pitch_spectrum_db | | 7.0 | 8.6 |
| yaw_spectrum_db | | 9.9 | 5.0 |
| score | | 2.234 | 2.013 |

`Stratos700_20261008_134116.0`, 46-76 s at 1700 rpm:

| | real | prior | new |
|---|---|---|---|
| roll_err_rms | 5.96 | 1.95 | 1.65 |
| pitch_err_rms | 6.02 | 3.12 | 2.89 |
| yaw_err_rms | 56.61 | 36.71 | 36.73 |
| throttle | 62.4 | 54.0 | 63.8 |
| tail | -19.4 | -20.4 | -20.6 |
| roll_spectrum_db | | 22.4 | 21.4 |
| pitch_spectrum_db | | 14.9 | 14.3 |
| yaw_spectrum_db | | 10.2 | 8.6 |
| score | | 1.676 | 1.077 |


Model changed: yes.
