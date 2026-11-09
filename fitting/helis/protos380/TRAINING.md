# MSH Protos 380 - training report

Trained 2026-11-09 on setup `b8916a32` (craft Protos, firmware 4.6, board RDMS NEXUS_XR, tail servo, collective -1083,1083, esc 0/0), from the prior: size prior (380 class, M7R dynamics). Written by `scripts/train.py`.

- fit logs (6): Protos_20260622_094806.0, Protos_20260622_094946.0, Protos_20260622_095704.0, Protos_20260622_095920.0, Protos_20260622_100428.0, Protos_20260622_101056.0
- held-out logs (2): Protos_20260622_094304.0, Protos_20260622_095452.0

## Stages

| stage | decision | held-out before | after |
|---|---|---|---|
| rotor | no proposal | - | - |
| thrust | no proposal | - | - |
| rate | taken | 0.1294 | 0.1009 |
| flap | kept | 0.2092 | 0.1246 |
| yaw | taken | 0.5878 | 0.5836 |
| power | no proposal | - | - |
| pack | no proposal | - | - |

**rotor**: no proposal
- A or B uncertain: 147 / 148 %

**thrust**: no proposal
- no gated hover with a stable collective (2000 rpm 4.6 s insufficient, 2200 rpm 2.5 s insufficient)

**rate**: taken: held-out 0.1294 -> 0.1009
- rate ~ rpm^2.26 over 2000/2200 rpm -> ctrl_rpm_exp 1.26
- at 2200 rpm: roll 5.51, pitch 5.33
- held-out before: ln(model/measured) roll 2000 +0.20, pitch 2000 -0.02, roll 2200 +0.25, pitch 2200 +0.04
- held-out after: ln(model/measured) roll 2000 -0.13, pitch 2000 -0.21, roll 2200 +0.04, pitch 2200 -0.02

**flap**: kept: makes rate 0.1009 -> 0.1829 worse
- roll 2200 rpm: L or tf uncertain (75 / 27 %)
- roll from 11 sharp inputs at 2400 rpm: L 2794 +- 14 %, tf 0.023 +- 13 %, VAF 0.94
- pitch from 40 sharp inputs at 2200 rpm: L 739 +- 5 %, tf 0.044 +- 5 %, VAF 0.95
- held-out before: 1 - VAF 0.209 over 8 sharp inputs
- held-out after: 1 - VAF 0.125 over 8 sharp inputs

**yaw**: taken: held-out 0.5878 -> 0.5836
- N_ped 31.4 at 2200 rpm (R^2 0.56) -> Nped 31.4 at 2200 rpm, Nr -0.10, Ncol 23.1
- held-out before: R^2 0.412
- held-out after: R^2 0.416

**power**: no proposal
- no gated hover at 2200 rpm with Vbat and Ibat

**pack**: no proposal
- resistance not identified (R^2 0.93, +- 79 %)
- capacity not identified (2.82 Ah +- 46 %, the largest draw 42 % of it)

## Held-out scores

Lower is better (`scripts/holdout.py`).

| stage | prior | current | new |
|---|---|---|---|
| rotor | 11.48 | 11.48 | 11.48 |
| thrust | 0.2427 | 0.2427 | 0.2427 |
| rate | 0.1294 | 0.1294 | 0.1009 |
| flap | 0.3488 | 0.3488 | 0.2092 |
| yaw | 0.5878 | 0.5878 | 0.5836 |
| power | - | - | - |
| pack | 0.2069 | 0.2069 | 0.2069 |

## Replay check

Held-out flights replayed through the SITL (`replay_check.py`) with the prior and the new model, scored against the real flight (lower is better): 3.653 -> 3.792, verdict **ok**.

`Protos_20260622_094304.0`, 11-67 s at 2000 rpm:

| | real | prior | new |
|---|---|---|---|
| roll_err_rms | | (11.99, 4.54) | (11.99, 3.93) |
| pitch_err_rms | | (13.47, 7.09) | (13.47, 7.01) |
| yaw_err_rms | | (15.67, 13.11) | (15.67, 20.04) |
| throttle | | (61.0, 54.7) | (61.0, 54.8) |
| tail | | (-44.0, -22.6) | (-44.0, -19.7) |
| roll_spectrum_db | | 19.2 | 20.2 |
| pitch_spectrum_db | | 9.3 | 11.2 |
| yaw_spectrum_db | | 14.6 | 16.5 |
| score | | 3.368 | 3.722 |

`Protos_20260622_095452.0`, 22-56 s at 2200 rpm:

| | real | prior | new |
|---|---|---|---|
| roll_err_rms | | (15.0, 3.75) | (15.0, 3.09) |
| pitch_err_rms | | (11.34, 2.23) | (11.34, 2.26) |
| yaw_err_rms | | (39.33, 28.66) | (39.33, 48.81) |
| throttle | | (65.4, 60.6) | (65.4, 60.85) |
| tail | | (-42.3, -18.8) | (-42.3, -19.6) |
| roll_spectrum_db | | 25.1 | 25.8 |
| pitch_spectrum_db | | 15.6 | 15.3 |
| yaw_spectrum_db | | 9.0 | 11.2 |
| score | | 3.939 | 3.861 |


Model changed: yes.
