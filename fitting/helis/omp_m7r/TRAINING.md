# OMP M7R - training report

Trained 2026-11-08 on setup `c3771b88` (craft OMP M7R, firmware 4.6, board RDMS NEXUS_XR, tail servo, collective -1250,1250, esc 0/0), from the prior: preset m7r. Written by `scripts/train.py`.

- fit logs (2): OMP_M7R_20261016_124328, OMP_M7R_20261022_143720
- held-out logs (1): OMP_M7R_20261022_155150

## Stages

| stage | decision | held-out before | after |
|---|---|---|---|
| rotor | taken | 6.042 | 1.742 |
| thrust | no proposal | - | - |
| rate | kept | 0.06792 | 0.08319 |
| flap | taken | 0.06076 | 0.05955 |
| yaw | taken | 1.214 | 0.7039 |
| power | no proposal | - | - |

**rotor**: taken: held-out 6.042 -> 1.742
- R^2 0.950, A 30.12 +- 13 %, B 3.703 +- 13 %
- held-out before: RMS throttle error 6.0 % over 31830 samples
- held-out after: RMS throttle error 1.7 % over 31830 samples

**thrust**: no proposal
- no gated hover with a stable collective ()

**rate**: kept: makes flap 0.06076 -> 0.1269 worse
- rate ~ rpm^1.16 over 1500/1800/2100 rpm -> ctrl_rpm_exp 0.16
- at 1800 rpm: roll 6.46, pitch 6.54
- held-out before: ln(model/measured) roll 1500 +0.05, pitch 1500 -0.14, roll 1800 +0.04, pitch 1800 -0.04, roll 2000 -0.07, pitch 2000 +0.06
- held-out after: ln(model/measured) roll 1500 -0.03, pitch 1500 -0.04, roll 1800 -0.01, pitch 1800 +0.10, roll 2000 -0.10, pitch 2000 +0.22

**flap**: taken: held-out 0.06076 -> 0.05955
- roll from 7 sharp inputs at 1800 rpm: L 1230 +- 21 %, tf 0.035 +- 28 %, VAF 0.98
- pitch from 29 sharp inputs at 1800 rpm: L 376 +- 19 %, tf 0.081 +- 16 %, VAF 0.96
- held-out before: 1 - VAF 0.061 over 11 sharp inputs
- held-out after: 1 - VAF 0.060 over 11 sharp inputs

**yaw**: taken: held-out 1.214 -> 0.7039
- N_ped 27.0 at 1800 rpm (R^2 0.38) -> Nped 27.0 at 1800 rpm, Nr -0.60
- held-out before: R^2 -0.214
- held-out after: R^2 0.296

**power**: no proposal
- no gated hover at 1800 rpm with Vbat and Ibat

## Held-out scores

Lower is better (`scripts/holdout.py`).

| stage | prior | current | new |
|---|---|---|---|
| rotor | 6.042 | 6.042 | 1.742 |
| thrust | 0.00344 | 0.00344 | 0.00344 |
| rate | 0.06792 | 0.06792 | 0.0683 |
| flap | 0.06076 | 0.06076 | 0.05955 |
| yaw | 1.214 | 1.214 | 0.7039 |
| power | - | - | - |

Model changed: yes.
