# MSH Protos 380 - training report

Trained 2026-11-08 on setup `b8916a32` (craft Protos, firmware 4.6, board RDMS NEXUS_XR, tail servo, collective -1083,1083, esc 0/0), from the prior: size prior (380 class, M7R dynamics). Written by `scripts/train.py`.

- fit logs (6): Protos_20260622_094806.0, Protos_20260622_094946.0, Protos_20260622_095704.0, Protos_20260622_095920.0, Protos_20260622_100428.0, Protos_20260622_101056.0
- held-out logs (2): Protos_20260622_094304.0, Protos_20260622_095452.0

## Stages

| stage | decision | held-out before | after |
|---|---|---|---|
| rotor | no proposal | - | - |
| thrust | no proposal | - | - |
| rate | taken | 0.1294 | 0.1009 |
| flap | kept | 0.2092 | 0.1283 |
| yaw | taken | 0.5901 | 0.4259 |
| power | no proposal | - | - |

**rotor**: no proposal
- A or B uncertain: 147 / 148 %

**thrust**: no proposal
- no gated hover with a stable collective (2000 rpm 4.6 s insufficient, 2200 rpm 2.5 s insufficient)

**rate**: taken: held-out 0.1294 -> 0.1009
- rate ~ rpm^2.26 over 2000/2200 rpm -> ctrl_rpm_exp 1.26
- at 2200 rpm: roll 5.51, pitch 5.33
- held-out before: ln(model/measured) roll 2000 +0.20, pitch 2000 -0.02, roll 2200 +0.25, pitch 2200 +0.04
- held-out after: ln(model/measured) roll 2000 -0.13, pitch 2000 -0.21, roll 2200 +0.04, pitch 2200 -0.02

**flap**: kept: makes rate 0.1009 -> 0.128 worse
- roll 2200 rpm: L or tf uncertain (75 / 27 %)
- roll from 11 sharp inputs at 2400 rpm: L 2794 +- 14 %, tf 0.023 +- 13 %, VAF 0.94
- pitch from 40 sharp inputs at 2200 rpm: L 739 +- 5 %, tf 0.044 +- 5 %, VAF 0.95
- held-out before: 1 - VAF 0.209 over 8 sharp inputs
- held-out after: 1 - VAF 0.128 over 8 sharp inputs

**yaw**: taken: held-out 0.5901 -> 0.4259
- N_ped 31.4 at 2200 rpm (R^2 0.56) -> Nped 31.4 at 2200 rpm, Nr -0.10
- held-out before: R^2 0.410
- held-out after: R^2 0.574

**power**: no proposal
- no gated hover at 2200 rpm with Vbat and Ibat

## Held-out scores

Lower is better (`scripts/holdout.py`).

| stage | prior | current | new |
|---|---|---|---|
| rotor | 11.48 | 11.48 | 11.48 |
| thrust | 0.2427 | 0.2427 | 0.2427 |
| rate | 0.1294 | 0.1294 | 0.1009 |
| flap | 0.3488 | 0.3488 | 0.2092 |
| yaw | 0.5901 | 0.5901 | 0.4259 |
| power | - | - | - |

Model changed: yes.
