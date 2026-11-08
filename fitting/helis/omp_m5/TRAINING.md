# OMP M5 - training report

Trained 2026-11-09 on setup `1cfbd66a` (craft OMP M5, firmware 4.6, board RDMS NEXUS_XR, tail servo, collective -1250,1250, esc 0/0), from the prior: size prior (? class, M7R dynamics). Written by `scripts/train.py`.

- fit logs (1): OMP_M5_20260829_130626
- held-out logs (1): OMP_M5_20260822_104240

## Stages

| stage | decision | held-out before | after |
|---|---|---|---|
| rotor | taken | 9.585 | 1.153 |
| thrust | no proposal | - | - |
| rate | kept | 0.1694 | 0.166 |
| flap | kept | 0.07779 | 0.09209 |
| yaw | taken | 4.094 | 2.196 |
| power | no proposal | - | - |
| pack | no proposal | - | - |

**rotor**: taken: held-out 9.585 -> 1.153
- R^2 0.980, A 42.48 +- 17 %, B 5.02 +- 16 %
- held-out before: RMS throttle error 9.6 % over 26321 samples
- held-out after: RMS throttle error 1.2 % over 26321 samples

**thrust**: no proposal
- no gated hover with a stable collective ()

**rate**: kept: makes flap 0.07779 -> 0.08056 worse
- at 2200 rpm: roll 7.72, pitch 6.52
- held-out before: ln(model/measured) roll 1600 +0.20, pitch 1900 -0.14
- held-out after: ln(model/measured) roll 1600 +0.33, pitch 1900 -0.00

**flap**: kept: makes rate 0.1694 -> 0.2187 worse
- roll from 5 sharp inputs at 2200 rpm: L 1754 +- 29 %, tf 0.018 +- 22 %, VAF 0.98
- pitch from 13 sharp inputs at 2200 rpm: L 316 +- 16 %, tf 0.063 +- 23 %, VAF 0.95
- held-out before: 1 - VAF 0.078 over 21 sharp inputs
- held-out after: 1 - VAF 0.092 over 21 sharp inputs

**yaw**: taken: held-out 4.094 -> 2.196
- N_ped 22.4 at 1600 rpm (R^2 0.51) -> Nped 42.4 at 2200 rpm, Nr -0.10, Ncol -15.3
- held-out before: R^2 -3.094
- held-out after: R^2 -1.196

**power**: no proposal
- no gated hover at 2200 rpm with Vbat and Ibat

**pack**: no proposal
- resistance not identified (R^2 0.94, +- inf %)
- capacity not identified (2.42 Ah +- inf %, the largest draw 56 % of it)

## Held-out scores

Lower is better (`scripts/holdout.py`).

| stage | prior | current | new |
|---|---|---|---|
| rotor | 9.585 | 9.585 | 1.153 |
| thrust | - | - | - |
| rate | 0.1694 | 0.1694 | 0.1694 |
| flap | 0.07779 | 0.07779 | 0.07779 |
| yaw | 2.344 | 2.344 | 2.196 |
| power | - | - | - |
| pack | 0.4971 | 0.4971 | 0.4971 |

Model changed: yes.
