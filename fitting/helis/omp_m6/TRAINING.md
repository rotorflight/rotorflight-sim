# OMP M6 - training report

Trained 2026-11-08 on setup `3e472a5b` (craft OMP M6, firmware 4.6, board RDMS NEXUS_XR, tail servo, collective -1250,1250, esc 0/0), from the prior: size prior (? class, M7R dynamics). Written by `scripts/train.py`.

- fit logs (1): OMP_M6_20260821_112524
- held-out logs (1): OMP_M6_20260822_102430

## Stages

| stage | decision | held-out before | after |
|---|---|---|---|
| rotor | taken | 11.46 | 0.8571 |
| thrust | no proposal | - | - |
| rate | kept | 0.04757 | 0.1423 |
| flap | no proposal | - | - |
| yaw | no proposal | - | - |
| power | no proposal | - | - |

**rotor**: taken: held-out 11.46 -> 0.8571
- R^2 0.982, A 50.68 +- 18 %, B 6.435 +- 18 %
- held-out before: RMS throttle error 11.5 % over 40400 samples
- held-out after: RMS throttle error 0.9 % over 40400 samples

**thrust**: no proposal
- no gated hover with a stable collective (2000 rpm 0.9 s insufficient)

**rate**: kept: makes flap 0.04627 -> 0.06696 worse
- at 2200 rpm: roll 7.95, pitch 6.08
- held-out before: ln(model/measured) pitch 1600 +0.06, roll 1900 +0.05, pitch 1900 +0.03
- held-out after: ln(model/measured) pitch 1600 +0.12, roll 1900 +0.21, pitch 1900 +0.09

**flap**: no proposal
- pitch 2200 rpm: L or tf uncertain (51 / 50 %)

**yaw**: no proposal
- no yaw fit passes R^2 >= 0.3, no gated hover at 2200 rpm

**power**: no proposal
- no gated hover at 2200 rpm with Vbat and Ibat

## Held-out scores

Lower is better (`scripts/holdout.py`).

| stage | prior | current | new |
|---|---|---|---|
| rotor | 11.46 | 11.46 | 0.8571 |
| thrust | 0.06951 | 0.06951 | 0.06951 |
| rate | 0.04757 | 0.04757 | 0.04757 |
| flap | 0.04627 | 0.04627 | 0.04627 |
| yaw | 0.4608 | 0.4608 | 0.4608 |
| power | - | - | - |

Model changed: yes.
