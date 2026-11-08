# ALZRC Devil 505 - training report

Trained 2026-11-08 on setup `683c976e` (craft Devil 505, firmware 4.5, board RDMS NEXUS_XR, tail servo, collective -1250,1250, esc 0/0), from the prior: size prior (500 class, M7R dynamics). Written by `scripts/train.py`.

- fit logs (2): Devil_505_20260129_125822.0, Devil_505_20260129_163848.0
- held-out logs (0): none

## Stages

| stage | decision | held-out before | after |
|---|---|---|---|
| rotor | no proposal | - | - |
| thrust | no proposal | - | - |
| rate | taken, not validated (over a size prior) | - | - |
| flap | kept, not validated | - | - |
| yaw | taken, not validated (over a size prior) | - | - |
| power | no proposal | - | - |

**rotor**: no proposal
- A or B uncertain: 55 / 66 %

**thrust**: no proposal
- no gated hover with a stable collective (2200 rpm 2.2 s insufficient, 2300 rpm 2.7 s insufficient)

**rate**: taken, not validated (over a size prior)
- at 2300 rpm: pitch 5.00
- held-out before: no held-out logs
- held-out after: no held-out logs

**flap**: kept, not validated: no held-out logs
- roll from 44 sharp inputs at 2200 rpm: L 4579 +- 10 %, tf 0.030 +- 3 %, VAF 0.90
- pitch 2300 rpm: L or tf uncertain (35 / 40 %)
- pitch from 45 sharp inputs at 2200 rpm: L 947 +- 15 %, tf 0.032 +- 17 %, VAF 0.86
- held-out before: no held-out logs
- held-out after: no held-out logs

**yaw**: taken, not validated (over a size prior)
- N_ped 19.9 at 2300 rpm (R^2 0.42) -> Nped 19.9 at 2300 rpm, Nr -0.11
- held-out before: no held-out logs
- held-out after: no held-out logs

**power**: no proposal
- no gated hover at 2300 rpm with Vbat and Ibat

## Held-out scores

Lower is better (`scripts/holdout.py`).

| stage | prior | current | new |
|---|---|---|---|
| rotor | - | - | - |
| thrust | - | - | - |
| rate | - | - | - |
| flap | - | - | - |
| yaw | - | - | - |
| power | - | - | - |

Model changed: yes.
