# Nimbus - training report

Trained 2026-11-08 on setup `4062f5c0` (craft Nimbus, firmware 4.6, board RDMS NEXUS_XR, tail servo, collective -1125,1125, esc 0/0), from the prior: size prior (? class, M7R dynamics). Written by `scripts/train.py`.

- fit logs (12): Nimbus_20260811_161206.0, Nimbus_all_20260811_112712.5, Nimbus_all_20260811_112712.6, Nimbus_all_20260811_112712.7, Nimbus_all_20260811_112712.8, Nimbus_all_20260811_112712.9, Nimbus_all_20260811_112712.10, Nimbus_all_20260811_112712.11, Nimbus_all_20260811_112712.12, Nimbus_all_20260811_112712.19, Nimbus_all_20260811_112712.20, Nimbus_all_20260811_112712.21
- held-out logs (6): Nimbus_20260811_160756.0, Nimbus_all_20260811_112712.2, Nimbus_all_20260811_112712.4, Nimbus_all_20260811_112712.22, Nimbus_all_20260811_112712.28, Nimbus_all_20260811_112712.29

## Stages

| stage | decision | held-out before | after |
|---|---|---|---|
| rotor | taken | 15.55 | 2.286 |
| thrust | taken | 0.01426 | 0.01171 |
| rate | taken | 0.4074 | 0.06506 |
| flap | kept | 0.1593 | 0.07986 |
| yaw | taken | 0.447 | 0.2825 |
| power | taken, not validated (over a size prior) | - | - |

**rotor**: taken: held-out 15.55 -> 2.286
- R^2 0.967, A 56.63 +- 14 %, B 3.799 +- 6 %
- held-out before: RMS throttle error 15.6 % over 50860 samples
- held-out after: RMS throttle error 2.3 % over 50860 samples

**thrust**: taken: held-out 0.01426 -> 0.01171
- hover collective 0.296 at 1900 rpm (9.7 s), the prior's inflow share 72 %
- hover collective at 1900 rpm 0.296
- held-out before: hover collective error -0.014 at 1700 rpm (9 s)
- held-out after: hover collective error +0.012 at 1700 rpm (9 s)

**rate**: taken: held-out 0.4074 -> 0.06506
- rate ~ rpm^1.07 over 1700/1900 rpm -> ctrl_rpm_exp 0.07
- at 1900 rpm: roll 4.05, pitch 4.26
- held-out before: ln(model/measured) pitch 1700 +0.26, roll 1900 +0.59, pitch 1900 +0.37
- held-out after: ln(model/measured) pitch 1700 -0.04, roll 1900 +0.07, pitch 1900 +0.08

**flap**: kept: makes rate 0.06506 -> 0.0727 worse
- roll from 22 sharp inputs at 1900 rpm: L 8837 +- 18 %, tf 0.038 +- 24 %, VAF 0.96
- pitch from 38 sharp inputs at 1900 rpm: L 1168 +- 1 %, tf 0.085 +- 3 %, VAF 0.96
- held-out before: 1 - VAF 0.159 over 83 sharp inputs
- held-out after: 1 - VAF 0.080 over 83 sharp inputs

**yaw**: taken: held-out 0.447 -> 0.2825
- N_ped 41.3 at 1900 rpm (R^2 0.82) -> Nped 41.3 at 1900 rpm, Nr -0.83
- hover tail trim -0.394 at 1900 rpm
- held-out before: R^2 0.553
- held-out after: R^2 0.717

**power**: taken, not validated (over a size prior)
- hover 626 W at 21.8 V, throttle 0.75 (1900 rpm)
- held-out before: no hover with power at 1900 rpm in the held-out logs
- held-out after: no hover with power at 1900 rpm in the held-out logs

## Held-out scores

Lower is better (`scripts/holdout.py`).

| stage | prior | current | new |
|---|---|---|---|
| rotor | 15.55 | 15.55 | 2.286 |
| thrust | 0.01426 | 0.01426 | 0.01171 |
| rate | 0.4074 | 0.4074 | 0.06506 |
| flap | 0.5962 | 0.5962 | 0.1593 |
| yaw | 0.447 | 0.447 | 0.2825 |
| power | - | - | - |

Model changed: yes.
