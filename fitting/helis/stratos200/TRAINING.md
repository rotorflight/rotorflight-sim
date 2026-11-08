# Stratos 200 (Bo105 fuselage) - training report

Trained 2026-11-08 on setup `cfbc5200` (craft Stratos 200, firmware 4.7, board RDMS NEXUS_XR, tail motor, collective -1083,1083, esc 6/1), from the prior: preset stratos200. Written by `scripts/train.py`.

- fit logs (5): 20261008_160146_11, 20261008_160146_12, 20261008_174938_00, 20261008_180641_00, 20261008_182311_01
- held-out logs (1): 20261008_172721_00

## Stages

| stage | decision | held-out before | after |
|---|---|---|---|
| rotor | kept (worse) | 0.6944 | 0.6994 |
| thrust | no proposal | - | - |
| rate | taken | 0.1259 | 0.006304 |
| flap | no proposal | - | - |
| yaw | no proposal | - | - |
| power | no proposal | - | - |

**rotor**: kept (worse): held-out 0.6944 -> 0.6994
- R^2 0.995, A 632.8 +- 15 %, B 10.6 +- 15 %
- held-out before: RMS throttle error 0.7 % over 6607 samples
- held-out after: RMS throttle error 0.7 % over 6607 samples

**thrust**: no proposal
- no gated hover with a stable collective (3300 rpm 0.4 s insufficient, 3600 rpm 3.3 s insufficient)

**rate**: taken: held-out 0.1259 -> 0.006304
- at 3600 rpm: roll 6.72, pitch 5.41
- held-out before: ln(model/measured) pitch 3600 -0.13
- held-out after: ln(model/measured) pitch 3600 +0.01

**flap**: no proposal
- no sharp-input fit passes VAF >= 0.85 with >= 5 events

**yaw**: no proposal
- motorized tail: the yaw stage fits servo tails only

**power**: no proposal
- no gated hover at 3600 rpm with Vbat and Ibat

## Held-out scores

Lower is better (`scripts/holdout.py`).

| stage | prior | current | new |
|---|---|---|---|
| rotor | 0.6944 | 0.6944 | 0.6944 |
| thrust | - | - | - |
| rate | 0.1259 | 0.1259 | 0.006304 |
| flap | - | - | - |
| yaw | - | - | - |
| power | - | - | - |

Model changed: yes.
