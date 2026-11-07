# Logging a flight for the sim

Every preset in this repository comes from real blackbox logs. A log of your heli helps in
two ways: it can become a preset of its own, and it puts another size on the scaling laws
of the physical model (`TODO.md`, sections 1 and 3). This page says how to set up the
logging, what to fly, and what to send.

## Setup

- **Firmware**: Rotorflight 4.6 or newer, which logs the unfiltered gyro (`gyroRAW`) at
  any debug mode.
- **Blackbox**: 1 kHz (`blackbox_rate_denom` to 1 kHz at your PID loop rate), all fields on.
  Below 1 kHz the vibration fit loses the tail and motor lines; below 250 Hz the rate gains
  go too.
- **Rotor speed**: an rpm sensor or ESC telemetry, so the log has `headspeed`. For the
  autorotation, an rpm source on the rotor side (a sensor on the main gear, not the motor):
  with a one-way bearing the motor stops while the rotor keeps turning.
- **Voltage and current**: Vbat and Ibat in the log (the rotor law fits motor volts).
- **Baro**: the flight controller's barometer, so `altitude` and `vario` are logged - the
  segmentation finds hover by them.
- **GPS** (optional): GPS speed in the log for the forward-flight derivatives.
- **CLI dump**: `diff all` (or `dump all`) from the same day as the flight.

## Manoeuvre card

One battery is enough. Calm air is best; note the wind otherwise. Each item says which fit
it feeds.

| | what | for |
|---|---|---|
| 1 | Hover 10 s at 2 m or higher (out of ground effect), on **each** headspeed you fly (switch profiles in the hover) | hover collective per headspeed, the thrust (inflow) law - needs two headspeeds |
| 2 | Hover 10 s at 0.3 m (in ground effect) | ground effect |
| 3 | Sharp roll doublets: full stick right, centre, left, centre, ~0.3 s each, 3-4 times, from a hover | control power, flap time constant |
| 4 | The same on pitch | as 3 |
| 5 | Collective doublets from a hover: up and down by ~3 deg of pitch, ~0.5 s each | collective sensitivity, heave damping |
| 6 | Slow stick sweeps on roll and pitch: a few degrees, from ~0.3 Hz up to ~5 Hz over 20 s | frequency response, the flap/body mode |
| 7 | Piro both ways at hover height | tail authority, yaw |
| 8 | Straight passes at constant height, slow, medium and fast | translational lift, drag, forward-flight derivatives (with GPS) |
| 9 | If the heli does them: an autorotation from height, with the rotor-side rpm source | the inflow model's sink rate and rotor decay |
| 10 | Whatever you normally fly | everything else, and the vibration over the whole envelope |

## The heli file

Fill in what you know of `fitting/helis/<heli>/heli.json` (copy one of the existing ones):
rotor radius, blade chord and mass, flying weight, gear and tail ratios, motor (Kv, poles),
ESC, pack (cells, capacity), tail type, the flight controller and its gyro chip. Roll and
pitch inertia, if you can measure them (a bifilar or compound pendulum), help the physical
model most.

## Checking a log

```sh
FIT_DATA_DIR=<a folder for decoded logs> python -I fitting/scripts/intake.py fitting/helis/<heli>/heli.json <log.bbl> ...
```

decodes the files, skips logs it already has, and reports per log what each fit can use
and what is missing (`--dry-run` leaves the heli file alone). Then `fit_heli.py`,
`vibration.py` and `segment.py` run on the heli file (`fitting/README.md`).

## Sending logs

Open an issue or a pull request with the heli file and a link to the logs and the CLI dump.
Logs stay out of the repository unless you say they may go in. Remove anything you do not
want to share first: a GPS track holds where you fly.
