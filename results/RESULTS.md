# Results

The full result set lives in the wiki, which carries the per car and per condition detail:
https://github.com/ITakeCake/DynamicABS/wiki

## What was measured

Four cars, the ETK 800, the Hirochi SBR4, the Hirochi Scintilla and the Gavril Roamer, against the
game's own ABS and against the game's ABS with traction control and stability control.

| Campaign | Stops | What it covers |
|---|---|---|
| Straight line | 450 | ten stops per car per speed, from 60 up to 180 miles per hour |
| Cornering | 99 | three steer angles at 60 miles per hour with full brake |
| Surface Suite | 324 | ice, grass, sand, inclines, a jump and three bump sections |
| Build comparison | 501 | 27 archived controller builds on one car, straight line and cornering |
| Measurement validation | 40 | the checks described below |

## Headline

Straight line, averaged across cars: plus 0.0046 g at 60 miles per hour, rising to plus 0.1156 g at 180.
Cornering, averaged across cars: 5.81 metres shorter, winning all twelve car and steer combinations.
Surfaces are car dependent and are best read per car in the wiki rather than as an average.

## Measurement validation

The campaigns were driven with the simulation running faster than real time, and in a headless instance.
Both were tested rather than assumed, because either could have handicapped the baseline. Forty stops on
one car at one speed, alternating one run at a time so drift falls on both sides equally:

| | normal speed | raised speed |
|---|---|---|
| headless | 0.8990 g | 0.8951 g |
| windowed | 0.9065 g | 0.8971 g |

The spread across all four is 0.011 g against a run to run spread of 0.026, so neither setting changes
a result. Stops are recorded by the brake test mod itself, ending when speed falls to 1.0 metre per
second, which is the game's own cutoff from wheels.lua. The automation calls the same function the mod's
own interface calls and reads the mod's own output file, so there is one measurement path, not two.

## A note on the baseline

Those forty stops also showed that the game's own ABS is bimodal on the Gavril Roamer. It either
modulates properly and stops in about 155 metres from 120 miles per hour, or it under modulates and
stops in about 166 metres. It found the better mode in 11 of 40 stops. Dynamic ABS reached the same
ceiling in 10 of 10. The gap on that car is therefore mostly consistency rather than peak grip, and the
raw data for every one of those stops is in [data](data) so the split can be checked.

## Raw data

Every run behind these numbers is in [data](data), one row per stop, including the runs that were
excluded for damage, for spinning, or for never reaching the recording speed. Exclusions carry their
reason in the status column.

## Older campaigns

[RESULTS_SHEET.md](RESULTS_SHEET.md) and [results.xlsx](results.xlsx) hold the 2026-09-05 campaign. That
campaign braked two miles per hour above the recording speed, so its numbers are internally consistent
but are not pooled with the current ones.
