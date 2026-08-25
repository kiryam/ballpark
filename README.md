# Ballpark

**[Русская версия](README.ru.md)**

Blind IDEX toolhead offset calibration for Klipper via a ball probe:
the head finds the ball by itself (no head geometry assumed), locks
onto the apex with a converging stencil of paraboloid fits, verifies
the dome signature, switches to head B, re-measures, and re-checks with
a revision pass. Offsets are applied through `IDEX_VARS` +
`SAVE_OFFSETS_TO_DISK`.

Validated against a physics harness (`tests/sim_harness.py`, ported
from the validated 3D simulator) on randomized heads, ball bumps, wire
noise and ball walk: 98.7% clean completions on unseen seeds, median
offset error **0.05 mm**, p95 0.12 mm, zero hangs (hard press/time
budgets by construction). See [ANALYSIS.md](ANALYSIS.md) for the v8/v9
problem catalog that motivated the rewrite.

## Installation

### Moonraker (recommended)

Add to `moonraker.conf`:

```ini
[update_manager ballpark]
type: git_repo
primary_branch: main
origin: https://github.com/kiryam/ballpark.git
path: ~/ballpark
install_script: install.sh
managed_services: klipper
```

Restart moonraker (`sudo systemctl restart moonraker`), then install
from Mainsail/Fluidd → Update Manager. Moonraker clones the repo, runs
`install.sh` on the host and restarts klipper; updates work the same way.

### Manually (on the klipper host)

```bash
git clone https://github.com/kiryam/ballpark.git ~/ballpark
~/ballpark/install.sh
sudo systemctl restart klipper
```

### From a workstation

```bash
./install.sh your-host             # push, install, restart, verify
```

`install.sh` copies the plugin to `~/printer_data/config/ballpark/`,
symlinks the module into klippy extras, and inserts
`[include ballpark/*.cfg]` into `printer.cfg` (BEFORE the `#*#` autosave
block, with a backup).

The only required config option is the probe pin — write it into
`~/printer_data/config/ballpark/tool_offset_sphere.cfg`:

```ini
[tool_offset_sphere]
pin: ^endstop7
```

### Removal

```bash
rm ~/klipper/klippy/extras/tool_offset_sphere.py
rm -rf ~/printer_data/config/ballpark ~/ballpark   # + remove the include line
sudo systemctl restart klipper
```

## Running

1. Clean nozzles, preload the head-B lane, home the axes. The ball goes
   anywhere the head can reach — for the very first run park the head
   over it (afterwards the remembered position is the hint).
2. `TOOL_SPHERE_CALIBRATE DRY_RUN=1` — a full pass, offsets not applied.
3. `TOOL_SPHERE_CALIBRATE` — measure, verify and apply.

The ball position and height are learned every run and persisted in
`.ball-state.json` next to the config — there is nothing to re-enter
after hardware changes; the algorithm re-anchors itself from head A's
own measurement each run.

`TOOL_SPHERE_QUERY_PROBE` — probe state.
`TOOL_SPHERE_NOISE_TEST` — shake each axis, count probe flickers.
`TOOL_SPHERE_PROBE_TEST` — one probing_move down + up, with pin states.

## Options

| Option | Default | Meaning |
|---|---|---|
| `pin` | — (required) | probe pin, e.g. `^endstop7` (NC) |
| `ball_radius` | 5 | probe ball radius |
| `floor_z` | 38 | never probe below this Z — probe body guard |
| `edge_margin` | 15 | stay this far from the axis limits |
| `probe_speed` / `travel_speed` / `lift_speed` | 4 / 80 / 15 | mm/s |
| `head_switch_b_gcode` / `head_switch_a_gcode` | `T1` / `T0` | carriage switch |

The plugin applies offsets via `SET_GCODE_VARIABLE MACRO=IDEX_VARS` +
`SAVE_OFFSETS_TO_DISK` — those macros must exist on the printer.

## How it works (v10, "converging stencil")

One primitive — a depth-limited vertical press — and three depth tiers
(top-cone / presence / cold) that only use ball geometry, so any head
with any ducts/shrouds works without configuration. A 5-press stencil
fits a paraboloid and jumps to the apex (Newton-style, 2-3 rounds); the
fit itself rejects foreign surfaces, and a ring of presses inside the
35-degree contact cone verifies the dome signature without being able
to nudge the ball. A robust exhaustive outlier fit ignores wire
glitches; head B and a revision pass of head A bracket the offset; a
fresh printer re-runs the B/A pair once with its own first estimate
before applying anything. Every loop has hard budgets — a failure is a
clear error, never a hang.

## Test harness

```bash
python3 tests/sim_harness.py all              # grid + regressions + budgets
python3 tests/sim_harness.py debug stale_bt   # verbose single case
```

Physics: element-based head contacts (ported from the validated
`sim/sim3d.html`), lever pre-travel, X/Y readout slop, readout glitches,
ball nudges from shoulder presses, ball bumps at the head switch, and
the probe body ring. The 3D visual simulator (`sim/sim3d.html`) still
contains the v8/v9 reference implementation.

## Licenses

Code: Apache-2.0 (see [LICENSE](LICENSE)). Vendored assets under their
own licenses: three.js examples (MIT), STL Voron StealthBurner /
Clockwork2 (CC-BY-NC-SA, VoronDesign) — see [NOTICE](NOTICE).
