# Ballpark

**[Русская версия](README.ru.md)**

Blind ball-probe calibration of IDEX toolhead offsets for Klipper: the head
finds the ball anywhere in the scan zone (no head geometry assumed), fits a
sphere from nozzle-touch points (LSQ + RANSAC), verifies, switches to head
B, and re-measures head A to catch a nudged ball. Offsets land in
`IDEX_VARS` + `SAVE_OFFSETS_TO_DISK`.

Validated in the [3D simulator](sim/sim3d.html) on stress grids (wire
noise, contact slop, ball bumps): 23/23 success, ~0.03 mm, zero ball moves.

## Install

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

Restart moonraker (`sudo systemctl restart moonraker`), then install from
Mainsail/Fluidd → Update Manager (or wait for the next update check).
Moonraker clones the repo, runs `install.sh` on the host and restarts
klipper; updates roll out the same way.

### Manual (on the klipper host)

```bash
git clone https://github.com/kiryam/ballpark.git ~/ballpark
~/ballpark/install.sh
sudo systemctl restart klipper
```

### From a workstation

```bash
./install.sh your-klipper-host     # pushes, installs, restarts, health-checks
```

`install.sh` copies the plugin to `~/printer_data/config/ballpark/`,
symlinks the module into klippy extras, inserts
`[include ballpark/*.cfg]` into `printer.cfg` (before the `#*#` autosave
block, with a backup) and migrates the old `tool_offset/` copy away.

The only required config option is the probe pin (a normally-closed ball
probe on an endstop input) — edit
`~/printer_data/config/ballpark/tool_offset_sphere.cfg`:

```ini
[tool_offset_sphere]
pin: ^endstop7
```

### Uninstall

```bash
rm ~/klipper/klippy/extras/tool_offset_sphere.py
rm -rf ~/printer_data/config/ballpark ~/ballpark   # + remove the include line
sudo systemctl restart klipper
```

## Run

1. Clean both nozzles, preload the head-B lane, home axes, drop the ball
   into the scan zone (roughly `search_center`, default X165 Y35 ±80/±60).
2. `TOOL_SPHERE_CALIBRATE DRY_RUN=1` — watch the log, offsets untouched.
3. `TOOL_SPHERE_CALIBRATE` — applies and saves the offsets.
4. The first run prints `SPEED-UP CONFIG: ball_top=..` — put that value
   into the config to speed up the next runs (ball XY is never saved).

`TOOL_SPHERE_QUERY_PROBE` reports the probe state.

## Options

| Option | Default | Meaning |
|---|---|---|
| `pin` | — (required) | probe pin, e.g. `^endstop7` (NC) |
| `ball_top` | `0` | ball top height; `0` = auto-discovery |
| `ball_radius` | `5` | probe ball radius |
| `floor_z` | `38` | never probe below this Z (probe body crash guard) |
| `safe_z_cold` | `58` | travel height while `ball_top` is unknown |
| `search_center_x/y`, `search_size_x/y` | 165/35, 160/120 | blind scan zone |
| `probe_speed` / `travel_speed` / `lift_speed` | 4 / 80 / 15 | mm/s |
| `head_switch_b_gcode` / `head_switch_a_gcode` | `T1` / `T0` | carriage switch |

The plugin applies offsets via `SET_GCODE_VARIABLE MACRO=IDEX_VARS` +
`SAVE_OFFSETS_TO_DISK` — make sure your printer defines those macros.

## How it works

Blind scan (multi-resolution grid, the final nozzle-only pass is guaranteed
for any head) → hill-climb by click height → escape rings + jumps along
element offsets → sphere point rings → LSQ sphere fit with RANSAC →
verification press at the fitted center → head B repeats → revision pass A
(guards against a ball nudged between passes) → offsets from the
closest-in-time measurement. Ball height is optional: a cold start
discovers it and prints the config value.

## Simulator

Open `sim/sim3d.html` in a browser (serve the repo root, e.g.
`python3 -m http.server 8799`). The algorithm generates G-code and a
bare-bones executor (G0/G1/G38.2/M117/G4/T0/T1) drives the toolheads:
shared X gantry with parking, a 400×260 bed, carriage nozzle offsets, and
real physics — any lateral touch of the ball with a non-nozzle element
physically moves it and fails the run. URL switches:
`?test=grid&reps=1&noise=1&seed=N` (stress grid),
`?br=4.5&pre=0.25&jit=0.15&bump=1.2&np=10` (ball radius, switch pre-travel,
coordinate slop, ball bump at the head switch, noise %), `?bt=49.5` (known
ball top), `?mirror=x|z` (STL orientation).

## Troubleshooting

- "Probe already triggered prior to movement" — stuck probe or wire noise;
  the module retries glitches automatically. Hardware fix: 4.7–10 kΩ
  external pull-up to 3.3 V.
- "Ball not found in the scan zone" — move the ball closer to
  `search_center` or enlarge the zone.
- A filament change triggered by `T1` would knock the ball off — preload
  the lane first, or set `head_switch_b_gcode` to a low-level switch
  (`SET_DUAL_CARRIAGE CARRIAGE=1` + `ACTIVATE_EXTRUDER EXTRUDER=extruder1`).

## Licenses

Code: Apache-2.0 (see [LICENSE](LICENSE)). Vendored assets keep their
licenses: three.js examples (MIT), Voron StealthBurner / Clockwork2 STLs
(CC-BY-NC-SA, VoronDesign) — see [NOTICE](NOTICE).
