# Ballpark

**[English version](README.md)**

Слепая калибровка офсетов сопел IDEX-голов для Klipper по шаровому пробу:
голова сама находит шар в зоне скана (без предположений о геометрии головы),
фитит сферу по точкам касания соплом (МНК + RANSAC), верифицирует,
переключается на голову B и перемеряет голову A (ловит смещённый шар).
Офсеты сохраняются через `IDEX_VARS` + `SAVE_OFFSETS_TO_DISK`.

Проверено в [3D-симуляторе](sim/sim3d.html) на стресс-сетках (шум провода,
люфт контакта, толчки шара): 23/23 успеха, ~0.03 мм, ноль смещений шара.

## Установка

### Moonraker (рекомендуется)

Добавить в `moonraker.conf`:

```ini
[update_manager ballpark]
type: git_repo
primary_branch: main
origin: https://github.com/kiryam/ballpark.git
path: ~/ballpark
install_script: install.sh
managed_services: klipper
```

Перезапустить moonraker (`sudo systemctl restart moonraker`), затем
установить из Mainsail/Fluidd → Update Manager (или дождаться проверки
обновлений). Moonraker клонирует репозиторий, выполняет `install.sh`
на хосте и перезапускает klipper; обновления накатываются так же.

### Вручную (на хосте с klipper)

```bash
git clone https://github.com/kiryam/ballpark.git ~/ballpark
~/ballpark/install.sh
sudo systemctl restart klipper
```

### С рабочей машины

```bash
./install.sh ваш-хост              # залить, установить, перезапустить, проверить
```

`install.sh` копирует плагин в `~/printer_data/config/ballpark/`, делает
симлинк модуля в klippy extras, вставляет `[include ballpark/*.cfg]` в
`printer.cfg` (ДО автосейв-блока `#*#`, с бэкапом) и переносит старую
копию из `tool_offset/`.

Единственный обязательный параметр конфига — пин проба (шаровой проб,
НЗ, на входе концевика) — вписать в
`~/printer_data/config/ballpark/tool_offset_sphere.cfg`:

```ini
[tool_offset_sphere]
pin: ^endstop7
```

### Удаление

```bash
rm ~/klipper/klippy/extras/tool_offset_sphere.py
rm -rf ~/printer_data/config/ballpark ~/ballpark   # + убрать строку include
sudo systemctl restart klipper
```

## Запуск

1. Сопла чистые, лейна головы B загружена, оси огомнены, шар в зоне скана
   (примерно `search_center`, по умолчанию X165 Y35 ±80/±60).
2. `TOOL_SPHERE_CALIBRATE DRY_RUN=1` — прогон с логом, офсеты не трогаются.
3. `TOOL_SPHERE_CALIBRATE` — применить и сохранить офсеты.
4. Первый прогон напечатает `SPEED-UP CONFIG: ball_top=..` — вписать
   значение в конфиг для ускорения следующих пусков (XY шара не
   сохраняется никогда).

`TOOL_SPHERE_QUERY_PROBE` — состояние проба.

## Параметры

| Параметр | По умолчанию | Смысл |
|---|---|---|
| `pin` | — (обязателен) | пин проба, напр. `^endstop7` (НЗ) |
| `ball_top` | `0` | высота вершины шара; `0` = автопоиск |
| `ball_radius` | `5` | радиус шара проба |
| `floor_z` | `38` | ниже этого Z не зондировать (защита от корпуса проба) |
| `safe_z_cold` | `58` | высота переходов, пока `ball_top` неизвестен |
| `search_center_x/y`, `search_size_x/y` | 165/35, 160/120 | зона слепого скана |
| `probe_speed` / `travel_speed` / `lift_speed` | 4 / 80 / 15 | мм/с |
| `head_switch_b_gcode` / `head_switch_a_gcode` | `T1` / `T0` | переключение кареток |

Плагин применяет офсеты через `SET_GCODE_VARIABLE MACRO=IDEX_VARS` +
`SAVE_OFFSETS_TO_DISK` — эти макросы должны быть определены на принтере.

## Как это работает

Слепой скан (многоразрешённая сетка; финальный проход «только сопло»
гарантирован для любой головы) → восхождение по высоте клика → кольца
побега + прыжки по выносам элементов → кольца точек сферы → МНК-фит с
RANSAC → верификационное нажатие в центре фита → голова B → ревизия A
(страховка от смещения шара между проходами) → офсет от ближайшего по
времени замера. Высота шара опциональна: холодный старт сам измеряет её и
печатает значение для конфига.

## Симулятор

Открыть `sim/sim3d.html` в браузере (каталог репозитория подать, например,
`python3 -m http.server 8799`). Алгоритм генерирует G-код, а чистый
исполнитель (G0/G1/G38.2/M117/G4/T0/T1) двигает головы: общая балка X с
парковками, стол 400×260, ноузл-офсеты кареток и честная физика — любое
боковое касание шара не соплом физически смещает шар и валит прогон.
Параметры URL: `?test=grid&reps=1&noise=1&seed=N` (стресс-сетка),
`?br=4.5&pre=0.25&jit=0.15&bump=1.2&np=10` (радиус шара, предпрогиб
концевика, люфт координат, толчок шара при смене головы, % шума),
`?bt=49.5` (известная высота шара), `?mirror=x|z` (ориентация STL).

## Если что-то не так

- «Probe already triggered prior to movement» — зажат проб или наводка;
  модуль сам ретраит наводки. Аппаратно: внешняя подтяжка 4.7–10 кОм к 3.3 В.
- «Ball not found in the scan zone» — подвинуть шар ближе к
  `search_center` или расширить зону.
- Смена филамента от `T1` снесёт шар — предзагрузить лейну или задать
  низкоуровневое переключение в `head_switch_b_gcode`
  (`SET_DUAL_CARRIAGE CARRIAGE=1` + `ACTIVATE_EXTRUDER EXTRUDER=extruder1`).

## Лицензии

Код: Apache-2.0 (см. [LICENSE](LICENSE)). Вендорные ассеты — под своими
лицензиями: примеры three.js (MIT), STL Voron StealthBurner / Clockwork2
(CC-BY-NC-SA, VoronDesign) — см. [NOTICE](NOTICE).
