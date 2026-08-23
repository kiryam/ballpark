# Ballpark — IDEX toolhead offset calibration via a ball probe.
# Port of the sphere-v8 algorithm from the 3D simulator (sim/sim3d.html),
# validated there on stress grids (noise/slop/ball bumps): 23/23, ~0.03 mm.
#
# Cycle: blind scan -> hill-climb by click height -> escape rings ->
# sphere rings -> LSQ fit + RANSAC -> verify -> head B -> revision A ->
# SET_GCODE_VARIABLE IDEX_VARS + SAVE_OFFSETS_TO_DISK.
#
# Minimal install (the ONLY required option is the pin):
#
#   [tool_offset_sphere]
#   pin: ^endstop7
#
# No head geometry is assumed (multi-resolution scan down to a
# nozzle-only grid). Ball height is optional: a cold start discovers it
# and prints the value to set for faster runs. Ball XY is never saved.

import math

class ToolOffsetSphere:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        ppins = self.printer.lookup_object('pins')
        pin = config.get('pin')
        ppins.allow_multi_use_pin(pin.replace('^', '').replace('!', ''))
        pin_params = ppins.lookup_pin(pin, can_invert=True, can_pullup=True)
        self.mcu_endstop = pin_params['chip'].setup_pin('endstop', pin_params)
        self.reactor = self.printer.get_reactor()
        self.ball_r = config.getfloat('ball_radius', 5., above=1.)
        self.ball_top = config.getfloat('ball_top', 0., minval=0.)
        self.floor_z = config.getfloat('floor_z', 38., above=0.)
        self.safe_z_cold = config.getfloat('safe_z_cold', 58., above=10.)
        self.probe_speed = config.getfloat('probe_speed', 4., above=0.)
        self.travel_speed = config.getfloat('travel_speed', 80., above=0.)
        self.lift_speed = config.getfloat('lift_speed', 15., above=0.)
        self.cx = config.getfloat('search_center_x', 165.)
        self.cy = config.getfloat('search_center_y', 35.)
        self.sx = config.getfloat('search_size_x', 160., above=20.)
        self.sy = config.getfloat('search_size_y', 120., above=20.)
        self.log_level = config.getint('log_level', 1, minval=0, maxval=2)
        self.climb_budget = config.getint('climb_budget', 350, minval=10)
        self.max_off_xy = config.getfloat('max_offset_xy', 15., above=1.)
        self.max_off_z = config.getfloat('max_offset_z', 5., above=0.5)
        self.y_margin = config.getfloat('y_margin', 15., above=0.)
        self.esc_offsets = []
        for tok in config.get('escape_offsets', '-15,0 15,0').split():
            dx, dy = [float(v) for v in tok.split(',')]
            self.esc_offsets.append((dx, dy))
        # plain T1/T0: fine when the head-B lane is preloaded (a filament
        # change at switch time would knock the ball off)
        self.switch_b = config.get('head_switch_b_gcode', 'T1')
        self.switch_a = config.get('head_switch_a_gcode', 'T0')
        # axis bounds and auto parking (filled at run start)
        self.bounds = None
        # homing_move computes the halt position from the steppers attached
        # to the endstop; without any it returns the start position and the
        # trigger arming misbehaves. Attach the Z rails (the probing axis)
        # plus the IDEX primary X rail, like the old tools_calibrate did.
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.printer.register_event_handler("klippy:ready",
                                            self._handle_connect)
    def _handle_connect(self, eventtime=None):
        if getattr(self, '_steppers_attached', False):
            return
        try:
            kin = self._toolhead().get_kinematics()
        except Exception:
            return
        for st in kin.get_steppers():
            # Z rails ONLY: probing_move's check_no_movement() requires every
            # registered stepper to move during the probe - a vertical
            # descent never moves X, so attaching stepper_x turns EVERY
            # probe into "Probe triggered prior to movement"
            if st.get_name().startswith('stepper_z'):
                self.mcu_endstop.add_stepper(st)
        self._steppers_attached = True
        self.gcode.register_command('TOOL_SPHERE_CALIBRATE', self.cmd_run,
                                    desc=self.cmd_run.__doc__)
        self.gcode.register_command('TOOL_SPHERE_QUERY_PROBE', self.cmd_query,
                                    desc="Ball probe state")
        self.gcode.register_command('TOOL_SPHERE_NOISE_TEST', self.cmd_noise,
                                    desc="Shake each axis and count probe flickers")
        self.gcode.register_command('TOOL_SPHERE_PROBE_TEST', self.cmd_probetest,
                                    desc="One probing_move down + up, with pin states")
    # ---------------- helpers ----------------
    def _log(self, msg, level=1):
        if level > self.log_level:
            return
        prefix = {0: '!!', 1: '//', 2: '##'}[min(level, 2)]
        self.gcode.respond_raw("%s %s" % (prefix, msg))
    def _toolhead(self):
        return self.printer.lookup_object('toolhead')
    def _pos(self):
        return self._toolhead().get_position()
    def _move(self, x, y, z, speed):
        # machine coordinates of the active head carriage
        self._toolhead().manual_move([x, y, z], speed)
    def _safe_z(self, ball_top):
        return (ball_top + 2.5) if ball_top else self.safe_z_cold
    def _bounds(self):
        # axis bounds, filled lazily (any command can run first)
        if self.bounds is None:
            kin = self._toolhead().get_kinematics().get_status(
                self.reactor.monotonic())
            self.bounds = (kin['axis_minimum'][0], kin['axis_maximum'][0],
                           kin['axis_minimum'][1], kin['axis_maximum'][1])
        return self.bounds
    def _park(self, left):
        b = self._bounds()
        x = (b[0] + 10.) if left else (b[1] - 10.)
        return x
    def _probe_down(self, x, y, safe_z, tag=''):
        # 3-phase travel + descent to floor_z, stop on click.
        # Returns (x, y, click_z) or None (miss)
        pos = self._pos()
        if pos[2] < safe_z - .01:
            self._move(pos[0], pos[1], safe_z, self.lift_speed)
        # travel at the CURRENT height if it is higher (e.g. the T1 macro's
        # z-hop) - never descend while moving XY: head B's nozzle sits
        # lower than A's by an unknown dZ and could plane through the ball
        tz = max(pos[2], safe_z)
        self._move(x, y, tz, self.travel_speed)
        if tz > safe_z + .01:
            self._move(x, y, safe_z, self.lift_speed)
        # settle: the ball lever rings for a few hundred ms after ANY move;
        # arming the endstop mid-ring latches a false instant trigger
        # ("Probe triggered prior to movement"). Let it decay first.
        self._toolhead().dwell(0.4)
        if self.mcu_endstop.query_endstop(self._toolhead().get_last_move_time()):
            raise self.printer.command_error(
                "ball probe reports TRIGGERED before the descent (pressed, "
                "jammed or wrong polarity) - check the switch")
        phoming = self.printer.lookup_object('homing')
        attempts = 3
        while True:
            try:
                tpos = [x, y, self.floor_z]
                epos = phoming.probing_move(self.mcu_endstop, tpos,
                                            self.probe_speed)
                break
            except self.printer.command_error as e:
                reason = str(e)
                if "No trigger" in reason:
                    # clean miss - the descent completed without a click
                    self._log(". miss %s(%.1f,%.1f)" % (tag, x, y), 2)
                    return None
                if "prior to movement" in reason and attempts > 0:
                    # the lever has not swung back yet (or a bounce): wait
                    # for a stable open state up to 2s, then retry
                    attempts -= 1
                    toolhead = self._toolhead()
                    ok = False
                    for _ in range(20):
                        toolhead.dwell(0.1)
                        if not self.mcu_endstop.query_endstop(
                                toolhead.get_last_move_time()):
                            ok = True
                            break
                    if ok:
                        self._log("probe still pressed after the click - "
                                  "waited for release, retrying (%d left)"
                                  % attempts)
                        continue
                raise self.printer.command_error(reason)
        z = epos[2]
        if z <= self.floor_z + 0.01:
            self._log(". miss %s(%.1f,%.1f)" % (tag, x, y), 2)
            return None
        if z >= safe_z - 1.5:
            # triggered right at the travel height: the switch is not on the
            # ball - noise or a knocked-off/unplugged probe. Never treat as
            # a ball contact (it used to poison the whole calibration).
            self._log("!! probe triggered at travel height %.1f (%s) - "
                      "switch noise or the ball probe is off the bed" % (z, tag), 0)
            return None
        # real click: give the ball lever time to swing back before the
        # next probe arms (else the next start sees the pin still pressed)
        self._toolhead().dwell(0.4)
        return (x, y, z)
    # ---------------- sphere math ----------------
    def _fit_sphere(self, pts):
        n = len(pts)
        if n < 4:
            return None
        M = [[0.]*5 for _ in range(4)]
        for p in pts:
            x, y, z = p
            row = [2*x, 2*y, 2*z, 1.]
            for i in range(4):
                for j in range(4):
                    M[i][j] += row[i]*row[j]
                M[i][4] += row[i]*(x*x + y*y + z*z)
        for col in range(4):
            piv = col
            for r2 in range(col+1, 4):
                if abs(M[r2][col]) > abs(M[piv][col]):
                    piv = r2
            M[col], M[piv] = M[piv], M[col]
            if abs(M[col][col]) < 1e-9:
                return None
            for r2 in range(col+1, 4):
                f = M[r2][col]/M[col][col]
                for c2 in range(col, 5):
                    M[r2][c2] -= f*M[col][c2]
        sol = [0.]*4
        for r2 in range(3, -1, -1):
            s = M[r2][4]
            for c2 in range(r2+1, 4):
                s -= M[r2][c2]*sol[c2]
            sol[r2] = s/M[r2][r2]
        cx, cy, cz, c = sol
        r2 = c + cx*cx + cy*cy + cz*cz
        if r2 <= 0:
            return None
        return (cx, cy, cz, math.sqrt(r2))
    def _ransac_sphere(self, pts):
        import random
        rng = random.Random()
        if len(pts) < 4:
            return None
        lo, hi = self.ball_r - .6, self.ball_r + .6
        best_set, best_cnt = None, -1
        for _ in range(80):
            idx = rng.sample(range(len(pts)), 4)
            f = self._fit_sphere([pts[i] for i in idx])
            if f is None or not (lo <= f[3] <= hi):
                continue
            cx, cy, cz, r = f
            inl = [p for p in pts
                   if abs(math.dist(p, (cx, cy, cz)) - r) < 0.2]
            if len(inl) > best_cnt:
                best_cnt, best_set = len(inl), inl
        if best_set and len(best_set) >= 4:
            return self._fit_sphere(best_set)
        return self._fit_sphere(pts)
    # ---------------- algorithm phases ----------------
    def _scan_pitches(self):
        # Multi-resolution blind scan, NO head geometry assumptions.
        # Pass 1 assumes a typical heater block listens wide (fast); the last
        # pass is nozzle-only and is GUARANTEED for any head: worst grid-node
        # to ball offset = pitch/sqrt(2) <= nozzle hearing radius (R + 0.5).
        r = self.ball_r
        sched = [min(20., math.floor((12. + r) * 1.35)),   # typical block
                 min(15., math.floor((6. + r) * 1.35)),    # sock-sized
                 max(6., math.floor((r + 0.5) * 1.35))]    # nozzle-only
        out = []
        for p in sched:
            if not out or p < out[-1] - .5:
                out.append(p)
        return out
    def _scan(self, ball_top):
        safe = self._safe_z(ball_top)
        x0 = self.cx - self.sx/2.; x1 = self.cx + self.sx/2.
        y0 = self.cy - self.sy/2.; y1 = self.cy + self.sy/2.
        # clamp the search rectangle to the machine's own axis bounds -
        # an out-of-range grid point would abort the whole run
        if self.bounds:
            x0 = max(x0, self.bounds[0] + 2.); x1 = min(x1, self.bounds[1] - 2.)
            y0 = max(y0, self.bounds[2] + self.y_margin)
            y1 = min(y1, self.bounds[3] - self.y_margin)
        pitches = self._scan_pitches()
        for pi, pitch in enumerate(pitches):
            xs, ys = [], []
            x = x0
            while x <= x1 + .01:
                xs.append(x); x += pitch
            y = y0
            while y <= y1 + .01:
                ys.append(y); y += pitch
            ys.sort(key=lambda v: abs(v - self.cy))
            pts = []
            flip = self.cx > (xs[0] + xs[-1])/2.
            for ry in ys:
                row = xs[:] if not flip else xs[::-1]
                for rx in row:
                    pts.append((rx, ry))
                flip = not flip
            self._log("blind search pass %d/%d: grid %.0fmm, %d points"
                      % (pi + 1, len(pitches), pitch, len(pts)))
            for (x, y) in pts:
                hit = self._probe_down(x, y, safe, 'scan')
                if hit:
                    return hit
        return None
    def _climb(self, best, ball_top):
        # hill-climb by click height + escape rings + jumps.
        # Returns the best click (x, y, z) and nozzle-point samples for the fit
        samples = []
        cur = best; d = 4.
        clicks = 0
        safe = self._safe_z(ball_top)
        xmin, xmax, ymin, ymax = self.bounds[0], self.bounds[1], self.bounds[2], self.bounds[3]
        clx = lambda v: max(xmin, min(xmax, v))
        cly = lambda v: max(ymin + self.y_margin,
                             min(ymax - self.y_margin, v))
        while True:
            if clicks >= self.climb_budget:
                self._log("climb budget (%d clicks) reached - settling on "
                          "the best click (a real ball settles slightly "
                          "between presses, so the climb self-terminates)"
                          % self.climb_budget, 0)
                break
            if clicks and clicks % 25 == 0:
                self._log("climb: %d clicks, best Z%.2f at %.1f,%.1f"
                          % (clicks, cur[2], cur[0], cur[1]))
            probes = []
            for (dx, dy) in ((1,0),(-1,0),(0,1),(0,-1)):
                clicks += 1
                hit = self._probe_down(clx(cur[0]+dx*d), cly(cur[1]+dy*d), safe, 'climb')
                if hit:
                    samples.append(hit); probes.append(hit)
            up = None
            for h in probes:
                if h[2] > cur[2] + 0.03 and (up is None or h[2] > up[2]):
                    up = h
            if up:
                cur = up; d = min(4., d*1.5); continue
            top = max(probes, key=lambda h: h[2]) if probes else None
            eq = any(abs(h[2]-cur[2]) <= 0.03 for h in probes)
            if eq and top and top[2] >= cur[2] - 0.01:
                # apex plateau: equal clicks - go straight to sphere rings
                return cur, samples
            if not eq and d > 0.7:
                d /= 2.; continue
            # escape rings (early exit on the first good click)
            escaped = None
            for rr in (5., 9., 14., 20.):
                for k in range(8):
                    a = k/8.*2.*math.pi
                    clicks += 1
                    hit = self._probe_down(clx(cur[0]+rr*math.cos(a)),
                                           cly(cur[1]+rr*math.sin(a)), safe, 'ring')
                    if hit:
                        samples.append(hit)
                        if hit[2] > cur[2] + 0.3:
                            escaped = hit; break
                if escaped: break
            if escaped:
                cur = escaped; d = 4.; continue
            # jumps along element offsets (known head geometry)
            jumped = None
            for (dx, dy) in self.esc_offsets:
                clicks += 1
                hit = self._probe_down(clx(cur[0]+dx), cly(cur[1]+dy), safe, 'jump')
                if hit and hit[2] > cur[2] - 1.0:
                    samples.append(hit); jumped = hit; break
            if jumped:
                cur = jumped; d = 2.; continue
            return cur, samples
    def _rings(self, center, ball_top, radii):
        samples = []
        safe = self._safe_z(ball_top)
        for rr in radii:
            for k in range(6):
                a = k/6.*2.*math.pi + (math.pi/6. if rr > 3. else 0.)
                hit = self._probe_down(center[0]+rr*math.cos(a),
                                       center[1]+rr*math.sin(a), safe, 'sphere')
                if hit:
                    samples.append(hit)
        return samples
    def _seed_b(self, apex, ball_top):
        # Find the ball again with head B. B's nozzle offset is exactly what
        # we are calibrating, so do NOT press at A's apex directly and do NOT
        # travel at the tight A-margin: scan a small serpentine around the
        # apex at a tall margin (B's dZ is unknown), vertical descents only.
        # The first click seeds the climb, which refines from there.
        safe_hi = ball_top + 8.
        self._log("head B: local scan around apex A (tall margin Z%.1f)"
                  % safe_hi)
        pitch = 10.
        pts = []
        for iy, dy in enumerate((20., 10., 0., -10., -20.)):
            row = (-20., -10., 0., 10., 20.)
            if iy % 2:
                row = row[::-1]
            for dx in row:
                pts.append((apex[0] + dx, apex[1] + dy))
        for (x, y) in pts:
            seed = self._probe_down(x, y, safe_hi, 'Bscan')
            if seed:
                self._log("head B: first click at %.1f,%.1f (Z%.2f)"
                          % seed)
                return seed
        return None
    def _measure(self, seed_hit, ball_top):
        # climb -> rings -> fit -> verify. Returns the apex (x,y,z)
        best, samples = self._climb(seed_hit, ball_top)
        R = self.ball_r
        def try_fit(pts):
            f = self._ransac_sphere(pts)
            if f and R-.6 <= f[3] <= R+.6:
                return f
            return None
        all_pts = samples[:]
        fit = None; fittry = 0; vretry = 0
        for radii in ((R*.52, R*.86), (R*.3, R*.64, R)):
            all_pts += self._rings(best, ball_top, radii)
            fit = try_fit(all_pts)
            if fit or fittry:
                break
            fittry += 1
        if fit:
            cx, cy, cz, r = fit
            top = cz + r
            # verify: a click at the center must match the sphere top
            # (a wire glitch is not a mismatch - just repeat the press)
            v = None
            for _ in range(3):
                v = self._probe_down(cx, cy, self._safe_z(ball_top), 'verify')
                if v:
                    break
            if v and abs(v[2] - top) <= 0.15:
                self._log("verify: Z%.2f = sphere %.2f - ok" % (v[2], top))
                return (cx, cy, top)
            if vretry < 2:
                vretry += 1
                all_pts += self._rings((cx, cy), ball_top, (R*.3, R*.64, R))
                fit = try_fit(all_pts)
                if fit:
                    cx, cy, cz, r = fit
                    v2 = self._probe_down(cx, cy, self._safe_z(ball_top), 'verify2')
                    if v2 and abs(v2[2] - (cz+r)) <= 0.15:
                        return (cx, cy, cz + r)
        self._log("sphere fit did not converge - using best click", 0)
        return best
    # ---------------- main cycle ----------------
    def cmd_query(self, gcmd):
        toolhead = self._toolhead()
        pt = toolhead.get_last_move_time()
        self.gcode.respond_info("ball probe: %s" % (
            "TRIGGERED" if self.mcu_endstop.query_endstop(pt) else "open"))
    def cmd_noise(self, gcmd):
        # Which motion shakes a false trigger out of the ball switch:
        # wiggle one axis at a time and sample the endstop between moves.
        toolhead = self._toolhead()
        if toolhead.get_status(self.reactor.monotonic())['homed_axes'] != 'xyz':
            raise gcmd.error("Home all axes first")
        self._bounds()
        pos = list(self._pos())
        z_safe = max(pos[2], self.safe_z_cold)
        def sample(n):
            hits = 0
            for _ in range(n):
                toolhead.dwell(0.02)
                if self.mcu_endstop.query_endstop(
                        toolhead.get_last_move_time()):
                    hits += 1
            return hits
        def wiggle(axis, delta, times):
            hits = 0
            for i in range(times):
                v = pos[axis] + (delta if i % 2 == 0 else -delta)
                p = list(self._pos()); p[axis] = v
                self._toolhead().manual_move(p, 60.)
                hits += sample(4)
            return hits
        res = {}
        res['idle'] = sample(20)
        # park away from the ball zone first (X edge, ball sits near center)
        self._move(self._park(True), pos[1], z_safe, self.travel_speed)
        pos = list(self._pos())
        res['X_wiggle'] = wiggle(0, 3., 5)
        res['Y_wiggle'] = wiggle(1, 3., 5)
        res['Z_wiggle'] = wiggle(2, 1., 5)
        # the real failure mode: 3 probe descents far from the ball
        inst = 0
        for _ in range(3):
            hit = self._probe_down(self._park(True) + 10., pos[1],
                                   z_safe, 'noise')
            if hit is None and self._log:
                pass
        self.gcode.respond_info(
            "noise test (flickers): idle %d/20, X %d/20, Y %d/20, Z %d/20"
            % (res['idle'], res['X_wiggle'], res['Y_wiggle'], res['Z_wiggle']))
    def cmd_probetest(self, gcmd):
        # Decisive test: a downward probing_move that "clicks" at its very
        # start while query_endstop says open AND an upward probing_move that
        # "clicks" too = the hardware trigger path is misbehaving, not the
        # switch. Prints pin state before/after each attempt.
        toolhead = self._toolhead()
        if toolhead.get_status(self.reactor.monotonic())['homed_axes'] != 'xyz':
            raise gcmd.error("Home all axes first")
        self._bounds()
        phoming = self.printer.lookup_object('homing')
        pos = list(self._pos())
        if pos[2] < self.safe_z_cold:
            self._move(pos[0], pos[1], self.safe_z_cold, self.lift_speed)
            pos = list(self._pos())
        def q(tag):
            v = self.mcu_endstop.query_endstop(
                toolhead.get_last_move_time())
            self.gcode.respond_info("// %s: query=%s" % (tag, v))
            return v
        for i in range(3):
            q("down %d before" % i)
            p0 = list(self._pos())
            try:
                epos = phoming.probing_move(self.mcu_endstop,
                                            [p0[0], p0[1], p0[2] - 2.],
                                            self.probe_speed)
            except self.printer.command_error as e2:
                q("down %d after" % i)
                self.gcode.respond_info(
                    "// down %d: clean miss (no trigger): %s" % (i, e2))
                self._move(p0[0], p0[1], p0[2], self.lift_speed)
                continue
            q("down %d after" % i)
            self.gcode.respond_info(
                "// down %d: start Z%.3f -> reported Z%.3f (triggered at +%0.3fmm)"
                % (i, p0[2], epos[2], p0[2] - epos[2]))
            # lift back
            self._move(p0[0], p0[1], p0[2], self.lift_speed)
            q("up %d before" % i)
            p1 = list(self._pos())
            epos2 = phoming.probing_move(self.mcu_endstop,
                                         [p1[0], p1[1], p1[2] + 2.],
                                         self.probe_speed)
            q("up %d after" % i)
            self.gcode.respond_info(
                "// up %d: start Z%.3f -> reported Z%.3f"
                % (i, p1[2], epos2[2]))
            self._move(p1[0], p1[1], p1[2], self.lift_speed)
    def cmd_run(self, gcmd):
        toolhead = self._toolhead()
        if toolhead.get_status(self.reactor.monotonic())['homed_axes'] != 'xyz':
            raise gcmd.error("Home all axes first")
        self._bounds()
        ball_top = gcmd.get_float('BALL_TOP', self.ball_top, minval=0.)
        dry = gcmd.get_int('DRY_RUN', 0)
        self._log("=== ball-probe offset calibration (sphere v8) ===")
        self._log("ball top: %s" % ("%.1f (from config)" % ball_top
                                       if ball_top else "auto-discovery"))
        travel_z = lambda: max(ball_top + 5., self.safe_z_cold)
        # --- pass A ---
        seed = self._scan(ball_top)
        if not seed:
            raise gcmd.error("Ball not found in the scan zone - move the probe")
        apex_a = self._measure(seed, ball_top)
        self._log("apex A: X%.2f Y%.2f Z%.2f" % apex_a, 1)
        if not ball_top:
            ball_top = apex_a[2]
            self._log("SPEED-UP CONFIG: ball_top=%.2f "
                      "(set it in [tool_offset_sphere]; ball XY is not saved)"
                      % ball_top, 0)
        # --- park A, head B ---
        self._move(apex_a[0], apex_a[1], travel_z(), self.lift_speed)
        self._move(self._park(True), apex_a[1], travel_z(), self.travel_speed)
        self.gcode.run_script_from_command(self.switch_b)
        seed = self._seed_b(apex_a, ball_top)
        if not seed:
            raise gcmd.error("Head B did not find the ball near apex A")
        apex_b = self._measure(seed, ball_top)
        self._log("apex B: X%.2f Y%.2f Z%.2f" % apex_b, 1)
        # --- park B, revision A ---
        self._move(self._park(False), apex_a[1], travel_z(), self.travel_speed)
        self.gcode.run_script_from_command(self.switch_a)
        seed = self._probe_down(apex_a[0], apex_a[1],
                                self._safe_z(ball_top), 'revision')
        apex_a2 = self._measure(seed, ball_top) if seed else apex_a
        drift = math.dist(apex_a, apex_a2)
        if drift > 0.2:
            self._log("ball moved between passes (drift %.2fmm) - "
                      "offset from the revision" % drift, 0)
        self._log("apex A (revision): X%.2f Y%.2f Z%.2f" % apex_a2, 1)
        # --- offset ---
        off = (apex_b[0]-apex_a2[0], apex_b[1]-apex_a2[1], apex_b[2]-apex_a2[2])
        self._log("MEASURED B offset: dX%.3f dY%.3f dZ%.3f" % off, 1)
        if (abs(off[0]) > self.max_off_xy or abs(off[1]) > self.max_off_xy
                or abs(off[2]) > self.max_off_z):
            # way outside what a head drift can be: the run was corrupted
            # (probe knocked the ball off / false triggers). NEVER apply.
            raise gcmd.error(
                "Measured offset dX%.2f dY%.2f dZ%.2f exceeds the plausible "
                "range (+/-%.0f XY, +/-%.0f Z) - the ball was probably "
                "knocked off mid-run or the switch false-triggered. "
                "Offsets NOT applied." % (off + (self.max_off_xy,
                                                 self.max_off_z)))
        if dry:
            self._log("DRY_RUN: offsets are not applied", 1)
            return
        for name, val in (('offset_x', off[0]), ('offset_y', off[1]),
                          ('offset_z', off[2])):
            self.gcode.run_script_from_command(
                "SET_GCODE_VARIABLE MACRO=IDEX_VARS VARIABLE=%s VALUE=%.3f"
                % (name, val))
        self.gcode.run_script_from_command("SAVE_OFFSETS_TO_DISK")
        try:
            self.gcode.run_script_from_command("DISPLAY_CURRENT_OFFSETS")
        except self.printer.command_error:
            pass
        self.gcode.run_script_from_command(
            "M117 B offset: dX%.3f dY%.3f dZ%.3f" % off)
    cmd_run.__doc__ = ("Full head-B offset calibration via the ball probe "
                       "(scan->climb->sphere->verify->revision A). "
                       "Params: BALL_TOP=.., DRY_RUN=1")

def load_config(config):
    return ToolOffsetSphere(config)
