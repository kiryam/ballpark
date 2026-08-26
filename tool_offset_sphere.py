# Ballpark v10 - IDEX toolhead offset calibration via a ball probe.
#
# Complete rewrite of the v8/v9 scan/hill-climb/hop-table algorithm
# (see ANALYSIS.md for the problem catalog). Core ideas:
#   - one primitive: a depth-limited vertical press; everything else is
#     built from presses;
#   - a local paraboloid fit is BOTH the apex estimator (Newton-style
#     convergence, 2-3 rounds) AND the "is it really the ball" test
#     (a flat foreign contact does not converge) - verification happens
#     inside every round, not after the whole measurement;
#   - no head geometry anywhere: foreign (side) clicks are handled by
#     expanding rings that only use ball geometry; works for any head;
#   - ball_top is self-healing: head A's measured apex re-anchors the
#     run and is persisted in the state file - no manual config;
#   - every loop has a budget; exhaustion raises a clear error instead
#     of hanging;
#   - the algorithm core is pure Python (class BallparkCore) and talks
#     to hardware through a small interface object - the same core runs
#     on the printer (Klipper adapter below) and in the test harness
#     (tests/sim_harness.py).
#
# Config: pin (required) + a handful of physical constants. Ball XY and
# ball_top live in <config>/ballpark/.ball-state.json and update themselves.

import math
import os
import json
import time


# =====================================================================
#  Core - pure algorithm, no Klipper dependencies.
# =====================================================================

class CoreError(Exception):
    pass


class BallparkCore:
    # tier depths (mm below the working top)
    DIP_TOP = 1.0          # top-cone tier: only the ~3.2mm cone clicks
    DIP_PRESENCE = 3.5     # presence tier: the ball shoulder is heard
    DIP_ASCEND = 0.7       # ring search floor below a foreign click
    # convergence
    STENCIL_S = 2.2        # first-round stencil radius
    # polish ring: 8 points on ONE radius - maximum slope leverage per
    # press (the apex xy comes from symmetric z-differences; sensitivity
    # dz/du grows with u). Must stay INSIDE the 35-degree contact cone
    # (R*sin35 = 2.87 for R=5): presses there never nudge the ball, so
    # head B and the revision measure the same surface. The rounds
    # deliver P within ~0.15mm, so 2.6 + 0.15 < 2.87 holds.
    POLISH_R = 2.6
    CONFIRM_TOL = 0.2      # confirm press vs fit (noise + ball walk)
    # fit acceptance window (effective radius must look like the ball)
    REFF_MIN_F = 0.5
    REFF_MAX_ADD = 3.5
    # budgets (hard guarantees against hangs)
    MAX_PRESSES = 700     # anti-hang bound, not a target: typical runs
                          # use ~110 presses, a fresh first-ever run with
                          # a far-off hint may legitimately use ~500
    MAX_SECONDS = 1800.
    MAX_CANDIDATES = 4     # anchors tried per head measurement
    MAX_ASCEND_HOPS = 4    # re-centers while climbing away from a side line

    def __init__(self, hw, log, clock, ball_r=5., bounds=(-10., 300., 0., 260.),
                 edge_margin=15., floor_z=38., travel_cold_z=58.):
        # hw interface: press, park, switch_b, switch_a, prior, position,
        # state_load, state_save
        self.hw = hw
        self.log = log        # callable(msg, level=1)
        self.clock = clock
        self.ball_r = ball_r
        self.bounds = bounds  # xmin, xmax, ymin, ymax (machine)
        self.edge = edge_margin
        self.floor_z = floor_z
        self._travel_cold = travel_cold_z
        self.press_count = 0
        self.t0 = clock()
        self._travel_z = travel_cold_z
        self.click_log = []

    # ---------------- infrastructure ----------------
    def _budget(self):
        if self.press_count >= self.MAX_PRESSES:
            raise CoreError("press budget exhausted (%d presses) - aborting "
                            "instead of looping forever. The ball was not "
                            "identified; nothing was applied." % self.MAX_PRESSES)
        if self.clock() - self.t0 > self.MAX_SECONDS:
            raise CoreError("time budget exhausted (%.0f min) - aborting. "
                            "Nothing was applied." % (self.MAX_SECONDS / 60.))

    def _clamp(self, x, y):
        xmin, xmax, ymin, ymax = self.bounds
        return (max(xmin + self.edge, min(xmax - self.edge, x)),
                max(ymin + self.edge, min(ymax - self.edge, y)))

    def _press(self, x, y, floor, tag):
        self._budget()
        x, y = self._clamp(x, y)
        floor = max(floor, self.floor_z)
        hit = self.hw.press(x, y, floor, self._travel_z)
        self.press_count += 1
        if hit:
            self.click_log.append(hit)
            self.log(". %s(%.1f,%.1f) click Z%.2f" % (tag, x, y, hit[2]), 2)
        return hit

    def _ring(self, cx, cy, rho, n):
        for k in range(n):
            a = k / float(n) * 2. * math.pi
            yield self._clamp(cx + rho * math.cos(a), cy + rho * math.sin(a))

    def _ring_presses(self, cx, cy, rho, floor, tag):
        n = max(8, int(math.ceil(2. * math.pi * rho / 8.)))
        out = []
        for (x, y) in self._ring(cx, cy, rho, n):
            h = self._press(x, y, floor, tag)
            if h:
                out.append(h)
        return out

    # ---------------- paraboloid fit ----------------
    def _fit_apex(self, pts):
        """Fit a dome; returns (x0, y0, z0, reff) or None.
        Robust against readout glitches: one false click tilts a plain
        least-squares paraboloid so much that honest points end up with
        the largest residuals (sequential dropping fails). Instead try
        every leave-one-out and leave-two-out subset, score by inliers
        against ALL points, keep the best. 9 points -> 46 tiny solves -
        negligible next to a physical press."""
        # z = c + bx*x + by*y + a*(x^2+y^2)  ->  apex (x0,y0,z0), reff=1/2a
        # NOTE: fit in coordinates centered on the stencil centroid - with
        # machine coordinates (~165) over a ~4mm stencil the raw normal
        # equations are catastrophically ill-conditioned.
        def solve(points):
            n = len(points)
            if n < 4:
                return None
            mx = sum(p[0] for p in points) / n
            my = sum(p[1] for p in points) / n
            # normal equations M*s = v, params [c, bx, by, a] (centered)
            M = [[0.] * 5 for _ in range(4)]
            for (x, y, z) in points:
                x -= mx
                y -= my
                row = [1., x, y, x * x + y * y]
                for i in range(4):
                    for j in range(4):
                        M[i][j] += row[i] * row[j]
                    M[i][4] += row[i] * z
            for col in range(4):
                piv = col
                for r2 in range(col + 1, 4):
                    if abs(M[r2][col]) > abs(M[piv][col]):
                        piv = r2
                M[col], M[piv] = M[piv], M[col]
                if abs(M[col][col]) < 1e-9:
                    return None
                for r2 in range(col + 1, 4):
                    f = M[r2][col] / M[col][col]
                    for c2 in range(col, 5):
                        M[r2][c2] -= f * M[col][c2]
            s = [0.] * 4
            for r2 in range(3, -1, -1):
                v = M[r2][4]
                for c2 in range(r2 + 1, 4):
                    v -= M[r2][c2] * s[c2]
                s[r2] = v / M[r2][r2]
            c, bx, by, a = s
            if a >= -1e-6:
                return None       # flat (a~0) or a bowl (a>0) - not a dome
            x0, y0 = -bx / (2. * a), -by / (2. * a)
            return (x0 + mx, y0 + my, c - (bx * bx + by * by) / (4. * a),
                    -1. / (2. * a))
        fit = solve(pts)
        if fit is None:
            return None
        # exhaustive glitch rejection (see docstring)
        def res(f, p):
            u2 = (p[0] - f[0]) ** 2 + (p[1] - f[1]) ** 2
            return abs(p[2] - (f[2] - u2 / (2. * f[3])))
        n = len(pts)
        subsets = [()]  # the empty drop = the full set
        subsets += [(i,) for i in range(n)]
        subsets += [(i, j) for i in range(n) for j in range(i + 1, n)]
        best = None       # (score, fit); score = -sum of capped residuals
        for drop in subsets:
            work = [p for k, p in enumerate(pts) if k not in drop]
            f = solve(work)
            if f is None:
                continue
            # capped residual sum over ALL points: a fit that smears one
            # outlier across the cloud scores worse than a tight fit
            # that ignores it - no inlier-count games
            score = -sum(min(res(f, p), 0.3) for p in pts)
            if best is None or score > best[0]:
                best = (score, f)
        return best[1] if best is not None else fit

    def _fit_ok(self, fit, strict=True):
        # strict: the final gate (polish). loose: round steering - a
        # glitch-starved 5-point fit still has to steer toward the apex,
        # only flat/bowl surfaces are rejected there
        if fit is None:
            return False
        if not strict:
            return 0.35 * self.ball_r <= fit[3] <= self.ball_r + 8.
        return (self.ball_r * self.REFF_MIN_F <= fit[3]
                <= self.ball_r + self.REFF_MAX_ADD)

    # ---------------- anchor search ----------------
    def _top_candidates(self, cx, cy, top):
        # top-cone presses around the hint; floor = top - DIP_TOP means
        # only the cone of radius sqrt(2*R*DIP) ~ 3.2mm can click - side
        # lines of foreign parts are below and never fire here. Lazy:
        # yield the FIRST clicks and let the stencil do the rest (every
        # extra shoulder press walks the ball a little).
        floor = top - self.DIP_TOP
        got = 0
        h = self._press(cx, cy, floor, 'top')
        if h:
            got += 1
            yield h
        for rho in (2.5, 5.):
            if got >= 2:
                return
            for (x, y) in self._ring(cx, cy, rho, 6 if rho < 3 else 8):
                h = self._press(x, y, floor, 'top')
                if h:
                    got += 1
                    yield h
                    if got >= 2:
                        return

    def _ascend(self, click):
        # starting from ANY click (usually a foreign part line): ring out
        # with a floor just below the current line and chase anything
        # HIGHER - the true dome top always beats every side line. No
        # head geometry used: only "higher means closer to the top".
        best = click
        hops = 0
        while hops < self.MAX_ASCEND_HOPS:
            hops += 1
            found = None
            for rho in (3., 6., 10., 14., 20., 26.):
                hits = self._ring_presses(best[0], best[1], rho,
                                          best[2] - self.DIP_ASCEND, 'asc')
                if hits:
                    top = max(hits, key=lambda p: p[2])
                    if top[2] > best[2] + 0.1:
                        found = top
                        break
                    # same line again (flat plateau): its extent bounds the
                    # ball - keep ringing outward from the original center
            if not found:
                return best
            best = found
            self.log("ascend: %.1f,%.1f Z%.2f -> %.1f,%.1f Z%.2f"
                     % (click[0], click[1], click[2], best[0], best[1], best[2]), 2)
        return best

    def _presence(self, cx, cy, top):
        # the hint lies badly (stale prior / moved probe): ring outward at
        # shoulder depth until anything clicks, then chase the height.
        # Lazy: stop the ring at the first click - the ascent plus the
        # stencil finish the job.
        for rho in (7., 12., 17., 22., 27., 32.):
            for (x, y) in self._ring(cx, cy, rho,
                                     max(8, int(math.ceil(
                                         2. * math.pi * rho / 8.)))):
                h = self._press(x, y, top - self.DIP_PRESENCE, 'pres')
                if h:
                    yield self._ascend(h)
                    yield h
                    return

    def _cold_find(self, cx, cy):
        # no ball_top at all: ring out at full depth (floor_z guards the
        # probe body); ANY click localizes the ball, then chase the height.
        for rho in (7., 12., 17., 22., 27., 32.):
            hits = self._ring_presses(cx, cy, rho, self.floor_z, 'cold')
            if hits:
                top = max(hits, key=lambda p: p[2])
                self.log("cold: first click %.1f,%.1f Z%.2f" % top, 1)
                return top
        return None

    # ---------------- convergence ----------------
    def _stencil(self, P, s, floor):
        pts = []
        h = self._press(P[0], P[1], floor, 'st')
        if h:
            pts.append(h)
        miss = []
        for (dx, dy) in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            h = self._press(P[0] + dx * s, P[1] + dy * s, floor, 'st')
            if h:
                pts.append(h)
            else:
                miss.append((P[0] + dx * s, P[1] + dy * s))
        if miss:
            # a stencil arm at the dome edge stops just short of the
            # surface - give the misses one deeper chance
            for (x, y) in miss:
                h = self._press(x, y, floor - 0.4, 'st-')
                if h:
                    pts.append(h)
        while len(pts) < 6:
            # top up with SHORT diagonals: presses near P always click
            # when the anchor is on the dome, keep the cloud two-sided,
            # and stay inside the 35-degree cone (no ball nudging). Six
            # points keep the robust fit meaningful after one glitch.
            for (dx, dy) in ((.5, .5), (-.5, .5), (.5, -.5), (-.5, -.5)):
                if len(pts) >= 6:
                    break
                h = self._press(P[0] + dx * s, P[1] + dy * s, floor, 'std')
                if h:
                    pts.append(h)
            else:
                break
        return pts

    def _converge(self, anchor, remeasure=False, top=0.):
        # Remeasure fast path: the revision re-measures a ball whose
        # apex height THIS SAME HEAD measured moments ago - a first
        # click within 0.15 of that height is the apex itself (frame
        # matches), and the Newton rounds would only press the shoulder
        # around it. Gated by the height so a stale hint's shoulder
        # click goes to the rounds instead; the verify ring gates what
        # reaches the polish either way.
        if remeasure and top and anchor[2] >= top - 0.15:
            # remeasure: polish FIRST (nothing in the path nudges the
            # ball, so the revision measures the exact surface state B
            # did), verify the polished apex afterwards - a post-failure
            # aborts the run without applying garbage
            apex = self._measure_apex(anchor[:3], verify_first=False)
            if apex is not None and self._verify_ring(apex):
                return apex
            if apex is not None:
                self.log("remeasure: post-verify failed - treating the "
                         "anchor as unknown", 1)
                return None
        # Stage 1 - Newton rounds ONCE per anchor: stencil -> paraboloid
        # fit -> jump to the apex. Flat/bowl/too-curved surfaces (a
        # foreign part line) never converge -> the caller tries the next
        # anchor. The round fit P is position-only truth: glitches move
        # it a little but the polish absorbs that.
        # Stage 2 - verify + polish + confirm, up to twice from the SAME
        # P: a readout glitch in the measurement data poisons one pass,
        # and glitches do not repeat at the same spots. Retrying only
        # stage 2 keeps the cost bounded.
        P = anchor
        for rnd in range(4):
            s = self.STENCIL_S if rnd == 0 else self.STENCIL_S * 0.8
            # floor covers the full arm drop even with P a couple mm off
            # the apex: a missing far arm both skews the fit AND makes the
            # press pattern one-sided (symmetric opposing presses cancel
            # their ball nudges - one-sided ones walk the ball)
            pts = self._stencil(P, s, P[2] - 1.3)
            if len(pts) < 4:
                return None
            fit = self._fit_apex(pts)
            if not self._fit_ok(fit, strict=False):
                self.log("converge: surface not ball-like (reff %s)" %
                         ("%.1f" % fit[3] if fit else "n/a"), 2)
                return None
            if math.dist(P[:2], fit[:2]) > 6.:
                return None
            P = fit
            if math.dist(P[:2], anchor[:2]) < 0.15:
                break
        for _ in range(2):
            apex = self._measure_apex(P)
            if apex is not None:
                return apex
        return None

    def _measure_apex(self, P, verify_first=True):
        # Decisive dome check: the ball must CONTINUE around the apex -
        # 4 presses on a r=3.5 ring, each exactly expect=r^2/2R below.
        # A foreign element's edge zone carries the ball's own curvature
        # (it passes the reff gate!) but its ring profile is flat or
        # falls off the ball - this rejects it. Head-independent.
        # Normally BEFORE the polish (reject cheap, and the ring's
        # nudges land before the final measurement); the remeasure path
        # verifies afterwards instead (see _converge).
        if verify_first and not self._verify_ring(P):
            self.log("converge: ring profile is not the ball top", 1)
            return None
        # polish: FRESH presses on one ring radius inside the contact
        # cone (never reuse a fitted value as data). Adaptive: 8 points
        # normally; extend to 16 when the first confirm smells like
        # noise - the apex xy comes from symmetric z-differences, more
        # ring points average the jitter down.
        floor = P[2] - 1.7
        base = self._press(P[0], P[1], floor, 'fin')

        def ring_point(k):
            # first 8: the full circle every 45 degrees; the extension
            # interleaves at 22.5 degrees (k = 8..15)
            a = k * math.pi / 4. if k < 8 else (2 * (k - 8) + 1) * math.pi / 8.
            return self._press(P[0] + self.POLISH_R * math.cos(a),
                               P[1] + self.POLISH_R * math.sin(a),
                               floor, 'fin')
        ring = [ring_point(k) for k in range(8)]
        for _ in range(8):               # extension, only if needed
            pts = ([base] if base else []) + [p for p in ring if p]
            if len(pts) < 8:
                return None              # most of the ring misses: not it
            fit = self._fit_apex(pts)
            if fit is None or not self._fit_ok(fit) \
                    or math.dist(P[:2], fit[:2]) > 1.5:
                return None
            # confirm: presses at the fitted apex, median z - robust to a
            # single readout glitch and a direct measurement of the apex
            # height (better than the fit extrapolation)
            zs = []
            for _ in range(3):
                h = self._press(fit[0], fit[1], fit[2] - 0.8, 'ok')
                if h:
                    zs.append(h[2])
            if len(zs) >= 2:
                zs.sort()
                z = zs[1] if len(zs) == 3 else (zs[0] + zs[1]) / 2.
                if abs(z - fit[2]) <= 0.25:
                    return (fit[0], fit[1], z)
            if len(ring) >= 16:
                return None
            self.log("polish: confirm off - extending the ring", 2)
            ring += [ring_point(k) for k in range(8, 16)]
        return None

    def _verify_ring(self, fit):
        # The ring radius sits INSIDE the 35-degree contact cone
        # (R*sin35 = 2.87 for R=5): these presses never nudge the ball,
        # so verifying cannot walk it mid-run. The drop r^2/2R still
        # separates the dome from flat foreign lines (0 vs 0.73mm) and
        # from edge-zone caps (profile clipped on one side).
        r = min(2.7, self.ball_r * 0.54)
        expect = r * r / (2. * self.ball_r)
        floor = fit[2] - expect - 0.9

        def drop(x, y):
            h = self._press(x, y, floor, 'ver')
            return None if h is None else fit[2] - h[2]

        for (dx, dy) in ((1, 0), (0, 1)):
            d1 = drop(fit[0] + dx * r, fit[1] + dy * r)
            d2 = drop(fit[0] - dx * r, fit[1] - dy * r)
            if d1 is not None and abs(d1 - expect) > 0.4:
                d1 = drop(fit[0] + dx * r, fit[1] + dy * r)  # glitch retry
            if d2 is not None and abs(d2 - expect) > 0.4:
                d2 = drop(fit[0] - dx * r, fit[1] - dy * r)
            if d1 is None or d2 is None:
                return False
            if abs(d1 - expect) > 0.4 or abs(d2 - expect) > 0.4:
                return False
        return True

    # ---------------- per-head measurement ----------------
    def _candidates(self, hint, top):
        """Lazily yield anchor clicks, cheapest/most-likely first:
        top-cone presses -> presence rings. A cold start first finds ANY
        click at full depth and uses its height as a provisional top, so
        the same chain works with zero knowledge."""
        cold = not top
        if cold:
            c = self._cold_find(hint[0], hint[1])
            if c is None:
                return
            self.log("cold: first click %.1f,%.1f Z%.2f - provisional top"
                     % c, 1)
            hint, top = c[:2], c[2] + 0.5
        for c in self._top_candidates(hint[0], hint[1], top):
            yield c
        for c in self._presence(hint[0], hint[1], top):
            yield c
        if not cold:
            return
        # last resort: the neighborhood is foreign lines and a walking
        # ball (every out-of-cone press nudges it); restart the search
        # from a different spot with the knowledge already gathered
        c = self._cold_find(hint[0] + 20., hint[1])
        if c is not None:
            yield self._ascend(c)
            yield c

    def measure(self, hint, top, tag, remeasure=False):
        """Find and measure the ball apex for the ACTIVE head.
        hint: (x, y) where the ball is expected (memory / prior / head)
        top:  known ball top height, 0 = cold (no knowledge)
        remeasure: this head measured this ball just moments ago - take
        the first anchor straight to verify+polish (frame matches `top`)
        Returns (x, y, z) or None."""
        for i, c in enumerate(self._candidates(hint, top)):
            if i >= self.MAX_CANDIDATES:
                break
            apex = self._converge(c, remeasure=remeasure and i == 0,
                                  top=top)
            if apex:
                return apex
            self.log("%s: anchor %d did not converge" % (tag, i + 1), 1)
        return self._settle_measure(tag)

    def _settle_measure(self, tag):
        # Last resort for a WALKING ball: out-of-cone presses attract it,
        # so it has been migrating toward the press cluster all run. The
        # highest logged clicks are the best settled guesses - and a
        # tight flower of presses INSIDE the contact cone cannot nudge
        # the ball any further, so the surface finally holds still under
        # the stencil. Try up to 2 spots, 2.5mm apart, highest first.
        if not self.click_log or self.press_count > self.MAX_PRESSES - 60:
            return None
        spots = []
        for c in sorted(self.click_log, key=lambda p: -p[2]):
            if all(math.dist(c[:2], s[:2]) > 2.5 for s in spots):
                spots.append(c)
            if len(spots) >= 2:
                break
        for best in spots:
            floor = max(best[2] - 1.5, self.floor_z)
            pts = []
            h = self._press(best[0], best[1], floor, 'set')
            if h:
                pts.append(h)
            for k in range(8):
                a = k * math.pi / 4.
                h = self._press(best[0] + 2.7 * math.cos(a),
                                best[1] + 2.7 * math.sin(a), floor, 'set')
                if h:
                    pts.append(h)
            if len(pts) < 6:
                continue
            fit = self._fit_apex(pts)
            if not self._fit_ok(fit) or math.dist(best[:2], fit[:2]) > 2.:
                continue
            self.log("%s: settle attempt at %.1f,%.1f" % (tag, *best[:2]), 1)
            apex = self._measure_apex(fit[:3])
            if apex is not None:
                return apex
        self.log("%s: settle attempts exhausted" % tag, 1)
        return None

    # ---------------- full run ----------------
    def run(self, dry=False):
        """Full calibration: A -> B -> revision A -> offset.
        Returns (dx, dy, dz) in the machine frame (IDEX_VARS convention
        verified on the real printer 2026-08-25: stored = measured as-is)."""
        st = self.hw.state_load() or {}
        # cold hint: where the head is NOW (the user parks it over the
        # ball before a first run) - no search-zone config needed
        hint = (st.get('x'), st.get('y'))
        if hint[0] is None or hint[1] is None:
            hint = self.hw.position()
        top = st.get('ball_top', 0.)
        self._travel_z = self._travel_cold
        # ---- head A ----
        self.log("head A: hint %.1f,%.1f  top %s" %
                 (hint[0], hint[1], "%.1f" % top if top else "unknown"), 1)
        apex_a = self.measure(hint, top, 'A')
        if apex_a is None:
            raise CoreError("Head A did not identify the ball (foreign "
                            "contact lines keep moving it?). Re-seat the "
                            "probe securely, park the head over it and "
                            "rerun; if it persists, clean the nozzle.")
        self.log("apex A: X%.3f Y%.3f Z%.3f" % apex_a, 1)
        # self-healing ball_top: what head A just measured IS the top
        top = apex_a[2]
        self._travel_z = top + 3.
        self.hw.state_save(apex_a[0], apex_a[1], top)
        # ---- B / revision loop ----
        # Hint signs: the stored offset IS the machine-frame measurement
        # (apexB_cmd - apexA), so "head B's nozzle over the ball" is the
        # command apexA + prior, and "head A there" is apexB - prior
        # (v9 subtracted the first and its seed was systematically off
        # by twice the offset; the second keeps the revision's search
        # short - searches walk the ball, and walks between the B and
        # revision polishes ARE the measurement error).
        # The pair repeats while the offset estimate still moves by more
        # than 0.5mm: a fresh printer (prior 0,0) walks the ball in the
        # first pass; one redo with the fresh estimate pins both hints.
        px, py = self.hw.prior()
        px0, py0 = px, py
        self.log("head B: prior %.2f,%.2f" % (px, py), 1)
        self.hw.park(True, self._travel_z)
        self.hw.switch_b()
        apex_b = apex_a2 = None
        for attempt in range(3):
            apex_b = self.measure((apex_a[0] + px, apex_a[1] + py),
                                  top, 'B')
            if apex_b is None:
                raise CoreError("Head B did not identify the ball (prior "
                                "%.1f,%.1f was off?)." % (px, py))
            self.log("apex B: X%.3f Y%.3f Z%.3f" % apex_b, 1)
            self.hw.park(False, self._travel_z)
            self.hw.switch_a()
            apex_a2 = None
            for _ in range(2):
                apex_a2 = self.measure((apex_b[0] - px, apex_b[1] - py),
                                       top, 'rev', remeasure=True)
                if apex_a2 is not None:
                    break
            if apex_a2 is None:
                raise CoreError("Revision pass A failed - cannot guarantee "
                                "the ball did not move. Nothing was applied.")
            self.log("apex A (revision): X%.3f Y%.3f Z%.3f" % apex_a2, 1)
            off = (apex_b[0] - apex_a2[0], apex_b[1] - apex_a2[1])
            if attempt == 2 or math.dist(off, (px, py)) <= 0.5:
                break
            if self.press_count > self.MAX_PRESSES - 150:
                # a fresh printer's first pass rides a far-off hint and
                # walks the ball; its raw offset is provisional. Never
                # apply it without the confirming redo.
                if math.hypot(px0, py0) <= 0.5:
                    raise CoreError(
                        "Budget ran out before the confirmation pass - the "
                        "offset was NOT applied. Rerun the calibration.")
                break
            self.log("offset estimate still moving (%.2f,%.2f -> %.2f,%.2f)"
                     " - one more pass" % (px, py, off[0], off[1]), 1)
            px, py = off
            self.hw.park(True, self._travel_z)
            self.hw.switch_b()
        # Drift guard. Only motion BETWEEN head B's polish and the
        # revision's polish corrupts the offset (before that, the fresh
        # hints re-measure the moved ball). The revision remeasured the
        # same surface B did: mapping B's apex back through the prior
        # must land on the revision's apex. Ball walks during searches
        # and a bump at the head switch are legal and cancel here.
        if math.hypot(px, py) > 0.5:
            gap = math.dist((apex_b[0] - px, apex_b[1] - py),
                            apex_a2[:2])
            if gap > 1.4:
                raise CoreError(
                    "Ball moved %.2fmm between the B and revision passes - "
                    "measurement invalid. Secure the probe and rerun. "
                    "Nothing was applied." % gap)
            if gap > 0.7:
                self.log("revision disagrees with B by %.2fmm (prior error? "
                         "walk?)" % gap, 0)
        if math.hypot(px, py) <= 0.5 \
                and math.dist(apex_a[:2], apex_a2[:2]) > 3.5:
            # no usable prior to map through (fresh printer): fall back
            # to a coarse total-walk check; the fresh hints already
            # tolerate most walks, so only flag the extreme ones
            raise CoreError("Ball moved %.2fmm during the run - measurement "
                            "invalid. Secure the probe and rerun. Nothing "
                            "was applied."
                            % math.dist(apex_a[:2], apex_a2[:2]))
        self.hw.state_save(apex_a2[0], apex_a2[1], apex_a2[2])
        off = (apex_b[0] - apex_a2[0], apex_b[1] - apex_a2[1],
               apex_b[2] - apex_a2[2])
        self.log("MEASURED (machine frame): dX%.3f dY%.3f dZ%.3f" % off, 1)
        if abs(off[0]) > 15. or abs(off[1]) > 15. or abs(off[2]) > 5.:
            raise CoreError("Measured dX%.2f dY%.2f dZ%.2f is outside the "
                            "plausible range (+/-15 XY, +/-5 Z) - the ball "
                            "was knocked or the switch glitched. Nothing was "
                            "applied." % off)
        if dry:
            self.log("DRY_RUN: offsets not applied", 1)
        return off


# =====================================================================
#  Klipper adapter
# =====================================================================

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
        # physical config - deliberately tiny (see ANALYSIS.md)
        self.ball_r = config.getfloat('ball_radius', 5., above=1.)
        self.floor_z = config.getfloat('floor_z', 38., above=0.)
        self.edge = config.getfloat('edge_margin', 15., minval=0.)
        self.probe_speed = config.getfloat('probe_speed', 4., above=0.)
        self.travel_speed = config.getfloat('travel_speed', 80., above=0.)
        self.lift_speed = config.getfloat('lift_speed', 15., above=0.)
        self.travel_cold_z = config.getfloat('travel_z_cold', 58., above=10.)
        self.log_level = config.getint('log_level', 1, minval=0, maxval=2)
        # legacy seed: used once, then the state file owns the value
        self._seed_top = config.getfloat('ball_top', 0., minval=0.)
        self.switch_b = config.get('head_switch_b_gcode', 'T1')
        self.switch_a = config.get('head_switch_a_gcode', 'T0')
        self.bounds = None
        self._trace_path = '/tmp/bp_trace.log'
        # state file next to the module copy in the config dir
        self._state_path = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), '.ball-state.json')
        self._oldpos_path = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), '.ball-pos.json')
        # homing_move computes the halt position from the steppers attached
        # to the endstop; attach the Z rails only (probing axis) - X/Y
        # steppers never move during a vertical press and would trip
        # check_no_movement() on every probe.
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

    # ---------------- misc helpers ----------------
    def _log(self, msg, level=1):
        if level > self.log_level:
            return
        prefix = {0: '!!', 1: '//', 2: '##'}[min(level, 2)]
        self.gcode.respond_raw("%s %s" % (prefix, msg))

    def _trace(self, msg):
        try:
            with open(self._trace_path, 'a') as f:
                f.write('%.3f %s\n' % (self.reactor.monotonic(), msg))
        except Exception:
            pass

    def _toolhead(self):
        return self.printer.lookup_object('toolhead')

    def _pos(self):
        return self._toolhead().get_position()

    def _get_bounds(self):
        if self.bounds is None:
            kin = self._toolhead().get_kinematics().get_status(
                self.reactor.monotonic())
            self.bounds = (kin['axis_minimum'][0], kin['axis_maximum'][0],
                           kin['axis_minimum'][1], kin['axis_maximum'][1])
        return self.bounds

    def _state_load(self):
        try:
            return json.load(open(self._state_path))
        except Exception:
            pass
        # one-time migration from v9 artifacts
        st = {}
        try:
            old = json.load(open(self._oldpos_path))
            st['x'], st['y'] = old['x'], old['y']
        except Exception:
            pass
        if self._seed_top:
            st['ball_top'] = self._seed_top
        return st or None

    def _state_save(self, x, y, top):
        try:
            json.dump({'x': round(x, 3), 'y': round(y, 3),
                       'ball_top': round(top, 3)},
                      open(self._state_path, 'w'))
        except Exception:
            pass

    # ---------------- hardware interface for the core ----------------
    def press(self, x, y, floor, travel_z):
        # 3-phase press: lift/travel (never descend while moving XY),
        # settle, wait for the lever to release, depth-limited descent.
        # Returns (x, y, click_z) or None.
        self._trace("press %.2f,%.2f floor=%.2f tz=%.2f" % (x, y, floor, travel_z))
        pos = self._pos()
        if pos[2] < travel_z - .01:
            self._toolhead().manual_move([pos[0], pos[1], travel_z],
                                         self.lift_speed)
        tz = max(pos[2], travel_z)
        self._toolhead().manual_move([x, y, tz], self.travel_speed)
        if tz > travel_z + .01:
            self._toolhead().manual_move([x, y, travel_z], self.lift_speed)
        # settle: the ball lever rings after any move; arming mid-ring
        # latches a false "triggered prior to movement"
        th = self._toolhead()
        th.dwell(0.4)
        for _ in range(20):
            th.dwell(0.1)
            if not self.mcu_endstop.query_endstop(th.get_last_move_time()):
                break
        else:
            raise self.printer.command_error(
                "ball probe reports TRIGGERED before the descent (pressed, "
                "jammed or wrong polarity) - check the switch")
        phoming = self.printer.lookup_object('homing')
        attempts = 3
        while True:
            try:
                epos = phoming.probing_move(self.mcu_endstop, [x, y, floor],
                                            self.probe_speed)
                break
            except self.printer.command_error as e:
                reason = str(e)
                if "No trigger" in reason:
                    return None
                if "prior to movement" in reason and attempts > 0:
                    attempts -= 1
                    ok = False
                    for _ in range(20):
                        th.dwell(0.1)
                        if not self.mcu_endstop.query_endstop(
                                th.get_last_move_time()):
                            ok = True
                            break
                    if ok:
                        continue
                raise self.printer.command_error(reason)
        z = epos[2]
        if z <= floor + 0.01:
            return None
        if z >= travel_z - 0.2:
            # triggered at the travel height: switch noise or the probe is
            # off the bed - never a ball contact
            self._log("!! probe triggered at travel height %.1f - noise or "
                      "missing probe" % z, 0)
            return None
        th.dwell(0.4)   # let the lever swing back before the next press
        self._trace("click %.2f,%.2f z=%.3f" % (x, y, z))
        return (x, y, z)

    def _hw(self):
        adapter = self

        class HW:
            def press(hw, x, y, floor, travel_z):
                return adapter.press(x, y, floor, travel_z)
            def park(hw, left, z):
                pos = adapter._pos()
                adapter._toolhead().manual_move([pos[0], pos[1], z],
                                                adapter.lift_speed)
                b = adapter._get_bounds()
                if left:
                    px = b[0] + 10.
                    # the kit toolchange re-approaches the park position
                    # with head B active (gcode offset applied) - stay in
                    # the window BOTH heads can reach: the dual carriage
                    # has its own min limit (1mm on vostok), so
                    # |offset_x| + margin from zero is the safe floor
                    px = max(px, abs(hw.prior()[0]) + 15.)
                else:
                    px = b[1] - 10.
                adapter._toolhead().manual_move([px, pos[1], z],
                                                adapter.travel_speed)
            def switch_b(hw):
                adapter.gcode.run_script_from_command(adapter.switch_b)
            def switch_a(hw):
                adapter.gcode.run_script_from_command(adapter.switch_a)
            def prior(hw):
                try:
                    iv = adapter.printer.lookup_object('gcode_macro IDEX_VARS')
                    st = iv.get_status(adapter.reactor.monotonic())
                    return (st.get('offset_x', 0.), st.get('offset_y', 0.))
                except Exception:
                    return (0., 0.)
            def position(hw):
                p = adapter._pos()
                return (p[0], p[1])
            def state_load(hw):
                return adapter._state_load()
            def state_save(hw, x, y, top):
                adapter._state_save(x, y, top)
        return HW()

    # ---------------- commands ----------------
    def cmd_run(self, gcmd):
        toolhead = self._toolhead()
        if toolhead.get_status(self.reactor.monotonic())['homed_axes'] != 'xyz':
            raise gcmd.error("Home all axes first")
        try:
            os.remove(self._trace_path)
        except OSError:
            pass
        self._log("=== ball-probe offset calibration (v10) ===")
        core = BallparkCore(self._hw(), self._log,
                            lambda: self.reactor.monotonic(),
                            ball_r=self.ball_r, bounds=self._get_bounds(),
                            edge_margin=self.edge, floor_z=self.floor_z,
                            travel_cold_z=self.travel_cold_z)
        dry = gcmd.get_int('DRY_RUN', 0)
        try:
            off = core.run(dry=dry)
        except CoreError as e:
            raise gcmd.error(str(e))
        self._log("%d presses, %.0fs" % (core.press_count,
                                         core.clock() - core.t0), 1)
        if dry:
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
    cmd_run.__doc__ = ("Calibrate the offsets between T0 and T1 via the "
                       "ball probe (v10 converging-stencil). "
                       "Params: DRY_RUN=1")

    def cmd_query(self, gcmd):
        toolhead = self._toolhead()
        pt = toolhead.get_last_move_time()
        self.gcode.respond_info("ball probe: %s" % (
            "TRIGGERED" if self.mcu_endstop.query_endstop(pt) else "open"))

    def cmd_noise(self, gcmd):
        toolhead = self._toolhead()
        if toolhead.get_status(self.reactor.monotonic())['homed_axes'] != 'xyz':
            raise gcmd.error("Home all axes first")
        self._get_bounds()
        pos = list(self._pos())
        z_safe = max(pos[2], self.travel_cold_z)

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
                p = list(self._pos())
                p[axis] = pos[axis] + (delta if i % 2 == 0 else -delta)
                toolhead.manual_move(p, 60.)
                hits += sample(4)
            return hits
        res = {'idle': sample(20)}
        b = self._get_bounds()
        self._toolhead().manual_move([b[0] + 10., pos[1], z_safe],
                                     self.travel_speed)
        pos = list(self._pos())
        res['X_wiggle'] = wiggle(0, 3., 5)
        res['Y_wiggle'] = wiggle(1, 3., 5)
        res['Z_wiggle'] = wiggle(2, 1., 5)
        for _ in range(3):
            self.press(b[0] + 20., pos[1], self.floor_z, z_safe)
        self.gcode.respond_info(
            "noise test (flickers): idle %d/20, X %d/20, Y %d/20, Z %d/20"
            % (res['idle'], res['X_wiggle'], res['Y_wiggle'], res['Z_wiggle']))

    def cmd_probetest(self, gcmd):
        toolhead = self._toolhead()
        if toolhead.get_status(self.reactor.monotonic())['homed_axes'] != 'xyz':
            raise gcmd.error("Home all axes first")
        self._get_bounds()
        phoming = self.printer.lookup_object('homing')
        pos = list(self._pos())
        if pos[2] < self.travel_cold_z:
            self._toolhead().manual_move([pos[0], pos[1], self.travel_cold_z],
                                         self.lift_speed)

        def q(tag):
            v = self.mcu_endstop.query_endstop(toolhead.get_last_move_time())
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
                continue
            q("down %d after" % i)
            self.gcode.respond_info(
                "// down %d: start Z%.3f -> reported Z%.3f (triggered at +%0.3fmm)"
                % (i, p0[2], epos[2], p0[2] - epos[2]))
            self._toolhead().manual_move(p0[:3], self.lift_speed)


def load_config(config):
    return ToolOffsetSphere(config)
