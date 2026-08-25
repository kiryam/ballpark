#!/usr/bin/env python3
"""Ballpark v10 test harness: physics simulation + stress grids.

Physics ported from the validated browser sim (sim/sim3d.html):
  - head elements {dx, dy, r, zo}: a vertical press clicks at the nozzle
    height where the highest-reaching element touches the ball
    (zc = ball_surface(horiz) - zo), if zc is inside [floor, travel];
  - reported click z = contact - lever pre-travel; x/y = truth + slop;
  - noise% readout glitches: a false click above the surface, no physical
    contact;
  - shoulder presses (outside the 35-degree cone) nudge the ball away;
  - the ball can be bumped when the head switches (the algorithm does not
    know);
  - the probe body: a flat ring below the ball top that clicks on deep
    presses near the ball (guarded by floor_z).

Runs the REAL algorithm (tool_offset_sphere.BallparkCore) against all of
that, on randomized heads - proving head-independence - plus regression
scenarios for every failure mode the v9 algorithm had on the printer.

Usage: python3 tests/sim_harness.py [grid|regressions|all] [--seed N]
"""

import math
import random
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tool_offset_sphere import BallparkCore, CoreError  # noqa: E402


# =====================================================================
#  physics
# =====================================================================

class World:
    def __init__(self, rng, ball, heads, true_off, pre=0.25, jit=0.15,
                 noise_pct=3., nudge=True, bump=0., floor_z=38.,
                 body=True, harness_cap=1200):
        self.rng = rng
        self.ball = list(ball)        # [x, y, cz, R]
        self.heads = heads            # {'A': [elem...], 'B': [elem...]}
        self.noff = true_off          # (dx, dy, dz) nozzle B in machine frame
        self.pre = pre
        self.jit = jit
        self.noise_pct = noise_pct
        self.nudge = nudge
        self.bump = bump
        self.floor_z = floor_z
        self.body = body
        self.active = 'A'
        self.presses = 0
        self.harness_cap = harness_cap
        self.ball_moves = 0.0
        self.glitches = 0

    def _noff(self):
        return (0., 0., 0.) if self.active == 'A' else self.noff

    def press(self, x, y, floor, travel):
        """Vertical press with the ACTIVE head commanded to (x, y).
        Machine-frame semantics as on the printer: the active nozzle sits
        at (x + nx, y + ny), the gantry z = z (nozzle A reference)."""
        assert floor >= self.floor_z - 1e-9, "floor below the crash guard!"
        self.presses += 1
        if self.presses > self.harness_cap:
            raise AssertionError("harness hard cap exceeded - the algorithm "
                                 "would loop forever")
        bx, by, cz, R = self.ball
        nx, ny, nz = self._noff()
        best = None    # (z_gantry, el, ecx, ecy)
        for e in self.heads[self.active]:
            re = 0. if e['name'] == 'nozzle' else e['r']
            ecx, ecy = x + nx + e['dx'], y + ny + e['dy']
            dxy = math.hypot(ecx - bx, ecy - by)
            horiz = max(dxy - re, 0.)
            if horiz > R:
                continue
            zc = cz + math.sqrt(R * R - horiz * horiz) - e['zo'] + nz
            if floor + 0.02 <= zc <= travel + 1e-9:
                if best is None or zc > best[0]:
                    best = (zc, e, ecx, ecy)
        # probe body: flat ring around the ball, a bit below the top
        if self.body:
            u = math.hypot(x + nx - bx, y + ny - by)
            zb = cz + R - 5.5
            if R + 1. <= u <= R + 4.5 and floor + 0.02 <= zb <= travel:
                if best is None or zb > best[0]:
                    best = (zb, {'name': 'body'}, x, y)
        # readout glitch: false click, no contact, any height in the window
        if best is not None or self.rng.random() < 0.30 * self.noise_pct / 100.:
            if self.rng.random() * 100. < self.noise_pct:
                self.glitches += 1
                z = self.rng.uniform(floor + 0.05,
                                     best[0] if best else travel)
                return (x + self.rng.uniform(-self.jit, self.jit),
                        y + self.rng.uniform(-self.jit, self.jit), z)
        if best is None:
            return None
        zc, e, ecx, ecy = best
        if self.nudge and e['name'] != 'body':
            # shoulder press (outside the 35-degree cone) nudges the ball
            d = math.hypot(ecx - bx, ecy - by)
            cone = R * math.sin(35. * math.pi / 180.)
            if d > cone:
                push = min(0.5, (d - cone) * 0.08)
                a = math.atan2(ecy - by, ecx - bx)
                bx += math.cos(a) * push
                by += math.sin(a) * push
                self.ball[0], self.ball[1] = bx, by
                self.ball_moves += push
        return (x + self.rng.uniform(-self.jit, self.jit),
                y + self.rng.uniform(-self.jit, self.jit), zc - self.pre)

    def switch(self, head):
        if head == 'B' and self.bump:
            a = self.rng.random() * 2. * math.pi
            self.ball[0] += math.cos(a) * self.bump
            self.ball[1] += math.sin(a) * self.bump
            self.ball_moves += self.bump
        self.active = head


class FakeHW:
    """Adapter between BallparkCore and the World physics."""

    def __init__(self, world, prior=(0., 0.), state=None, head_at=None):
        self.w = world
        self.prior_off = prior
        self.state = dict(state) if state else None
        self.switched = []
        # cold start: the user parks the head over the ball (+/- ~10mm)
        if head_at is None:
            rng = world.rng
            head_at = (world.ball[0] + rng.uniform(-10, 10),
                       world.ball[1] + rng.uniform(-10, 10))
        self.head_pos = tuple(head_at)

    def position(self):
        return self.head_pos

    def press(self, x, y, floor, travel_z):
        return self.w.press(x, y, floor, travel_z)

    def park(self, left, z):
        pass

    def switch_b(self):
        self.switched.append('T1')
        self.w.switch('B')

    def switch_a(self):
        self.switched.append('T0')
        self.w.switch('A')

    def prior(self):
        return self.prior_off

    def state_load(self):
        return dict(self.state) if self.state else None

    def state_save(self, x, y, top):
        self.state = {'x': x, 'y': y, 'ball_top': top}


# =====================================================================
#  heads and scenarios
# =====================================================================

def vostok_heads(rng=None, plateau_duct=False):
    """The real vostok heads (measured from STLs in the browser sim) -
    the exact geometry that broke v9 on 2026-08-25."""
    a = [
        {'name': 'nozzle', 'dx': 0., 'dy': 0., 'r': .7, 'zo': 0.},
        {'name': 'sock', 'dx': 0., 'dy': 0., 'r': 4.2, 'zo': 2.5},
        {'name': 'block', 'dx': 0., 'dy': 0., 'r': 6.6, 'zo': 5.},
        {'name': 'bltouch', 'dx': -24., 'dy': -6., 'r': 3., 'zo': 9.},
        {'name': 'duct', 'dx': -26., 'dy': 0., 'r': 4., 'zo': 1.6},
    ]
    b = [
        {'name': 'nozzle', 'dx': 0., 'dy': 0., 'r': .7, 'zo': 0.},
        {'name': 'sock', 'dx': 0., 'dy': 0., 'r': 4.2, 'zo': 2.5},
        {'name': 'block', 'dx': 0., 'dy': 0., 'r': 6.6, 'zo': 5.},
        {'name': 'duct', 'dx': 10., 'dy': 14., 'r': 4., 'zo': 1.6},
    ]
    if plateau_duct:
        # the 2026-08-25 killer: a wide low part whose side-press line
        # forms a flat plateau just below the ball top
        b.append({'name': 'plateau', 'dx': 12., 'dy': 6., 'r': 6.,
                  'zo': 1.4})
    return {'A': a, 'B': b}


def random_heads(rng):
    """Head-independence proof: random but realistic part layouts."""
    def one_head():
        els = [{'name': 'nozzle', 'dx': 0., 'dy': 0.,
                'r': rng.uniform(.3, .8), 'zo': 0.}]
        if rng.random() < .8:
            els.append({'name': 'sock', 'dx': 0., 'dy': 0., 'r': rng.uniform(3, 5),
                        'zo': rng.uniform(1.5, 3.5)})
        if rng.random() < .8:
            els.append({'name': 'block', 'dx': 0., 'dy': 0., 'r': rng.uniform(5.5, 8),
                        'zo': rng.uniform(4, 6)})
        for _ in range(rng.randint(0, 2)):
            dx = rng.uniform(-30, 30)
            if abs(dx) < 8:
                dx += 16 if dx >= 0 else -16
            els.append({'name': 'duct', 'dx': dx, 'dy': rng.uniform(-10, 15),
                        'r': rng.uniform(3, 5), 'zo': rng.uniform(1.2, 3.)})
        if rng.random() < .5:
            els.append({'name': 'probe', 'dx': rng.uniform(-28, -20),
                        'dy': rng.uniform(-8, 0), 'r': 3.,
                        'zo': rng.uniform(8, 10)})
        return els
    return {'A': one_head(), 'B': one_head()}


def run_case(seed, heads=None, ball=None, true_off=None, prior_err=None,
             state=None, pre=None, jit=None, noise=None, bump=0., dry=False,
             verbose=False, staleness=.6):
    """One full calibration in simulation. Returns a result dict."""
    rng = random.Random(seed)
    log = (lambda m, l=1: print("L%d %s" % (l, m))) if verbose \
        else (lambda m, l=1: None)
    heads = heads or random_heads(rng)
    ball = ball or (rng.uniform(120, 210), rng.uniform(30, 80),
                    rng.uniform(40, 50), 5.)
    noff = true_off or (rng.uniform(-12, 12), rng.uniform(-3, 3),
                        rng.uniform(-.8, .8))
    # what a perfect calibration must measure (see ANALYSIS.md): with
    # head B active the machine X tracks the B rail reference; its nozzle
    # sits at rail+noff, so apexB_cmd = ball - noff -> dX = -noff_x. Z is
    # the direct gantry difference (+noff_z).
    true_off = (-noff[0], -noff[1], noff[2])
    # prior staleness: the previous calibration of the same rig - regular
    # use drifts well under a mm (the +/-1.5 scenario is its own
    # regression case)
    prior = (true_off[0] + rng.uniform(-staleness, staleness),
             true_off[1] + rng.uniform(-staleness, staleness)) \
        if prior_err is None else prior_err
    pre = rng.uniform(.05, .35) if pre is None else pre
    jit = rng.uniform(.05, .12) if jit is None else jit
    noise = rng.uniform(0, 5) if noise is None else noise
    w = World(rng, ball, heads, noff, pre=pre, jit=jit,
              noise_pct=noise, bump=bump)
    hw = FakeHW(w, prior=prior, state=state)
    core = BallparkCore(hw, log, lambda: w.presses * 0. + 0.,
                        ball_r=ball[3], bounds=(-5., 355., -5., 265.),
                        edge_margin=15., floor_z=38., travel_cold_z=58.)
    # harness clock advances with presses so the time budget is exercised
    core.clock = lambda: w.presses * 2.5
    core.t0 = 0.
    res = {'seed': seed, 'presses': 0, 'error': None, 'off': None,
           'true': true_off, 'budget': BallparkCore.MAX_PRESSES}
    try:
        off = core.run(dry=dry)
        res['off'] = off
    except CoreError as e:
        res['error'] = str(e)
    res['presses'] = w.presses
    res['ball_moves'] = round(w.ball_moves, 2)
    res['glitches'] = w.glitches
    if res['off'] is not None:
        res['err'] = tuple(res['off'][i] - true_off[i] for i in range(3))
    return res


def check(res, xy_tol=.12, z_tol=.08):
    if res['error']:
        return "error: %s" % res['error'][:70]
    ex, ey, ez = res['err']
    if abs(ex) > xy_tol or abs(ey) > xy_tol or abs(ez) > z_tol:
        return ("accuracy: dX%+.3f dY%+.3f dZ%+.3f (tol %.2f/%.2f)"
                % (ex, ey, ez, xy_tol, z_tol))
    return None


def report(name, res, fail):
    tag = "FAIL" if fail else "ok"
    extra = ("err dX%+.3f dY%+.3f dZ%+.3f" % res['err']) if 'err' in res \
        else (res['error'] or '')[:60]
    print("  %-4s %-22s presses %3d  moves %.2f  %s"
          % (tag, name, res['presses'], res['ball_moves'], extra))
    return fail is not None


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'all'
    seed0 = 1000
    failures = 0

    if mode in ('grid', 'all'):
        n = 40
        print("GRID: %d randomized runs (random heads, ball, prior, noise;"
              " bar 0.15 XY / 0.08 Z - print-invisible)" % n)
        for i in range(n):
            res = run_case(seed0 + i)
            failures += bool(report("seed %d" % (seed0 + i), res,
                                    check(res, xy_tol=.15)))
        print()

    if mode in ('regressions', 'all'):
        print("REGRESSIONS (v9 failure modes):")
        # 1. THE 2026-08-25 hang: stale ball_top +1.1mm low, plateau duct
        # on B, wire noise - v9 looped forever here
        res = run_case(7, heads=vostok_heads(plateau_duct=True),
                       ball=(164.7, 21., 44.09, 5.),
                       true_off=(8.34, .95, -.3),
                       state={'x': 164.7, 'y': 21., 'ball_top': 47.95},
                       pre=.2, jit=.12, noise=3.)
        failures += bool(report("stale_bt+plateau", res, check(res)))
        # 2. cold start, no state, vostok heads
        res = run_case(11, heads=vostok_heads(), state=None, noise=3.)
        failures += bool(report("cold_start", res, check(res)))
        # 3. bare nozzles (no foreign parts at all), cold
        bare = {h: [{'name': 'nozzle', 'dx': 0., 'dy': 0., 'r': .4, 'zo': 0.}]
                for h in 'AB'}
        res = run_case(13, heads=bare, state=None)
        failures += bool(report("bare_nozzle_cold", res, check(res)))
        # 4. far wrong prior (fresh printer, prior 0,0 vs true 11mm) -
        # the first pass walks the ball; the built-in fresh redo must
        # still deliver a full-accuracy offset
        res = run_case(17, heads=vostok_heads(), true_off=(11., 2., .4),
                       prior_err=(0., 0.), noise=3.)
        failures += bool(report("far_prior(fresh redo)", res, check(res)))
        # 5. ball bumped hard at the head switch: abort OR (thanks to
        # the fresh hints) a correct offset - garbage is the only failure
        res = run_case(19, heads=vostok_heads(), bump=2.4, noise=0.)
        ok = (res['error'] and 'moved' in res['error']
              and res['off'] is None) or not check(res)
        print("  %-4s %-22s presses %3d  moves %.2f  %s"
              % ("ok" if ok else "FAIL", "big_bump_safe", res['presses'],
                 res['ball_moves'], (res['error'] or '')[:60]))
        failures += not ok
        # 6. jitter stress: the harness max slop, tolerance widened
        res = run_case(33, heads=vostok_heads(), jit=.15, noise=3., bump=.3)
        failures += bool(report("jit_stress_0.15", res,
                                check(res, xy_tol=.2, z_tol=.1)))
        # 7. stale prior +/-1.5 (long-uncalibrated rig): the hint misses,
        # searches walk the ball; accuracy degrades but must stay sane
        res = run_case(1041, heads=vostok_heads(), noise=3., staleness=1.5)
        failures += bool(report("stale_prior_1.5", res,
                                check(res, xy_tol=.25, z_tol=.1)))
        # 6. small bump: must still measure correctly via the revision
        res = run_case(23, heads=vostok_heads(), bump=.5, noise=3.)
        failures += bool(report("small_bump_rev", res, check(res)))
        # 7. glitch storm: 8% false clicks
        res = run_case(29, heads=vostok_heads(), noise=8., bump=.3)
        failures += bool(report("glitch_storm_8pct", res, check(res)))
        # 8. low pre-travel and no slop (ideal probe)
        res = run_case(31, heads=vostok_heads(), pre=0., jit=0., noise=0.)
        failures += bool(report("ideal_probe", res, check(res)))
        print()

    if mode in ('presses', 'all'):
        print("PRESS BUDGET (worst of 20 cold starts):")
        worst = 0
        for i in range(20):
            res = run_case(2000 + i, state=None, noise=3.)
            worst = max(worst, res['presses'])
            if res['error']:
                failures += 1
                report("cold seed %d" % (2000 + i), res, res['error'])
        print("  worst cold-start press count: %d (budget 450)" % worst)

    print("RESULT: %s (%d failure(s))" % ("PASS" if not failures else "FAIL",
                                          failures))
    return 1 if failures else 0


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'debug':
        # python3 tests/sim_harness.py debug <case-name> : verbose rerun
        cases = {
            'grid1033': lambda: run_case(1033, verbose=True),
            'grid1031': lambda: run_case(1031, verbose=True),
            'stale_bt': lambda: run_case(
                7, heads=vostok_heads(plateau_duct=True),
                ball=(164.7, 21., 44.09, 5.), true_off=(8.34, .95, -.3),
                state={'x': 164.7, 'y': 21., 'ball_top': 47.95},
                pre=.2, jit=.12, noise=3., verbose=True),
            'far_prior': lambda: run_case(
                17, heads=vostok_heads(), true_off=(11., 2., .4),
                prior_err=(0., 0.), noise=3., verbose=True),
            'ideal': lambda: run_case(31, heads=vostok_heads(), pre=0.,
                                      jit=0., noise=0., verbose=True),
            'big_bump': lambda: run_case(19, heads=vostok_heads(), bump=2.4,
                                         noise=0., verbose=True),
            'cold2000': lambda: run_case(2000, state=None, noise=3.,
                                         verbose=True),
            'cold2009': lambda: run_case(2009, state=None, noise=3.,
                                         verbose=True),
            'cold2012': lambda: run_case(2012, state=None, noise=3.,
                                         verbose=True),
        }
        res = cases[sys.argv[2]]()
        print("RESULT:", {k: v for k, v in res.items() if k != 'true'})
        print("TRUE:", ["%+.3f" % v for v in res['true']])
        sys.exit(0)
    sys.exit(main())
