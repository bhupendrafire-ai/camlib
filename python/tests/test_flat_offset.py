"""Flat-area finishing in contour-parallel passes, outside in (#38).

Every check here is measured on the motion stream and the simulated stock
against analytic parts, independently of the planner.
"""

import math
import unittest

import numpy as np

import camlib
from camlib.cadquery import export_stock_mesh, export_surface_mesh
from camlib.simulation import OperationUpdate, Simulation, Update
from camlib.tools import HolderSection, ToolAssembly

R = 2.0  # cutter radius
STEPOVER = 1.5
CELL = 0.25
INITIAL_Z = 14


def tool():
    return ToolAssembly(
        "mill4",
        camlib.EndMill("em4", diameter=4, flute_length=20, overall_length=50),
        30,
        60,
        (HolderSection(30, 60, 4, 4),),
    )


# Parts as unions of boxes (x0, y0, z0, x1, y1, z1), with pockets cut out.
# `boxes` are added, `pockets` removed, `bosses` added after the pockets.
STEPPED = dict(
    boxes=[(0, 0, 0, 24, 16, 4)],
    pockets=[],
    bosses=[(8, 4, 4, 16, 12, 7)],
    extent=(0, 0, 24, 16),
)
# Doubling the footprint puts short, nearly parallel chords next to each
# other on a remaining-floor boundary that also contains arcs.
STEPPED_DOUBLE = dict(
    boxes=[(0, 0, 0, 48, 32, 4)],
    pockets=[],
    bosses=[(16, 8, 4, 32, 24, 7)],
    extent=(0, 0, 48, 32),
)
# A block with a pocket; in the pocket a short island and, 3 mm beside it
# (narrower than the 4 mm tool), a boss standing well above the block.
ISLAND = dict(
    boxes=[(0, 0, 0, 30, 20, 6)],
    pockets=[(3, 3, 3, 27, 17, 6)],
    bosses=[(7, 8, 3, 13, 12, 5), (16, 8, 3, 20, 12, 9)],
    extent=(0, 0, 30, 20),
)
# A wide block with a boss in the middle whose offsets split the block's
# level into separate areas either side of the boss.
WIDE = dict(
    boxes=[(0, 0, 0, 60, 40, 4)],
    pockets=[],
    bosses=[(20, 10, 4, 40, 30, 7)],
    extent=(0, 0, 60, 40),
)
# STEPPED at 1.89 times the size: a lead-in that starts over floor the first
# lap finished.
STEPPED_LARGE = dict(
    boxes=[(0, 0, 0, 45.4, 30.2, 4)],
    pockets=[],
    bosses=[(15.1, 7.6, 4, 30.2, 22.7, 7)],
    extent=(0, 0, 45.4, 30.2),
)


def solid(spec, skin=0.0):
    """The part, or with `skin` mm left on every flat (walls to size): the
    blank a flat finishing pass meets after roughing."""
    import cadquery as cq

    def box(b, top_skin):
        x0, y0, z0, x1, y1, z1 = b
        return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0 + top_skin, centered=False).translate((x0, y0, z0))

    shape = None
    for b in spec["boxes"]:
        s = box(b, skin)
        shape = s if shape is None else shape.union(s)
    for p in spec["pockets"]:
        x0, y0, z0, x1, y1, z1 = p
        shape = shape.cut(box((x0, y0, z0 + skin, x1, y1, z1 + skin + 1), 0))
    for b in spec["bosses"]:
        shape = shape.union(box(b, skin))
    return shape


def height(spec, x, y):
    """Analytic top of the part at (x, y) (numpy arrays)."""
    h = np.zeros(np.broadcast(x, y).shape)
    inside = lambda r: (x >= r[0]) & (x <= r[3]) & (y >= r[1]) & (y <= r[4])  # noqa: E731
    for b in spec["boxes"]:
        h = np.where(inside(b), np.maximum(h, b[5]), h)
    for p in spec["pockets"]:
        strict = (x > p[0]) & (x < p[3]) & (y > p[1]) & (y < p[4])
        h = np.where(strict, p[2], h)
    for b in spec["bosses"]:
        h = np.where(inside(b), np.maximum(h, b[5]), h)
    return h


class Field:
    """The part sampled on a fine grid: distance from a point to the part
    standing above a height."""

    def __init__(self, spec, step=0.05):
        x0, y0, x1, y1 = spec["extent"]
        xs = np.arange(x0, x1 + step / 2, step)
        ys = np.arange(y0, y1 + step / 2, step)
        self.X, self.Y = np.meshgrid(xs, ys)
        self.H = height(spec, self.X, self.Y)
        self.step = step
        self._above = {}

    def above(self, z):
        key = round(z, 6)
        if key not in self._above:
            m = self.H > z + 1e-6
            self._above[key] = np.stack([self.X[m], self.Y[m]], axis=1)
        return self._above[key]

    def clearance(self, p, z):
        """XY distance from p to the part above z (inf if none); exact to
        the sampling step."""
        pts = self.above(z)
        if len(pts) == 0:
            return math.inf
        return float(np.min(np.hypot(pts[:, 0] - p[0], pts[:, 1] - p[1])))


def flat_op(spec, entry=camlib.RampEntry(angle_deg=3, min_length_mm=1), **changes):
    surface = export_surface_mesh(solid(spec), source=dict(model="flat-offset", revision="1"))
    t = tool()
    op = camlib.Flat(
        "flats",
        t,
        surface,
        0,
        STEPOVER,
        14,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
        entry=entry,
        **changes,
    )
    return t, op


def run(spec, skin=0.1, **changes):
    """Run Flat on the roughed blank; return (report, motions, stock
    snapshots before each motion as (nx, ny, dx, dy, heights), final state)."""
    t, op = flat_op(spec, **changes)
    stock = camlib.StockSpec.from_mesh(export_stock_mesh(solid(spec, skin)), CELL)
    motions, before = [], []
    report = None
    with Simulation(
        stock=stock,
        tools=(t,),
        operations=(op,),
        revision="1",
        initial_position_mm=(0, 0, INITIAL_Z),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    ) as sim:
        s = sim.state
        snapshot = list(s.heights)
        for update in sim.run_operations():
            if isinstance(update, OperationUpdate) and update.stage == "end":
                report = update.report
            if isinstance(update, Update):
                motions.append(update.motion)
                before.append(snapshot)
                snapshot = list(s.heights)
        assert sim.state.finished
        grid = (s.nx, s.ny, s.dx, s.dy, tuple(s.min_mm))
        final = list(s.heights)
    return report, motions, before, grid, final


def stock_top_near(grid, heights, p, radius):
    """Highest stock cell upper bound whose cell rectangle comes within
    `radius` of p (what the tool's disk at p can touch)."""
    nx, ny, dx, dy, lo = grid
    top = -math.inf
    for j in range(ny):
        for i in range(nx):
            x0, y0 = lo[0] + i * dx, lo[1] + j * dy
            ex = max(x0 - p[0], 0, p[0] - (x0 + dx))
            ey = max(y0 - p[1], 0, p[1] - (y0 + dy))
            if math.hypot(ex, ey) < radius:
                top = max(top, heights[2 * (j * nx + i) + 1])
    return top


def points(m, n=None):
    c = m["curve"]
    a, b = c["start"], c["end"]
    if c.get("kind", "line") == "line":
        n = n or max(2, math.ceil(math.dist(a, b) / 0.05) + 1)
        return [[a[k] + (b[k] - a[k]) * i / (n - 1) for k in range(3)] for i in range(n)]
    u, v, w = {"xy": (0, 1, 2), "xz": (2, 0, 1), "yz": (1, 2, 0)}[c["plane"]]
    center, sweep = c["center"], c["sweep_rad"]
    radius = math.hypot(a[u] - center[u], a[v] - center[v])
    start = math.atan2(a[v] - center[v], a[u] - center[u])
    n = n or max(2, math.ceil(math.hypot(radius * sweep, b[w] - a[w]) / 0.05) + 1)
    samples = []
    for i in range(n):
        t = i / (n - 1)
        q = list(a)
        q[u] = center[u] + radius * math.cos(start + t * sweep)
        q[v] = center[v] + radius * math.sin(start + t * sweep)
        q[w] = a[w] + t * (b[w] - a[w])
        samples.append(q)
    return samples


def vertical_plunges(motions):
    out = []
    for m in motions:
        a, b = m["curve"]["start"], m["curve"]["end"]
        if m["kind"] == "plunge" and math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-9:
            out.append(m)
    return out


def level_cuts(motions, z):
    return [
        m
        for m in motions
        if m["kind"] in ("cut", "link", "lead")
        and all(abs(p[2] - z) < 1e-6 for p in (m["curve"]["start"], m["curve"]["end"]))
    ]


def seg_distance(p, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    vx, vy = bx - ax, by - ay
    l2 = vx * vx + vy * vy
    t = 0 if l2 == 0 else max(0, min(1, ((p[0] - ax) * vx + (p[1] - ay) * vy) / l2))
    return math.hypot(p[0] - ax - t * vx, p[1] - ay - t * vy)


def level_distance(p, c):
    a, b = c["start"], c["end"]
    if c.get("kind", "line") == "line":
        return seg_distance(p, a, b)
    assert c["plane"] == "xy" and a[2] == b[2]
    x, y = c["center"][:2]
    start = math.atan2(a[1] - y, a[0] - x)
    angle = math.atan2(p[1] - y, p[0] - x)
    sweep = c["sweep_rad"]
    if ((angle - start) * math.copysign(1, sweep)) % math.tau <= abs(sweep):
        return abs(math.hypot(p[0] - x, p[1] - y) - math.hypot(a[0] - x, a[1] - y))
    return min(math.dist(p, a[:2]), math.dist(p, b[:2]))


def rect_outline_distance(p, r):
    """Distance from p to the outline of rectangle r = (x0, y0, x1, y1)."""
    x0, y0, x1, y1 = r
    inside = x0 <= p[0] <= x1 and y0 <= p[1] <= y1
    if inside:
        return min(p[0] - x0, x1 - p[0], p[1] - y0, y1 - p[1])
    return math.hypot(max(x0 - p[0], 0, p[0] - x1), max(y0 - p[1], 0, p[1] - y1))


_cache = {}


def cached(name):
    if name not in _cache:
        _cache[name] = run({"stepped": STEPPED, "island": ISLAND, "double": STEPPED_DOUBLE}[name])
    return _cache[name]


class FlatOffsetTest(unittest.TestCase):
    def test_measurement_follows_arc_and_helix_instead_of_chord(self):
        c = dict(kind="arc", plane="xy", start=[2, 0, 4], end=[-2, 0, 4], center=[0, 0, 4], sweep_rad=-math.pi)
        self.assertEqual(level_distance((0, 0), c), 2)
        self.assertEqual(level_distance((0, -2), c), 0)
        self.assertAlmostEqual(level_distance((0, 2), c), math.sqrt(8))
        h = dict(c, kind="helix", end=[-2, 0, 2])
        self.assertTrue(np.allclose(points({"curve": h}, 3)[1], [0, -2, 3]))

    def test_few_entries_and_links_at_depth(self):
        # Entries are the vertical plunges. The stepped part needs one per
        # ring along an edge or wall that cannot be linked to (boss top,
        # base outer edge, ring round the boss) plus one per piece the
        # offsets split off out of link reach (the base's two side strips
        # at d = 3): at most 6. Today: one per raster run (39).
        report, motions, *_ = cached("stepped")
        plunges = len(vertical_plunges(motions))
        self.assertLessEqual(plunges, 6, f"{plunges} entries; runs={report['runs']}")
        self.assertEqual(plunges, report["runs"])
        self.assertGreater(report.get("links_at_depth", 0), 0)

    def test_no_vertical_plunge_onto_a_finished_face(self):
        # No plunge marks: a vertical plunge that ends at a flat's cutting
        # height never has the tool's disk over that flat's face; the tool
        # comes down beside the face (side entry) or stops above the stock
        # and ramps.
        for name, spec in (("stepped", STEPPED), ("island", ISLAND)):
            _, motions, *_ = cached(name)
            field = Field(spec)
            worst = (math.inf, None)
            for m in vertical_plunges(motions):
                b = m["curve"]["end"]
                # Within the level tolerance: the mesh puts a level a few
                # 1e-5 off its analytic height (3.00001 for Z3).
                face = np.abs(field.H - b[2]) <= 1e-3
                if not face.any():
                    continue
                d = float(np.min(np.hypot(field.X[face] - b[0], field.Y[face] - b[1])))
                if d < worst[0]:
                    worst = (d, [round(v, 4) for v in b])
            self.assertGreaterEqual(
                worst[0],
                R - 1e-6 - field.step,
                f"{name}: plunge onto a finished face at {worst[1]} ({worst[0]:.4f} mm)",
            )

    def test_open_edges_are_entered_from_the_side_in_air(self):
        # The boss top and the base's outer edge are open: their first ring
        # is entered by a vertical plunge where the tool's disk touches no
        # stock above the plunge's end (measured on the stock before the
        # move), then a horizontal lead into the ring. Only the ring round
        # the boss (walled, 0.1 mm skin left on the flat) may need a ramp.
        report, motions, before, grid, _ = cached("stepped")
        in_air = 0
        for m in vertical_plunges(motions):
            b = m["curve"]["end"]
            top = stock_top_near(grid, before[motions.index(m)], b, R)
            if top <= b[2] + 1e-6:
                in_air += 1
        self.assertGreaterEqual(in_air, 2, f"in-air entries {in_air}")
        self.assertLessEqual(report.get("ramped_entries", 0), 1)

    def test_first_ring_of_each_level_follows_its_edge(self):
        # Outside in: the first loop cut at each level runs round the
        # level's outer edge (boss top: the 8 x 8 boss; base: the 24 x 16
        # block), all the way round, its centre the radius less one stepover
        # outside the edge: its disk takes one stepover of the face, like
        # every later pass, rather than rolling a radius outside it in air.
        _, motions, *_ = cached("stepped")
        for z, rect in ((7.0, (8, 4, 16, 12)), (4.0, (0, 0, 24, 16))):
            cuts = [m for m in level_cuts(motions, z) if m["kind"] == "cut"]
            self.assertTrue(cuts, z)
            start = cuts[0]["curve"]["start"]
            ring, travelled = [], 0.0
            for m in cuts:
                ring.append(m)
                a, b = m["curve"]["start"], m["curve"]["end"]
                travelled += math.hypot(b[0] - a[0], b[1] - a[1])
                if travelled > 1 and math.hypot(b[0] - start[0], b[1] - start[1]) < 1e-6:
                    break
            worst = max(abs(rect_outline_distance(p, rect) - (R - STEPOVER)) for m in ring for p in points(m))
            self.assertLessEqual(worst, 0.01, f"Z{z}: first loop strays {worst:.4f} mm from the edge ring")
            perimeter = 2 * (rect[2] - rect[0] + rect[3] - rect[1]) + 2 * math.pi * (R - STEPOVER)
            self.assertGreater(travelled, 0.98 * perimeter, f"Z{z}: first loop {travelled:.3f} of {perimeter:.3f} mm")

    def test_coverage_of_every_reachable_flat_point(self):
        # Every flat point at least a radius (+0.01) from the part standing
        # above it lies under the tool's disk on some cut at its level.
        for name, spec in (("stepped", STEPPED), ("island", ISLAND), ("double", STEPPED_DOUBLE)):
            _, motions, *_ = cached(name)
            field = Field(spec)
            x0, y0, x1, y1 = spec["extent"]
            levels = {z: level_cuts(motions, z) for z in np.unique(field.H) if z > 0}
            worst = (0.0, None)
            for x in np.arange(x0 + 0.125, x1, 0.25):
                for y in np.arange(y0 + 0.125, y1, 0.25):
                    z = float(height(spec, np.array(x), np.array(y)))
                    if z <= 0 or field.clearance((x, y), z) < R + 0.01:
                        continue
                    d = min((level_distance((x, y), m["curve"]) for m in levels[z]), default=math.inf)
                    if d > worst[0]:
                        worst = (d, (x, y, z))
            self.assertLessEqual(
                worst[0], R - 1e-6, f"{name}: uncovered flat point {worst[1]} {worst[0]:.4f} mm from any cut"
            )

    def test_tool_never_reaches_into_the_part_above_its_tip(self):
        # On every feed move (cut, link, lead, plunge, ramp) the tool's
        # axis stays at least a radius (less 1e-4) from every part point
        # above its tip: measured against the analytic part, including the
        # tall boss 3 mm beside the island.
        for name, spec in (("stepped", STEPPED), ("island", ISLAND)):
            _, motions, *_ = cached(name)
            field = Field(spec)
            worst = (math.inf, None)
            for m in motions:
                if m["kind"] == "rapid":
                    continue
                for p in points(m, 9):
                    c = field.clearance(p, p[2])
                    if c < worst[0]:
                        worst = (c, (m["id"], m["kind"], [round(v, 4) for v in p]))
            self.assertGreaterEqual(
                worst[0],
                R - 1e-4 - field.step,
                f"{name}: tool reaches {R - worst[0]:.4f} mm into the part at {worst[1]}",
            )

    def test_never_cuts_below_the_part(self):
        for name, spec in (("stepped", STEPPED), ("island", ISLAND)):
            _, _, _, grid, final = cached(name)
            nx, ny, dx, dy, lo = grid
            worst = 0.0
            for j in range(ny):
                for i in range(nx):
                    xs = np.array([lo[0] + i * dx, lo[0] + (i + 1) * dx])
                    ys = np.array([lo[1] + j * dy, lo[1] + (j + 1) * dy])
                    X, Y = np.meshgrid(xs, ys)
                    lowest = float(np.min(height(spec, X, Y)))
                    worst = max(worst, lowest - final[2 * (j * nx + i) + 1])
            self.assertLessEqual(worst, 1e-6, f"{name}: stock cut {worst:.4f} mm below the part")

    def test_no_moves_of_kernel_artefacts(self):
        # A kernel's flattened circle (chords mixed with micro-arcs) and its
        # snapped corner stubs are not cut as moves of their own: no cut,
        # link or lead shorter than 5 microns (the arcs' own move sagitta),
        # measured in 3D (a rounded plunge's chords descend mostly in Z).
        for name in ("stepped", "island"):
            _, motions, *_ = cached(name)
            tiny = [
                m["id"]
                for m in motions
                if m["kind"] in ("cut", "link", "lead") and math.dist(m["curve"]["end"], m["curve"]["start"]) < 0.005
            ]
            self.assertEqual(tiny, [], f"{name}: moves under 5 microns: {tiny[:10]}")

    def test_passes_climb_mill(self):
        # With a clockwise spindle a pass climbs when the stock it cuts lies on
        # the right of travel. Measured on the live stock: just outside the
        # disk, beside the middle of each straight cut, which side still has
        # stock above the cut height.
        for name in ("stepped", "island"):
            _, motions, before, grid, _ = cached(name)
            nx, ny, dx, dy, lo = grid
            side = {"climb": 0.0, "conventional": 0.0}
            for m, snap in zip(motions, before):
                c = m["curve"]
                a, b = c["start"], c["end"]
                length = math.hypot(b[0] - a[0], b[1] - a[1])
                if m["kind"] != "cut" or c.get("kind", "line") != "line" or length < 1e-6:
                    continue
                tx, ty = (b[0] - a[0]) / length, (b[1] - a[1]) / length
                mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)

                def stock(sign):
                    for d in (R * 0.8, R * 0.95):
                        x, y = mid[0] + sign * ty * d, mid[1] - sign * tx * d
                        i, j = int((x - lo[0]) / dx), int((y - lo[1]) / dy)
                        if 0 <= i < nx and 0 <= j < ny and snap[2 * (j * nx + i) + 1] > a[2] + 0.02:
                            return True
                    return False

                right, left = stock(1), stock(-1)
                if right != left:
                    side["climb" if right else "conventional"] += length
            self.assertGreater(side["climb"], 3 * side["conventional"], f"{name}: {side}")

    def test_passes_do_not_run_over_finished_floor(self):
        # Measured on the live stock before each move: the length of the
        # level passes (cuts, links and leads at a flat's height) whose disk
        # (less 0.1 mm) has no stock standing 0.02 mm above the cut — air
        # beside an open edge, or floor an earlier pass already finished.
        # A first ring rolling a radius outside every open edge, and inner
        # rings run all the way round a rim or top that earlier rings had
        # already covered, made this 47 % (stepped) and 45 % (island).
        for name in ("stepped", "island"):
            _, motions, before, grid, _ = cached(name)
            nx, ny, dx, dy, lo = grid
            xs = lo[0] + (np.arange(nx) + 0.5) * dx
            ys = lo[1] + (np.arange(ny) + 0.5) * dy
            X, Y = np.meshgrid(xs, ys)
            total = idle = 0.0
            for m, snap in zip(motions, before):
                c = m["curve"]
                z = c["start"][2]
                if m["kind"] not in ("cut", "link", "lead") or abs(c["end"][2] - z) > 1e-6:
                    continue
                H = np.array(snap[1::2]).reshape(ny, nx)
                ps = points(m)
                length = sum(math.dist(p[:2], q[:2]) for p, q in zip(ps, ps[1:]))
                live = [bool((H[(X - p[0]) ** 2 + (Y - p[1]) ** 2 < (R - 0.1) ** 2] > z + 0.02).any()) for p in ps]
                total += length
                idle += length * (1 - sum(live) / len(live))
            self.assertLess(idle, 0.1 * total, f"{name}: {idle:.1f} of {total:.1f} mm over no stock")

    def test_entries_plunge_in_air_and_lead_in_and_out_on_tangent_arcs(self):
        # A run is entered by a vertical plunge in air and a quarter-circle
        # lead-in of the tool's radius, tangent to the pass where it joins it
        # (flat beside a face; over a finished face a helix stops above the
        # floor and descends at the declared 3 degrees over its chord), and
        # left by a tangent quarter-circle lead-out of the same radius before
        # the retract. Measured on the motion stream: radius, sweep, slope,
        # and the direction on both sides of each joint.
        def tangent(c, end):
            if c.get("kind", "line") == "line":
                d = [c["end"][0] - c["start"][0], c["end"][1] - c["start"][1]]
            else:
                p = c["end"] if end else c["start"]
                ce, sign = c["center"], math.copysign(1, c["sweep_rad"])
                d = [-(p[1] - ce[1]) * sign, (p[0] - ce[0]) * sign]
            n = math.hypot(*d)
            return [d[0] / n, d[1] / n]

        for name in ("stepped", "island"):
            report, motions, *_ = cached(name)
            self.assertGreater(report.get("arc_lead_ins", 0), 0, name)
            self.assertGreater(report.get("arc_lead_outs", 0), 0, name)
            ins = outs = 0
            for i, m in enumerate(motions[1:-1], 1):
                c = m["curve"]
                if (
                    c.get("kind") not in ("arc", "helix")
                    or c.get("plane") != "xy"
                    or m["kind"] not in ("lead", "plunge")
                ):
                    continue
                prev, nxt = motions[i - 1], motions[i + 1]
                # A ramp's helix legs also end on a pass: a lead-in is told
                # apart by its shape (quarter circle of the tool's radius);
                # the counts below must then match the report's.
                a, ce = c["start"], c["center"]
                quarter = (
                    abs(math.hypot(a[0] - ce[0], a[1] - ce[1]) - R) < 1e-6
                    and abs(abs(c["sweep_rad"]) - math.pi / 2) < 1e-9
                )
                lead_in = nxt["kind"] == "cut" and (m["kind"] == "lead" or quarter)
                rise = (
                    nxt["curve"]["end"][2] > nxt["curve"]["start"][2]
                    and math.dist(nxt["curve"]["end"][:2], nxt["curve"]["start"][:2]) < 1e-9
                )
                lead_out = m["kind"] == "lead" and prev["kind"] in ("cut", "link") and rise
                if not (lead_in or lead_out):
                    continue
                self.assertAlmostEqual(math.hypot(a[0] - ce[0], a[1] - ce[1]), R, delta=1e-6)
                self.assertAlmostEqual(abs(c["sweep_rad"]), math.pi / 2, delta=1e-9)
                if lead_in:
                    ins += 1
                    t0, t1 = tangent(c, True), tangent(nxt["curve"], False)
                    drop = c["start"][2] - c["end"][2]
                    chord = math.dist(c["start"][:2], c["end"][:2])
                    self.assertLessEqual(drop / chord, math.tan(math.radians(3)) + 1e-9, (name, c))
                else:
                    outs += 1
                    t0, t1 = tangent(prev["curve"], True), tangent(c, False)
                self.assertGreater(t0[0] * t1[0] + t0[1] * t1[1], 1 - 1e-9, (name, i, c))
            self.assertEqual(ins, report["arc_lead_ins"], name)
            self.assertEqual(outs, report["arc_lead_outs"], name)

    def test_plunges_turn_to_horizontal_on_a_quarter_circle(self):
        # A vertical plunge stops a tool radius above where it used to land
        # and turns to horizontal on a quarter circle of the tool's radius in
        # the vertical plane of the move that follows: the turn ends a radius
        # lower and a radius on, every point of it on that circle (its chords
        # within 5 microns), leaving horizontally in the direction the
        # next move starts in.
        for name in ("stepped", "island"):
            report, motions, *_ = cached(name)
            self.assertGreater(report.get("rounded_plunges", 0), 0, name)
            turns = 0
            for i, m in enumerate(motions):
                if m not in vertical_plunges([m]):
                    continue
                j, path = i + 1, []
                while (
                    motions[j]["kind"] == "lead"
                    and motions[j]["curve"]["end"][2] < motions[j]["curve"]["start"][2] - 1e-9
                ):
                    path.append(motions[j])
                    j += 1
                if not path or motions[j]["kind"] == "plunge":
                    # No turn, or an arc ramp's upper arc (its own test).
                    continue
                turns += 1
                top, end = path[0]["curve"]["start"], path[-1]["curve"]["end"]
                self.assertEqual(top, m["curve"]["end"], name)
                self.assertAlmostEqual(top[2] - end[2], R, delta=1e-9)
                self.assertAlmostEqual(math.dist(top[:2], end[:2]), R, delta=1e-9)
                centre = [end[0], end[1], top[2]]
                for t in path:
                    c = t["curve"]
                    for q in (c["start"], c["end"]):
                        self.assertAlmostEqual(math.dist(q, centre), R, delta=1e-9)
                    if c["kind"] == "line":
                        mid = [(a + b) / 2 for a, b in zip(c["start"], c["end"])]
                        self.assertLessEqual(R - math.dist(mid, centre), 0.005 + 1e-9)
                d = [end[0] - top[0], end[1] - top[1]]
                nxt = motions[j]["curve"]
                if nxt.get("kind", "line") == "line":
                    e = [nxt["end"][0] - nxt["start"][0], nxt["end"][1] - nxt["start"][1]]
                else:
                    a, ce, sign = nxt["start"], nxt["center"], math.copysign(1, nxt["sweep_rad"])
                    e = [-(a[1] - ce[1]) * sign, (a[0] - ce[0]) * sign]
                cos = (d[0] * e[0] + d[1] * e[1]) / (math.hypot(*d) * math.hypot(*e))
                self.assertGreater(cos, 1 - 1e-9, (name, motions[j]))
            self.assertEqual(turns, report["rounded_plunges"], name)

    def test_radial_stock_is_left_on_every_wall(self):
        # With radial_stock_to_leave_mm the passes cut only the flats: on every
        # feed move the tool's axis stays a radius plus that stock (less the
        # sampling step and 1e-4) from every part point above its tip,
        # measured against the analytic part (the short island's walls, the
        # tall boss beside it, the pocket and boss walls).
        radial = 0.1
        for name, spec in (("stepped", STEPPED), ("island", ISLAND)):
            _, motions, *_ = run(spec, radial_stock_to_leave_mm=radial)
            field = Field(spec)
            worst = (math.inf, None)
            for m in motions:
                if m["kind"] == "rapid":
                    continue
                for p in points(m, None if m["curve"].get("kind") == "arc" else 9):
                    c = field.clearance(p, p[2])
                    if c < worst[0]:
                        worst = (c, (m["id"], m["kind"], [round(v, 4) for v in p]))
            self.assertGreaterEqual(
                worst[0],
                R + radial - 1e-4 - field.step,
                f"{name}: tool within {worst[0] - R:.4f} mm of a wall at {worst[1]}",
            )

    def test_mesh_only_wall_pass_finishes_after_the_floor(self):
        report, motions, *_ = run(STEPPED, finish_walls=True)
        self.assertEqual(report["wall_passes"], 1)
        self.assertEqual(report["runs_completed"], report["runs"])
        last = [move for move in level_cuts(motions, 4) if move["kind"] == "cut"][-1]
        endpoint = last["curve"]["end"]
        self.assertAlmostEqual(Field(STEPPED).clearance(endpoint, 4), R, delta=0.01)

    def test_each_area_is_finished_before_crossing_to_the_next(self):
        # The base level of WIDE splits into areas left and right of the
        # boss. Each is finished before the tool moves on: between cutting
        # moves on the base level, the tool crosses from one side of the boss
        # to the other through the air (a retract and rapid) at most once.
        _, motions, *_ = run(WIDE)
        x0, x1 = WIDE["bosses"][0][0], WIDE["bosses"][0][3]
        z = WIDE["boxes"][0][5]
        crossings, side, lifted = 0, None, False
        for m in motions:
            c = m["curve"]
            if m["kind"] in ("rapid", "retract"):
                lifted = True
                continue
            if m["kind"] != "cut" or abs(c["start"][2] - z) > 1e-6:
                continue
            x = c["start"][0]
            here = "left" if x < x0 else "right" if x > x1 else None
            if here is None:
                continue
            if lifted and side is not None and here != side:
                crossings += 1
            side, lifted = here, False
        self.assertLessEqual(crossings, 1)

    def test_entries_over_finished_floor_plunge_rather_than_spiral(self):
        # A pass whose arc lead-in starts over floor already finished (no
        # stock above the pass under the tool) is entered by a plunge that
        # turns to horizontal, not by a helix down onto the floor.
        parts = (("stepped", STEPPED), ("large", STEPPED_LARGE), ("wide", WIDE), ("island", ISLAND))
        for name, spec in parts:
            _, motions, before, grid, _ = run(spec)
            for i in range(1, len(motions)):
                c, prev = motions[i]["curve"], motions[i - 1]["curve"]
                vertical = (
                    motions[i - 1]["kind"] == "plunge"
                    and prev.get("kind", "line") == "line"
                    and math.hypot(prev["end"][0] - prev["start"][0], prev["end"][1] - prev["start"][1]) < 1e-9
                )
                lead_in = (
                    vertical
                    and c.get("kind") == "helix"
                    and i + 1 < len(motions)
                    and motions[i + 1]["kind"] != "plunge"
                )
                if not lead_in:
                    continue
                # A helix between a vertical plunge and the pass is a helical
                # lead-in (a ramp continues with more plunging moves); it is
                # only allowed where stock under the plunge point stands above
                # the pass (no plunge in air there).
                p, z = c["start"], c["end"][2]
                top = stock_top_near(grid, before[i], p, R)
                self.assertGreater(top, z + 0.05, (name, motions[i]["id"], p))

    def test_arc_ramps_meet_the_stock_at_the_declared_angle(self):
        # A pass start under stock is entered by one smooth descent: a
        # vertical plunge, an arc of the tool's radius through air above the
        # stock under the tool (measured on the live stock), then moves that
        # descend no steeper than the declared 3 degree ramp onto the pass,
        # arriving level. Each is counted in arc_ramps.
        slope = math.tan(math.radians(3)) + 1e-9
        for name in ("island", "wide"):
            if name == "island":
                report, motions, before, grid, _ = cached(name)
            else:
                report, motions, before, grid, _ = run(WIDE, radial_stock_to_leave_mm=0.1)
            ramps = 0
            for i, m in enumerate(motions):
                if m not in vertical_plunges([m]):
                    continue
                j = i + 1
                air = []
                while (
                    motions[j]["kind"] == "lead"
                    and motions[j]["curve"]["end"][2] < motions[j]["curve"]["start"][2] - 1e-9
                ):
                    air.append(j)
                    j += 1
                if not air or motions[j]["kind"] != "plunge":
                    continue
                ramps += 1
                for k in air:
                    q = motions[k]["curve"]["end"]
                    top = stock_top_near(grid, before[k], q, R)
                    self.assertGreaterEqual(q[2], top - 1e-9, (name, motions[k]["id"]))
                last = None
                while motions[j]["kind"] == "plunge":
                    c = motions[j]["curve"]
                    a, b = c["start"], c["end"]
                    span = math.hypot(b[0] - a[0], b[1] - a[1])
                    self.assertGreater(span, 0, (name, motions[j]["id"]))
                    self.assertLessEqual((a[2] - b[2]) / span, slope, (name, motions[j]["id"]))
                    last = c
                    j += 1
                # A ramp started beside the pass leads into it on a flat arc.
                if motions[j]["kind"] == "lead":
                    c = motions[j]["curve"]
                    self.assertAlmostEqual(c["start"][2], last["end"][2], delta=1e-9)
                    self.assertAlmostEqual(c["end"][2], last["end"][2], delta=1e-9)
                    j += 1
                self.assertEqual(motions[j]["kind"], "cut", name)
                self.assertAlmostEqual(last["end"][2], motions[j]["curve"]["start"][2], delta=1e-9)
            self.assertEqual(ramps, report.get("arc_ramps", 0), name)
            self.assertGreater(ramps, 0, name)


if __name__ == "__main__":
    unittest.main()
