"""Run a posted program in LinuxCNC's own interpreter and compare what it
executes with the plan.

camlib decodes every program it posts and compares the result with the plan,
but that decoder is camlib's reading of the LinuxCNC manual. This asks
LinuxCNC itself: `rs274`, the standalone interpreter shipped with LinuxCNC
(package `linuxcnc-uspace`; "Stand Alone Interpreter" in the LinuxCNC
documentation), reads a program and prints the canonical machining commands
it would send to the machine (STRAIGHT_TRAVERSE, STRAIGHT_FEED, ARC_FEED,
SET_FEED_RATE, CHANGE_TOOL, ...). No machine, no GUI.

It is a simulator of the controller's interpreter, not of a machine: it says
nothing about a real machine's limits, tool table, offsets or kinematics.

`rs274` is found on the PATH. Tests that need it are skipped where it is not
installed, unless CAMLIB_REQUIRE_LINUXCNC is set (the CI job that installs
LinuxCNC sets it, so a missing interpreter fails there instead of skipping).
"""

import math
import os
from pathlib import Path
import re
import shutil
import subprocess
from tempfile import TemporaryDirectory
import unittest

# The interpreter prints lengths to four decimals: a printed point is within
# this of the point it stands for. A straight move is known by its printed
# end points; an arc by three printed points (start, end and centre), so a
# point of it can be that much further off twice more.
PRINTED_MM = 0.00005 * math.sqrt(3)

_COMMAND = re.compile(r"^\s*\d+\s+N\S*\s+([A-Z_0-9]+)\((.*)\)\s*$")


def require():
    """Skip the calling test where LinuxCNC's interpreter is not installed."""
    if shutil.which("rs274"):
        return
    if os.environ.get("CAMLIB_REQUIRE_LINUXCNC"):
        raise AssertionError("CAMLIB_REQUIRE_LINUXCNC is set but rs274 (linuxcnc-uspace) is not on the PATH")
    raise unittest.SkipTest("LinuxCNC's standalone interpreter (rs274) is not installed")


def tool_table(numbers):
    """A LinuxCNC tool table with an entry for each number, all zero.

    Offsets are zero so the canonical positions are the programmed ones: the
    tool's length belongs to the machine, and this checks the program.
    """
    return "".join(f"T{n} P{n} Z0 D0\n" for n in sorted(set(numbers)))


# Marks where the program under test begins in the interpreter's output.
_START = "camlib program starts here"


def interpret(program, tools=(), start=None):
    """Canonical commands LinuxCNC's interpreter executes for `program`:
    a list of (name, [arguments as text]). Raises AssertionError with the
    interpreter's message if it rejects a block.

    The standalone interpreter starts at X0 Y0 Z0, and a program relies on
    the machine being where its `Machine` declares the tool is when it
    starts (a canned cycle, for one, takes its initial plane from there).
    With `start`, the interpreter is first moved there by one rapid that is
    not part of the program; what that rapid emits is left out.
    """
    if start is not None:
        x, y, z = start
        program = f"G21 G90 G0 X{x:.6f} Y{y:.6f} Z{z:.6f}\n({_START})\n" + program
    with TemporaryDirectory(prefix="camlib-rs274-") as folder:
        folder = Path(folder)
        (folder / "program.ngc").write_text(program)
        (folder / "tool.tbl").write_text(tool_table(tools))
        result = subprocess.run(
            ["rs274", "-g", "-t", "tool.tbl", "program.ngc", "canon.txt"],
            cwd=folder,
            capture_output=True,
            text=True,
            timeout=300,
        )
        output = folder / "canon.txt"
        if result.returncode != 0 or not output.exists():
            raise AssertionError(
                f"LinuxCNC's interpreter rejected the program (exit {result.returncode}):\n"
                f"{result.stdout}{result.stderr}"
            )
        commands = []
        for line in output.read_text().splitlines():
            match = _COMMAND.match(line)
            if not match:
                raise AssertionError(f"unreadable interpreter output: {line!r}")
            commands.append((match[1], [a.strip() for a in match[2].split(",")] if match[2].strip() else []))
    if not commands or commands[-1][0] != "ON_RESET" or not any(c[0] == "PROGRAM_END" for c in commands):
        raise AssertionError("the interpreter did not reach the end of the program")
    if start is not None:
        marker = next(i for i, c in enumerate(commands) if c[0] == "COMMENT" and _START in c[1][0])
        commands = [c for c in commands[:marker] if "TRAVERSE" not in c[0] and "FEED" != c[0][-4:]] + commands[
            marker + 1 :
        ]
    return commands


class Machine:
    """The interpreter's state while its canonical commands are replayed."""

    def __init__(self, start):
        self.position = tuple(start)
        self.plane = "XY"
        self.feed = 0.0
        self.spindle = 0.0  # signed speed: + clockwise (M3), - counterclockwise (M4), 0 stopped
        self.speed = 0.0  # rpm, or m/min while constant surface speed is on
        self.css_max = 0.0  # G96's rpm limit; 0 when the speed is an rpm (G97)
        self.per_rev = False  # G95: the feed is per revolution
        self.lead = None  # the lead of a synchronised (G33) move in progress
        self.outputs = frozenset()  # digital outputs switched with motion (M62/M63) that are on
        self.adaptive_feed = False  # M52 P1
        self.flood = False
        self.mist = False
        self.selected = None
        self.tool = None

    def apply(self, name, args):
        """Update the state; return the moves [(kind, sample(t), end)] of a
        motion command (none for any other)."""
        if name == "SELECT_PLANE":
            self.plane = args[0].removeprefix("CANON_PLANE_")
        elif name == "SET_FEED_RATE":
            self.feed = float(args[0])
        elif name == "SET_FEED_MODE":
            self.per_rev = int(args[-1]) == 1
        elif name == "SET_SPINDLE_MODE":
            # One argument, "spindle limit": 0 is a fixed speed (G97).
            self.css_max = float(" ".join(args).split()[-1])
        elif name == "SET_MOTION_OUTPUT_BIT":
            self.outputs = self.outputs | {int(args[0])}
        elif name == "CLEAR_MOTION_OUTPUT_BIT":
            self.outputs = self.outputs - {int(args[0])}
        elif name in ("ENABLE_ADAPTIVE_FEED", "DISABLE_ADAPTIVE_FEED"):
            self.adaptive_feed = name == "ENABLE_ADAPTIVE_FEED"
        elif name == "START_SPEED_FEED_SYNC":
            self.lead = float(args[0])
        elif name == "STOP_SPEED_FEED_SYNCH":
            self.lead = None
        elif name == "SET_SPINDLE_SPEED":
            self.speed = float(args[-1])
            if self.spindle:
                self.spindle = math.copysign(self.speed, self.spindle)
        elif name == "START_SPINDLE_CLOCKWISE":
            self.spindle = self.speed
        elif name == "START_SPINDLE_COUNTERCLOCKWISE":
            self.spindle = -self.speed
        elif name == "STOP_SPINDLE_TURNING":
            self.spindle = 0.0
        elif name in ("FLOOD_ON", "FLOOD_OFF"):
            self.flood = name == "FLOOD_ON"
        elif name in ("MIST_ON", "MIST_OFF"):
            self.mist = name == "MIST_ON"
        elif name == "SELECT_TOOL":
            self.selected = int(args[0])
        elif name == "CHANGE_TOOL":
            # CHANGE_TOOL's argument is the pocket; the tool is the one the
            # T word selected (SELECT_TOOL).
            self.tool = self.selected
        elif name in ("STRAIGHT_TRAVERSE", "STRAIGHT_FEED"):
            start, end = self.position, tuple(float(a) for a in args[:3])
            self.position = end
            kind = "traverse" if name == "STRAIGHT_TRAVERSE" else "feed" if self.lead is None else "thread"
            return [(kind, _line(start, end), end)]
        elif name == "ARC_FEED":
            sample = self._arc(args)
            return [("arc", sample, self.position)]
        elif name == "RIGID_TAP":
            # G33.1: down to the point with the spindle synchronised, then
            # back out to where it started (LinuxCNC G-code reference,
            # G33.1). The lead (K) is not in the canonical command.
            start, bottom = self.position, tuple(float(a) for a in args[:3])
            return [("tap", _line(start, bottom), bottom), ("tap", _line(bottom, start), start)]
        return []

    def _arc(self, args):
        """ARC_FEED(first_end, second_end, first_centre, second_centre,
        rotation, axis_end): the plane's two axes in its own order (XY: X, Y;
        YZ: Y, Z; XZ: Z, X), `rotation` turns counterclockwise seen from the
        positive third axis (negative: clockwise), the third axis linear."""
        first, second, linear = {"XY": (0, 1, 2), "YZ": (1, 2, 0), "XZ": (2, 0, 1)}[self.plane]
        e1, e2, c1, c2 = (float(a) for a in args[:4])
        turns = int(args[4])
        start = self.position
        end = [0.0, 0.0, 0.0]
        end[first], end[second], end[linear] = e1, e2, float(args[5])
        a0 = math.atan2(start[second] - c2, start[first] - c1)
        a1 = math.atan2(e2 - c2, e1 - c1)
        sweep = a1 - a0
        if turns > 0:
            sweep = sweep % math.tau or math.tau
            sweep += math.tau * (turns - 1)
        else:
            sweep = -((-sweep) % math.tau or math.tau)
            sweep -= math.tau * (-turns - 1)
        r0 = math.hypot(start[first] - c1, start[second] - c2)
        r1 = math.hypot(e1 - c1, e2 - c2)
        self.position = tuple(end)

        def sample(t):
            radius = r0 + (r1 - r0) * t
            angle = a0 + sweep * t
            p = [0.0, 0.0, 0.0]
            p[first] = c1 + radius * math.cos(angle)
            p[second] = c2 + radius * math.sin(angle)
            p[linear] = start[linear] + (end[linear] - start[linear]) * t
            return tuple(p)

        return sample


def _line(start, end):
    return lambda t: tuple(a + (b - a) * t for a, b in zip(start, end))


def distance_to_curve(point, curve):
    """Distance from a point to a planned curve (a motion's `curve` record)."""
    a, b = curve["start"], curve["end"]
    if curve["kind"] == "line":
        d = [q - p for p, q in zip(a, b)]
        length2 = sum(v * v for v in d)
        t = 0.0 if length2 == 0 else max(0.0, min(1.0, sum((x - p) * v for x, p, v in zip(point, a, d)) / length2))
        return math.dist(point, [p + v * t for p, v in zip(a, d)])
    first, second, linear = {"xy": (0, 1, 2), "yz": (1, 2, 0), "zx": (2, 0, 1), "xz": (2, 0, 1)}[
        str(curve.get("plane", "xy")).lower()
    ]
    centre, sweep = curve["center"], curve["sweep_rad"]
    radius = math.hypot(a[first] - centre[first], a[second] - centre[second])
    start = math.atan2(a[second] - centre[second], a[first] - centre[first])
    angle = math.atan2(point[second] - centre[second], point[first] - centre[first])
    # Every turn of the curve the point's angle can lie on, and both ends.
    turn = ((angle - start) * math.copysign(1, sweep)) % math.tau
    fractions = {0.0, 1.0}
    k = 0
    while turn + k * math.tau <= abs(sweep) + 1e-12:
        fractions.add(min(1.0, (turn + k * math.tau) / abs(sweep)))
        k += 1
    best = math.inf
    for t in fractions:
        q = [0.0, 0.0, 0.0]
        q[first] = centre[first] + radius * math.cos(start + sweep * t)
        q[second] = centre[second] + radius * math.sin(start + sweep * t)
        q[linear] = a[linear] + (b[linear] - a[linear]) * t
        best = min(best, math.dist(point, q))
    return best


def _interpreted(program, tools, start):
    """The interpreter's moves for `program`, each with the state it is made in."""
    machine = Machine(start)
    moves = []
    for name, args in interpret(program, tools, start):
        for kind, sample, end in machine.apply(name, args):
            moves.append(
                dict(
                    kind="feed" if kind == "arc" else kind,
                    printed=3 * PRINTED_MM if kind == "arc" else PRINTED_MM,
                    sample=sample,
                    end=end,
                    feed=machine.feed,
                    per_rev=machine.per_rev,
                    spindle=machine.spindle,
                    css_max=machine.css_max,
                    lead=machine.lead,
                    outputs=machine.outputs,
                    adaptive_feed=machine.adaptive_feed,
                    flood=machine.flood,
                    mist=machine.mist,
                    tool=machine.tool,
                )
            )
    return moves


def _match(test, planned, moves):
    """Assert the interpreter's `moves` execute the `planned` motions.

    `planned`: dicts with the motion's `curve`, the `kind` of interpreter
    move it must be (traverse, feed, tap, thread), a `tolerance` (to which
    what the interpreter's printing can hide is added), a `where`
    for messages and `state`, the interpreter state its cutting moves must
    be made in. Every planned motion, in order, must be executed by one or
    more interpreter moves that stay on its curve and end where it ends;
    nothing the interpreter moves may be left over. Returns the number of
    interpreter moves.
    """
    used = 0
    for item in planned:
        curve, where = item["curve"], item["where"]
        tolerance = item["tolerance"] + PRINTED_MM
        if curve["kind"] == "line" and math.dist(curve["start"], curve["end"]) <= tolerance:
            # Shorter than the comparison resolves: the interpreter may or
            # may not move for it.
            if used < len(moves) and math.dist(moves[used]["end"], curve["end"]) <= tolerance:
                if all(distance_to_curve(moves[used]["sample"](t), curve) <= tolerance for t in (0.0, 0.5, 1.0)):
                    used += 1
            continue
        while True:
            test.assertLess(used, len(moves), f"the interpreter stopped moving before {where}")
            move = moves[used]
            used += 1
            tolerance = item["tolerance"] + move["printed"]
            for t in (0.0, 0.25, 0.5, 0.75, 1.0):
                off = distance_to_curve(move["sample"](t), curve)
                test.assertLessEqual(off, tolerance, f"interpreter move {used} is {off:.5f} mm off {where}")
            test.assertEqual(move["kind"], item["kind"], where)
            for name, value in item["state"].items():
                if isinstance(value, float):
                    test.assertAlmostEqual(move[name], value, delta=5e-4, msg=f"{name} at {where}")
                else:
                    test.assertEqual(move[name], value, f"{name} at {where}")
            if math.dist(move["end"], curve["end"]) <= tolerance:
                break
    test.assertEqual(used, len(moves), f"the interpreter makes {len(moves) - used} more moves than the plan has")
    return used


def compare(test, posted, program, operations, tools, start):
    """Assert that LinuxCNC's interpreter executes `program` as `posted` planned it.

    `posted`: a finished `PostedRun` (a milling setup). `operations`:
    {operation id: object with spindle_rpm, coolant and tool_number}.
    `tools`: the machine's tool numbers and registers (for the tool table).
    `start`: the tip position the program starts from.

    Each planned motion is compared within the post's own deviation bound
    for that motion plus the interpreter's print resolution. A rapid must be
    a traverse and anything else a feed at the planned rate; while it feeds,
    the spindle, coolant and tool must be the operation's. A synchronised
    tapping move (G33.1) must be a tap move of the plan, down and back; its
    lead is not in the interpreter's output and is not compared here.
    Returns the number of interpreter moves compared.
    """
    planned = []
    for index in range(posted.status()["available"]):
        record = posted.posted_motion(index)
        motion = record["motion"]
        operation = operations[motion["operation"]]
        kind = "tap" if motion["kind"] == "tap" else "traverse" if motion["feed_mm_min"] is None else "feed"
        state = {}
        if kind != "traverse":
            state = dict(
                spindle=float(operation.spindle_rpm),
                flood=operation.coolant == "flood",
                mist=operation.coolant == "mist",
                tool=operation.tool_number,
            )
        if kind == "feed":
            state.update(feed=float(motion["feed_mm_min"]), per_rev=False)
        planned.append(
            dict(
                curve=motion["curve"],
                kind=kind,
                tolerance=record["mapping"]["deviation_bound_mm"],
                where=f"motion {motion['id']} ({motion['operation']}, {motion['kind']}), "
                f"NC {[n['text'] for n in record['nc']]}",
                state=state,
            )
        )
    return _match(test, planned, _interpreted(program, tools, start))


def compare_program(test, events, program, tools, start, tolerance):
    """Assert that LinuxCNC's interpreter executes `program` as the planned
    `events` say (a `Program`'s events as dictionaries: `to_dict()["events"]`).

    For a turning program. The state each cutting move must be made in is
    the plan's own, followed through its events: the tool of the last tool
    change; a fixed spindle speed, or constant surface speed with its rpm
    limit (G96: the interpreter's speed is then the surface speed); feed per
    minute, or per revolution (G95) at the declared feed; coolant. A
    threading move must be synchronised (G33) with the lead its planned feed
    and speed give. `tolerance`: the program's tolerance plus half a unit of
    the machine's last decimal on each axis.
    """
    tool = None
    spindle = dict(spindle=0.0, css_max=0.0)
    per_rev = None
    rpm = 0.0
    coolant = dict(flood=False, mist=False)
    planned = []
    for event in events:
        kind = event["kind"]
        if kind == "tool_change":
            tool = event["number"]
        elif kind == "spindle":
            sign = -1.0 if event["direction"] == "ccw" else 1.0
            rpm = float(event["rpm"])
            spindle = dict(spindle=sign * rpm, css_max=0.0)
        elif kind == "surface_speed":
            sign = -1.0 if event["direction"] == "ccw" else 1.0
            spindle = dict(spindle=sign * float(event["m_min"]), css_max=float(event["max_rpm"]))
        elif kind == "feed_per_rev":
            # 0 returns to feeds per minute.
            per_rev = float(event["mm_rev"]) or None
        elif kind == "coolant":
            coolant = dict(flood=event["mode"] == "flood", mist=event["mode"] == "mist")
        elif kind == "motion":
            motion = event["motion"]
            where = f"motion {motion['id']} ({motion['operation']}, {motion['kind']})"
            state = {}
            if motion["kind"] == "thread":
                expected = "thread"
                state = dict(lead=motion["feed_mm_min"] / rpm, tool=tool, **spindle, **coolant)
            elif motion["feed_mm_min"] is None:
                expected = "traverse"
            else:
                expected = "feed"
                feed = dict(feed=float(motion["feed_mm_min"]), per_rev=False)
                if per_rev is not None:
                    feed = dict(feed=per_rev, per_rev=True)
                state = dict(tool=tool, **spindle, **coolant, **feed)
            planned.append(dict(curve=motion["curve"], kind=expected, tolerance=tolerance, where=where, state=state))
    return _match(test, planned, _interpreted(program, tools, start))


def planned_events(job):
    """The events a job's run plans, as dictionaries in stream order (each
    operation's own events and every motion): what a post is given."""
    from camlib.jobs import JobRun
    from camlib.simulation import OperationUpdate

    events = []
    for update in JobRun(job.compile(), None):
        u = update.update
        if isinstance(u, OperationUpdate):
            events.extend(u.events)
        elif hasattr(u, "motion"):
            events.append(dict(kind="motion", motion=u.motion))
    return events


def deviation_bounds(posted):
    """{planned motion id: the post's own bound on how far the written
    blocks may lie from that motion}, of a finished `PostedRun`. A motion
    the post writes as several blocks (an arc as checked line segments) has
    a bound for each; the largest is the motion's."""
    bounds = {}
    for index in range(posted.status()["available"]):
        mapping = posted.posted_motion(index)["mapping"]
        source = mapping["source_motion_id"]
        bounds[source] = max(bounds.get(source, 0.0), mapping["deviation_bound_mm"])
    return bounds


def compare_thermal(test, events, program, start, bounds, *, torch_tool, full_power_s, thc_off_output):
    """Assert that LinuxCNC's interpreter executes a plasma `program` as the
    planned `events` say (each motion within its bound in `bounds`, see
    `deviation_bounds`), for a post that writes the torch as a spindle and
    height control as a digital output switched with motion (QtPlasmaC:
    `M3 $0 S1` / `M5 $0`, and `M62 P2` to disable torch height control,
    `M63 P2` to enable it).

    Besides the path and feeds: every feed move is made with the torch as
    the plan has it (on at the planned power, as the spindle speed
    `full_power_s` x power; off for a move the plan makes with the beam
    off), every traverse with the torch off, the height-control output is
    as planned from the first time the plan sets it, the tool is the
    post's torch tool, and feed hold on a paused motion (`M52 P1`, which
    QtPlasmaC requires) is on.
    """
    power = 0.0
    height = None
    planned = []
    for event in events:
        kind = event["kind"]
        if kind in ("beam", "beam_mark"):
            # Cutting or marking: QtPlasmaC fires the torch the same way for
            # both (the material selects the current).
            power = float(event["power_percent"])
        elif kind == "height_control":
            height = bool(event["on"])
        elif kind == "motion":
            motion = event["motion"]
            where = f"motion {motion['id']} ({motion['operation']}, {motion['kind']})"
            state = dict(spindle=full_power_s * power / 100.0, tool=torch_tool, adaptive_feed=True)
            if motion["feed_mm_min"] is None:
                expected = "traverse"
                test.assertEqual(power, 0.0, f"the plan rapids with the beam on at {where}")
            else:
                expected = "feed"
                state.update(feed=float(motion["feed_mm_min"]), per_rev=False)
            if height is not None:
                state["outputs"] = frozenset() if height else frozenset({thc_off_output})
            tolerance = bounds[motion["id"]]
            planned.append(dict(curve=motion["curve"], kind=expected, tolerance=tolerance, where=where, state=state))
    return _match(test, planned, _interpreted(program, [torch_tool], start))
