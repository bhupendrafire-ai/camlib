"""A vertical lift after an arc whose words moved its end stays vertical.

At coarse precision an arc's radius-consistent words can end several words
from its source end (`post_rounding::circular_words` searches up to six words
around it). A straight-up move from there must keep the controller's decoded
XY whenever the block stays within the machine tolerance of its source;
writing the source XY instead adds a sideways step to a move leaving the
surface the cutter touches, a rapid that the stock-contact check cannot then
prove clear. Half the tolerance per axis was too tight: the arc below moves
its end 5.8 um in Y at tolerance 0.01 mm.
"""

from dataclasses import replace
import math
import unittest

from camlib.motion import Arc, Line, Motion
from camlib.post import LoweringOptions, post_program
from test_post import fixture
from test_rust_post import rust_program

# Found by search: 3-decimal words for this arc exist only with the end moved
# (0.0039, 0.0058) mm from the source end, more than half of 0.01 in Y.
START = (27.893, -8.196, 3.0)
END = (22.0148760318273, -5.787754034117453, 3.0)
CENTER = (24.384, -8.383, 3.0)
SWEEP = 2.257435796417693
TOP = (END[0], END[1], 10.0)


def lift_program():
    p, m = fixture()
    events = list(p.events)
    i = next(k for k, e in enumerate(events) if isinstance(e, Motion))
    events[i : i + 1] = [
        Motion(1, 'pocket', 't1', Line((5, 0, 3), START), 1200),
        Motion(2, 'pocket', 't1', Arc(START, END, CENTER, SWEEP), 1200),
        Motion(3, 'pocket', 't1', Line(END, TOP), None, kind='rapid'),
        Motion(4, 'pocket', 't1', Line(TOP, (5, 0, 10)), None, kind='rapid'),
    ]
    return replace(p, events=tuple(events)), replace(m, decimals=3, tolerance_mm=0.01)


def decoded(lowering):
    program, machine = lift_program()
    chunks = post_program(program, machine=machine, lowering=lowering)
    return [mv for c in chunks for mv in c.motions]


class VerticalLiftPosting(unittest.TestCase):
    def check(self, lowering):
        moves = decoded(lowering)
        arc = next(mv.curve for mv in moves if isinstance(mv.curve, Arc))
        moved = (arc.end[0] - END[0], arc.end[1] - END[1])
        self.assertGreater(max(map(abs, moved)), 0.005, 'the arc moved its end word past half the tolerance')
        self.assertLessEqual(math.hypot(*moved) + 0.0005, 0.01)
        lift = next(mv.curve for mv in moves if isinstance(mv.curve, Line) and mv.curve.end[2] == 10.0)
        self.assertEqual(lift.start[:2], arc.end[:2])
        self.assertEqual(lift.end[:2], lift.start[:2], 'the lift keeps the decoded XY: no sideways step')

    def test_default_post(self):
        self.check(None)

    def test_native_arc_lowering(self):
        self.check(LoweringOptions(native_arcs=True))

    def test_rust_post_writes_the_same_vertical_lift(self):
        program, machine = lift_program()
        for lowering in (None, LoweringOptions(native_arcs=True)):
            python = ''.join(c.text for c in post_program(program, machine=machine, lowering=lowering))
            self.assertIn('\nG0 X22.011 Y-5.782 Z10\n', python)
            self.assertEqual(rust_program(program, machine, lowering)['program'], python)


if __name__ == '__main__':
    unittest.main()
