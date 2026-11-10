"""engagement_front link feed: fast only where links are proven clear of stock.

With link_feed_mm_min, a stay-down link runs at that feed only on stretches
whose cutter footprint the committed stock proves clear; the rest keeps the
cutting feed. The stream's own stock checks stay clear, and the program is
shorter.

With link_chip_thinning, stretches that may touch stock lightly feed between
the cutting and link feeds (radial chip thinning, capped at the link feed).
"""

import unittest

from camlib.operations import AdaptiveClearing, Roughing
from camlib.simulation import Simulation, StockSpec, Update, VerificationUpdate
from camlib.surface import SurfaceMesh
from camlib.verification import StockContactOptions, StreamChecks

from test_engagement_front_rapid_descent import CUT, TOOL, box


def rough(link_feed, chip_thinning=False):
    mesh = SurfaceMesh(box(12, 28, 12, 28, 0, 9), 0.0)
    extra = {} if link_feed is None else dict(link_feed_mm_min=link_feed)
    if chip_thinning:
        extra["link_chip_thinning"] = True
    op = Roughing(
        "rough",
        TOOL,
        mesh,
        top_z_mm=10,
        floor_z_mm=1,
        stepdown_mm=4,
        clearance_z_mm=20,
        retract_z_mm=11,
        feed_z_mm=11,
        clearing=AdaptiveClearing(1.0, path_strategy="engagement_front"),
        radial_allowance_mm=0.2,
        bottom_allowance_mm=0.2,
        **CUT,
        **extra,
    )
    feeds, statuses = [], []
    with Simulation(
        stock=StockSpec((0, 0, 0), (40, 40, 10), 0.5),
        tools=(TOOL,),
        operations=(op,),
        revision="1",
        initial_position_mm=(0, 0, 20),
        verification=StreamChecks(stock_contact=StockContactOptions()),
    ) as sim:
        for u in sim.run_operations():
            if isinstance(u, Update) and u.motion["kind"] == "link":
                feeds.append(u.motion["feed_mm_min"])
            elif isinstance(u, VerificationUpdate) and u.status not in (None, "clear"):
                statuses.append((u.completed, u.status))
        assert sim.state.finished
    return feeds, statuses


class LinkFeedTest(unittest.TestCase):
    def test_links_run_at_the_link_feed_only_where_proven_clear(self):
        feeds, statuses = rough(3000)
        self.assertEqual(statuses, [])
        self.assertIn(3000, feeds)
        self.assertTrue(set(feeds) <= {3000, CUT["feed_mm_min"]})
        plain, plain_statuses = rough(None)
        self.assertEqual(plain_statuses, [])
        self.assertEqual(set(plain), {CUT["feed_mm_min"]})

    def test_chip_thinning_feeds_light_links_between_the_two_feeds(self):
        feeds, statuses = rough(3000, chip_thinning=True)
        self.assertEqual(statuses, [])
        cut = CUT["feed_mm_min"]
        self.assertTrue(all(cut <= f <= 3000 for f in feeds))
        self.assertTrue(any(cut < f < 3000 for f in feeds))

    def test_chip_thinning_needs_a_link_feed(self):
        with self.assertRaises(Exception):
            rough(None, chip_thinning=True)

    def test_link_feed_must_be_positive(self):
        with self.assertRaises(Exception):
            rough(0)


if __name__ == "__main__":
    unittest.main()
