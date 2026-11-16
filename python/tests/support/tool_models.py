"""Synthetic closed tool models shared by regression tests and benchmarks."""

import math


def dense_bar_mesh(around):
    import camlib

    rings = [(0.5 + 0.25 * k, 5 * (0.8 + 0.005 * k)) for k in range(41)]
    rings.extend((10.5 + 0.69 * k, 5) for k in range(1, 101))

    def at(ring, k):
        z, r = ring
        angle = math.tau * (k % around) / around
        return (-6 + r * math.cos(angle), r * math.sin(angle), z)

    triangles = []
    for lo, hi in zip(rings, rings[1:]):
        for k in range(around):
            a, b, c, d = at(lo, k), at(lo, k + 1), at(hi, k), at(hi, k + 1)
            triangles.extend(((a, b, d), (a, d, c)))
    for k in range(around):
        triangles.extend(
            (
                ((-6, 0, rings[0][0]), at(rings[0], k + 1), at(rings[0], k)),
                ((-6, 0, rings[-1][0]), at(rings[-1], k), at(rings[-1], k + 1)),
            )
        )
    return camlib.SurfaceMesh(triangles, 5 * (1 - math.cos(math.pi / around)))
