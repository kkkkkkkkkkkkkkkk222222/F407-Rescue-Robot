"""Plan-view sweep against the four yellow blocks. Dimensions in mm.

For translations the slab test is continuous, not a sampled path. An optional
disk centred on the pose origin models a conservative/illustrative footprint;
its exact rounded-corner clearance is computed from segment-to-rectangle
distance. A real wheel, claw, yaw change or climb is not modelled by that disk.
"""
from argparse import ArgumentParser
from math import hypot


def point_segment(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    scale = dx*dx + dy*dy
    t = max(0, min(1, ((p[0]-a[0])*dx + (p[1]-a[1])*dy)/scale)) if scale else 0
    return hypot(p[0]-a[0]-dx*t, p[1]-a[1]-dy*t)


def clearance(a, b, rect):
    x, y, X, Y = rect
    lo, hi = 0., 1.
    for start, end, low, high in zip(a, b, (x, y), (X, Y)):
        d = end-start
        if not d:
            if not low <= start <= high:
                break
        else:
            t0, t1 = sorted(((low-start)/d, (high-start)/d))
            lo, hi = max(lo, t0), min(hi, t1)
    else:
        if lo <= hi:
            return 0.
    corners = [(x, y), (x, Y), (X, Y), (X, y)]
    return min([point_segment(c, a, b) for c in corners] +
               [point_segment(p, corners[i-1], c) for p in (a, b)
                for i, c in enumerate(corners)])


blocks = [(x, y, x+150, y+100) for x in (-400, 250) for y in (-660, 560)]


def gap(a, b):
    return min(clearance(a, b, r) for r in blocks)


if __name__ == "__main__":
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--radius", type=float, default=0, help="disk radius, not measured")
    args = parser.parse_args()
    if args.radius < 0:
        parser.error("radius must be nonnegative")
    routes = {
        "centre to red material, nominal": ((0, 0), (-150, 540)),
        "red material aligned push through band": ((-150, 540), (-150, 850)),
        "illustrative visual shift OUT, 80 mm": ((-150, 540), (-230, 540)),
        "centreline through band (candidate)": ((0, 0), (0, 800)),
        "station 1 to 3": ((-400, 0), (400, 0)),
        "illustrative return, same side": ((-150, 900), (-400, 0)),
        "illustrative return, opposite side": ((-150, 900), (400, 0)),
    }
    for side in (-1, 1):
        for station in (-400, 400):
            routes[f"attack return side={side}, station={station}"] = (
                (side*150, -side*590), (station, -side*590))
    assert gap((0, 0), (0, 800)) == 250
    assert gap((-150, 540), (-150, 850)) == 100
    assert gap((-400, 560), (-250, 560)) == 0
    assert round(gap((-150, 540), (-150, 540)), 3) == round(hypot(100, 20), 3)
    for name, (a, b) in routes.items():
        g = gap(a, b)
        print(f"{name}: centre clearance={g:.1f}; disk margin={g-args.radius:.1f} mm")
