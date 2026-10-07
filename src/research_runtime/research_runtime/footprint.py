"""Conservative convex outer envelope and polygon intersection in metres."""
import math


def convex_hull(points):
    if any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
        return ()
    points = sorted(set((float(x), float(y)) for x, y in points))
    if len(points) < 3:
        return tuple(points)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return tuple(lower[:-1] + upper[:-1])


def convex_polygons_intersect(first, second):
    # Separating-axis theorem; touching a cell is conservatively collision.
    for polygon in (first, second):
        for a, b in zip(polygon, polygon[1:] + polygon[:1]):
            axis = (a[1] - b[1], b[0] - a[0])
            p = [x * axis[0] + y * axis[1] for x, y in first]
            q = [x * axis[0] + y * axis[1] for x, y in second]
            if max(p) < min(q) - 1e-12 or max(q) < min(p) - 1e-12:
                return False
    return True
