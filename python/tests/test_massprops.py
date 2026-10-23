import numpy as np
import pytest

from vsim import massprops


def box(cx, cy, cz, lx, ly, lz):
    xs, ys, zs = (np.array([-0.5, 0.5]) * l + c for l, c in ((lx, cx), (ly, cy), (lz, cz)))
    v = np.array([[x, y, z] for x in xs for y in ys for z in zs])
    # Outward-facing triangles of the 6 faces (vertex index = 4x + 2y + z).
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    t = []
    for a, b, c, d in faces:
        t += [(a, b, c), (a, c, d)]
    return v, np.array(t)


def test_box_matches_the_formulas():
    v, t = box(1.0, -2.0, 0.5, 0.4, 1.0, 2.0)
    p = massprops.mesh_properties(v, t, 2700.0)
    m = 2700.0 * 0.4 * 1.0 * 2.0
    assert p.mass == pytest.approx(m)
    assert p.centroid == pytest.approx([1.0, -2.0, 0.5])
    expected = m / 12 * np.diag([1.0 ** 2 + 2.0 ** 2, 0.4 ** 2 + 2.0 ** 2, 0.4 ** 2 + 1.0 ** 2])
    assert p.inertia == pytest.approx(expected, abs=1e-9)
    # Inside-out gives the same answer.
    q = massprops.mesh_properties(v, t[:, ::-1], 2700.0)
    assert q.mass == pytest.approx(m)


def test_parallel_axis_combination():
    v, t = box(0, 0, 0, 1, 1, 1)
    a = massprops.mesh_properties(v + [1, 0, 0], t, 1.0)
    b = massprops.mesh_properties(v - [1, 0, 0], t, 1.0)
    total = massprops.combine([a, b])
    assert total.mass == pytest.approx(2.0)
    assert total.centroid == pytest.approx([0, 0, 0])
    # Each cube: 1/6 about its centre, +1 m^2 * 1 kg offset about y and z.
    assert total.inertia[1, 1] == pytest.approx(2 * (1 / 6 + 1))
    assert total.inertia[0, 0] == pytest.approx(2 / 6)
    assert massprops.scaled(a, 5.0).inertia[0, 0] == pytest.approx(5 / 6)


def test_open_mesh_is_rejected():
    v, t = box(0, 0, 0, 1, 1, 1)
    with pytest.raises(ValueError):
        massprops.mesh_properties(v, t[:2], 1.0)
