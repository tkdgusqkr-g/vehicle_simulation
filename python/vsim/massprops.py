"""Mass properties of closed triangle meshes (design doc §5.4).

The vehicle builder page (ui/builder) computes the same in the browser; this
module is the reference and the command-line tool:

    python -m vsim.massprops car.glb --density 1600       # every mesh
    python -m vsim.massprops part.stl --density 2700 --units mm

Volumes come from the divergence theorem over tetrahedra from the origin,
so each mesh must be closed (watertight) with consistent winding; inside-out
meshes give negative volume and are flipped.
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
from dataclasses import dataclass

import numpy as np


@dataclass
class MassProperties:
    mass: float               # kg
    volume: float             # m^3
    centroid: np.ndarray      # m, (3,)
    inertia: np.ndarray       # kg m^2 about the centroid, (3, 3)

    def to_dict(self) -> dict:
        return {"mass": self.mass, "volume": self.volume, "centroid": self.centroid.tolist(),
                "inertia": self.inertia.tolist()}


def weld(vertices: np.ndarray, triangles: np.ndarray, tolerance: float = 1e-7) -> tuple[np.ndarray, np.ndarray]:
    """Merges coincident vertices (STL and per-face CAD tessellations repeat them)."""
    v = np.asarray(vertices, dtype=float)
    scale = max(float(np.abs(v).max()), 1e-9) if len(v) else 1.0
    keys = np.round(v / (tolerance * scale)).astype(np.int64)
    unique, inverse = np.unique(keys, axis=0, return_inverse=True)
    merged = np.zeros((len(unique), 3))
    merged[inverse.ravel()] = v
    return merged, inverse.ravel()[np.asarray(triangles, dtype=np.int64)]


def is_closed(triangles: np.ndarray) -> bool:
    """Every edge shared by exactly two triangles."""
    t = np.asarray(triangles, dtype=np.int64)
    edges = np.sort(np.concatenate([t[:, [0, 1]], t[:, [1, 2]], t[:, [2, 0]]]), axis=1)
    _, counts = np.unique(edges, axis=0, return_counts=True)
    return bool(len(counts)) and bool(np.all(counts == 2))


def mesh_properties(vertices: np.ndarray, triangles: np.ndarray, density: float) -> MassProperties:
    """Mass properties of a closed mesh of uniform density (kg/m^3)."""
    v, t = weld(vertices, triangles)
    t = t[(t[:, 0] != t[:, 1]) & (t[:, 1] != t[:, 2]) & (t[:, 2] != t[:, 0])]  # degenerate after welding
    if not is_closed(t):
        raise ValueError("mesh is not closed (every edge must join two triangles)")
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    vol = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6.0
    volume = vol.sum()
    sign = 1.0 if volume >= 0 else -1.0   # inside-out mesh
    volume *= sign
    vol *= sign
    if volume <= 0:
        raise ValueError("mesh encloses no volume (open or degenerate)")
    s = a + b + c
    centroid = (vol[:, None] * s).sum(axis=0) / 4.0 / volume
    # Second moment about the origin: V/20 (aa' + bb' + cc' + ss') per tetrahedron.
    outer = (np.einsum("i,ij,ik->jk", vol, a, a) + np.einsum("i,ij,ik->jk", vol, b, b)
             + np.einsum("i,ij,ik->jk", vol, c, c) + np.einsum("i,ij,ik->jk", vol, s, s)) / 20.0
    second = outer - volume * np.outer(centroid, centroid)   # about the centroid
    inertia = density * (np.trace(second) * np.eye(3) - second)
    return MassProperties(density * volume, volume, centroid, inertia)


def point_mass(mass: float, position) -> MassProperties:
    return MassProperties(mass, 0.0, np.asarray(position, dtype=float), np.zeros((3, 3)))


def scaled(p: MassProperties, mass: float) -> MassProperties:
    """The same shape with a given mass (e.g. a measured part weight)."""
    k = mass / p.mass if p.mass > 0 else 0.0
    return MassProperties(mass, p.volume, p.centroid.copy(), p.inertia * k)


def combine(parts: list[MassProperties]) -> MassProperties:
    """Total of several bodies, inertia about the common centroid."""
    mass = sum(p.mass for p in parts)
    centroid = sum(p.mass * p.centroid for p in parts) / mass
    inertia = np.zeros((3, 3))
    for p in parts:
        d = p.centroid - centroid
        inertia += p.inertia + p.mass * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return MassProperties(mass, sum(p.volume for p in parts), centroid, inertia)


# --- Files -------------------------------------------------------------------------

def read_stl(path: str) -> list[tuple[str, np.ndarray, np.ndarray]]:
    data = open(path, "rb").read()
    if data[:5].lower() == b"solid" and b"facet" in data[:1000]:
        points = [list(map(float, line.split()[1:4])) for line in data.decode(errors="replace").splitlines()
                  if line.strip().startswith("vertex")]
        v = np.array(points, dtype=float)
    else:
        count = struct.unpack_from("<I", data, 80)[0]
        rec = np.frombuffer(data, dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]), count=count, offset=84)
        v = rec["v"].reshape(-1, 3).astype(float)
    return [("stl", v, np.arange(len(v)).reshape(-1, 3))]


def read_glb(path: str) -> list[tuple[str, np.ndarray, np.ndarray]]:
    """Meshes of a binary glTF in world coordinates (node transforms applied), y up."""
    data = open(path, "rb").read()
    magic, version, _ = struct.unpack_from("<III", data, 0)
    if magic != 0x46546C67 or version != 2:
        raise ValueError("not a binary glTF 2.0 file")
    at, doc, binary = 12, None, b""
    while at + 8 <= len(data):
        length, kind = struct.unpack_from("<II", data, at)
        chunk = data[at + 8:at + 8 + length]
        if kind == 0x4E4F534A:
            doc = json.loads(chunk)
        elif kind == 0x004E4942:
            binary = chunk
        at += 8 + length
    types = {5126: "<f4", 5125: "<u4", 5123: "<u2", 5121: "u1"}
    sizes = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}

    def accessor(i):
        a = doc["accessors"][i]
        view = doc["bufferViews"][a["bufferView"]]
        n, k = a["count"], sizes[a["type"]]
        dtype = np.dtype(types[a["componentType"]])
        stride = view.get("byteStride", dtype.itemsize * k)
        start = view.get("byteOffset", 0) + a.get("byteOffset", 0)
        out = np.empty((n, k), dtype=float)
        for j in range(k):
            out[:, j] = np.ndarray((n,), dtype=dtype, buffer=binary, offset=start + j * dtype.itemsize,
                                   strides=(stride,))
        return out

    def matrix(node):
        if "matrix" in node:
            return np.array(node["matrix"], dtype=float).reshape(4, 4).T
        x, y, z, w = node.get("rotation", [0, 0, 0, 1])
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        m = np.eye(4)
        m[:3, :3] = r * np.array(node.get("scale", [1, 1, 1]))
        m[:3, 3] = node.get("translation", [0, 0, 0])
        return m

    meshes = []
    scene = doc.get("scenes", [{"nodes": list(range(len(doc.get("nodes", []))))}])[doc.get("scene", 0)]
    stack = [(i, np.eye(4)) for i in scene.get("nodes", [])]
    while stack:
        index, parent = stack.pop()
        node = doc["nodes"][index]
        world = parent @ matrix(node)
        if "mesh" in node:
            for prim in doc["meshes"][node["mesh"]]["primitives"]:
                pos = accessor(prim["attributes"]["POSITION"])
                v = (world @ np.c_[pos, np.ones(len(pos))].T).T[:, :3]
                idx = accessor(prim["indices"]).astype(np.int64).reshape(-1, 3) if "indices" in prim \
                    else np.arange(len(v)).reshape(-1, 3)
                meshes.append((node.get("name", f"node{index}"), v, idx))
        stack.extend((c, world) for c in node.get("children", []))
    return meshes


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Mass properties of closed meshes (.glb, .stl)")
    parser.add_argument("file")
    parser.add_argument("--density", type=float, default=1000.0, help="kg/m^3 (default 1000)")
    parser.add_argument("--units", choices=["m", "mm", "in"], default="m", help="model length unit")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    scale = {"m": 1.0, "mm": 0.001, "in": 0.0254}[args.units]
    meshes = read_glb(args.file) if args.file.lower().endswith(".glb") else read_stl(args.file)
    results = []
    for name, v, t in meshes:
        try:
            results.append((name, mesh_properties(v * scale, t, args.density)))
        except ValueError as error:
            print(f"skip {name}: {error}", file=sys.stderr)
    if not results:
        print("no closed meshes", file=sys.stderr)
        return 1
    total = combine([p for _, p in results])
    if args.json:
        print(json.dumps({"parts": {n: p.to_dict() for n, p in results}, "total": total.to_dict()}, indent=2))
    else:
        for name, p in results + [("TOTAL", total)]:
            c = p.centroid
            print(f"{name:24s} m {p.mass:9.3f} kg  cg ({c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f}) m  "
                  f"I diag ({p.inertia[0, 0]:.4f}, {p.inertia[1, 1]:.4f}, {p.inertia[2, 2]:.4f}) kg m^2")
    return 0


if __name__ == "__main__":
    sys.exit(main())
