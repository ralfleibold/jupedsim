# SPDX-License-Identifier: LGPL-3.0-or-later
"""Build a flat QQuick3DGeometry mesh from a shapely (Multi)Polygon.

We triangulate the polygon (with holes respected) on the Python side and
hand the resulting triangle soup to Quick 3D as a single mesh. The mesh
lives in the XY plane (z = 0). Future floor support: add z-offset per
polygon when constructing.
"""
from __future__ import annotations

import struct

import shapely
from PySide6.QtCore import QByteArray
from PySide6.QtGui import QVector3D
from PySide6.QtQuick3D import QQuick3DGeometry


def _iter_polygons(geom):
    """Yield shapely.Polygon for Polygon / MultiPolygon / GeometryCollection."""
    if geom.is_empty:
        return
    if geom.geom_type == "Polygon":
        yield geom
        return
    if hasattr(geom, "geoms"):
        for sub in geom.geoms:
            yield from _iter_polygons(sub)


def _triangulate_polygon_with_holes(polygon: shapely.Polygon):
    """Return a list of (p0, p1, p2) triangles tiling *polygon* (holes
    respected). Uses shapely's constrained Delaunay triangulation — the
    important word being *constrained*: hole edges are forced into the
    triangulation, so no triangle ever crosses a hole boundary."""
    tris = shapely.constrained_delaunay_triangles(polygon)
    out = []
    for tri in tris.geoms:
        coords = list(tri.exterior.coords)[:3]
        out.append(tuple(coords))
    return out


def build_mesh_edges_geometry(
    vertices: list[tuple[float, float]],
    polygons: list[list[int]],
    origin: tuple[float, float] = (0.0, 0.0),
    z: float = 0.05,
) -> QQuick3DGeometry:
    """Build a Lines geometry containing every polygon edge of *polygons*.

    *vertices*  : list of (x, y) pairs, indexed by *polygons*.
    *polygons*  : list of vertex-index sequences (any size — triangles,
                  general convex polygons, …).
    *origin*    : same translation as `build_polygon_geometry`. Pass the
                  walls' centroid so the overlay is co-located with them.
    *z*         : small positive offset so the lines sit *above* the wall
                  fill (which is at z=0). Quick 3D won't z-fight here.
    """
    ox, oy = origin
    stride = 24  # position (3 float) + normal (3 float)
    buf = bytearray()
    nx, ny, nz = 0.0, 0.0, 1.0  # normals unused for Lines but keep layout consistent

    for poly in polygons:
        n = len(poly)
        for i in range(n):
            ax, ay = vertices[poly[i]]
            bx, by = vertices[poly[(i + 1) % n]]
            buf += struct.pack(
                "<ffffff", float(ax) - ox, float(ay) - oy, z, nx, ny, nz
            )
            buf += struct.pack(
                "<ffffff", float(bx) - ox, float(by) - oy, z, nx, ny, nz
            )

    geometry = QQuick3DGeometry()
    geometry.clear()
    geometry.setStride(stride)
    geometry.setVertexData(QByteArray(bytes(buf)))
    geometry.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Lines)
    geometry.addAttribute(
        QQuick3DGeometry.Attribute.PositionSemantic,
        0,
        QQuick3DGeometry.Attribute.F32Type,
    )
    geometry.addAttribute(
        QQuick3DGeometry.Attribute.NormalSemantic,
        12,
        QQuick3DGeometry.Attribute.F32Type,
    )

    if vertices:
        xs = [vx - ox for vx, _ in vertices]
        ys = [vy - oy for _, vy in vertices]
        geometry.setBounds(
            QVector3D(min(xs), min(ys), z),
            QVector3D(max(xs), max(ys), z),
        )
    return geometry


def build_grid_geometry(
    xmin: float,
    ymin: float,
    xmax: float,
    ymax: float,
    spacing: float,
    origin: tuple[float, float] = (0.0, 0.0),
    z: float = 0.075,
) -> QQuick3DGeometry:
    """Build a Lines geometry: a rectangular axis-aligned grid covering
    ``[xmin..xmax] × [ymin..ymax]`` with the given world-space *spacing*.
    Translated by *origin* the same way as walls / mesh / path so it
    aligns with the rest of the scene. Z is set above the wall fill but
    below the routed path."""
    import math

    ox, oy = origin
    sx = math.floor(xmin / spacing) * spacing
    ex = math.ceil(xmax / spacing) * spacing
    sy = math.floor(ymin / spacing) * spacing
    ey = math.ceil(ymax / spacing) * spacing

    stride = 24
    buf = bytearray()

    def emit(x, y):
        nonlocal buf
        buf += struct.pack("<ffffff", float(x) - ox, float(y) - oy, z, 0.0, 0.0, 1.0)

    # Vertical lines.
    x = sx
    while x <= ex + 1e-9:
        emit(x, sy)
        emit(x, ey)
        x += spacing
    # Horizontal lines.
    y = sy
    while y <= ey + 1e-9:
        emit(sx, y)
        emit(ex, y)
        y += spacing

    geometry = QQuick3DGeometry()
    geometry.clear()
    geometry.setStride(stride)
    geometry.setVertexData(QByteArray(bytes(buf)))
    geometry.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Lines)
    geometry.addAttribute(
        QQuick3DGeometry.Attribute.PositionSemantic,
        0,
        QQuick3DGeometry.Attribute.F32Type,
    )
    geometry.addAttribute(
        QQuick3DGeometry.Attribute.NormalSemantic,
        12,
        QQuick3DGeometry.Attribute.F32Type,
    )
    geometry.setBounds(
        QVector3D(sx - ox, sy - oy, z),
        QVector3D(ex - ox, ey - oy, z),
    )
    return geometry


def _grid_spacing_for_extent(extent: float) -> float:
    """Power-of-two grid spacing chosen for a scene of size *extent*.

    Matches the legacy VTK grid logic (`grid.py::__compute_scale`):
    spacing = 2^max(1, ceil(log2(parallel_scale)) - 4), where
    parallel_scale = extent/2 (half-viewport height in world units at
    the fit zoom). Examples:
      *  ~50 m extent (BUW)         → 2 m
      *  ~20 m extent (office_rooms)→ 2 m
      *  ~676 m extent (SiB2023)    → 32 m
    """
    import math

    parallel_scale = max(1.0, extent / 2.0)
    pow2 = math.ceil(math.log2(parallel_scale))
    return float(2 ** max(1, pow2 - 4))


def build_path_geometry(
    points: list[tuple[float, float]],
    origin: tuple[float, float] = (0.0, 0.0),
    z: float = 0.1,
    width: float = 0.0,
) -> QQuick3DGeometry:
    """Build a thick polyline (Triangles primitive) through *points*, with a
    given world-space *width*. We can't rely on OpenGL line width — most
    drivers cap GL_LINES at 1 px — so we extrude each segment perpendicular
    to its direction. No miter joints; per-segment quads are good enough."""
    ox, oy = origin
    stride = 24
    buf = bytearray()
    nx, ny, nz = 0.0, 0.0, 1.0
    half = max(width / 2.0, 1e-6)
    xs: list[float] = []
    ys: list[float] = []

    def emit(p):
        nonlocal buf
        buf += struct.pack("<ffffff", float(p[0]), float(p[1]), z, nx, ny, nz)
        xs.append(p[0])
        ys.append(p[1])

    for (ax, ay), (bx, by) in zip(points[:-1], points[1:]):
        ax2, ay2 = ax - ox, ay - oy
        bx2, by2 = bx - ox, by - oy
        dx, dy = bx2 - ax2, by2 - ay2
        d = (dx * dx + dy * dy) ** 0.5
        if d <= 0:
            continue
        # Perpendicular, length = half-width.
        px, py = -dy / d * half, dx / d * half
        AL = (ax2 + px, ay2 + py)
        AR = (ax2 - px, ay2 - py)
        BL = (bx2 + px, by2 + py)
        BR = (bx2 - px, by2 - py)
        # Two triangles per segment.
        emit(AL); emit(BL); emit(AR)
        emit(BL); emit(BR); emit(AR)

    geometry = QQuick3DGeometry()
    geometry.clear()
    geometry.setStride(stride)
    geometry.setVertexData(QByteArray(bytes(buf)))
    geometry.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles)
    geometry.addAttribute(
        QQuick3DGeometry.Attribute.PositionSemantic,
        0,
        QQuick3DGeometry.Attribute.F32Type,
    )
    geometry.addAttribute(
        QQuick3DGeometry.Attribute.NormalSemantic,
        12,
        QQuick3DGeometry.Attribute.F32Type,
    )
    if xs:
        geometry.setBounds(
            QVector3D(min(xs), min(ys), z),
            QVector3D(max(xs), max(ys), z),
        )
    return geometry


def triangulate(geom: shapely.geometry.base.BaseGeometry) -> list[tuple]:
    """Public for debugging: return the triangle list used to build the mesh."""
    triangles: list[tuple] = []
    for poly in _iter_polygons(geom):
        triangles.extend(_triangulate_polygon_with_holes(poly))
    return triangles


def build_polygon_geometry(
    geom: shapely.geometry.base.BaseGeometry,
    origin: tuple[float, float] = (0.0, 0.0),
) -> QQuick3DGeometry:
    """Build a QQuick3DGeometry with one triangle list covering *geom*.

    The mesh is flat (z = 0), oriented +Z up. One normal per vertex
    pointing +Z. No UVs, no colors — the material handles color uniformly.
    """
    triangles: list[tuple] = []
    for poly in _iter_polygons(geom):
        triangles.extend(_triangulate_polygon_with_holes(poly))

    # Translate so the scene is near (0, 0) — orthographic projection at the
    # numeric scale of real geometries (SiB2023 is around (-2000, -300)) is
    # noticeably jittery without this.
    ox, oy = origin
    # Vertex layout: position (3 float) + normal (3 float) = 24 bytes / vertex.
    stride = 24
    buf = bytearray()
    for tri in triangles:
        for (x, y) in tri:
            buf += struct.pack(
                "<ffffff", float(x) - ox, float(y) - oy, 0.0, 0.0, 0.0, 1.0
            )

    geometry = QQuick3DGeometry()
    geometry.clear()
    geometry.setStride(stride)
    geometry.setVertexData(QByteArray(bytes(buf)))
    geometry.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles)
    geometry.addAttribute(
        QQuick3DGeometry.Attribute.PositionSemantic,
        0,
        QQuick3DGeometry.Attribute.F32Type,
    )
    geometry.addAttribute(
        QQuick3DGeometry.Attribute.NormalSemantic,
        12,
        QQuick3DGeometry.Attribute.F32Type,
    )

    # Provide bounds so Quick 3D can frame / cull properly.
    xs, ys = [], []
    for tri in triangles:
        for (x, y) in tri:
            xs.append(x)
            ys.append(y)
    if xs:
        geometry.setBounds(
            QVector3D(min(xs) - ox, min(ys) - oy, 0.0),
            QVector3D(max(xs) - ox, max(ys) - oy, 0.0),
        )

    return geometry
