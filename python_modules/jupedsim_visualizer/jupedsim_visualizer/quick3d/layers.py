# SPDX-License-Identifier: LGPL-3.0-or-later
"""Render-layer descriptor for the Quick 3D geometry view.

A single flat dataclass — deliberately not a class hierarchy. Each visual
overlay (walls, grid, nav-mesh, routed path, …) is one `Layer`. Adding a new
overlay is then: build its geometry, append one `Layer(...)` in the view, and
add the matching ``Model { geometry: <prop> }`` block in GeometryScene.qml.
"""
from __future__ import annotations

from dataclasses import dataclass

from PySide6.QtQuick3D import QQuick3DGeometry


@dataclass
class Layer:
    # QML context property holding this layer's QQuick3DGeometry.
    geometry_prop: str
    # The geometry object. Held here so the Python reference stays alive —
    # QML only borrows it.
    geometry: QQuick3DGeometry | None = None
    # Bool context property toggling the layer's Model visibility. None for
    # layers that are always shown or gate visibility in QML another way
    # (e.g. the path layer hides itself when its geometry is null).
    show_prop: str | None = None
    # Initial value pushed for `show_prop` (ignored when show_prop is None).
    visible: bool = False
