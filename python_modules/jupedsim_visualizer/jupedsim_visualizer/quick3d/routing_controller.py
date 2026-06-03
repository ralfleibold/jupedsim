# SPDX-License-Identifier: LGPL-3.0-or-later
"""Live shortest-path routing state machine for the geometry view.

Owns the route endpoints, the drag flag, the computed waypoints and their
total length, and turns waypoints into a QQuick3DGeometry ribbon. It is pure
state + logic: it does not know about QML or widgets. The view converts
pixels to world coordinates (via the camera) before handing endpoints here,
and passes in a `push_geometry` sink that is called with the new path
geometry (or None to clear it).

Frame coalescing — *when* the expensive recompute / rebuild actually run —
stays in the view; this class just exposes `recompute()` and
`rebuild_geometry()` for the view's flush to call.
"""
from __future__ import annotations

import math
from collections.abc import Callable

from jupedsim_visualizer.quick3d.camera import Camera2D
from jupedsim_visualizer.quick3d.polygon_geometry import build_path_geometry

_Point = tuple[float, float]


class RoutingController:
    def __init__(
        self,
        navi,
        camera: Camera2D,
        extent: float,
        push_geometry: Callable[[object], None],
    ) -> None:
        self._navi = navi
        self._camera = camera
        self._extent = extent  # for the path-width fallback at mag <= 0
        self._push_geometry = push_geometry

        self.route_from: _Point | None = None
        self.route_to: _Point | None = None
        self.dragging = False
        self.waypoints: list[_Point] | None = None
        self.distance = 0.0

    @property
    def enabled(self) -> bool:
        """Routing is only possible when the view was given a navi."""
        return self._navi is not None

    @property
    def has_path(self) -> bool:
        return self.waypoints is not None

    # ----- drag lifecycle (called from the view's input handlers) ---------

    def start(self, world: _Point) -> None:
        """Begin a new route at *world*. Drops any previously drawn path."""
        self.dragging = True
        self._clear()
        self.route_from = world
        self.route_to = None

    def set_target(self, world: _Point) -> None:
        self.route_to = world

    def end(self) -> None:
        # Don't clear the path — leave it on screen until the next route.
        self.dragging = False

    # ----- coalesced work (called from the view's flush) ------------------

    def recompute(self) -> None:
        """Recompute the path for the current endpoints (the expensive A*)
        and rebuild its geometry. Clears the path if either endpoint is
        outside the walkable region."""
        if (
            self._navi is None
            or self.route_from is None
            or self.route_to is None
        ):
            return
        try:
            if not self._navi.is_routable(
                self.route_from
            ) or not self._navi.is_routable(self.route_to):
                # Endpoint outside the walkable region — drop the path
                # entirely (matches the old VTK behaviour).
                self._clear()
                return
            waypoints = self._navi.compute_waypoints(
                self.route_from, self.route_to
            )
        except Exception:
            self._clear()
            return

        self.waypoints = list(waypoints)
        self.distance = sum(
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(waypoints[:-1], waypoints[1:])
        )
        self.rebuild_geometry()

    def rebuild_geometry(self) -> None:
        """Rebuild the path ribbon from the existing waypoints (no A*). Used
        when only the magnification changed (zoom / resize) — the path width
        is in world units and tracks mag."""
        if not self.waypoints:
            return
        geo = build_path_geometry(
            self.waypoints,
            origin=(self._camera.cx, self._camera.cy),
            width=self._path_width(),
        )
        self._push_geometry(geo)

    # ----- internal -------------------------------------------------------

    def _clear(self) -> None:
        self.waypoints = None
        self.distance = 0.0
        self._push_geometry(None)

    def _path_width(self) -> float:
        """Path thickness in world units, targeting ~2 px on screen — matches
        the old VTK impl's `SetLineWidth(3)` look (most GL drivers cap line
        width at 1 px in practice, so the old result was ~1-2 px anyway)."""
        mag = self._camera.mag
        if mag <= 0:
            return self._extent * 0.002
        return 2.0 / mag
