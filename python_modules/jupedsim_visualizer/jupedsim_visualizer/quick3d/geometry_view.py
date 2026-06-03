# SPDX-License-Identifier: LGPL-3.0-or-later
"""A QQuickWidget that shows a flat polygon (top-down) with hover + routing.

Geometry source is shapely — *not* `navi.mesh()`. The nav-mesh may be inset
away from walls (wall-clearance), and we want the visualizer to show the
original walls plus, optionally, the mesh overlay on top.

Controls:
  - LMB drag           : live shortest path from press-position to cursor
  - Middle-button drag : pan
  - Mouse wheel        : zoom (anchored to cursor)
  - q / e              : zoom out / in (back-compat)
  - w / a / s / d      : pan up / left / down / right (back-compat)
"""
from __future__ import annotations

import math
from pathlib import Path

import shapely
from PySide6.QtCore import Qt, QSize, QTimer, QUrl, Signal
from PySide6.QtQuickWidgets import QQuickWidget
from PySide6.QtWidgets import (
    QLabel,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from jupedsim_visualizer.quick3d.polygon_geometry import (
    _grid_spacing_for_extent,
    build_grid_geometry,
    build_mesh_edges_geometry,
    build_path_geometry,
    build_polygon_geometry,
)

_QML_DIR = Path(__file__).parent / "qml"

# Zoom factor per wheel-notch (120 angle-delta units = one notch on most mice).
_WHEEL_NOTCH = 120.0
_WHEEL_ZOOM_FACTOR = 1.15  # per notch
_KEY_ZOOM_FACTOR = 1.05
_KEY_PAN_FRACTION = 0.035  # of the visible extent


class Quick3DGeometryView(QWidget):
    """Top-down view of a shapely polygon with a world-coord hover readout
    and live shortest-path routing (when a navi is supplied)."""

    hovered = Signal(float, float)  # world (x, y)

    def __init__(
        self,
        polygon: shapely.geometry.base.BaseGeometry,
        navi=None,
        parent=None,
    ):
        super().__init__(parent)

        xmin, ymin, xmax, ymax = polygon.bounds
        self._cx = (xmin + xmax) / 2.0
        self._cy = (ymin + ymax) / 2.0
        self._extent_x = (xmax - xmin) or 1.0
        self._extent_y = (ymax - ymin) or 1.0
        self._extent = max(self._extent_x, self._extent_y)  # kept for path width
        self._navi = navi

        # Pan is in the *translated* frame (geometry sits around origin).
        self._pan_x = 0.0
        self._pan_y = 0.0
        # Magnification model:
        #   mag = base_fit_mag * resize_scale(w/base_w, h/base_h) * user_zoom
        # The base is snapshot once at first layout; user_zoom is the
        # accumulated factor from wheel / q / e. The resize_scale is computed
        # fresh from (current size / base size) every time — no event-to-
        # event accumulation, so a round-trip back to the base size restores
        # exactly the original mag (no drift, no matter what path got us
        # there).
        self._base_w = 0
        self._base_h = 0
        self._base_fit_mag = 1.0
        self._user_zoom = 1.0
        self._mag = 1.0

        # Routing state. `_route_from` / `_route_to` persist across release —
        # the path stays drawn until the user starts a new route.
        self._route_from: tuple[float, float] | None = None
        self._route_to: tuple[float, float] | None = None
        self._dragging = False
        self._path_waypoints: list[tuple[float, float]] | None = None
        self._path_distance = 0.0

        # Geometry objects we hold a reference to (QML borrows).
        self._wall_geometry = build_polygon_geometry(
            polygon, origin=(self._cx, self._cy)
        )
        self._mesh_geometry = None
        self._face_polys: list = []
        self._face_tree = None
        if navi is not None:
            verts, polys = navi.mesh()
            self._mesh_geometry = build_mesh_edges_geometry(
                verts, polys, origin=(self._cx, self._cy)
            )
            # Spatial index so the hover-id lookup is O(log N) per cursor move.
            self._face_polys = [
                shapely.Polygon([verts[i] for i in face]) for face in polys
            ]
            self._face_tree = shapely.STRtree(self._face_polys)
        self._path_geometry = None

        # Grid overlay: spans the polygon bbox with margin. Spacing follows
        # the legacy VTK grid's power-of-2 rule (see _grid_spacing_for_extent).
        grid_margin = 0.5 * self._extent
        spacing = _grid_spacing_for_extent(self._extent)
        self._grid_geometry = build_grid_geometry(
            xmin - grid_margin,
            ymin - grid_margin,
            xmax + grid_margin,
            ymax + grid_margin,
            spacing,
            origin=(self._cx, self._cy),
        )

        # QQuickWidget renders the Quick 3D scene to an offscreen FBO and
        # composites it as an ordinary widget. We deliberately do NOT use
        # QQuickView + createWindowContainer: embedding a native (foreign)
        # QQuickView window into the QTabWidget hierarchy deadlocks on
        # Wayland during the wl_subsurface reparenting (the app froze on
        # tab insertion). QQuickWidget has no native subsurface and is robust
        # across X11 / Wayland / WSL, which is the whole point of this viewer.
        self._view = QQuickWidget()
        self._view.setResizeMode(QQuickWidget.ResizeMode.SizeRootObjectToView)
        self._view.setFocusPolicy(Qt.StrongFocus)  # keyboard events reach QML
        self._view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        ctx = self._view.rootContext()
        ctx.setContextProperty("wallGeometry", self._wall_geometry)
        ctx.setContextProperty("meshGeometry", self._mesh_geometry)
        ctx.setContextProperty("pathGeometry", self._path_geometry)
        ctx.setContextProperty("gridGeometry", self._grid_geometry)
        ctx.setContextProperty("showMesh", False)
        ctx.setContextProperty("showGrid", False)
        ctx.setContextProperty("panX", self._pan_x)
        ctx.setContextProperty("panY", self._pan_y)
        ctx.setContextProperty("viewMag", self._mag)
        self._view.setSource(QUrl.fromLocalFile(str(_QML_DIR / "GeometryScene.qml")))

        # QQuickWidget *is* the rendering widget; no createWindowContainer
        # wrapper. `self._quick` stays the name used by the size/layout math
        # (_view_size) and the layout below.
        self._quick = self._view

        # Single combined status line — cursor x/y, nav id under cursor, and
        # path length (when a path is drawn). Always-current so the user sees
        # feedback during drag too (drag suppresses the hover signal in QML).
        self._info_label = QLabel("")
        self._cursor_xy: tuple[float, float] | None = None
        self._cursor_nav_id: int | None = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._quick, 1)
        layout.addWidget(self._info_label)

        root = self._view.rootObject()
        if root is not None:
            root.hovered.connect(self._on_hover_px)
            root.exited.connect(self._on_exit)
            root.routeStart.connect(self._on_route_start)
            root.routeMove.connect(self._on_route_move)
            root.routeEnd.connect(self._on_route_end)
            root.panStart.connect(self._on_pan_start)
            root.panMove.connect(self._on_pan_move)
            root.panEnd.connect(self._on_pan_end)
            root.wheelZoom.connect(self._on_wheel)
            root.keyPressed.connect(self._on_key)

        # Internal drag state.
        self._pan_anchor_px: tuple[float, float] | None = None
        self._pan_anchor_world: tuple[float, float] | None = None

        # Frame coalescing. Mouse-move / pan / wheel events arrive faster than
        # the compositor can usefully render (libinput can fire >100/s), and
        # every render on QQuickWidget is a full GUI-thread render + FBO
        # composite. So input handlers only mutate state + set dirty flags;
        # the actual route recompute + context-property push (which triggers
        # the render) is flushed at most once per frame by this timer.
        #   _camera_dirty    -> re-push pan/zoom to QML
        #   _route_dirty     -> recompute waypoints (A*) + rebuild path
        #   _path_geom_dirty -> rebuild path geometry only (mag changed; same
        #                       waypoints) — skipped when _route_dirty covers it
        self._camera_dirty = False
        self._route_dirty = False
        self._path_geom_dirty = False
        self._frame_timer = QTimer(self)
        self._frame_timer.setSingleShot(True)
        self._frame_timer.setInterval(16)  # ~60 Hz cap
        self._frame_timer.timeout.connect(self._flush)

    def sizeHint(self) -> QSize:
        # Without this, QQuickWidget reports a tiny default hint which
        # propagates up through QTabWidget → QMainWindow and causes the
        # main window to shrink when this tab is inserted.
        return QSize(800, 600)

    def minimumSizeHint(self) -> QSize:
        return QSize(320, 240)

    # ----- public API -----------------------------------------------------

    def show_mesh(self, state: bool) -> None:
        self._view.rootContext().setContextProperty("showMesh", bool(state))

    def show_grid(self, state: bool) -> None:
        self._view.rootContext().setContextProperty("showGrid", bool(state))

    def reset_camera(self) -> None:
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._mag = self._fit_magnification()
        self._push_camera()

    # ----- camera helpers -------------------------------------------------

    def _view_size(self) -> tuple[int, int]:
        """Size used for all pixel-to-world math. `self._quick` is the
        QQuickWidget that owns the rendered surface; its size tracks the
        layout at every resize callback. QML's MouseArea reports coordinates
        in that same space, so this works for input math too."""
        return (max(1, self._quick.width()), max(1, self._quick.height()))

    def _fit_magnification(self) -> float:
        """Pick the largest magnification that still fits both axes of the
        polygon inside the viewport, with a small margin. Handles non-square
        polygons in non-square windows correctly — previously we used
        min(window) / max(extent), which tightly framed the long axis of
        the polygon against the *short* axis of the window and left
        unnecessary empty space along the matching dimension."""
        w, h = self._view_size()
        mag_w = w / self._extent_x
        mag_h = h / self._extent_y
        return min(mag_w, mag_h) * 0.97  # ~3% margin

    def _push_camera(self) -> None:
        ctx = self._view.rootContext()
        ctx.setContextProperty("panX", self._pan_x)
        ctx.setContextProperty("panY", self._pan_y)
        ctx.setContextProperty("viewMag", self._mag)

    def _schedule_flush(self) -> None:
        if not self._frame_timer.isActive():
            self._frame_timer.start()

    def _flush(self) -> None:
        """Apply the latest coalesced state: push the camera and/or rebuild
        the path. Runs at most once per frame (timer-driven) or immediately
        at drag/pan end so the final state is never left stale."""
        if self._camera_dirty:
            self._push_camera()
            self._camera_dirty = False
        if self._route_dirty:
            self._update_path()  # recompute waypoints + rebuild path geometry
            self._route_dirty = False
            self._path_geom_dirty = False
        elif self._path_geom_dirty:
            self._rebuild_path_geometry()
            self._path_geom_dirty = False

    def _flush_now(self) -> None:
        """Cancel any pending frame and flush synchronously (drag/pan end)."""
        self._frame_timer.stop()
        self._flush()

    def _px_to_translated(self, px: float, py: float) -> tuple[float, float]:
        """View pixel -> world coords in the *translated* (camera) frame.
        Reverse this with `+ (cx, cy)` to recover original WKT coords."""
        w, h = self._view_size()
        dx = (px - w / 2.0) / self._mag
        dy = -(py - h / 2.0) / self._mag  # screen-y goes down
        return (self._pan_x + dx, self._pan_y + dy)

    def _to_world(self, tx: float, ty: float) -> tuple[float, float]:
        return (self._cx + tx, self._cy + ty)

    def _auto_mag(self, w: int, h: int) -> float:
        """Magnification chosen automatically by the resize logic (i.e.
        ignoring the user's wheel/key zoom multiplier).

        Behavior the user observed they want:
          * Both axes grew relative to the base size  →  polygon grows
            proportionally by the smaller of the two growth ratios.
          * Only one axis changed (grew *or* shrunk)  →  polygon size
            unchanged. The empty space along the changing axis just
            grows/shrinks.
          * Both axes shrank, or one axis shrank enough that the polygon
            no longer fits at base size  →  polygon shrinks to fit.

        Achieved as `min( base * max(1, min(rw, rh)), fit(w, h) )`:
          * `max(1, min(rw, rh))` is 1 unless BOTH axes have grown — so a
            one-axis change is a no-op for the target.
          * Capping by `fit(w, h)` enforces no-overflow when the window
            can no longer hold the polygon at the target mag.
          * Continuous at every axis-crossing — no jumps.
          * Stateless w.r.t. the previous event — round-trips can't drift.
        """
        rw = w / self._base_w
        rh = h / self._base_h
        growth = max(1.0, min(rw, rh))
        target = self._base_fit_mag * growth
        fit = min(w / self._extent_x, h / self._extent_y) * 0.97
        return min(target, fit)

    def _recompute_mag(self) -> None:
        if self._base_w == 0 or self._base_h == 0:
            return
        w, h = self._view_size()
        # User wheel/key zoom multiplies on top — and is allowed to exceed
        # the no-overflow cap. Zooming-in past fit is the whole point of
        # the wheel; the polygon then extends off-screen and the user can
        # pan to see it.
        self._mag = self._auto_mag(w, h) * self._user_zoom

    def resizeEvent(self, event):
        """Snapshot the base size on the first real resize; thereafter just
        recompute the derived mag. Lives on Quick3DGeometryView (a real
        Python QWidget) — the container returned by `createWindowContainer`
        is C++-only and won't dispatch resize events back to Python."""
        super().resizeEvent(event)
        w, h = self._view_size()
        if self._base_w == 0 or self._base_h == 0:
            self._base_w, self._base_h = w, h
            self._base_fit_mag = self._fit_magnification()
        self._recompute_mag()
        self._camera_dirty = True
        if self._path_waypoints is not None:
            self._path_geom_dirty = True
        self._schedule_flush()

    # ----- event handlers -------------------------------------------------

    def _refresh_info(self):
        parts: list[str] = []
        if self._cursor_xy is not None:
            wx, wy = self._cursor_xy
            parts.append(f"x: {wx:.2f}   y: {wy:.2f}")
        if self._cursor_nav_id is not None:
            parts.append(f"Nav ID: {self._cursor_nav_id}")
        if self._path_waypoints is not None:
            parts.append(f"path length: {self._path_distance:.2f}")
        self._info_label.setText("    ".join(parts))

    def _nav_id_at(self, wx: float, wy: float) -> int | None:
        if self._face_tree is None:
            return None
        pt = shapely.Point(wx, wy)
        # STRtree applies the predicate as `query_geom.<predicate>(tree_geom)`,
        # so we need `point.within(face)` here, not `contains`.
        hits = self._face_tree.query(pt, predicate="within")
        if len(hits) == 0:
            return None
        return int(hits[0])

    def _on_hover_px(self, px: float, py: float):
        tx, ty = self._px_to_translated(px, py)
        self._cursor_xy = self._to_world(tx, ty)
        self._cursor_nav_id = self._nav_id_at(*self._cursor_xy)
        self._refresh_info()
        self.hovered.emit(*self._cursor_xy)

    def _on_exit(self):
        self._cursor_xy = None
        self._cursor_nav_id = None
        self._refresh_info()

    def _on_wheel(self, px: float, py: float, delta_y: float):
        if delta_y == 0.0:
            return
        # Center-anchored zoom: cursor position is intentionally ignored;
        # the world point at the window centre stays at the window centre.
        # We tweak `user_zoom` so the zoom is preserved across resizes.
        notches = delta_y / _WHEEL_NOTCH
        self._user_zoom *= _WHEEL_ZOOM_FACTOR ** notches
        self._recompute_mag()
        self._camera_dirty = True
        if self._path_waypoints is not None:
            self._path_geom_dirty = True  # path width tracks mag
        self._schedule_flush()

    def _on_pan_start(self, px: float, py: float):
        self._pan_anchor_px = (px, py)
        self._pan_anchor_world = (self._pan_x, self._pan_y)

    def _on_pan_move(self, px: float, py: float):
        if self._pan_anchor_px is None or self._pan_anchor_world is None:
            return
        ax, ay = self._pan_anchor_px
        wx, wy = self._pan_anchor_world
        # Pixel delta -> world delta, inverted (dragging right shows scene
        # moving right, i.e. camera moves left).
        dx = (px - ax) / self._mag
        dy = -(py - ay) / self._mag
        self._pan_x = wx - dx
        self._pan_y = wy - dy
        self._camera_dirty = True
        self._schedule_flush()

    def _on_pan_end(self):
        self._pan_anchor_px = None
        self._pan_anchor_world = None
        self._flush_now()

    def _on_route_start(self, px: float, py: float):
        if self._navi is None:
            return
        self._dragging = True
        # A new drag starts a new route — drop the previously drawn path.
        self._path_waypoints = None
        self._path_distance = 0.0
        self._set_path_geometry(None)
        tx, ty = self._px_to_translated(px, py)
        self._route_from = self._to_world(tx, ty)
        self._cursor_xy = self._route_from
        self._route_to = None
        self._refresh_info()
        self._update_path()

    def _on_route_move(self, px: float, py: float):
        # QML suppresses the hover signal while LMB is held, so refresh the
        # cursor here too — the user wants to see x/y change during drag.
        tx, ty = self._px_to_translated(px, py)
        self._cursor_xy = self._to_world(tx, ty)
        self._cursor_nav_id = self._nav_id_at(*self._cursor_xy)
        if self._navi is None or self._route_from is None or not self._dragging:
            self._refresh_info()
            return
        self._route_to = self._cursor_xy
        self._route_dirty = True
        self._schedule_flush()
        self._refresh_info()

    def _on_route_end(self):
        # Don't clear the path — leave it on screen until the next route.
        self._dragging = False
        self._flush_now()  # render the final drag target without waiting a frame

    def _update_path(self):
        if self._navi is None or self._route_from is None or self._route_to is None:
            return
        try:
            if not self._navi.is_routable(self._route_from) or not self._navi.is_routable(
                self._route_to
            ):
                # Endpoint outside the walkable region — drop the path
                # entirely (matches the old VTK behaviour).
                self._path_waypoints = None
                self._path_distance = 0.0
                self._set_path_geometry(None)
                return
            waypoints = self._navi.compute_waypoints(
                self._route_from, self._route_to
            )
        except Exception:
            self._path_waypoints = None
            self._path_distance = 0.0
            self._set_path_geometry(None)
            return

        self._path_waypoints = list(waypoints)
        self._path_distance = sum(
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(waypoints[:-1], waypoints[1:])
        )
        self._rebuild_path_geometry()

    def _path_width_world(self) -> float:
        """Path thickness in world units, targeting ~2 px on screen — matches
        the old VTK impl's `SetLineWidth(3)` look (most GL drivers cap line
        width at 1 px in practice, so the old result was ~1-2 px anyway)."""
        if self._mag <= 0:
            return self._extent * 0.002
        return 2.0 / self._mag

    def _rebuild_path_geometry(self) -> None:
        if not self._path_waypoints:
            return
        geo = build_path_geometry(
            self._path_waypoints,
            origin=(self._cx, self._cy),
            width=self._path_width_world(),
        )
        self._set_path_geometry(geo)

    def _set_path_geometry(self, geometry):
        # Hold a Python reference (QML side only borrows).
        self._path_geometry = geometry
        self._view.rootContext().setContextProperty("pathGeometry", geometry)

    def _on_key(self, key: int, text: str):
        if text == "e":
            self._user_zoom *= _KEY_ZOOM_FACTOR
            self._recompute_mag()
        elif text == "q":
            self._user_zoom /= _KEY_ZOOM_FACTOR
            self._recompute_mag()
        elif text in ("w", "a", "s", "d"):
            step = (1.0 / self._mag) * (
                min(*self._view_size()) * _KEY_PAN_FRACTION
            )
            if text == "w":
                self._pan_y += step
            elif text == "s":
                self._pan_y -= step
            elif text == "a":
                self._pan_x -= step
            elif text == "d":
                self._pan_x += step
        else:
            return
        self._camera_dirty = True
        if self._path_waypoints is not None:
            self._path_geom_dirty = True
        self._schedule_flush()
