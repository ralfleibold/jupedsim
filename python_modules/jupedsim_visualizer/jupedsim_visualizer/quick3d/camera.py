# SPDX-License-Identifier: LGPL-3.0-or-later
"""Top-down 2D orthographic camera for the Quick 3D geometry view.

Pure math — no Qt, no QML — so it is unit-testable in isolation. The view
owns the widget/QML side (reading the viewport size, pushing pan/mag as
context properties) and delegates all coordinate and zoom math here.

World coordinates are centred at ``(cx, cy)``; the QML scene is laid out
around ``(0, 0)``, so callers translate by subtracting ``(cx, cy)``.
``mag`` is pixels per world unit.

Magnification model (deliberately stateless w.r.t. the previous event — a
round-trip back to the base window size restores the original mag exactly,
so resizing never drifts)::

    mag      = auto_mag(w, h) * user_zoom
    auto_mag = min(base_fit_mag * max(1, min(rw, rh)), fit(w, h))

where ``rw, rh`` are the current/base width and height ratios. ``base_*`` is
snapshot once at the first resize; ``user_zoom`` accumulates wheel/key zoom.
"""
from __future__ import annotations

_FIT_MARGIN = 0.97  # ~3% margin so the polygon doesn't touch the viewport edge


class Camera2D:
    def __init__(
        self, cx: float, cy: float, extent_x: float, extent_y: float
    ) -> None:
        self.cx = cx
        self.cy = cy
        self._extent_x = extent_x or 1.0
        self._extent_y = extent_y or 1.0
        # Pan is in the *translated* frame (geometry sits around origin).
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._base_w = 0
        self._base_h = 0
        self._base_fit_mag = 1.0
        self._user_zoom = 1.0
        self._mag = 1.0

    @property
    def mag(self) -> float:
        return self._mag

    @property
    def pan(self) -> tuple[float, float]:
        return (self._pan_x, self._pan_y)

    def fit_magnification(self, w: int, h: int) -> float:
        """Largest magnification that fits both axes of the polygon inside a
        ``w × h`` viewport, with a small margin. Fitting both axes (rather
        than min(window)/max(extent)) frames non-square polygons in
        non-square windows without leaving extra empty space on one axis."""
        return min(w / self._extent_x, h / self._extent_y) * _FIT_MARGIN

    def on_resize(self, w: int, h: int) -> None:
        """Snapshot the base size on the first real resize; thereafter just
        recompute the derived mag from the current size."""
        if self._base_w == 0 or self._base_h == 0:
            self._base_w, self._base_h = w, h
            self._base_fit_mag = self.fit_magnification(w, h)
        self._recompute_mag(w, h)

    def zoom_by(self, factor: float, w: int, h: int) -> None:
        """Multiply the user zoom (wheel / q / e). User zoom is allowed to
        exceed the no-overflow fit cap — zooming in past fit is the point of
        the wheel; the polygon then extends off-screen and the user pans."""
        self._user_zoom *= factor
        self._recompute_mag(w, h)

    def pan_by(self, dx: float, dy: float) -> None:
        self._pan_x += dx
        self._pan_y += dy

    def set_pan(self, x: float, y: float) -> None:
        self._pan_x = x
        self._pan_y = y

    def reset(self, w: int, h: int) -> None:
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._mag = self.fit_magnification(w, h)

    def px_to_world(
        self, px: float, py: float, w: int, h: int
    ) -> tuple[float, float]:
        """View pixel -> world coords in the original (WKT) frame."""
        dx = (px - w / 2.0) / self._mag
        dy = -(py - h / 2.0) / self._mag  # screen-y goes down
        return (self.cx + self._pan_x + dx, self.cy + self._pan_y + dy)

    # ----- internal -------------------------------------------------------

    def _auto_mag(self, w: int, h: int) -> float:
        """Magnification chosen automatically by the resize logic (ignoring
        the user's zoom multiplier):

          * Both axes grew vs base  -> polygon grows by the smaller of the
            two growth ratios.
          * Only one axis changed   -> polygon size unchanged; the empty
            space along the changing axis just grows/shrinks.
          * Both axes shrank (or one shrank enough that the polygon no longer
            fits at base size) -> polygon shrinks to fit.

        ``max(1, min(rw, rh))`` is 1 unless BOTH axes grew, so a one-axis
        change is a no-op; capping by ``fit`` enforces no-overflow.
        Continuous at every axis crossing and stateless w.r.t. prior events.
        """
        rw = w / self._base_w
        rh = h / self._base_h
        growth = max(1.0, min(rw, rh))
        target = self._base_fit_mag * growth
        return min(target, self.fit_magnification(w, h))

    def _recompute_mag(self, w: int, h: int) -> None:
        if self._base_w == 0 or self._base_h == 0:
            return
        self._mag = self._auto_mag(w, h) * self._user_zoom
