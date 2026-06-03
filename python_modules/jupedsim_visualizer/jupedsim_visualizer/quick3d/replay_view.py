# SPDX-License-Identifier: LGPL-3.0-or-later
"""Quick 3D replay viewer: a geometry view plus a frame player.

Composes the existing `Quick3DGeometryView` (walls / nav-mesh / grid / camera /
hover / routing — all reused as-is) with the shared `PlayerControlWidget`, and
drives the geometry view's agent layer one recording frame at a time. No VTK.

The widget owns the current frame index and clamps it; the geometry view owns
the agent disks (`set_agents`). `show_mesh` / `show_grid` forward to the inner
view so the Settings-menu toggles in `MainWindow` reach it.
"""
from __future__ import annotations

from jupedsim import RoutingEngine
from jupedsim.recording import Recording
from PySide6.QtCore import QSignalBlocker, QTimer
from PySide6.QtWidgets import QVBoxLayout, QWidget

from jupedsim_visualizer.player_controls import PlayerControlWidget
from jupedsim_visualizer.quick3d.geometry_view import Quick3DGeometryView


class Quick3DReplayWidget(QWidget):
    def __init__(
        self,
        navi: RoutingEngine,
        rec: Recording,
        parent=None,
    ):
        super().__init__(parent)
        self._rec = rec
        self._num_frames = rec.num_frames
        self._index = 0
        self._timer: QTimer | None = None

        self._view = Quick3DGeometryView(
            rec.geometry(), navi=navi, parent=self
        )
        self._control = PlayerControlWidget(parent=self)
        self._control.slider.setMaximum(max(0, self._num_frames - 1))

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._view, 1)
        layout.addWidget(self._control)

        self._control.play.toggled.connect(self._play)
        self._control.forward.clicked.connect(self._frame_forward)
        self._control.backward.clicked.connect(self._frame_backward)
        self._control.begin.clicked.connect(lambda: self._goto(0))
        self._control.end.clicked.connect(
            lambda: self._goto(self._num_frames - 1)
        )
        self._control.slider.valueChanged.connect(self._goto)

        self._goto(0)

    # ----- forwarded settings toggles -------------------------------------

    def show_mesh(self, state: bool) -> None:
        self._view.show_mesh(state)

    def show_grid(self, state: bool) -> None:
        self._view.show_grid(state)

    # ----- frame transport ------------------------------------------------

    def _clamp(self, index: int) -> int:
        return max(0, min(self._num_frames - 1, index))

    def _goto(self, index: int) -> None:
        self._index = self._clamp(index)
        frame = self._rec.frame(self._index)
        self._view.set_agents([agent.position for agent in frame.agents])
        self._control.update_replay_time(self._index / self._rec.fps)
        with QSignalBlocker(self._control.slider):
            self._control.slider.setValue(self._index)

    def _frame_forward(self) -> None:
        self._goto(self._index + self._control.speed_selector.value())

    def _frame_backward(self) -> None:
        self._goto(self._index - self._control.speed_selector.value())

    def _play(self, checked: bool) -> None:
        if checked:
            self._timer = QTimer(self)
            self._timer.setInterval(int(1000.0 / self._rec.fps))
            self._timer.timeout.connect(self._frame_forward)
            self._timer.start()
        elif self._timer is not None:
            self._timer.stop()
