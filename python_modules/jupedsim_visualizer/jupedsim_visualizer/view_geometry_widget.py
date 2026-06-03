# SPDX-License-Identifier: LGPL-3.0-or-later
import jupedsim as jps
from PySide6.QtCore import Qt
from PySide6.QtGui import QPaintEvent
from PySide6.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from jupedsim_visualizer.geometry import Geometry
from jupedsim_visualizer.geometry_widget import RenderWidget

_ROUTING_ENGINES = ["AStar", "SurfaceMeshShortestPath", "DirectPath"]


def _provides_mesh(navi) -> bool:
    """An engine can display a triangulation iff it exposes a mesh()."""
    return callable(getattr(navi, "mesh", None))


class ViewGeometryWidget(QWidget):
    def __init__(
        self,
        navi: jps.AStarRoutingEngine,
        geo: Geometry,
        name_text: str,
        info_text: str,
        geometry=None,
        parent=None,
    ):
        QWidget.__init__(self, parent)
        self.geo = geo
        self._geometry = geometry
        # Lazily-built engines keyed by combo label; AStar is the one already set up.
        self._navis = {"AStar": navi}
        self._show_triangulation_requested = False

        bottom_layout = QHBoxLayout()
        geometry_label = QLabel(name_text)
        geometry_label.setAlignment(Qt.AlignmentFlag.AlignLeft)
        bottom_layout.addWidget(geometry_label, 1, Qt.AlignmentFlag.AlignLeft)

        properties_label = QLabel(info_text)
        properties_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        bottom_layout.addWidget(
            properties_label, 1, Qt.AlignmentFlag.AlignRight
        )

        toolbar_layout = QHBoxLayout()

        routing_combo = QComboBox()
        routing_combo.addItems(_ROUTING_ENGINES)
        toolbar_layout.addWidget(routing_combo, 1)
        self._routing_combo = routing_combo

        reset_cam_bt = QPushButton("Reset Camera")
        toolbar_layout.addWidget(reset_cam_bt, 2)

        layout = QVBoxLayout()
        layout.addLayout(toolbar_layout)

        self.render_widget = RenderWidget(geo, navi, [geo], parent=self)
        layout.addWidget(self.render_widget)

        self.hover_label = QLabel("")
        self.hover_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.hover_label)

        layout.addLayout(bottom_layout)
        self.setLayout(layout)

        reset_cam_bt.clicked.connect(self.render_widget.reset_camera)
        self.render_widget.on_hover_triangle.connect(self.hover_label.setText)
        routing_combo.currentTextChanged.connect(self._on_routing_engine_changed)

    def _navi_for(self, engine_name: str):
        navi = self._navis.get(engine_name)
        if navi is None:
            if engine_name == "SurfaceMeshShortestPath":
                navi = jps.SurfaceMeshShortestPathRoutingEngine()
                navi.set_geometry(self._geometry)
            elif engine_name == "DirectPath":
                navi = jps.DirectPathRoutingEngine()
            else:
                raise ValueError(f"Unknown routing engine: {engine_name}")
            self._navis[engine_name] = navi
        return navi

    def _on_routing_engine_changed(self, engine_name: str) -> None:
        navi = self._navi_for(engine_name)
        self.geo.show_triangulation(
            _provides_mesh(navi) and self._show_triangulation_requested
        )
        self.render_widget.set_routing_engine(navi)

    def set_triangulation_visible(self, state: bool) -> None:
        self._show_triangulation_requested = state
        if _provides_mesh(self.render_widget.navi):
            self.geo.show_triangulation(state)

    def render(self):
        self.render_widget.render()

    def paintEvent(self, event: QPaintEvent) -> None:
        self.render()
        return super().paintEvent(event)
