# SPDX-License-Identifier: LGPL-3.0-or-later
import math
from pathlib import Path

import jupedsim as jps
import shapely
from jupedsim.recording import Recording
from PySide6.QtCore import QSettings, QSize
from PySide6.QtStateMachine import QFinalState, QState, QStateMachine
from PySide6.QtWidgets import (
    QApplication,
    QFileDialog,
    QMainWindow,
    QMessageBox,
    QTabWidget,
)

from jupedsim_visualizer.geometry import Geometry
from jupedsim_visualizer.quick3d.geometry_view import Quick3DGeometryView
from jupedsim_visualizer.quick3d.replay_view import Quick3DReplayWidget
from jupedsim_visualizer.replay_widget import ReplayWidget
from jupedsim_visualizer.trajectory import Trajectory
from jupedsim_visualizer.view_geometry_widget import ViewGeometryWidget


class MainWindow(QMainWindow):
    def __init__(self, parent=None) -> None:
        QMainWindow.__init__(self, parent)
        self.settings = QSettings("jupedsim", "jupedsim_visualizer")
        self.setWindowTitle("jupedsim_visualizer")
        self._build_central_tabs_widget()
        self._build_menu_bar()
        self._build_state_machine()
        self.setVisible(True)

    def _build_central_tabs_widget(self):
        tabs = QTabWidget(self)
        tabs.setMinimumSize(QSize(640, 480))
        tabs.setMovable(True)
        tabs.setDocumentMode(True)
        tabs.setTabsClosable(True)
        tabs.setTabBarAutoHide(True)
        tabs.tabCloseRequested.connect(tabs.removeTab)
        self.setCentralWidget(tabs)
        self.tabs = tabs

    def _build_menu_bar(self) -> None:
        menu = self.menuBar()
        open_menu = menu.addMenu("File")
        open_wkt_act = open_menu.addAction("Open wkt file")
        open_wkt_act.triggered.connect(self._open_wkt)
        open_wkt_q3d_act = open_menu.addAction("Open wkt file (Quick 3D, preview)")
        open_wkt_q3d_act.triggered.connect(self._open_wkt_quick3d)
        open_replay_act = open_menu.addAction("Open replay file")
        open_replay_act.triggered.connect(self._open_replay)
        open_replay_q3d_act = open_menu.addAction(
            "Open replay file (Quick 3D, preview)"
        )
        open_replay_q3d_act.triggered.connect(self._open_replay_quick3d)
        settings_menu = menu.addMenu("Settings")
        self._show_triangulation = settings_menu.addAction("show triangulation")
        self._show_triangulation.setCheckable(True)
        self._show_triangulation.toggled.connect(self._toggle_triangulation)
        self._show_triangulation.setChecked(
            bool(
                self.settings.value(
                    "show_triangulation", type=bool, defaultValue=False
                )
            )
        )
        self._show_grid = settings_menu.addAction("show grid")
        self._show_grid.setCheckable(True)
        self._show_grid.toggled.connect(self._toggle_grid)
        self._show_grid.setChecked(
            bool(
                self.settings.value("show_grid", type=bool, defaultValue=False)
            )
        )

    def _build_state_machine(self) -> None:
        sm = QStateMachine(self)
        sm.finished.connect(QApplication.quit)

        start = self._build_start_state()
        sm.addState(start)

        exit = self._build_exit_state()
        sm.addState(exit)

        # start.addTransition(self.button.clicked, exit)

        sm.setInitialState(start)
        sm.start()
        self.state_machine = sm

    def _build_start_state(self) -> QState:
        state = QState()
        return state

    def _build_show_wkt_state(self) -> QState:
        state = QState()
        return state

    def _build_exit_state(self) -> QFinalState:
        state = QFinalState()
        return state

    def _toggle_triangulation(self, state: bool) -> None:
        self.settings.setValue("show_triangulation", state)
        for idx in range(self.tabs.count()):
            tab = self.tabs.widget(idx)
            if hasattr(tab, "geo"):
                tab.geo.show_triangulation(state)
            if hasattr(tab, "show_mesh"):
                tab.show_mesh(state)
        self.repaint()

    def _toggle_grid(self, state: bool) -> None:
        self.settings.setValue("show_grid", state)
        for idx in range(self.tabs.count()):
            tab = self.tabs.widget(idx)
            if hasattr(tab, "render_widget"):
                tab.render_widget.show_grid(state)
            elif hasattr(tab, "show_grid"):
                tab.show_grid(state)
        self.repaint()

    def _open_wkt(self):
        base_path_obj = self.settings.value(
            "files/last_wkt_location",
            type=str,
            defaultValue=Path("~").expanduser(),
        )
        base_path = Path(str(base_path_obj))
        file, _ = QFileDialog.getOpenFileName(
            self, caption="Open WKT file", dir=str(base_path)
        )
        if not file:
            return
        file = Path(file)
        self.settings.setValue("files/last_wkt_location", str(file.parent))
        try:
            polygon = shapely.from_wkt(Path(file).read_text(encoding="UTF-8"))
            navi = jps.RoutingEngine(polygon)
            xmin, ymin, xmax, ymax = polygon.bounds
            info_text = f"Dimensions: {math.ceil(xmax - xmin)}m x {math.ceil(ymax - ymin)}m Polygons: {len(navi.mesh()[1])}"
            name_text = f"Geometry: {file}"
            self.setUpdatesEnabled(False)
            geo = Geometry(navi)
            geo.show_triangulation(self._show_triangulation.isChecked())
            tab = ViewGeometryWidget(
                navi, geo, name_text, info_text, parent=self
            )
            tab.render_widget.show_grid(self._show_grid.isChecked())
            tab_idx = self.tabs.insertTab(0, tab, file.name)
            self.tabs.setCurrentIndex(tab_idx)
            self.setUpdatesEnabled(True)
        except Exception as e:
            QMessageBox.critical(
                self,
                "Error importing WKT geometry",
                f"Error importing WKT geometry:\n{e}",
            )
            return

    def _open_wkt_quick3d(self):
        base_path_obj = self.settings.value(
            "files/last_wkt_location",
            type=str,
            defaultValue=Path("~").expanduser(),
        )
        base_path = Path(str(base_path_obj))
        file, _ = QFileDialog.getOpenFileName(
            self, caption="Open WKT file (Quick 3D)", dir=str(base_path)
        )
        if not file:
            return
        file = Path(file)
        self.settings.setValue("files/last_wkt_location", str(file.parent))
        try:
            polygon = shapely.from_wkt(file.read_text(encoding="UTF-8"))
            navi = jps.RoutingEngine(polygon)
            # Capture geometry/maximized state so Qt's layout system doesn't
            # auto-shrink the main window when the new tab's sizeHint kicks in.
            saved_geometry = self.saveGeometry()
            was_maximized = self.isMaximized()
            tab = Quick3DGeometryView(polygon, navi=navi, parent=self)
            tab.show_mesh(self._show_triangulation.isChecked())
            tab.show_grid(self._show_grid.isChecked())
            tab_idx = self.tabs.insertTab(0, tab, f"[Q3D] {file.name}")
            self.tabs.setCurrentIndex(tab_idx)
            self.restoreGeometry(saved_geometry)
            if was_maximized:
                self.showMaximized()
        except Exception as e:
            QMessageBox.critical(
                self,
                "Error importing WKT geometry",
                f"Error importing WKT geometry:\n{e}",
            )
            return

    def _open_replay_quick3d(self):
        base_path_obj = self.settings.value(
            "files/last_replay_location",
            type=str,
            defaultValue=Path("~").expanduser(),
        )
        base_path = Path(str(base_path_obj))
        file, _ = QFileDialog.getOpenFileName(
            self, caption="Open recording (Quick 3D)", dir=str(base_path)
        )
        if not file:
            return
        file = Path(file)
        self.settings.setValue("files/last_replay_location", str(file.parent))
        try:
            rec = Recording(file.as_posix())
            navi = jps.RoutingEngine(rec.geometry())
            # Preserve window geometry the same way _open_wkt_quick3d does, so
            # the new tab's sizeHint doesn't shrink the main window.
            saved_geometry = self.saveGeometry()
            was_maximized = self.isMaximized()
            tab = Quick3DReplayWidget(navi, rec, parent=self)
            tab.show_mesh(self._show_triangulation.isChecked())
            tab.show_grid(self._show_grid.isChecked())
            tab_idx = self.tabs.insertTab(0, tab, f"[Q3D] {file.name}")
            self.tabs.setCurrentIndex(tab_idx)
            self.restoreGeometry(saved_geometry)
            if was_maximized:
                self.showMaximized()
        except Exception as e:
            import traceback

            traceback.print_exception(e)
            QMessageBox.critical(
                self,
                "Error importing simulation recording",
                f"Error importing simulation recording:\n{e}",
            )
            return

    def _open_replay(self):
        base_path_obj = self.settings.value(
            "files/last_replay_location",
            type=str,
            defaultValue=Path("~").expanduser(),
        )
        base_path = Path(str(base_path_obj))
        file, _ = QFileDialog.getOpenFileName(
            self, caption="Open recording", dir=str(base_path)
        )
        if not file:
            return
        file = Path(file)
        self.settings.setValue("files/last_replay_location", str(file.parent))
        try:
            rec = Recording(file.as_posix())
            self.setUpdatesEnabled(False)
            navi = jps.RoutingEngine(rec.geometry())
            geo = Geometry(navi)
            geo.show_triangulation(self._show_triangulation.isChecked())
            trajectory = Trajectory(rec)
            tab = ReplayWidget(navi, rec, geo, trajectory, parent=self)
            tab.render_widget.show_grid(self._show_grid.isChecked())
            tab_idx = self.tabs.insertTab(0, tab, file.name)
            self.tabs.setCurrentIndex(tab_idx)
            self.setUpdatesEnabled(True)
            self.update()
        except Exception as e:
            import traceback

            traceback.print_exception(e)
            QMessageBox.critical(
                self,
                "Error importing simulation recording",
                f"Error importing simulation recording:\n{e}",
            )
            return
