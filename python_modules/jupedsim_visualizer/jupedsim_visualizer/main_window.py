# SPDX-License-Identifier: LGPL-3.0-or-later
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

from jupedsim_visualizer.quick3d.geometry_view import Quick3DGeometryView
from jupedsim_visualizer.quick3d.replay_view import Quick3DReplayWidget


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
        open_replay_act = open_menu.addAction("Open replay file")
        open_replay_act.triggered.connect(self._open_replay)
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

        sm.setInitialState(start)
        sm.start()
        self.state_machine = sm

    def _build_start_state(self) -> QState:
        state = QState()
        return state

    def _build_exit_state(self) -> QFinalState:
        state = QFinalState()
        return state

    def _toggle_triangulation(self, state: bool) -> None:
        self.settings.setValue("show_triangulation", state)
        for idx in range(self.tabs.count()):
            tab = self.tabs.widget(idx)
            if hasattr(tab, "show_mesh"):
                tab.show_mesh(state)
        self.repaint()

    def _toggle_grid(self, state: bool) -> None:
        self.settings.setValue("show_grid", state)
        for idx in range(self.tabs.count()):
            tab = self.tabs.widget(idx)
            if hasattr(tab, "show_grid"):
                tab.show_grid(state)
        self.repaint()

    def _insert_tab(self, tab, title: str) -> None:
        """Insert a new viewer tab, preserving the main window geometry.

        Capturing/restoring geometry (and the maximized flag) stops Qt's
        layout system from auto-shrinking the window to the new tab's
        sizeHint when it is inserted."""
        saved_geometry = self.saveGeometry()
        was_maximized = self.isMaximized()
        tab_idx = self.tabs.insertTab(0, tab, title)
        self.tabs.setCurrentIndex(tab_idx)
        self.restoreGeometry(saved_geometry)
        if was_maximized:
            self.showMaximized()

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
            polygon = shapely.from_wkt(file.read_text(encoding="UTF-8"))
            navi = jps.RoutingEngine(polygon)
            tab = Quick3DGeometryView(polygon, navi=navi, parent=self)
            tab.show_mesh(self._show_triangulation.isChecked())
            tab.show_grid(self._show_grid.isChecked())
            self._insert_tab(tab, file.name)
        except Exception as e:
            QMessageBox.critical(
                self,
                "Error importing WKT geometry",
                f"Error importing WKT geometry:\n{e}",
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
            navi = jps.RoutingEngine(rec.geometry())
            tab = Quick3DReplayWidget(navi, rec, parent=self)
            tab.show_mesh(self._show_triangulation.isChecked())
            tab.show_grid(self._show_grid.isChecked())
            self._insert_tab(tab, file.name)
        except Exception as e:
            import traceback

            traceback.print_exception(e)
            QMessageBox.critical(
                self,
                "Error importing simulation recording",
                f"Error importing simulation recording:\n{e}",
            )
            return
