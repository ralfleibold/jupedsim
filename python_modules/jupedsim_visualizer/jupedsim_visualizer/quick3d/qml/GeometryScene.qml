// SPDX-License-Identifier: LGPL-3.0-or-later
import QtQuick
import QtQuick3D

Item {
    id: root
    anchors.fill: parent
    focus: true

    // ---- signals (Python is authoritative for camera/route state) ----
    signal hovered(real px, real py)
    signal exited()
    signal routeStart(real px, real py)
    signal routeMove(real px, real py)
    signal routeEnd()
    signal panStart(real px, real py)
    signal panMove(real px, real py)
    signal panEnd()
    signal wheelZoom(real px, real py, real deltaY)
    signal keyPressed(int key, string text)

    View3D {
        id: view
        anchors.fill: parent

        environment: SceneEnvironment {
            // Match the legacy VTK palette (jupedsim_visualizer.config.Colors).
            clearColor: "#74a892"   // teal background (Colors.d)
            backgroundMode: SceneEnvironment.Color
            antialiasingMode: SceneEnvironment.MSAA
        }

        OrthographicCamera {
            id: cam
            // Pan moves the camera laterally; mag changes its frustum scale.
            position: Qt.vector3d(panX, panY, 100)
            horizontalMagnification: viewMag
            verticalMagnification: viewMag
            clipNear: 0.1
            clipFar: 10000
        }

        Model {
            geometry: wallGeometry
            materials: DefaultMaterial {
                diffuseColor: "#fbf2c4"   // cream fill (Colors.c)
                lighting: DefaultMaterial.NoLighting
                cullMode: Material.NoCulling
            }
        }

        // Optional grid overlay (Lines, z=0.075 baked in).
        Model {
            geometry: gridGeometry
            visible: showGrid && gridGeometry !== null
            materials: DefaultMaterial {
                diffuseColor: "#008585"   // dark teal (Colors.e)
                lighting: DefaultMaterial.NoLighting
            }
        }

        // Nav-mesh overlay (Lines, z=0.05 baked in).
        Model {
            geometry: meshGeometry
            visible: showMesh && meshGeometry !== null
            materials: DefaultMaterial {
                diffuseColor: "#c7522a"   // orange-red edges (Colors.a)
                lighting: DefaultMaterial.NoLighting
            }
        }

        // Routed path overlay (thick triangle strip, z=0.1 baked in).
        Model {
            geometry: pathGeometry
            visible: pathGeometry !== null
            materials: DefaultMaterial {
                diffuseColor: "#ff0000"   // red path (legacy used pure red)
                lighting: DefaultMaterial.NoLighting
                cullMode: Material.NoCulling
            }
        }
    }

    MouseArea {
        anchors.fill: parent
        hoverEnabled: true
        acceptedButtons: Qt.LeftButton | Qt.MiddleButton

        onPositionChanged: (m) => {
            if (m.buttons & Qt.LeftButton) {
                root.routeMove(m.x, m.y)
            } else if (m.buttons & Qt.MiddleButton) {
                root.panMove(m.x, m.y)
            } else {
                root.hovered(m.x, m.y)
            }
        }
        onPressed: (m) => {
            // The Item below this MouseArea won't get focus naturally; force it.
            root.forceActiveFocus()
            if (m.button === Qt.LeftButton) {
                root.routeStart(m.x, m.y)
            } else if (m.button === Qt.MiddleButton) {
                root.panStart(m.x, m.y)
            }
        }
        onReleased: (m) => {
            if (m.button === Qt.LeftButton) {
                root.routeEnd()
            } else if (m.button === Qt.MiddleButton) {
                root.panEnd()
            }
        }
        onExited: root.exited()
        onWheel: (w) => root.wheelZoom(w.x, w.y, w.angleDelta.y)
    }

    Keys.onPressed: (e) => root.keyPressed(e.key, e.text)
}
