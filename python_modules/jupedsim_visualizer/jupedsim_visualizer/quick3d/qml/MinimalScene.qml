// SPDX-License-Identifier: LGPL-3.0-or-later
// Absolutely minimal Quick 3D scene. No external properties. One sphere at
// origin, default-ish camera. If THIS doesn't render, Quick 3D itself is
// not actually drawing into the window.
import QtQuick
import QtQuick3D

View3D {
    id: view
    anchors.fill: parent

    environment: SceneEnvironment {
        clearColor: "#003366"   // distinctive blue — easy to tell apart from desktop bleed-through
        backgroundMode: SceneEnvironment.Color
    }

    PerspectiveCamera {
        id: cam
        position: Qt.vector3d(0, 0, 300)
    }

    Model {
        source: "#Sphere"
        materials: DefaultMaterial {
            diffuseColor: "#ff8800"
            lighting: DefaultMaterial.NoLighting
        }
    }
}
