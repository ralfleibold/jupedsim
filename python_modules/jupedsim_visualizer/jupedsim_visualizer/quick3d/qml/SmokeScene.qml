// SPDX-License-Identifier: LGPL-3.0-or-later
import QtQuick
import QtQuick3D

View3D {
    id: view
    anchors.fill: parent

    environment: SceneEnvironment {
        clearColor: "#1e1e1e"
        backgroundMode: SceneEnvironment.Color
        antialiasingMode: SceneEnvironment.MSAA
    }

    // Top-down orthographic camera. When 3D floors land, swap this for
    // PerspectiveCamera and bump z.
    OrthographicCamera {
        id: cam
        position: Qt.vector3d(centerX, centerY, 100)
        eulerRotation: Qt.vector3d(0, 0, 0)
        // horizontalMagnification / verticalMagnification scale world -> view.
        horizontalMagnification: viewMag
        verticalMagnification: viewMag
        clipNear: 0.1
        clipFar: 10000
    }

    // The polygon mesh, provided from Python as a context property `wallGeometry`.
    // Debug sanity primitive — known-good built-in sphere at the scene center.
    // If you see this but not the polygon, framing/geometry is wrong.
    // If you see nothing, the renderer/backend is wrong.
    Model {
        source: "#Sphere"
        position: Qt.vector3d(centerX, centerY, 0)
        scale: Qt.vector3d(probeScale, probeScale, probeScale)
        materials: DefaultMaterial {
            diffuseColor: "#ff3030"
            lighting: DefaultMaterial.NoLighting
        }
    }

    Model {
        geometry: wallGeometry
        materials: DefaultMaterial {
            diffuseColor: "#d4c89a"
            lighting: DefaultMaterial.NoLighting
            // Triangle winding from shapely's Delaunay isn't guaranteed CCW,
            // so don't cull back faces — we're drawing a flat slab from above.
            cullMode: Material.NoCulling
        }
    }
}
