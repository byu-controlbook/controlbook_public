// Copyright (C) 2019 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR GPL-3.0-only

import QtQuick
import QtQuick3D

Rectangle {
    property var source: null
    width: layout.width + 10
    height: layout.height + 10
    color: "#80778BA5"
    radius: 5

    Column {
        id: layout
        anchors.centerIn: parent

        Text {
            text: source.renderStats.fps + " FPS (" + (source.renderStats.frameTime).toFixed(3) + "ms)"
            font.pointSize: 13
            color: "white"
        }
        Text {
            text: "Sync: " + (source.renderStats.syncTime).toFixed(3) + "ms"
            font.pointSize: 9
            color: "white"
        }
        Text {
            text: "Render: " + (source.renderStats.renderTime).toFixed(3) + "ms (prep: " + (source.renderStats.renderPrepareTime).toFixed(3) + "ms)"
            font.pointSize: 9
            color: "white"
        }
        Text {
            text: "Max: " + (source.renderStats.maxFrameTime).toFixed(3) + "ms"
            font.pointSize: 9
            color: "white"
        }
    }
}
