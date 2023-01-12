// Copyright (C) 2021 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR GPL-3.0-only

import QtQuick 2.15
import QtQuick.Layouts 1.15
import HelperWidgets 2.0
import StudioTheme 1.0 as StudioTheme

Section {
    caption: qsTr("Model")

    SectionLayout {
        id: tessellationSection

        PropertyLabel {
            text: qsTr("Source")
            tooltip: qsTr("Sets the location of the mesh file containing the geometry of this model.")
        }

        SecondColumnLayout {
            UrlChooser {
                id: sourceUrlChooser
                backendValue: backendValues.source
                filter: "*.mesh"
                defaultItems: ["#Rectangle" ,"#Sphere" ,"#Cube" ,"#Cone" ,"#Cylinder"]
            }

            ExpandingSpacer {}
        }

        function hasTessellationMode(mode) {
            if (tessellationModeComboBox.backendValue.valueToString !== "" &&
                tessellationModeComboBox.backendValue.valueToString !== mode)
                return false

            if (tessellationModeComboBox.backendValue.enumeration !== "" &&
                tessellationModeComboBox.backendValue.enumeration !== mode)
                return false

            return true
        }

        PropertyLabel {
            text: qsTr("Casts Reflections")
            tooltip: qsTr("Enables rendering the model by reflection probes and making it visible in the reflections.")
        }

        SecondColumnLayout {
            CheckBox {
                text: backendValues.castsReflections.valueToString
                backendValue: backendValues.castsReflections
                implicitWidth: StudioTheme.Values.twoControlColumnWidth
                               + StudioTheme.Values.actionIndicatorWidth
            }

            ExpandingSpacer {}
        }

        PropertyLabel {
            text: qsTr("Casts Shadows")
            tooltip: qsTr("Enables the geometry of this model to be rendered to the shadow maps.")
        }

        SecondColumnLayout {
            CheckBox {
                text: backendValues.castsShadows.valueToString
                backendValue: backendValues.castsShadows
                implicitWidth: StudioTheme.Values.twoControlColumnWidth
                               + StudioTheme.Values.actionIndicatorWidth
            }

            ExpandingSpacer {}
        }

        PropertyLabel {
            text: qsTr("Receives Shadows")
            tooltip: qsTr("Enables the geometry of this model to receive shadows.")
        }

        SecondColumnLayout {
            CheckBox {
                text: backendValues.receivesShadows.valueToString
                backendValue: backendValues.receivesShadows
                implicitWidth: StudioTheme.Values.twoControlColumnWidth
                               + StudioTheme.Values.actionIndicatorWidth
            }

            ExpandingSpacer {}
        }

        PropertyLabel {
            text: qsTr("Receives Reflections")
            tooltip: qsTr("Enables the geometry of this model to receive reflections from the nearest reflection probe. The model must be inside at least one reflection probe to start receiving reflections.")
        }

        SecondColumnLayout {
            CheckBox {
                text: backendValues.receivesReflections.valueToString
                backendValue: backendValues.receivesReflections
                implicitWidth: StudioTheme.Values.twoControlColumnWidth
                               + StudioTheme.Values.actionIndicatorWidth
            }

            ExpandingSpacer {}
        }

        PropertyLabel {
            text: qsTr("Casts Reflections")
            tooltip: qsTr("Enables reflection probes to reflect this model.")
        }

        SecondColumnLayout {
            CheckBox {
                text: backendValues.castsReflections.valueToString
                backendValue: backendValues.castsReflections
                implicitWidth: StudioTheme.Values.twoControlColumnWidth
                               + StudioTheme.Values.actionIndicatorWidth
            }

            ExpandingSpacer {}
        }

        PropertyLabel {
            text: qsTr("Pickable")
            tooltip: qsTr("Enables model picking.")
        }

        SecondColumnLayout {
            CheckBox {
                text: backendValues.pickable.valueToString
                backendValue: backendValues.pickable
                implicitWidth: StudioTheme.Values.twoControlColumnWidth
                               + StudioTheme.Values.actionIndicatorWidth
            }

            ExpandingSpacer {}
        }

        PropertyLabel {
            text: qsTr("Materials")
            Layout.alignment: Qt.AlignTop
            Layout.topMargin: 5
        }

        SecondColumnLayout {
            EditableListView {
                backendValue: backendValues.materials
                model: backendValues.materials.expressionAsList
                Layout.fillWidth: true

                onAdd: function(value) { backendValues.materials.idListAdd(value) }
                onRemove: function(idx) { backendValues.materials.idListRemove(idx) }
                onReplace: function (idx, value) { backendValues.materials.idListReplace(idx, value) }
            }

            ExpandingSpacer {}
        }

        PropertyLabel {
            text: qsTr("Geometry")
            tooltip: qsTr("Sets a custom geometry for the model")
        }

        SecondColumnLayout {
            IdComboBox {
                id: geometryComboBox
                typeFilter: "QtQuick3D.Geometry"
                backendValue: backendValues.geometry
                implicitWidth: StudioTheme.Values.singleControlColumnWidth
                               + StudioTheme.Values.actionIndicatorWidth

                Connections {
                    target: geometryComboBox.backendValue
                    function onExpressionChanged() {
                        if (geometryComboBox.backendValue.expression !== "" &&
                                sourceUrlChooser.backendValue.expression !== "")
                            sourceUrlChooser.backendValue.resetValue()
                    }
                }
            }

            ExpandingSpacer {}
        }

        PropertyLabel {
            text: qsTr("Instancing")
            tooltip: qsTr("If this property is set, the model will not be rendered normally. Instead, a number of instances of the model will be rendered, as defined by the instance table.")
        }

        SecondColumnLayout {
            IdComboBox {
                typeFilter: "QtQuick3D.Instancing"
                backendValue: backendValues.instancing
                implicitWidth: StudioTheme.Values.singleControlColumnWidth
                               + StudioTheme.Values.actionIndicatorWidth
            }

            ExpandingSpacer {}
        }

        PropertyLabel {
            text: qsTr("Instance Root")
            tooltip: qsTr("Sets the origin of the instance’s coordinate system.")
        }

        SecondColumnLayout {
            IdComboBox {
                typeFilter: "QtQuick3D.Node"
                backendValue: backendValues.instanceRoot
                implicitWidth: StudioTheme.Values.singleControlColumnWidth
                               + StudioTheme.Values.actionIndicatorWidth
            }

            ExpandingSpacer {}
        }

        PropertyLabel {
            text: qsTr("Skeleton")
            tooltip: qsTr("Sets the skeleton for the model.")
        }

        SecondColumnLayout {
            IdComboBox {
                typeFilter: "QtQuick3D.Skeleton"
                backendValue: backendValues.skeleton
                implicitWidth: StudioTheme.Values.singleControlColumnWidth
                               + StudioTheme.Values.actionIndicatorWidth
            }

            ExpandingSpacer {}
        }

        PropertyLabel {
            text: qsTr("Morph Targets")
            tooltip: qsTr("Sets a list of MorphTargets used to render the provided geometry.")
            Layout.alignment: Qt.AlignTop
            Layout.topMargin: 5
        }

        SecondColumnLayout {
            EditableListView {
                backendValue: backendValues.morphTargets
                model: backendValues.morphTargets.expressionAsList
                Layout.fillWidth: true
                typeFilter: "QtQuick3D.MorphTarget"

                onAdd: function(value) { backendValues.morphTargets.idListAdd(value) }
                onRemove: function(idx) { backendValues.morphTargets.idListRemove(idx) }
                onReplace: function (idx, value) { backendValues.morphTargets.idListReplace(idx, value) }
            }

            ExpandingSpacer {}
        }

        PropertyLabel {
            text: qsTr("Depth Bias")
            tooltip: qsTr("Sets the depth bias of the model.")
        }

        SecondColumnLayout {
            SpinBox {
                minimumValue: -9999999
                maximumValue: 9999999
                decimals: 0
                backendValue: backendValues.depthBias
                implicitWidth: StudioTheme.Values.twoControlColumnWidth
                               + StudioTheme.Values.actionIndicatorWidth
            }

            ExpandingSpacer {}
        }
    }
}
