
import QtQuick          2.3
import QtQuick.Controls 1.2
import QtQuick.Controls.Styles 1.4
import QtQuick.Dialogs  1.2
import QtQuick.Extras   1.4
import QtQuick.Layouts  1.2

import QGroundControl               1.0
import QGroundControl.ScreenTools   1.0
import QGroundControl.Vehicle       1.0
import QGroundControl.Controls      1.0
import QGroundControl.FactControls  1.0
import QGroundControl.Palette       1.0
import QGroundControl.FlightMap     1.0

TransectStyleComplexItemEditor {
    transectAreaDefinitionComplete: _missionItem.corridorPolyline.isValid
    transectAreaDefinitionHelp:     qsTr("Use the Polyline Tools to create the polyline which defines the start and end goals.")
    transectValuesHeaderName:       qsTr("Pathing Settings")
    transectValuesComponent:        _transectValuesComponent
    presetsTransectValuesComponent: _transectValuesComponent

    // The following properties must be available up the hierarchy chain
    //  property real   availableWidth    ///< Width for control
    //  property var    missionItem       ///< Mission Item for editor

    property real   _margin:        ScreenTools.defaultFontPixelWidth / 2
    property var    _missionItem:   missionItem

    Component {
        id: _transectValuesComponent

        GridLayout {
            columnSpacing:  _margin
            rowSpacing:     _margin
            columns:        2

            QGCLabel { text: qsTr("Avoidance Radius") }
            FactTextField {
                fact:               _missionItem.avoidanceRadius
                Layout.fillWidth:   true
            }
            QGCLabel { text: qsTr("Start Altitude") }
            FactTextField {
                fact:               _missionItem.startAltitude
                Layout.fillWidth:   true
            }
            QGCLabel { text: qsTr("End Altitude") }
            FactTextField {
                fact:               _missionItem.endAltitude
                Layout.fillWidth:   true
            }
            QGCLabel { text: qsTr("Grid Size") }
            FactTextField {
                fact:               _missionItem.gridSize
                Layout.fillWidth:   true
            }
            FactCheckBox {
                Layout.columnSpan:  2
                text:               qsTr("Altitudes Relative to Surface")
                fact:               _missionItem.relativeToSurface
            }
        }
    }
}
