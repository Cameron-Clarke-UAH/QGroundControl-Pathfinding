/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/
#ifndef PATHFINDING_H
#define PATHFINDING_H

#pragma once

#include "TransectStyleComplexItem.h"
#include "MissionItem.h"
#include "SettingsFact.h"
#include "QGCLoggingCategory.h"
#include "QGCMapPolyline.h"
#include "QGCMapPolygon.h"



class PathfindingComplexItem : public TransectStyleComplexItem
{
    Q_OBJECT

public:
    /// @param flyView true: Created for use in the Fly View, false: Created for use in the Plan View
    /// @param kmlFile Polyline comes from this file, empty for default polyline
    PathfindingComplexItem(PlanMasterController* masterController, bool flyView, const QString& kmlFile);

    Q_PROPERTY(QGCMapPolyline*  corridorPolyline    READ corridorPolyline   CONSTANT)
    Q_PROPERTY(Fact*            avoidanceRadius     READ avoidanceRadius      CONSTANT)
    Q_PROPERTY(Fact*            startAltitude       READ startAltitude        CONSTANT)
    Q_PROPERTY(Fact*            endAltitude         READ endAltitude        CONSTANT)
    Q_PROPERTY(Fact*            gridSize            READ gridSize             CONSTANT)
    Q_PROPERTY(Fact*            relativeToSurface   READ relativeToSurface             CONSTANT)

    Fact*           avoidanceRadius     (void) { return &_avoidanceRadiusFact; }
    Fact*           startAltitude       (void) { return &_startAltitudeFact; }
    Fact*           endAltitude         (void) { return &_endAltitudeFact; }
    Fact*           gridSize            (void) { return &_gridSizeFact; }
    Fact*           relativeToSurface   (void) { return &_relativeToSurfaceFact; }
    QGCMapPolyline* corridorPolyline    (void) { return &_corridorPolyline; }

    Q_INVOKABLE void rotateEntryPoint(void);
    Q_INVOKABLE void loadObstacleFile(const QString& filename);
    // Overrides from TransectStyleComplexItem
    QString patternName         (void) const final { return name; }
    void    save                (QJsonArray&  planItems) final;
    bool    specifiesCoordinate (void) const final;

    // Overrides from ComplexMissionItem
    bool    load                (const QJsonObject& complexObject, int sequenceNumber, QString& errorString) final;
    QString mapVisualQML        (void) const final { return QStringLiteral("CorridorScanMapVisual.qml"); }
    QString presetsSettingsGroup(void) { return settingsGroup; }
    void    savePreset          (const QString& name);
    void    loadPreset          (const QString& name);

    // Overrides from VisualMissionionItem
    QString             commandDescription  (void) const final { return tr("Pathing Item"); }
    QString             commandName         (void) const final { return tr("Pathing Item"); }
    QString             abbreviation        (void) const final { return tr("C"); }
    ReadyForSaveState   readyForSaveState   (void) const final;
    double              additionalTimeDelay (void) const final { return 0; }

    static const QString name;

    static const char* jsonComplexItemTypeValue;

    static const char* settingsGroup;
    static const char* avoidanceRadiusName;
    static const char* startAltitudeName;
    static const char* endAltitudeName;
    static const char* gridSizeName;
    static const char* relativeToSurfaceName;

private slots:
    void _polylineDirtyChanged          (bool dirty);
    void _updateWizardMode              (void);
    void _rebuildCorridor        (void);
    // Overrides from TransectStyleComplexItem
    void _rebuildTransectsPhase1    (void) final;
    void _recalcCameraShots         (void) final;

private:
    void    _saveCommon             (QJsonObject& complexObject);
    bool    _loadWorker              (const QJsonObject& complexObject, int sequenceNumber, QString& errorString, bool forPresets);
    QGCMapPolyline                  _corridorPolyline;
    QGCMapPolyline                  _transectPolyline;
    QList<QList<QGeoCoordinate>>    _transectSegments;      ///< Internal transect segments including grid exit, turnaround and internal camera points

    int                             _entryPoint;
    void pathToWaypoint(QGeoCoordinate startLoc,QGeoCoordinate endLoc);
    QMap<QString, FactMetaData*>    _metaDataMap;
    SettingsFact                    _avoidanceRadiusFact;
    SettingsFact                    _startAltitudeFact;
    SettingsFact                    _endAltitudeFact;
    SettingsFact                    _gridSizeFact;
    SettingsFact                    _relativeToSurfaceFact;
    static const char* _jsonEntryPointKey;
};
#endif // PATHFINDING_H
