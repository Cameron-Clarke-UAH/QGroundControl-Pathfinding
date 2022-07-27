#ifndef PATHFINDING_H
#define PATHFINDING_H

#include "QGCToolbox.h"
#include "SimpleMissionItem.h"
#include "qgeocoordinate.h"

class QGCApplication;

class PathfindingPlugin : public QGCTool
{
    Q_OBJECT
public:
    explicit PathfindingPlugin(QGCApplication* app = nullptr, QGCToolbox* toolbox = nullptr);

    Q_INVOKABLE void qmlDebug(QString msg);
    Q_INVOKABLE QGeoCoordinate getCoord(int index);
    Q_INVOKABLE int listSize();
    Q_INVOKABLE void pathToWaypoint(SimpleMissionItem *visualItem,double endLong, double endLat,double endAlt);
signals:

};

#endif // PATHFINDING_H
