/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "PathingComplexItem.h"
#include "JsonHelper.h"
#include "MissionController.h"
#include "QGCGeo.h"
#include "QGCQGeoCoordinate.h"
#include "SettingsManager.h"
#include "AppSettings.h"
#include "QGCQGeoCoordinate.h"
#include "PlanMasterController.h"
#include "QGCApplication.h"
#include "UTM.h"
#include <QPolygonF>
#include "geometry.h"
#include <iostream>
#include <cstdio>
#include <random>
#include <fstream>
#include <chrono>
#include <string>
#include <limits>
#include <QQmlEngine>


QList<QGeoCoordinate> Path;


using namespace std;
using namespace std::chrono;
vector < Point > Cloud;
KDNode* CloudTree;
double stepResolution = 8;
Point MinPoint;
Point MaxPoint;
Point originOffset = Point(0.0,0,0);
using namespace std::placeholders;
double CollisionRadius = 2;
const double INF = 1e18;
bool North = true;
double UTMZone = 16;

bool AStar::Vec3i::operator == (const Vec3i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y && z == coordinates_.z);
}

AStar::Vec3i operator + (const AStar::Vec3i& left_, const AStar::Vec3i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y , left_.z + right_.z };
}

Point CoordToPoint(AStar::Vec3i a) {
    return Point(a.x * stepResolution, a.y * stepResolution, a.z * stepResolution) + originOffset;
}

AStar::Vec3i CoordFromPoint(Point a) {
    Point Scaled = (a - originOffset)/stepResolution;
    return {int(round(Scaled.x)),int(round(Scaled.y)),int(round(Scaled.z))};
}

AStar::Node::Node(Vec3i coordinates_, Node* parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

AStar::uint AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setHeuristic(&Heuristic::euclideanApproximation);
    direction = {
        {1,0,0},
        {-1,0,0},
        {0,1,0},
        {0,-1,0},
        {0,0,-1},
        {0,0,1},

        {-1,-1,0},  // Diagonals
        {-1,0,-1},
        {-1,0,1},
        {-1,1,0},
        {0,-1,-1},
        {0,-1,1},
        {0,1,-1},
        {0,1,1},
        {1,-1,0},
        {1,0,-1},
        {1,0,1},
        {1,1,0},

        {-1,-1,1}, // Double Diagonals
        {-1,1,-1},
        {-1,1,1},
        {-1,-1,-1},
        {1,-1,-1},
        {1,-1,1},
        {1,1,-1},
        {1,1,1},
    };
}

void AStar::Generator::setOrigin(Point Origin)
{
    originOffset = Origin;
}


void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}


AStar::CoordinateList AStar::Generator::findPath(Point start, Point end)
{
    Vec3i source_ = CoordFromPoint(start);
    Vec3i target_ = CoordFromPoint(end);

    //Check if target is reachable
    bool Reachable = false;
    for(Vec3i Shift : direction){

        if (!detectCollision(target_,target_+Shift)){
            Reachable = true;
        }

    }
    CoordinateList path;
    if(!Reachable) {return path;}

    Node* current = nullptr;
    NodeSet openSet, closedSet;
    openSet.reserve(100);
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));
    while (!openSet.empty()) {

        auto current_it = openSet.begin();
        current = *current_it;

        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            auto node = *it;
            if (node->getScore() <= current->getScore()) {
                current = node;
                current_it = it;
            }
        }

        if (current->coordinates == target_) {
            break;
        }
        closedSet.push_back(current);
        openSet.erase(current_it);

        for (uint i = 0; i < directions; ++i) {

            Vec3i newCoordinates(current->coordinates + direction[i]);

            /*printPoint(CoordToPoint(current->coordinates));
            printPoint(CoordToPoint(newCoordinates));
            cout << "Pass? " << detectCollision(current->coordinates, newCoordinates) << "\n";
            */
            if (detectCollision(current->coordinates,newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }

            uint totalCost = current->G + ((i < 6) ? 10 : (i < 18) ? 14 : 17); // Attributes a distance cost to each of the traveled distances

            Node* successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.push_back(successor);
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }


    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    bool Changed = 1;
    while(Changed)
    {
        Changed = false;
        for(int i=0; i < size(path)-2; i++){
            if(!detectCollision(path[i],path[i+2]))
            {
                Changed = true;
                path.erase(path.begin()+(i+1));
                break;
            }
        }
    }


    qDebug() << "Path reduction complete\n";

    //Check for path reduction errors

    for(int j=0;j<size(path)-1;j++)
    {
        if(detectCollision(path[j],path[j+1]))
        {
            qDebug()<<"COLLISION DETECTED AFTER PATH REDUCTION";
        }
    }
    return path;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec3i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete* it;
        it = nodes_.erase(it);
    }
}

void RangeSearchRec(Point& LocA, Point& LocB, KDNode* root, vector <Point>& pointlist, vector<int>& tmpVec, int depth) {
    if (root == NULL) return;
    unsigned cd = depth % 3;
    Point LocalPos = pointlist[root->index];
    //Check if current point is in range on axis

    bool Left = LocB[cd] >= LocalPos[cd];
    bool Right = LocA[cd] <= LocalPos[cd];
    if (Left && Right) {
        // Check if current node is completly in range
        if (LocA < LocalPos && LocB> LocalPos) {
            tmpVec.push_back(root->index);
        }
        // Search left && Right
        RangeSearchRec(LocA, LocB, root->left, pointlist, tmpVec, depth + 1);
        RangeSearchRec(LocA, LocB, root->right, pointlist, tmpVec, depth + 1);
    }
    //Check if current point is left of range
    else if (Left) {
        RangeSearchRec(LocA, LocB, root->right, pointlist, tmpVec, depth + 1);
    }
    else {
        RangeSearchRec(LocA, LocB, root->left, pointlist, tmpVec, depth + 1);
    }

}

// Finds all points in XYZ rectangular range defined by two points
void RangeSearch(Point& LocA, Point& LocB, KDNode* root, vector <Point>& pointlist, vector<int>& Box) {

    RangeSearchRec(LocA, LocB, root, pointlist, Box, 0);

}



bool isEdgeObstacleFree(Point a, Point b) {
    //return true;
    vector<int> Contained;
    bool MinXPoint = a.x < b.x;
    bool MinYPoint = a.y < b.y;
    bool MinZPoint = a.z < b.z;


    Point MinPoint = Point((MinXPoint ? a.x : b.x) - CollisionRadius, (MinYPoint ? a.y : b.y) - CollisionRadius, (MinZPoint ? a.z : b.z) - CollisionRadius);
    Point MaxPoint = Point((MinXPoint ? b.x : a.x) + CollisionRadius, (MinYPoint ? b.y : a.y) + CollisionRadius, INF);
    RangeSearch(MinPoint, MaxPoint, CloudTree, Cloud, Contained);
    for (int i = 0; i < size(Contained); i++) {
        if (distanceBetweenSegments(a, b, Point(0, 0, -1000) + Cloud[Contained[i]], Cloud[Contained[i]], true, true, true, true) < CollisionRadius) {
            return false;
        }
    }
    return true;

}

double getLocationAltitude(Point Location)
{
    double Alt = 0;
    vector<int> Contained;
    double MinXYDistance = INF;

    // Creates a search region within 0.6 meters of the XY location
    Point MinPoint = Point(Location.x-0.3,Location.y-0.3,-INF);
    Point MaxPoint = Point(Location.x+0.3,Location.y+0.3,INF);

    // Gets points near the XY coordinates of the specified location
    RangeSearch(MinPoint, MaxPoint, CloudTree, Cloud, Contained);

    // Finds the closest of the returned points
    for (int i = 0; i < size(Contained); i++) {
        double dis = sqrt((Cloud[Contained[i]].x-Location.x)*(Cloud[Contained[i]].x-Location.x)+(Cloud[Contained[i]].y-Location.y)*(Cloud[Contained[i]].y-Location.y));
        if (dis < MinXYDistance)
        {
           MinXYDistance = dis;
           Alt =  Cloud[Contained[i]].z;
        }
    }
    return Alt;
}

bool AStar::Generator::detectCollision(Vec3i prevCoord,Vec3i coordinates_)
{
    if (!isEdgeObstacleFree(CoordToPoint(prevCoord), CoordToPoint(coordinates_))) {
        return true;
    }
    return false;
}



AStar::Vec3i AStar::Heuristic::getDelta(Vec3i source_, Vec3i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y),  abs(source_.z - target_.z) };
}


AStar::uint AStar::Heuristic::manhattan(Vec3i source_, Vec3i target_)
{
    auto delta = getDelta(source_, target_);
    return static_cast<uint>(10 * (delta.x + delta.y + delta.z));
}

AStar::uint AStar::Heuristic::euclidean(Vec3i source_, Vec3i target_)
{
    auto delta = getDelta(source_, target_);
    return static_cast<uint>(10*sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z));
}

AStar::uint AStar::Heuristic::euclideanApproximation(Vec3i source_, Vec3i target_)
{
    auto delta = getDelta(source_, target_);
    int minimum = min({delta.x, delta.y, delta.z});
    int maximum = max({delta.x, delta.y, delta.z});

    int doubleAxis = max(delta.x + delta.y + delta.z - maximum - 2 * minimum, 0);

    int singleAxis = maximum - doubleAxis - minimum;

    return (10 * singleAxis + 14 * doubleAxis + 17 * minimum);
}

AStar::uint AStar::Heuristic::octagonal(Vec3i source_, Vec3i target_)
{
    auto delta = getDelta(source_, target_);
    return 10 * (delta.x + delta.y + delta.z) + (-6) * std::min(delta.x, delta.y);
}




QGC_LOGGING_CATEGORY(PathfindingComplexItemLog, "PathfindingComplexItemLog")

const QString PathfindingComplexItem::name(PathfindingComplexItem::tr("Pathfinding"));

const char* PathfindingComplexItem::settingsGroup =            "PathfindingComplex";
const char* PathfindingComplexItem::avoidanceRadiusName =      "AvoidanceRadius";
const char* PathfindingComplexItem::startAltitudeName =        "StartAltitude";
const char* PathfindingComplexItem::endAltitudeName =          "EndAltitude";
const char* PathfindingComplexItem::gridSizeName =             "GridSize";
const char* PathfindingComplexItem::relativeToSurfaceName=     "RelativeToSurface";
const char* PathfindingComplexItem::_jsonEntryPointKey =       "EntryPoint";

const char* PathfindingComplexItem::jsonComplexItemTypeValue = "PathfindingComplex";

PathfindingComplexItem::PathfindingComplexItem(PlanMasterController* masterController, bool flyView, const QString& kmlFile)
    : TransectStyleComplexItem  (masterController, flyView, settingsGroup)
    , _entryPoint               (0)
    , _metaDataMap              (FactMetaData::createMapFromJsonFile(QStringLiteral(":/json/Pathing.SettingsGroup.json"), this))
    , _avoidanceRadiusFact      (settingsGroup, _metaDataMap[avoidanceRadiusName])
    , _startAltitudeFact        (settingsGroup, _metaDataMap[startAltitudeName])
    , _endAltitudeFact          (settingsGroup, _metaDataMap[endAltitudeName])
    , _gridSizeFact             (settingsGroup, _metaDataMap[gridSizeName])
    , _relativeToSurfaceFact    (settingsGroup, _metaDataMap[relativeToSurfaceName])
{
    _editorQml = "qrc:/qml/PathingItemEditor.qml";

    // We override the altitude to the mission default
    if (_cameraCalc.isManualCamera() || !_cameraCalc.valueSetIsDistance()->rawValue().toBool()) {
        _cameraCalc.distanceToSurface()->setRawValue(qgcApp()->toolbox()->settingsManager()->appSettings()->defaultMissionItemAltitude()->rawValue());
    }

    connect(&_avoidanceRadiusFact,  &Fact::valueChanged,                            this, &PathfindingComplexItem::_setDirty);
    connect(&_startAltitudeFact,    &Fact::valueChanged,                            this, &PathfindingComplexItem::_setDirty);
    connect(&_endAltitudeFact,      &Fact::valueChanged,                            this, &PathfindingComplexItem::_setDirty);
    connect(&_gridSizeFact,         &Fact::valueChanged,                            this, &PathfindingComplexItem::_setDirty);
    connect(&_relativeToSurfaceFact,&Fact::valueChanged,                            this, &PathfindingComplexItem::_setDirty);
    connect(&_corridorPolyline,     &QGCMapPolyline::pathChanged,                   this, &PathfindingComplexItem::_setDirty);

    connect(&_corridorPolyline,     &QGCMapPolyline::dirtyChanged,                  this, &PathfindingComplexItem::_polylineDirtyChanged);
    connect(&_transectPolyline,     &QGCMapPolyline::pathChanged,                   this, &PathfindingComplexItem::_rebuildCorridor);

    connect(&_corridorPolyline,     &QGCMapPolyline::isValidChanged,                this, &PathfindingComplexItem::_updateWizardMode);
    connect(&_corridorPolyline,     &QGCMapPolyline::traceModeChanged,              this, &PathfindingComplexItem::_updateWizardMode);

    if (!kmlFile.isEmpty()) {
        _corridorPolyline.loadKMLFile(kmlFile);
        _corridorPolyline.setDirty(false);
    }
    setDirty(false);
}

void PathfindingComplexItem::save(QJsonArray&  planItems)
{
    QJsonObject saveObject;

    _saveCommon(saveObject);
    planItems.append(saveObject);
}

void PathfindingComplexItem::savePreset(const QString& name)
{
    QJsonObject saveObject;

    _saveCommon(saveObject);
    _savePresetJson(name, saveObject);
}

void PathfindingComplexItem::_saveCommon(QJsonObject& saveObject)
{
    TransectStyleComplexItem::_save(saveObject);

    saveObject[JsonHelper::jsonVersionKey] =                    2;
    saveObject[VisualMissionItem::jsonTypeKey] =                VisualMissionItem::jsonTypeComplexItemValue;
    saveObject[ComplexMissionItem::jsonComplexItemTypeKey] =    jsonComplexItemTypeValue;
    saveObject[avoidanceRadiusName] =                           _avoidanceRadiusFact.rawValue().toDouble();
    saveObject[startAltitudeName] =                             _startAltitudeFact.rawValue().toDouble();
    saveObject[endAltitudeName] =                               _endAltitudeFact.rawValue().toDouble();
    saveObject[gridSizeName] =                                  _gridSizeFact.rawValue().toDouble();
    saveObject[relativeToSurfaceName] =                                  _relativeToSurfaceFact.rawValue().toDouble();
    saveObject[_jsonEntryPointKey] =                            _entryPoint;

    _corridorPolyline.saveToJson(saveObject);
}

void PathfindingComplexItem::loadPreset(const QString& name)
{
    QString errorString;

    QJsonObject presetObject = _loadPresetJson(name);
    if (!_loadWorker(presetObject, 0, errorString, true /* forPresets */)) {
        qgcApp()->showAppMessage(QStringLiteral("Internal Error: Preset load failed. Name: %1 Error: %2").arg(name).arg(errorString));
    }
    _rebuildTransects();
}

bool PathfindingComplexItem::_loadWorker(const QJsonObject& complexObject, int sequenceNumber, QString& errorString, bool forPresets)
{
    _ignoreRecalc = !forPresets;

    QList<JsonHelper::KeyValidateInfo> keyInfoList = {
        { JsonHelper::jsonVersionKey,                   QJsonValue::Double, true },
        { VisualMissionItem::jsonTypeKey,               QJsonValue::String, true },
        { ComplexMissionItem::jsonComplexItemTypeKey,   QJsonValue::String, true },
        { avoidanceRadiusName,                          QJsonValue::Double, true },
        { startAltitudeName,                            QJsonValue::Double, true },
        { endAltitudeName,                              QJsonValue::Double, true },
        { gridSizeName,                                 QJsonValue::Double, true },
        { relativeToSurfaceName,                        QJsonValue::Double, true },
        { _jsonEntryPointKey,                           QJsonValue::Double, true },
        { QGCMapPolyline::jsonPolylineKey,              QJsonValue::Array,  true },
    };
    if (!JsonHelper::validateKeys(complexObject, keyInfoList, errorString)) {
        _ignoreRecalc = false;
        return false;
    }

    QString itemType = complexObject[VisualMissionItem::jsonTypeKey].toString();
    QString complexType = complexObject[ComplexMissionItem::jsonComplexItemTypeKey].toString();
    if (itemType != VisualMissionItem::jsonTypeComplexItemValue || complexType != jsonComplexItemTypeValue) {
        errorString = tr("%1 does not support loading this complex mission item type: %2:%3").arg(qgcApp()->applicationName()).arg(itemType).arg(complexType);
        _ignoreRecalc = false;
        return false;
    }

    int version = complexObject[JsonHelper::jsonVersionKey].toInt();
    if (version != 2) {
        errorString = tr("%1 complex item version %2 not supported").arg(jsonComplexItemTypeValue).arg(version);
        _ignoreRecalc = false;
        return false;
    }

    if (!forPresets) {
        if (!_corridorPolyline.loadFromJson(complexObject, true, errorString)) {
            _ignoreRecalc = false;
            return false;
        }
    }

    setSequenceNumber(sequenceNumber);

    if (!_load(complexObject, forPresets, errorString)) {
        _ignoreRecalc = false;
        return false;
    }


    _avoidanceRadiusFact.setRawValue(complexObject[avoidanceRadiusName].toDouble());

    _startAltitudeFact.setRawValue(complexObject[startAltitudeName].toDouble());

    _endAltitudeFact.setRawValue(complexObject[endAltitudeName].toDouble());

    _gridSizeFact.setRawValue(complexObject[gridSizeName].toDouble());

    _relativeToSurfaceFact.setRawValue(complexObject[relativeToSurfaceName].toDouble());

    _entryPoint = complexObject[_jsonEntryPointKey].toInt();

    _ignoreRecalc = false;

    _recalcComplexDistance();
    if (_cameraShots == 0) {
        // Shot count was possibly not available from plan file
        _recalcCameraShots();
    }

    return true;
}
void PathfindingComplexItem::loadObstacleFile(const QString& filename)
{
    double DistanceBetweenPoints;
    double OffsetX;
    double OffsetY;
    int XCount;
    ifstream ReadFile(filename.toStdString().c_str(), ios::in | ios::binary);
    ReadFile.read((char*)&UTMZone, sizeof(int));
    ReadFile.read((char*)&North, sizeof(bool));
    ReadFile.read((char*)&OffsetX, sizeof(double));
    ReadFile.read((char*)&OffsetY, sizeof(double));
    ReadFile.read((char*)&DistanceBetweenPoints, sizeof(double));
    ReadFile.read((char*)&XCount, sizeof(int));
    Cloud.clear();

    qDebug() << "UTMZone: " << UTMZone << "\n";
    qDebug() << "North: " << North << "\n";
    qDebug() << "MinPointx: " << OffsetX << "\n";
    qDebug() << "MinPointy: " << OffsetY << "\n";
    qDebug() << "Scale: " << DistanceBetweenPoints << "\n";
    qDebug() << "XCount: " << XCount << "\n";
    int Index = 0;
    while (ReadFile) {
        float Z;
        ReadFile.read((char*)&Z, sizeof(float));
        if (Z > -9000)
        {
            //qDebug()<<"X: " << OffsetX+DistanceBetweenPoints*(Index % XCount)<<"\tY: "<< OffsetY+DistanceBetweenPoints*int(Index / XCount) << "\tZ: " <<Z+74.92<<"\n";
            Cloud.push_back(Point(OffsetX+DistanceBetweenPoints*(Index % XCount), OffsetY+DistanceBetweenPoints*int(Index / XCount), Z+74.92));
        }
        Index++;
    }

    ReadFile.close();
    CloudTree = BuildOrderedKDTree(Cloud);
    qDebug()  << "KD Tree Built\n";


}

void PathfindingComplexItem::pathToWaypoint(QGeoCoordinate startLoc,QGeoCoordinate endLoc)
{
    Path.clear();

       AStar::Generator generator;
       if (size(Cloud))
       {
           MinPoint = Point(INF, INF, INF);
           for (int i = 0; i < size(Cloud); i++)
           {
               if (Cloud[i].x < MinPoint.x) { MinPoint.x = Cloud[i].x; }

               if (Cloud[i].y < MinPoint.y) { MinPoint.y = Cloud[i].y; }

               if (Cloud[i].z < MinPoint.z) { MinPoint.z = Cloud[i].z; }
           }

           generator.setOrigin(MinPoint);
       }else{generator.setOrigin({0,0,0});}


       stepResolution = _gridSizeFact.rawValue().toDouble();


       // You can use a few heuristics : manhattan, euclidean or octagonal.
       generator.setHeuristic(AStar::Heuristic::euclideanApproximation);
       CollisionRadius = _avoidanceRadiusFact.rawValue().toDouble();
       double X=0,Y=0;



       bool RelativeAlt = _relativeToSurfaceFact.rawValue().toBool();
       double StartAltInput = _startAltitudeFact.rawValue().toDouble();
       double EndAltInput = _endAltitudeFact.rawValue().toDouble();

       LatLonToUTMXY(startLoc.latitude(), startLoc.longitude(), UTMZone, X, Y);
       Point start = Point(X, Y, RelativeAlt ? getLocationAltitude(Point(X,Y,0)) + StartAltInput : StartAltInput );

       LatLonToUTMXY(endLoc.latitude(), endLoc.longitude(), UTMZone, X, Y);
       Point stop = Point(X, Y,  RelativeAlt ? getLocationAltitude(Point(X,Y,0)) + EndAltInput : EndAltInput);

       auto startT = high_resolution_clock::now();
       auto path = generator.findPath(start,stop);
       auto stopT = high_resolution_clock::now();


       auto duration = duration_cast<milliseconds>(stopT - startT);

       qDebug()  << "Time taken by function: "
           << duration.count() << " milliseconds\n";

       for (int i=size(path)-1;i>=0; i--) {
           Point RealCoord = CoordToPoint(path[i]);
           double lat,lon;
           UTMXYToLatLon(RealCoord.x,RealCoord.y,UTMZone,!North,lat,lon);
           Path.append(QGeoCoordinate(lat*180/PI,lon*180/PI,RealCoord.z));
       }


}

bool PathfindingComplexItem::load(const QJsonObject& complexObject, int sequenceNumber, QString& errorString)
{
    return _loadWorker(complexObject, sequenceNumber, errorString, false /* forPresets */);
}

bool PathfindingComplexItem::specifiesCoordinate(void) const
{
    return _corridorPolyline.count() > 1;
}


void PathfindingComplexItem::_polylineDirtyChanged(bool dirty)
{
    if (dirty) {
        setDirty(true);
    }
}

void PathfindingComplexItem::rotateEntryPoint(void)
{
    // If the transects are getting rebuilt then any previsouly loaded mission items are now invalid
    if (_loadedMissionItemsParent) {
        _loadedMissionItems.clear();
        _loadedMissionItemsParent->deleteLater();
        _loadedMissionItemsParent = nullptr;
    }


    if (_corridorPolyline.count() >= 2) {
        // Turn transect into CoordInfo transect
        QList<TransectStyleComplexItem::CoordInfo_t> transect;
        QList<QGeoCoordinate> pathCoordinates = _corridorPolyline.coordinateList();
        pathToWaypoint(pathCoordinates[0],pathCoordinates[std::size(pathCoordinates)-1]);
        _transectPolyline.clear();
        for (QGeoCoordinate Coordinate : Path)
        {
            _transectPolyline.appendVertex(Coordinate);
        }
    }

    _rebuildTransects();
    setDirty(false);
}


void PathfindingComplexItem::_rebuildTransectsPhase1(void)
{
    if (_ignoreRecalc) {
        return;
    }
    // If the transects are getting rebuilt then any previsouly loaded mission items are now invalid
    if (_loadedMissionItemsParent) {
        _loadedMissionItems.clear();
        _loadedMissionItemsParent->deleteLater();
        _loadedMissionItemsParent = nullptr;
    }


    if (_corridorPolyline.count() >= 2) {
        // Turn transect into CoordInfo transect
        QList<TransectStyleComplexItem::CoordInfo_t> transect;
        for (QGeoCoordinate Coordinate : Path)
        {
            TransectStyleComplexItem::CoordInfo_t coordInfo = { Coordinate, CoordTypePathing };
            transect.append(coordInfo);
        }

        _transects.append(transect);
    }

}

void PathfindingComplexItem::_rebuildCorridor(void)
{
    if (_transectPolyline.count() < 2) {
        _surveyAreaPolygon.clear();
        return;
    }

    QList<QGeoCoordinate> firstSideVertices = _transectPolyline.offsetPolyline(CollisionRadius);
    QList<QGeoCoordinate> secondSideVertices = _transectPolyline.offsetPolyline(-CollisionRadius);

    _surveyAreaPolygon.clear();

    QList<QGeoCoordinate> rgCoord;
    for (const QGeoCoordinate& vertex: firstSideVertices) {
        rgCoord.append(vertex);

    }
    for (int i=secondSideVertices.count() - 1; i >= 0; i--) {

        rgCoord.append(secondSideVertices[i]);
    }
    _surveyAreaPolygon.appendVertices(rgCoord);
}

void PathfindingComplexItem::_recalcCameraShots(void)
{
    emit cameraShotsChanged();
}

PathfindingComplexItem::ReadyForSaveState PathfindingComplexItem::readyForSaveState(void) const
{
    return TransectStyleComplexItem::readyForSaveState();
}


void PathfindingComplexItem::_updateWizardMode(void)
{
    if (_corridorPolyline.isValid() && !_corridorPolyline.traceMode()) {
        setWizardMode(false);
    }
}
