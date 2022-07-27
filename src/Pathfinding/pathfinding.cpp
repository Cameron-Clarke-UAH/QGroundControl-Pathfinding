#include "pathfinding.h"
#include "QGCApplication.h"
#include <iostream>
#include <cstdio>
#include <random>
#include "geometry.h"
#include <fstream>
#include <chrono>
#include <string>
#include <limits>
#include "UTM.cpp"
#include <QQmlEngine>

QVector<QGeoCoordinate> Path;


using namespace std;
using namespace std::chrono;
vector < Point > Cloud;
KDNode* CloudTree;
double stepResolution = 4;
Point MinPoint;
Point MaxPoint;
Point originOffset = Point(0.0,0,0);
using namespace std::placeholders;
double CollisionRadius = 2;
const double INF = 1e18;

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
    AStar::Vec3i Coord;
    Point Scaled = (a - originOffset) * (1/stepResolution);
    Coord.x = Scaled.x;
    Coord.y = Scaled.y;
    Coord.z = Scaled.z;
    return Coord;
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
    setHeuristic(&Heuristic::euclidean);
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

void AStar::Generator::setOriginandMaximumPoint(Point Origin, Point Maximum)
{
    originOffset = Origin;
    worldSize = CoordFromPoint(Maximum);

}


void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}


AStar::CoordinateList AStar::Generator::findPath(Point start, Point end)
{
    Vec3i source_ = CoordFromPoint(start);
    Vec3i target_ = CoordFromPoint(end);
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

    CoordinateList path;
    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

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



bool AStar::Generator::detectCollision(Vec3i prevCoord,Vec3i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        coordinates_.z < 0 || coordinates_.z >= worldSize.z ||
        !isEdgeObstacleFree(CoordToPoint(prevCoord), CoordToPoint(coordinates_))) {
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

AStar::uint AStar::Heuristic::octagonal(Vec3i source_, Vec3i target_)
{
    auto delta = getDelta(source_, target_);
    return 10 * (delta.x + delta.y + delta.z) + (-6) * std::min(delta.x, delta.y);
}



PathfindingPlugin::PathfindingPlugin(QGCApplication* app, QGCToolbox* toolbox)
    : QGCTool(app, toolbox)
{

}

void PathfindingPlugin::qmlDebug(QString msg){
    qDebug() << msg;
}

QGeoCoordinate PathfindingPlugin::getCoord(int index){
    return Path[index];
}
int PathfindingPlugin::listSize()
{
    return Path.length();
};

void PathfindingPlugin::pathToWaypoint(SimpleMissionItem *visualItem,double endLong, double endLat,double endAlt){
    Path.clear();

    AStar::Generator generator;

    if (size(Cloud)<1)
    {
        double Coords[3] = {};
        ifstream ReadTest("C:\\Users\\ccc0032\\Documents\\Refined.pnt", ios::in | ios::binary);
        while (ReadTest) {
            ReadTest.read((char*)&Coords, sizeof(double) * 3);
            Cloud.push_back(Point(Coords[0], Coords[1], Coords[2]));

        }
        MinPoint = Point(INF, INF, INF);
        MaxPoint = Point(-INF, -INF, -INF);
        ReadTest.close();
        for (int i = 0; i < size(Cloud); i++)
        {
            if (Cloud[i].x < MinPoint.x) { MinPoint.x = Cloud[i].x; }

            if (Cloud[i].y < MinPoint.y) { MinPoint.y = Cloud[i].y; }

            if (Cloud[i].z < MinPoint.z) { MinPoint.z = Cloud[i].z; }

            if (Cloud[i].x > MaxPoint.x) { MaxPoint.x = Cloud[i].x; }

            if (Cloud[i].y > MaxPoint.y) { MaxPoint.y = Cloud[i].y; }

            if (Cloud[i].z > MaxPoint.z) { MaxPoint.z = Cloud[i].z; }

        }
        CloudTree = BuildOrderedKDTree(Cloud);
        qDebug()  << "KD Tree Built\n";
    }

    generator.setOriginandMaximumPoint(MinPoint, MaxPoint+Point(0,0,20));

    // You can use a few heuristics : manhattan, euclidean or octagonal.
    generator.setHeuristic(AStar::Heuristic::euclidean);
    QGeoCoordinate PriorPos = visualItem->coordinate();
    double PriorAlt = visualItem->altitude()->rawValue().toDouble();
    double X=0,Y=0;
    LatLonToUTMXY(PriorPos.latitude(), PriorPos.longitude(), 16, X, Y);
    Point start = Point(X, Y, PriorAlt);
    qDebug()<< "StartPoint("<<start.x<<","<<start.y<<","<<PriorAlt<<")\n";
    LatLonToUTMXY(endLat, endLong, 16, X, Y);
    Point stop = Point(X, Y, 130);
    qDebug()<< "endPoint("<<stop.x<<","<<stop.y<<","<<stop.z<<")\n";

    auto startT = high_resolution_clock::now();
    auto path = generator.findPath(start,stop);
    auto stopT = high_resolution_clock::now();

    auto duration = duration_cast<milliseconds>(stopT - startT);

    qDebug()  << "Time taken by function: "
        << duration.count() << " milliseconds\n";

    for (int i=size(path)-1;i>=0; i--) {
        Point RealCoord = CoordToPoint(path[i]);
        double lat,lon;
        UTMXYToLatLon(RealCoord.x,RealCoord.y,16,false,lat,lon);
        Path.append(QGeoCoordinate(lat*180/PI,lon*180/PI,RealCoord.z));
    }

}
