#ifndef GEOMETRY_H
#define GEOMETRY_H
// ^To make sure I don't declare any function more than once by including the header multiple times.

#include <math.h>
#include <vector>
#include <algorithm>
#include <numeric>
using namespace std;

// Type of data type to be used for all calculations (Ex: long double)
#define ftype double
#define PI 3.14159265
/*  NOTE: Most of the calculations are done using EPS as a factor of difference
    since double/long double doesn't store floating point values precisely (limited precision) */
const ftype EPS = 1e-6;

struct Quaternion
{
    double w, x, y, z;

    Quaternion(double X, double Y, double Z, double W)
    {
        x = X;
        y = Y;
        z = Z;
        w = W;
    }
};

void NormQuaternion(Quaternion& input)
{

    double Length = sqrt(pow(input.x, 2) + pow(input.y, 2) + pow(input.z, 2) + pow(input.w, 2));
    input.x /= Length;
    input.y /= Length;
    input.z /= Length;
    input.w /= Length;

}



Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    yaw = -yaw;
    pitch = -pitch;
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q = Quaternion(sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy);

    return q;
}

Quaternion MulQuaternion(Quaternion Q1, Quaternion Q2)
{
    Quaternion QO = Quaternion(Q1.w * Q2.x + Q1.x * Q2.w + Q1.y * Q2.z - Q1.z * Q2.y, Q1.w * Q2.y - Q1.x * Q2.z + Q1.y * Q2.w + Q1.z * Q2.x, Q1.w * Q2.z + Q1.x * Q2.y - Q1.y * Q2.x + Q1.z * Q2.w, Q1.w * Q2.w - Q1.x * Q2.x - Q1.y * Q2.y - Q1.z * Q2.z);

    return QO;
}



Quaternion GlobalToLocal(Quaternion LocalRotation, double GlobalX, double GlobalY, double GlobalZ) {

    Quaternion ConjugateRotation = Quaternion(-LocalRotation.x, -LocalRotation.y, -LocalRotation.z, LocalRotation.w);

    Quaternion GlobalCoordinate = Quaternion(GlobalX, GlobalY, GlobalZ, 0);

    return MulQuaternion(MulQuaternion(ConjugateRotation, GlobalCoordinate), LocalRotation);

}

Quaternion LocalToGlobal(Quaternion LocalRotation, double GlobalX, double GlobalY, double GlobalZ) {

    Quaternion ConjugateRotation = Quaternion(-LocalRotation.x, -LocalRotation.y, -LocalRotation.z, LocalRotation.w);

    Quaternion LocalCoordinate = Quaternion(GlobalX, GlobalY, GlobalZ, 0);

    return MulQuaternion(MulQuaternion(LocalRotation, LocalCoordinate), ConjugateRotation);

}

struct Point {
    ftype x, y, z;
    Point() {}
    Point(ftype x, ftype y, ftype z) : x(x), y(y), z(z) {}
    Point& operator+=(const Point& t) {
        x += t.x;
        y += t.y;
        z += t.z;
        return *this;
    }
    Point& operator-=(const Point& t) {
        x -= t.x;
        y -= t.y;
        z -= t.z;
        return *this;
    }
    Point& operator*=(ftype t) {
        x *= t;
        y *= t;
        z *= t;
        return *this;
    }
    Point& operator/=(ftype t) {
        x /= t;
        y /= t;
        z /= t;
        return *this;
    }
    Point operator+(const Point& t) const {
        return Point(*this) += t;
    }
    Point operator-(const Point& t) const {
        return Point(*this) -= t;
    }
    Point operator*(ftype t) const {
        return Point(*this) *= t;
    }
    Point operator/(ftype t) const {
        return Point(*this) /= t;
    }
    ftype dot(const Point& t) const {
        return (x * t.x + y * t.y + z * t.z);
    }
    ftype distance(const Point& t) const {
        const double x_diff = x - t.x, y_diff = y - t.y, z_diff = z - t.z;
        return sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
    }
    ftype operator[](int i) {
        switch (i) {
        case 0:
            return x;
        case 1:
            return y;
        case 2:
            return z;
        default:
            return NULL;
        }
    }

    //Fix || figure out what it does
    Point steer(const Point& t, ftype DELTA) {
        if (this->distance(t) < DELTA) {
            return t;
        }
        else {
            return *this + (t - *this) * (DELTA / distance(t));
        }
    }
    bool operator==(const Point& rhs) const
    {
        return fabs(x - rhs.x) < EPS && fabs(y - rhs.y) < EPS && fabs(z - rhs.z) < EPS; // || another approach as above
    }
    ftype len() {
        return sqrt((x * x) + (y * y) + (z * z));
    }
};

Point operator*(ftype a, Point b) {
    return b * a;
}
bool operator<(Point a, Point b) {
    return a.x<b.x && a.y<b.y && a.z<b.z;
}
bool operator>(Point a, Point b) {
    return a.x > b.x && a.y > b.y && a.z > b.z;
}
Point Norm(Point A) {
    return A / A.len();
}

ftype distance(Point& a, Point& b) {
    const ftype x_diff = a.x - b.x, y_diff = a.y - b.y, z_diff = a.z - b.z;
    return sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
}

ftype dot(Point a, Point b) {
    return (a.x * b.x + a.y * b.y + a.z * b.z);
}

Point cross(Point a, Point b) {
    return Point(a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x);
}


ftype distanceToSegment( Point a, Point b, Point v)
{
    Point ab = b - a;
    Point av = v - a;
    if (av.dot(ab) <= 0.0)           // Point is lagging behind start of the segment, so perpendicular distance is not viable.
        return av.len();         // Use distance to start of segment instead.

    Point bv = v - b;
    if (bv.dot(ab) >= 0.0)           // Point is advanced past the end of the segment, so perpendicular distance is not viable.
        return bv.len();         // Use distance to end of the segment instead.
    return (cross(ab, av)).len() / ab.len();       // Perpendicular distance of point to segment.
}

ftype ThreePointDeterminate(Point A, Point B, Point C) {
    // A [a b c]
    // B [d e f]
    // C [g h i]
    return A.x * B.y * C.z - A.x * B.z * C.y - A.y * B.x * C.z + A.y * B.z * C.x + A.z * B.x * C.y - A.z * B.y * C.x;
}

ftype distanceBetweenSegments(Point a0, Point a1, Point b0, Point b1, bool clampA0, bool clampA1, bool clampB0, bool clampB1 ) {
    //
    Point A = a1 - a0;
    Point B = b1 - b0;
    ftype magA = A.len();
    ftype magB = B.len();
    Point _A = A/magA;     // Unit Vector of line A
    Point _B = B/magB;     // Unit Vector of line B
    Point CrossAB = cross(_A, _B);
    ftype Crosslen = CrossAB.len();
    ftype denom = Crosslen * Crosslen;
    if (abs(denom) <= EPS)
    {
        ftype d0 = _A.dot((b1 - a0));
        if (clampA0 || clampA1 || clampB0 || clampB1)
        {
            ftype d1 = _A.dot((b1 - a0));
            if (d0 <= 0 && 0 >= d1 && clampA0 && clampB1)
            {
                return abs(d0) < abs(d1) ? (a0 - b0).len() : (a0 - b1).len();
            }

            if (d0 <= magA && magA >= d1 && clampA1 && clampB0)
            {
                return abs(d0) < abs(d1) ? (a1 - b0).len() : (a1 - b1).len();
            }

        }

        return ((d0 * _A + a0) - b0).len();
    }
    Point t = (b0 - a0);

    ftype detA = ThreePointDeterminate(t, _B, CrossAB);
    ftype detB = ThreePointDeterminate(t, _A, CrossAB);

    ftype t0 = detA / denom;
    ftype t1 = detB / denom;
    Point pA = a0 + (_A * t0);
    Point pB = b0 + (_A * t1);
    // Clamp projections
    if (clampA0 || clampA1 || clampB0 || clampB1)
    {
        if (clampA0 && t0 < 0) pA = a0;
        else if (clampA1 && t0 > magA) pA = a1;

        if (clampB0 && t1 < 0) pB = b0;
        else if (clampB1 && t1 > magB) pB = b1;

        // Clamp projection A
        if ((clampA0 && t0 < 0) || (clampA1 && t0 > magA))
        {
            ftype _dot = _B.dot((pA - b0));
            if (clampB0 && _dot < 0) _dot = 0;
            else if (clampB1 && _dot > magB) _dot = magB;
            pB = b0 + (_B * _dot);
        }

        // Clamp projection B
        if ((clampB0 && t1 < 0) || (clampB1 && t1 > magB))
        {
            ftype _dot = _A.dot((pB - a0));
            if (clampA0 && _dot < 0) _dot = 0;
            else if (clampA1 && _dot > magA) _dot = magA;
            pA = a0 + (_A * _dot);
        }
    }
    return (pA - pB).len();
}

/*  Returns a point in the direction of (p2 - p1) vector such that
    the new point is within a DELTA distance of point1  */
Point stepNear(Point& p1, Point& p2, ftype DELTA) {
    if (distance(p1, p2) <= DELTA)
        return p2;
    else {
        return p1 + (p2 - p1) * (DELTA / distance(p1, p2));
    }
}

/*  Return true if the given line segment intersects the circle whose center
    is at location */
bool checkCollision(Point a, Point b, Point p3, ftype r)
{
    if (distanceToSegment(a, b, p3) < r)
    { return true; }
    else { return false; }
}

struct KDNode {
    int index;
    KDNode *left, * right;
    ~KDNode() {
        delete left; // delete does nothing if ptr is 0
        delete right; // || recurses if there's an object
    }
};

// A method to create a node of K D tree
struct KDNode* newKDNode(int PIndex)
{
    struct KDNode* temp = new KDNode;

    temp->index = PIndex;
    temp->left = temp->right = NULL;
    return temp;
}


struct ParameterSort {
    ParameterSort(int Axis_, vector<Point>& pointlist_) : Axis(Axis_), pointlist(pointlist_){}
    bool operator () (int A, int B) { return pointlist[A][Axis] < pointlist[B][Axis]; }
    int Axis;
    vector<Point>& pointlist;
};


KDNode* BuildOrderedKDTreeRec(vector <Point>& pointlist,vector<int> &indexList, int depth, KDNode* root)
{
    unsigned Axis = depth % 3;

    nth_element(indexList.begin(), indexList.begin() + indexList.size() / 2, indexList.end(), ParameterSort(Axis, pointlist));
    vector <int> leftElements(indexList.begin(), indexList.begin() + indexList.size() / 2);
    vector <int> rightElements(indexList.begin() + indexList.size() / 2 +1,indexList.end());

    root = newKDNode(indexList[indexList.size() / 2]);

    int llen = size(leftElements);
    if (llen > 1)
    {
        root->left = BuildOrderedKDTreeRec(pointlist, leftElements, depth + 1, root->left);
    }
    else if(llen == 1)
    {
        root->left= newKDNode(leftElements[0]);
    }
    int rlen = size(rightElements);
    if (rlen > 1)
    {
        root->right = BuildOrderedKDTreeRec(pointlist, rightElements, depth + 1, root->right);
    }
    else if(rlen == 1)
    {
        root->right = newKDNode(rightElements[0]);
    }

    return root;
}

KDNode* BuildOrderedKDTree(vector <Point>& pointlist) {
// Ordering of depth = XYZ
    std::vector<int> indexList(size(pointlist));
    std::iota(indexList.begin(), indexList.end(), 0);
   return BuildOrderedKDTreeRec(pointlist,indexList, 0,NULL);
}


// Inserts a new node && returns root of modified tree
// The parameter depth is used to decide axis of comparison
KDNode* insertRec(KDNode* root, int Pindex, unsigned depth, vector <Point> &pointlist)
{
    // Tree is empty?
    if (root == NULL)
        return newKDNode(Pindex);

    // Calculate current dimension (cd) of comparison
    unsigned cd = depth % 3;

    // Compare the new point with root on current dimension 'cd'
    // && decide the left || right subtree
    if (pointlist[Pindex][cd] < (pointlist[root->index][cd]))
        root->left = insertRec(root->left, Pindex, depth + 1, pointlist);
    else
        root->right = insertRec(root->right, Pindex, depth + 1, pointlist);

    return root;
}



// Function to insert a new point with given point in
// KD Tree && return new root. It mainly uses above recursive
// function "insertRec()"
KDNode* insert(KDNode* root, int Pindex, vector <Point>& pointlist)
{
    return insertRec(root, Pindex, 0, pointlist);
}

// Searches a Point represented by "point[]" in the K D tree.
// The parameter depth is used to determine current axis.
bool searchRec(KDNode* root, int Pindex, unsigned depth, vector <Point>& pointlist)
{
    // Base cases
    if (root == NULL)
        return false;
    if (root->index == Pindex)
        return true;

    // Current dimension is computed using current depth && total
    // dimensions (k)
    unsigned cd = depth % 3;

    // Compare point with root with respect to cd (Current dimension)
    if (pointlist[Pindex][cd] < (pointlist[root->index][cd]))
        return searchRec(root->left, Pindex, depth + 1,pointlist);

    return searchRec(root->right, Pindex, depth + 1, pointlist);
}

// Searches a Point in the K D tree. It mainly uses
// searchRec()
bool search(KDNode* root, int Pindex, vector <Point>& pointlist)
{
    // Pass current depth as 0
    return searchRec(root, Pindex, 0 , pointlist);
}




// helper function
int sign(const ftype x) {
    return x >= 0 ? x ? 1 : 0 : -1;
}



namespace AStar
{
    struct Vec3i
    {
        int x, y, z;

        bool operator == (const Vec3i& coordinates_);
    };

    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(Vec3i, Vec3i)>;
    using CoordinateList = std::vector<Vec3i>;

    struct Node
    {
        uint G, H;
        Vec3i coordinates;
        Node* parent;

        Node(Vec3i coord_, Node* parent_ = nullptr);
        uint getScore();
    };

    using NodeSet = std::vector<Node*>;

    class Generator
    {
        bool detectCollision(Vec3i prevCoord, Vec3i coordinates_);
        Node* findNodeOnList(NodeSet& nodes_, Vec3i coordinates_);
        void releaseNodes(NodeSet& nodes_);

    public:
        Generator();
        void setOrigin(Point Origin);
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        CoordinateList findPath(Point start, Point end);
        void addCollision(Vec3i coordinates_);
        void removeCollision(Vec3i coordinates_);
        void clearCollisions();

    private:
        HeuristicFunction heuristic;
        CoordinateList direction, walls;
        Vec3i worldSize;
        uint directions = 26;
    };

    class Heuristic
    {
        static Vec3i getDelta(Vec3i source_, Vec3i target_);

    public:
        static uint manhattan(Vec3i source_, Vec3i target_);
        static uint euclidean(Vec3i source_, Vec3i target_);
        static uint euclideanApproximation(Vec3i source_, Vec3i target_);
        static uint octagonal(Vec3i source_, Vec3i target_);
    };
}

#endif
