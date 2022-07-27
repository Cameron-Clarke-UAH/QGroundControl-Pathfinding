# Obstacle File Creation:
In order to create an obstacle file to work with this software one must have a georefrenced point cloud representing the area they want to path through.
## Data collection:
The recommended way to collect the data required to generate the obstacle file is through structure from motion based photogrammetry using aerial photography.

## Data processing
Using the aerial photos a point cloud of the area should be generated. This point cloud should be used to generate a height map. The compatible height map file type for the conversion script XYZDatConversion.cpp should be an ascii XYZ point cloud.

The density of the generated height map should be around 0.5 meters.

## Obstacle File Format Details
~4.6 bytes per point depending on the cloud being used, missing data will result in worse bytes per point as missing points are assigned even if a value does not exist.
911184*4+4+1+8+8+8+8 = 3644773

Total Size : 3,644,773 bytes
3644773/793693 = 
4.59216977 bytes per point

Requires Coordinates to be sorted by X from minimum to maximum and then sorted from y minimum to maximum, missing data points have their z assigned to a feasibly impossible value

[XY coordinate order implicit 

X=XOffset+Scale*(Index%Width)

Y=YOffset+Scale*Index//Width]


Index = (Y-YMinimum)*PointsPerMeter*Width+(X-XMinimum)*PointsPerMeter

Header:

(int) UTM Zone  					(16)

(bool) North/South 					(True/False)

(double)OffsetX					(MinX)

(double)OffsetY					(MinY)

(double)DistanceBetweenPoints 		(0.5 m)

(int) XCount 						((MaxX-MinX)/Scale)

Data:

(float) Z 

(float) Z 

(float) Z 

........



