# QGroundControl Ground Control Station

[![Releases](https://img.shields.io/github/release/mavlink/QGroundControl.svg)](https://github.com/mavlink/QGroundControl/releases)
[![Travis Build Status](https://travis-ci.org/mavlink/qgroundcontrol.svg?branch=master)](https://travis-ci.org/mavlink/qgroundcontrol)
[![Appveyor Build Status](https://ci.appveyor.com/api/projects/status/crxcm4qayejuvh6c/branch/master?svg=true)](https://ci.appveyor.com/project/mavlink/qgroundcontrol)

[![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/mavlink/qgroundcontrol?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)


*QGroundControl* (QGC) is an intuitive and powerful ground control station (GCS) for UAVs.

The primary goal of QGC is ease of use for both first time and professional users.
It provides full flight control and mission planning for any MAVLink enabled drone, and vehicle setup for both PX4 and ArduPilot powered UAVs. Instructions for *using QGroundControl* are provided in the [User Manual](https://docs.qgroundcontrol.com/en/) (you may not need them because the UI is very intuitive!)

All the code is open-source, so you can contribute and evolve it as you want.
The [Developer Guide](https://dev.qgroundcontrol.com/en/) explains how to [build](https://dev.qgroundcontrol.com/en/getting_started/) and extend QGC.

Key Links:
* [Website](http://qgroundcontrol.com) (qgroundcontrol.com)
* [User Manual](https://docs.qgroundcontrol.com/en/)
* [Developer Guide](https://dev.qgroundcontrol.com/en/)
* [Discussion/Support](https://docs.qgroundcontrol.com/en/Support/Support.html)
* [Contributing](https://dev.qgroundcontrol.com/en/contribute/)
* [License](https://github.com/mavlink/qgroundcontrol/blob/master/COPYING.md)


# QGroundControl Pathfinding
![HeightMapVIs](https://user-images.githubusercontent.com/110111597/181274173-b5c820e9-c35b-400b-848b-8eef87aa7a5a.png)

This QGroundControl implementation adds a pathfinding system for mission planning under the complex mission item tab. The pathfinding makes use of a custom binary obstacle file format that stores UTM coordinates as a height map.


# Installation: 
The program can be installed by the installer found under the releases tab. Additionally, an example height map file of the University of Alabama in Huntsville is provided.

# Usage:
To make use of the pathfinding system a high resolution ( less than 1 meter between points) is required. Documentation on how this file is formatted can be found under the obstacle generation folder.

After installation, start the QGroundControl software. 
Upon startup you will be asked to select general settings. 

Upon completion of the setting selection the flight planning symbol can be pressed in the top left corner:

![image](https://user-images.githubusercontent.com/110111597/181282119-52f66903-e146-458e-bb80-e8ec7330cdd4.png)

After clicking this button the user should select the blank mission preset. 


![image](https://user-images.githubusercontent.com/110111597/181282713-1002df79-65c3-4fb0-88ed-16d0d31baa28.png)

After selecting the blank mission preset the window should go away. 

The user then needs to select a takeoff location, which can be done by clicking the takeoff button and then clicking on the desired takeoff location on the map.

![image](https://user-images.githubusercontent.com/110111597/181282824-e113ac55-c1b3-4836-8471-2fa4ec714d63.png)

![image](https://user-images.githubusercontent.com/110111597/181282874-3f787f49-0383-4d9e-a3a7-460b0a51856c.png)


The next step is clicking the pattern mission item menu button: 
![image](https://user-images.githubusercontent.com/110111597/181283177-af5eff0f-6c71-4be4-916e-ae6637970825.png)

Doing this will bring up a menu containing 4 options

![image](https://user-images.githubusercontent.com/110111597/181283342-72683b18-abef-4f28-9e9c-cf45b85e2673.png)

The pathfinding option should be clicked. This will create a new pathfinding mission item.

In order to select the start and end locations one should press the basic button in the polyline menu shown at the top of the screen: 
![image](https://user-images.githubusercontent.com/110111597/181283685-2505ed49-7f9e-416c-b6a1-d008a583e996.png)

This will create a single polyline: 

![image](https://user-images.githubusercontent.com/110111597/181283883-ea4f7271-4661-4b11-bc2a-9a9300d6abec.png)

By dragging the ends of the polyline the user can select the start and end locations. The top circle is the start location by default.

After creating the polyline multiple menu options will be shown on the right side.

![image](https://user-images.githubusercontent.com/110111597/181284215-906b4819-9c67-41ea-b1e2-e922de76902b.png)

These menu options allow the user to load a custom obstacle file, adjust the safe distance, adjust the initial heights of the start and end location, and select the grid size used for the A* pathing algorithm.

To select the obstacle file being loaded the "Load Obstacle File" button should be clicked. This will bring up a file selection menu from which the obstacle file should be selected. After selecting the obstacle file the program will take a few seconds to load the file.

After all of the relavent paremeters have been set, the obstacle file has been loaded, and the start and end locations are correct, click the refresh path button to generate the path. WARNING: Setting the grid size to anything less than 1 meter will take signifigant time to process. It is recommended for short distances to keep the grid size at 2 meters. For long distances, setting the grid size to a larger value such as 4 meters may be preferred for speed.

![image](https://user-images.githubusercontent.com/110111597/181285942-3ee92e3e-c98b-4fe8-863a-aad26ca8a2e0.png)

After clicking refresh path the generated path between the two locations will be shown.
In order for QGroundControl to accept the height values generated by this path the user must select Calc Above Terrain from the menu that is found under the mountain icon.

![image](https://user-images.githubusercontent.com/110111597/181286439-2b6344f6-e9a2-47ec-9a32-cebd86b4de52.png)

After generating the path and selecting Calc Above Terrain the pathfinding should be complete and ready to be flown. 

