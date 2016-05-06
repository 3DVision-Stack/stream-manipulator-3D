# PaCMaN Vision ROS node #
![PaCMan Vision](https://cloud.githubusercontent.com/assets/1950251/12723299/22f2d844-c909-11e5-9621-142a1d49dcd4.png)
Collection of 3D Vision-Oriented utilities packed in a single ROS node.

PaCMan Vision (or just PACV) is a modular ROS node for robot Vision handling point cloud streams from various RGB-D sensors, like the Asus Xtion PRO or the Microsft Kinect One and republishing
the modified stream to the ROS network, making it available for other nodes.
The node is composed by a basic node (or __bare-bone__ node), providing some simple and fast point cloud filters, like a cropping filter, a voxel grid downsampling and a RANSAC plane
segmentation. All the filters can be modified at runtime using the built-in Qt Gui.

The node has other functionalities like the possibility to dynamically change point cloud stream subscription or to save a single processed point cloud from the stream that's being republished.

PaCMan Vision also introduces the concept of dynamic modules, a dynamic module is a part of the node, providing some additional functionality, you can dynamically load or kill at runtime.
Each module has its own ROS node handle with its topics and services you can exploit and when you don't need it anymore you can kill the module and its functionality will disappear.

As of now, PaCMan Vision has a few modules providing object recognition and pose estimation and 3D object tracking, but it is planned to release more modules in the near future.
Each module has its own section on the wiki, with install and usage instructions.

## Get PaCMaN Vision ##
Clone these two repositories into your ROS catkin workspace (you need and [SSH key setup on GitHub](https://help.github.com/articles/generating-an-ssh-key/)):
```
roscd && cd ../src
git clone git@github.com:Tabjones/pacman_vision.git pacv
git clone git@github.com:Tabjones/pacman_vision_communications.git pacv_com
```
Pacman_vision repository contains all the software needed to build the node, while vision_communication repository is just a bunch of defined messages and services PaCMan Vision uses
You will need both to build and run the node.
The reason why those two packages are separated is to provide the user the flexibility of having PaCMan Vision messages and services without actually building the
whole PaCMan Vision package, thus avoiding its dependencies, while still being able to call its services or subscribe to its messages from other packages.

## Build Instructions ##
Detailed install instructions are found on the [package wiki](https://github.com/Tabjones/pacman_vision/wiki)

## License ##

```
 Software License Agreement (BSD License)

   PaCMan Vision (PaCV) - https://github.com/Tabjones/pacman_vision
   Copyright (c) 2015-2016, Federico Spinelli (fspinelli@gmail.com)
   All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

 * Neither the name of the copyright holder(s) nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ANY OTHER LIBRARIES OR PACKAGES USED WITHIN THIS PROGRAM RETAIN THEIR ORIGINAL
LICENSE.
```

## Usage ##

Launch the node with:
```
    roslaunch pacman_vision pacman_vision.launch
```

To check available parameters:
```
    roslaunch pacman_vision pacman_vision.launch --ros-args
```

### Code Info
http://cloc.sourceforge.net v 1.60  T=1.40 s (43.5 files/s, 6293.6 lines/s)
___


|Language  |  files |    blank |  comment |     code|    scale |  3rd gen. equiv|
|:--      | :--:    | :--:    | :--:      | :--:    | :--:    | --:           |
|C++          |        20  |     220  |    1022   |   3694 x  | 1.51 =   |     5577.94|
|C/C++ Header |        25  |     122  |    1197   |   1515 x  | 1.00 =   |     1515.00|
|CMake        |        10  |      66  |     367   |    358 x  | 1.00 =   |      358.00|
|YAML         |         2  |       0  |       0   |     98 x  | 0.90 =   |       88.20|
|IDL          |         3  |      24  |       0   |     63 x  | 3.80 =   |      239.40|
|XML          |         1  |      15  |      34   |     24 x  | 1.90 =   |       45.60|
|             |            |          |           |           |          |            |
|SUM:         |        61  |     447  |    2620   |   5752 x  | 1.36 =   |     7824.14|

___

### Mirrors ###
This project is mirrored on:

  * [Github](https://github.com/Tabjones/pacman_vision).
  * [Bitbucket](https://bitbucket.org/Tabjones/pacman_vision).
  * [Gitlab](https://gitlab.com/fspinelli/pacman_vision).


