# Stream Manipulator 3D #
[![Build Status](https://travis-ci.org/3DVision-Stack/stream-manipulator-3D.svg?branch=indigo-devel)](https://travis-ci.org/3DVision-Stack/stream-manipulator-3D)

Stream Manipulator 3D is a modular ROS node for 3D Vision, handling point cloud streams from various RGB-D sensors, like the Asus Xtion PRO or the Microsft Kinect One. It lets you modify the raw point cloud stream by applying various filters, in the order you choose, and eventually republish the modified stream to the ROS network or save it disk.

It is composed by a Ros node which handles Ros communications and processing (stream_manipulator_3d) and a GUI to control it (rqt_stream_manipulator_3d).
The GUI application is an rqt plugin, so you can run it by simply loading rqt and load stream_manipulator_3d from the configuration menu. 

*NOTE:* Package is still in beta and currently the GUI is the only way to control stream manipulator 3d node effectively.
