# Turtlebotto
Turtlebotto is a 3D mapping system under the environment **ROS Gazebo virtual environment**.
## Introduction
This task’s objective is to develop a system capable to perform the 3D mapping of the interior house shown in the following image.

![alt text](https://i.gyazo.com/4ab8c289b6e8335999c93c4a7f8c9f1f.png)

This can be achieved by using a Kinect camera attached to our Turtlebot robot for the ROS Gazebo virtual environment.
Since robot’s movement can be teleoperated or autonomous, we decided that will be teleoperated for the sake of capturing data easily as we need.
3D mapping will be done by using ***Point Cloud Library***, which will help us processing and filtering the point clouds captured by our robot’s topic.
As the robot moves, it will capture point clouds and will be modeling the house in a 3D model in real time.
In addition to our task and, in order to optimize it and save computing time, we applied methods such as **Iterative Closest Point** and outlier removal methods such as **Statistical Outlier Removal (SOR)**.
## Deployment
The process for executing the code is as follows:
```
1. run “roscore”.
2. run “roslaunch load_model init.launch”.
3. run “python main.py” in the /src/turtlebotto/teleop/ folder. This is for the teleop node, a pygame window will open and the robot can be moved through it by using the arrow keys.
4. run “rosrun mapping_3d mapping_3d_node + [parameters]”.
```
The following flags are to be used in the [parameters] section:

- **Original**: ```--o```.
- **Voxel Grid**: ```--v```.
- **SOR**: ```--s```.
- **SOR + Voxel Grid**: ```--sv```.
One of the following Keypoints detector flags:
- **SIFT3D**: ```--sift3d```.
- **Harris3D**: ```--harris3d```.

If Harris3D is used, one of the following flags can be chosen (if not, the default Harris3d method is used):
- **Tomasi**: ```--TOMASI```.
- **Lowe**: ```--LOWE```.
- **Noble**: ```--NOBLE```.
- **Curvature**: ```--CURVATURE```.

## Final results
The following images are examples of final cloud applying **Voxel Grid** and **SOR**. The first and second image is by using Sift3D and the second and third image is by using Harris3D and Tomasi.
![alt text](https://i.gyazo.com/e893316801dd1be46d2d8bddaeec70b6.png)
![alt text](https://i.gyazo.com/d8470d4e493b6ab2567bf90c5335b031.png)
![alt text](https://i.gyazo.com/84ab639d5dcb549e62734c7c0f64cd77.png)
![alt text](https://i.gyazo.com/7e3558d4f00db8fa3f70c8b8e7154a8e.png)

## Built With

* [ROS](http://wiki.ros.org/hydro/Installation) - An open-source, meta-operating system for robots.
* [Gazebo](http://gazebosim.org/tutorials?tut=ros_overview) -  A 3D dynamic simulator.
* [Turtlebot](http://wiki.ros.org/turtlebot_gazebo) - A robot simulator.
* [PointCloud Library](http://pointclouds.org/) - Used to process point cloud.
## Authors
* **Tudor** - [TudorMN](https://github.com/tudorMN)
* **Alicia** - [Alice6N](https://github.com/alice6n)
