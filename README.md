# Husky robot with LiDAR for avoidance path

This repository presents a solution for object classification and creation of local avoidance path in the presence of obstacles. Considerations for this first version of the implementation: the robot already would have a global path to follow, objects are static and with shape known a priori.

The simulations were done with:
<table>
  <tbody>
    <tr>
      <td align="center"><b> Ubuntu version </b></td>
      <td>
        Description: Ubuntu 18.04.4 LTS<br>
        Release: 18.04<br>
        Codename:bionic
      </td>
    </tr>
    <tr>
      <td align="center"><b> ROS Version </b></td>
      </td>
      <td>melodic</td>
    </tr>
    <tr>
      <td align="center"><b> Gazebo version </b></td>
      <td>gazebo9</td>
    </tr>
    <tr>
      <td align="center"><b> Shell </b></td>
      <td>bash</td>
    </tr>
    <tr>
      <td align="center"><b> Details </b></td>
      <td>
        <ul>
            <li>Velodyne HDL-32E </li>
            <li>PCL library</li>
        </ul>
      </td>
    </tr>
  </tbody>
</table>


# Installation and initial Setups

Install **Ros melodic** following the steps from the [ROS website](http://wiki.ros.org/melodic/Installation). Once done, begin to install gazebo and the Husky. The velodyne model was downloaded from the [DataspeedInc git](https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/).

To install gazebo9, follow the steps below:

Install **equivs**:
```
sudo apt-get install equivs
```

Create a text file `gdal-abi.control` with the following text:
```
Section: misc
Priority: optional
Standards-Version: 3.9.2

Package: gdal-abi-2-2-3
Version: 2.2.3
Depends: libgdal20
Description: Fake package for libgazebo9 which needs a gdal-abi-2-2-3
```

Execute the following commands

```
sudo equivs-build gdal-abi.control
sudo dpkg -i gdal-abi-2-2-3_2.2.3_all.deb
sudo apt install libgazebo9
sudo apt install gazebo9
```
Now, install the [Husky](http://wiki.ros.org/Robots/Husky) package:

```
sudo apt-get install ros-melodic-husky-gazebo
sudo apt-get install ros-melodic-husky-simulator
sudo apt-get install ros-melodic-teleop-twist-keyboard
```

Great! You have ROS, gazebo and the Husky robot!

Now, let's set up the workspace within the .bashrc file:
```
$gedit ~/.bashrc
# Add the line at the end of the .bashrc file:
source <MyLocalPath>/huskylidar/devel/setup.bash
$source ~/.bashrc
```

# Working with the workspace
Remember to always update the workspace after doing changes, by doing the following command in the **~/huskylidar** folder using the **catkin_make** command.

To create new packages within the `huskylidar` workspace:
```
catkin_create_pkg [NAME] rospy roscpp
```

If you want to go to the location of a specific package already installed on the computer:
```
roscd [package_name]
```

## Simulating the Husky robot

See more:
[Simulating Husky](http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky), [Teleop Twist Keyboard](http://wiki.ros.org/teleop_twist_keyboard)

1. To verify if the Husky robot was correct installed, you can try running it with the **husky_gazebo** package. Additionally, you can control the robot using your keyboard with the help of the **teleop_twist_keyboard** package.
```
# Terminal 1:
roslaunch husky_gazebo husky_empty_world.launch
# Terminal 2:
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

2. The original Husky robot doesn't come with the LiDAR. The [velodyne_simulator](https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/) packages offer a implementation for the VLP-16 and HDL-32E, which can be combined with the Husky robot. The LiDAR chosen for this simulation was the HDL-32E. It is possible to run the Husky robot with a LiDAR and see the output point cloud using the commands below. As mentioned before, you can also control the robot using your keyboard with the help of the **teleop_twist_keyboard** package.
```
# Terminal 1:
roslaunch velodyne_description husky_with_velodyne.launch
# Terminal 2:
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## The cloud processing and avoidance path

To run the cloud processing code, do:
```
rosrun cloud_processing object_classification

```

The cloud processing includes filters implemented in both launch files and c++ files, as shown below. The avoidance path uses information obtained at the cloud processing stages and creates a local avoidance path for the global path using a Python script.
```
cloud_processing
├── CMakeLists.txt
├── package.xml
├── include
│   └── cloud_processing
│       ├── avoidance_path.h
│       ├── config.h
│       ├── filters.h
│       └── object_classification.h
├── launch
│   ├── cropbox_filter.launch
│   └── passthrough_filter.launch
└── src
    └── obstacle_detection
    │   ├── avoidance_path.cpp
    │   ├── filters.cpp
    │   ├── object_classification.cpp
    │   └── obstacledetection_node.cpp
    └── python
        ├── GenerateBezier.py
        └── lmpc.py
```

For the simulation stages, the dataflow below was followed:

1. Raw data is filtered
2. A object classification is done with the filtered cloud, with information whether they are obstacles or not.
3. After the classification and under the base assumptions, in case a object is considered as obstacle (that means, object in the global path), a local avoidance path is created and then the robot goes back to the global path.

The cloud processing can also be done using the [velodyne_height_map](http://wiki.ros.org/velodyne_height_map), which also takes a raw set of data from a point cloud and identifies whether objects are obstacles or not.


# Final Considerations
* You might see the error `No p gain specified for pid`. This error won't change the outcome of the present repository. If you will need to control the robot with a PID, these parameters need to be configured.
* It is possible to see the circles used by the avoidance path algorithm by changing the value of `ENABLE_OBJECT_CLASSIFICATION_CLOUDS` and `ENABLE_AVOIDANCE_PATH_CLOUDS` on the [config](huskylidar/src/cluster_objects/cloud_processing/include/cloud_processing/config.h) file.
