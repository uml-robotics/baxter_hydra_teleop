Baxter Hydra Teleop
===================

Control Baxter's arms with Razer Hydra controllers. (Video: https://www.youtube.com/watch?v=duClzR2592o)

As Indigo does not support using the Oculus Rift w/ RVIZ, we now use 2 machines (in addition to baxter)
1. Ubuntu 12.04 and ROS Hydro - has the Rift and Hydra connected to it, and runs the Rift, the Hydra, and hydra_teleop node
2. Ubuntu 14.04 and ROS Indigo- has the Asus XTION connected to it, and runs openni, static transform publishers, and PCL


Dependencies (not exhaustive. sorry)
------------------
On the rift+hydra (hydro) PC:
```
sudo apt-get install ros-hydro-razer-hydra ros-hydro-oculus-rviz-plugins ros-hydro-baxter-sdk
# THEN... in a catkin or ROS workspace (nothing is compiled, but should be somewhere ROS knows about)
git clone https://github.com/uml-robotics/baxter_hydra_teleop
git clone https://github.com/uml-robotics/baxter_faces
```

On the openni+tf+pcl PC:
```
sudo apt-get install ros-indigo-perception-pcl ros-indigo-pcl-ros ros-indigo-openni-camera
# THEN... in a catkin or ROS workspace (nothing is compiled, but should be somewhere ROS knows about)
git clone https://github.com/uml-robotics/baxter_hydra_teleop
```


TO RUN
------------------
On the Hydro PC:
```
roslaunch baxter_hydra_teleop hydra_teleop.launch
```

On the Indigo PC:
```
rosrun baxter_hydra_teleop kinect_stuff
```

The teleop is initially disabled. To engage, press either RB or LB.
Currently pressing any other button stops the teleop and shuts off motors. Left analog stick is mapped to head pan.
