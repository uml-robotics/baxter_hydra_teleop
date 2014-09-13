Baxter Hydra Teleop
===================

Control Baxter's arms with Razer Hydra controllers.

Tested in Ros Hydro with Baxter SDK 1.0.0
Note that the hydro deb repository ros-hydro-baxter-sdk is not compatible at the time of writing this.
You can use ros-groovy-baxter-sdk, or do a source checkout following rethinks workstation installation guide
on the wiki for hydro compatibility.

Video: https://www.youtube.com/watch?v=duClzR2592o

Quick Start (Hydro)
------------------

```
sudo apt-get install ros-hydro-razer-hydra
```

Then grab required packages and put them on your ros path somewhere:

```
git clone https://github.com/uml-robotics/baxter_hydra_teleop.git
git clone https://github.com/uml-robotics/baxter_faces
```

You should be able to
```
roslaunch baxter_hydra_teleop hydra_teleop.launch
```

The teleop is initially disabled. To engage, press either RB or LB.
Currently pressing any other button stops the teleop and shuts off motors. Left analog stick is mapped to head pan.
