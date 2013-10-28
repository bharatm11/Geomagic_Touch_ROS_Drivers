phantom-omni
============

ROS Packages for Sensable Phantom Omni device. On going development continues in the hydro-devel branch

ROS packages developed by the [Group of Robots and Intelligent Machines](http://www.romin.upm.es/) from the [Universidad Politécnica de Madrid](http://www.upm.es/internacional). This group is part of the [Centre for Automation and Robotics](http://www.car.upm-csic.es/) (CAR UPM-CSIC). On going development continues in the hydro-devel branch.

**Maintainer:** Francisco Suárez Ruiz, [http://www.romin.upm.es/fsuarez/](http://www.romin.upm.es/fsuarez/)

---
# Documentation
* See the installation instructions below.
* This repository.
* Throughout the various files in the packages.
* For questions, please use [http://answers.ros.org](http://answers.ros.org)
---
# Installation

Go to your ROS working directory. e.g.
```
cd ~/catkin_ws/src
``` 
Use the `wstool` to install the repository
```
wstool init .
wstool merge https://raw.github.com/fsuarez6/phantom_omni/hydro-devel/omni.rosinstall
wstool update
``` 
Check for any missing dependencies using rosdep:
```
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro hydro
``` 
After installing the missing dependencies compile your ROS workspace. e.g.
```
cd ~/catkin_ws && catkin_make
``` 

## Testing Installation

Be sure to always source the appropriate ROS setup file, which for Hydro is done like so:
```
source /opt/ros/hydro/setup.bash
``` 
You might want to add that line to your `~/.bashrc`

Try any of the `.launch` files in the `grips_gazebo` package: (e.g. `cordless_drill.launch`)
```
roslaunch omni_common omni.launch
``` 

---
# Changelog
### 0.1.0 (2013-10-28)
* Initial Release

---
# Roadmap
TODO

---
# Tutorials
TODO
