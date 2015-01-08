phantom-omni
============

<img align="right" src="https://raw.github.com/fsuarez6/phantom_omni/hydro-devel/omni_description/resources/OmniRviz.png" />

ROS Packages for Sensable PHANToM Omni device, **firewire** version. On going development continues in the hydro-devel branch

ROS packages developed by the [Group of Robots and Intelligent Machines](http://www.romin.upm.es/) from the [Universidad Politécnica de Madrid](http://www.upm.es/internacional). This group is part of the [Centre for Automation and Robotics](http://www.car.upm-csic.es/) (CAR UPM-CSIC). On going development continues in the hydro-devel branch.

**Maintainer:** Francisco Suárez Ruiz, [http://www.romin.upm.es/fsuarez/](http://www.romin.upm.es/fsuarez/)

### Documentation
* See the installation instructions below.
* This repository.
* Throughout the various files in the packages.
* For questions, please use [http://answers.ros.org](http://answers.ros.org)

## Installation

This instructions have been tested with OpenHaptics SDK 3.0 in Ubuntu 12.04, both 32 and 64 bits.

Before going any further please check the following:
* You have a **firewire** PHANToM Omni NOT the Ethernet one.
* Your development machine has a **firewire** port. I have tested both Firewire400 and Firewire800 and they work just fine.
* You understand what a **catkin workspace**, a **symbolic link** and a **git repository** are.

### OpenHaptics SDK

This package requires OpenHaptics SDK. You can get it from (http://dsc.sensable.com/).

### ROS Metapackage

**Note:** This instructions are for **existing** catkin workspaces. To create a catking workspace check [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

Go to your ROS working directory. e.g.
```
cd ~/catkin_ws/src
``` 
Use the `wstool` to install the repository
```
wstool init .
wstool merge https://raw.github.com/fsuarez6/phantom_omni/hydro-devel/phantom_omni.rosinstall
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

### Testing Installation

Be sure to always source the appropriate ROS setup file, which for Hydro is done like so:
```
source /opt/ros/hydro/setup.bash
``` 
You might want to add that line to your `~/.bashrc`

Try the `omni.launch` file in the `omni_common` package:
```
roslaunch omni_common omni.launch
``` 


## Changelog
### 0.1.0 (2013-10-28)
* Initial Release


## Roadmap
TODO


## Tutorials
TODO
