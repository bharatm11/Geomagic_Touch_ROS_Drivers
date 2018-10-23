3D Systems Geomagic Touch ROS Driver
============

ROS Packages for 3D Systems Geomagic Touch haptic device, **USB** version.

This repository has been forked from the original repository by Francisco Suárez Ruiz, [http://fsuarez6.github.io](http://fsuarez6.github.io) for the Sensable PHANToM haptic device (https://github.com/fsuarez6/phantom_omni).

ROS packages developed by the [Group of Robots and Intelligent Machines](http://www.romin.upm.es/) from the [Universidad Politécnica de Madrid](http://www.upm.es/internacional). This group is part of the [Centre for Automation and Robotics](http://www.car.upm-csic.es/) (CAR UPM-CSIC). 

## Installation

1. Install Dependencies

```
sudo apt-get install --no-install-recommends freeglut3-dev g++ libdrm-dev libexpat1-dev libglw1-mesa libglw1-mesa-dev libmotif-dev libncurses5-dev libraw1394-dev libx11-dev libxdamage-dev libxext-dev libxt-dev libxxf86vm-dev tcsh unzip x11proto-dri2-dev x11proto-gl-dev x11proto-print-dev
```

2. Download and Extract OpenHaptics and Haptic Device drivers

Download drivers using instructions at: https://3dsystems.teamplatform.com/pages/102863?t=fptvcy2zbkcc

3. Install Openhaptics

```
cd ~/openhaptics_3.4-0-developer-edition-amd64/
sudo ./install
This gets installed in the following directory
/opt/OpenHaptics/ 
```







