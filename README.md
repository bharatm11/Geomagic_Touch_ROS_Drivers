3D Systems Geomagic Touch ROS Driver
============

ROS Packages for 3D Systems Geomagic Touch haptic device, **USB** version.

This repository has been forked from the original repository by Francisco Suárez Ruiz, [http://fsuarez6.github.io](http://fsuarez6.github.io) for the Sensable PHANToM haptic device (https://github.com/fsuarez6/phantom_omni).

ROS packages developed by the [Group of Robots and Intelligent Machines](http://www.romin.upm.es/) from the [Universidad Politécnica de Madrid](http://www.upm.es/internacional). This group is part of the [Centre for Automation and Robotics](http://www.car.upm-csic.es/) (CAR UPM-CSIC). 

## Installation

#1. Install Dependencies

```
sudo apt-get install --no-install-recommends freeglut3-dev g++ libdrm-dev libexpat1-dev libglw1-mesa libglw1-mesa-dev libmotif-dev libncurses5-dev libraw1394-dev libx11-dev libxdamage-dev libxext-dev libxt-dev libxxf86vm-dev tcsh unzip x11proto-dri2-dev x11proto-gl-dev x11proto-print-dev
```

2. Download and Extract OpenHaptics and Haptic Device drivers

Download drivers using instructions at: https://3dsystems.teamplatform.com/pages/102863?t=fptvcy2zbkcc

3. Install Openhaptics

```
cd ~/openhaptics_3.4-0-developer-edition-amd64/
sudo ./install
# This gets installed in the following directory
/opt/OpenHaptics/ 
```
4. Install Geomagic Driver

```
cd ~/geomagic_touch_device_driver_2015.5-26-amd64/
sudo ./install
# This gets installed in the following directory
/opt/geomagic_touch_device_driver/ 
```
5. **Only For 64-bit Systems** Create Symbolic Links to OpenHaptics SDK Libraries 
```
sudo ln -s /usr/lib/x86_64-linux-gnu/libraw1394.so.11.0.1 /usr/lib/libraw1394.so.8
sudo ln -s /usr/lib64/libPHANToMIO.so.4.3 /usr/lib/libPHANToMIO.so.4
sudo ln -s /usr/lib64/libHD.so.3.0.0 /usr/lib/libHD.so.3.0
sudo ln -s /usr/lib64/libHL.so.3.0.0 /usr/lib/libHL.so.3.0 
```
6. Device setup
The haptic device always creates a COM Port as /dev/ttyACM0 and requires admin priviliges
```
chmod 777 /dev/ttyACM0
```
Run Geomagic_Touch_Setup in /opt/geomagic_touch_device_driver/
Ensure that the device serial number is displayed 

7. Device Diagnostics

Run Geomagic_Touch_Diagnostic in /opt/geomagic_touch_device_driver/
This can be used to calibrate the device, read encoders, apply test forces etc. 

8. Launch ROS Node

Clone and build this repository.
```
cd <ROS_workspace>/devel
source setup.bash
roslaunch omni_common omni_state.launch 
```

Data from the haptic device can be read from the following topics:
  /phantom/button
  
  /phantom/force_feedback
  
  /phantom/joint_states
  
  /phantom/pose
  
  /phantom/state 

## Resources

https://3dsystems.teamplatform.com/pages/102863?t=fptvcy2zbkcc

https://fsuarez6.github.io/projects/geomagic-touch-in-ros/

https://github.com/fsuarez6/phantom_omni

http://dsc.sensable.com/viewtopic.php?t=5730&sid=9866fe798e24bc745fdb7fce08ee99eb



