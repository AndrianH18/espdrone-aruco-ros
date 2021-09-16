# espdrone-aruco-ros
This repository contains ROS packages for ESP-drone camera image processing and ArUco marker detection and pose tracking.

![demo_gif](./documentation/demo_flight.gif)

This stack works in conjunction with [`espdrone-ros`](https://github.com/NelsenEW/espdrone-ros), a set of ROS packages for interfacing with the ESP-drone using ROS, including for fetching camera image from the drone. Both this repository and `espdrone-ros` are specialized for the NTU EEE UAVIONICS DIP project.

This stack as well as `aruco-3.1.12`, the required dependency not in this repository, have been tested on **Ubuntu 20.04 and ROS Noetic** (both using Docker and native system installation).


## Overview
The packages in this repository allow for **processing the camera image from multiple ESP-drones concurrently**, specifically to:
* Process the camera image using ROS `image_proc`, e.g. for camera undistortion and convertion to grayscale (`MONO8`) format
* Perform pose estimation (position and orientation) of the camera/drone using multiple [ArUco](https://www.uco.es/investiga/grupos/ava/node/26) fiducial markers scattered across the drone's environment, forming a [marker map](http://www.uco.es/investiga/grupos/ava/node/57)

Pose estimation is possible if at least 2 ArUco markers are in the field of view of the drone's camera. This is the current limitation of the `aruco-3.1.12` library. 

## Computer Hardware Requirement
This stack is CPU bound; no powerful GPU is required. The stack has been tested to run fine on a Raspberry Pi 4B (8GB model) running Ubuntu 20.04 MATE, though a computer with a more powerful CPU is highly recommended if you intend to launch more than one ESP-drones.  

The computer running this stack also needs to connect to the same network as the ESP-drone.


## Setup and Usage Guide
For guides on setup and usage or notes on this stack, please head to the [Wiki section](https://github.com/AndrianH18/espdrone-aruco-ros/wiki) of this repository.
