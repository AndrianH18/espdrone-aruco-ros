# uavionics-dip-ros
This reposiroty contains ROS packages for the NTU EEE UAVIONICS DIP project.

These packages (as well as `aruco-3.1.12` and `librealsense`, the two required dependencies) have been tested on **Ubuntu 20.04 and ROS Noetic**.

## Setup
1. Download the `aruco-3.1.12` library [from here](https://sourceforge.net/projects/aruco/files/3.1.12/), then put it in your home directory. 
2. Setup the `librealsense` library required by the `realsense_ros` package *(skip this step if you do not require testing with Intel Realsense cameras)*:
    * Fetch the library from [GitHub](https://github.com/IntelRealSense/librealsense/tree/legacy).
    * We are using *Intel Realsense R200*, which is a legacy device. Make sure that you are on the `legacy` branch before proceeding to the next steps.
    * Install `libglfw3-dev`:
      ```bash
      sudo apt-get install libglfw3-dev
      ```
    * Build and install the library:
      ```bash
      cd librealsense
      mkdir build
      cd build
      cmake ..
      make && sudo make install
      ```
    * Copy `udev` rules:
      ```bash
      cd ..
      sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
      sudo udevadm control --reload-rules && udevadm trigger
      ```
3. Clone this repository to your Catkin workspace's `src` folder.
4. Build the workspace:
   ```bash
   cd ~/catkin_ws
   catkin build
   ```
   *Note: remove `realsense_ros` if you did not install `librealsense`.*

## References
Steps for setting up `librealsense` and `realsense-ros` for legacy devices on Ubuntu 18.04/20.04:  https://github.com/IntelRealSense/realsense-ros/issues/386.

* Not mentioned in the link: ROS PCL requires C++14 to compile, edit line 13 inside `realsense_camera/CMakeLists.txt` from `-std=c++11` to `-std=c++14`.
