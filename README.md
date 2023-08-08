
# zed2_on_ros

ZED2 Open Capture ROS Wrapper built without CUDA

## Dependencies

 - ROS Noetic
 - OpenCV

## Installation

Clone this repository

    git clone https://github.com/Projectredunimore/zed2_on_ros.git

Clone zed-open-capture official repository inside the package

    cd zed2_on_ros/
    git clone https://github.com/stereolabs/zed-open-capture.git

Build the library

    cd zed-open-capture/
    mkdir build
    cd build/
    cmake ..
    make
    sudo make install
    
   Build the ROS package
   

    cd your_catkin_ws/
    catkin build
   
  
