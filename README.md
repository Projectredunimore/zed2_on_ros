
# zed2_on_ros

ZED2 Open Capture ROS Wrapper built without CUDA

## Dependencies

 - ROS Noetic
 - OpenCV
 -  PointCloudLibrary (see installation instruction below)
 - Eigen3

## Installation

Clone this repository within you src folder

    git clone https://github.com/Projectredunimore/zed2_on_ros.git

Install `zed-open-capture` prerequisites

    sudo apt install build-essential
    sudo apt install cmake
    sudo apt install libusb-1.0-0-dev libhidapi-libusb0 libhidapi-dev
    sudo apt install libopencv-dev libopencv-viz-dev
    sudo apt install libeigen3-dev

Clone zed-open-capture official repository inside the package

    cd zed2_on_ros/
    git clone https://github.com/stereolabs/zed-open-capture.git

Add udev rules (Stereo cameras such as ZED 2 and ZED Mini have built-in sensors (e.g. IMU) that are identified as USB HID devices. To be able to access the USB HID device, you must add a udev rule contained in the udev folder)
    
    cd zed-open-capture/
    cd udev
    bash install_udev_rule.sh
    cd ..


Build the library
    
    cd zed-open-capture/
    mkdir build
    cd build/
    cmake ..
    make
    sudo make install
    sudo ldconfig
   
   ## Build PointCloudLibrary
  
  Download .tar Release file from [GitHub pcl repository](https://github.com/PointCloudLibrary/pcl)

Add pcl package into your src folder

    git clone https://github.com/PointCloudLibrary/pcl

Build and install (jump over the first two lines if you downloaded through terminal)

    tar xvf pcl-pcl-1.x.x.tar.gz
    cd pcl-pcl-1.x.x && 
    
    mkdir build && cd build
	cmake -DCMAKE_BUILD_TYPE=Release ..
	make -j2
	sudo make -j2 install

Create symlink for ROS includes

    cd /usr/local/include
    sudo ln -s pcl-1.x/pcl pcl 
    cd
    sudo ln -s /usr/include/eigen3/Eigen/ /usr/local/include/Eigen

Build the ROS package   

    cd your_catkin_ws/
    catkin build
