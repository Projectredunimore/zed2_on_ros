
# zed2_on_ros

ZED2 Open Capture ROS Wrapper built without CUDA

## Dependencies

 - ROS Noetic
 - OpenCV
 -  PointCloudLibrary (see installation instruction below)
 - Eigen3

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
   
   ## Build PointCloudLibrary
  Download .tar Release file from [GitHub pcl repository](https://github.com/PointCloudLibrary/pcl)

Build and install

    tar xvf pcl-pcl-1.x.x.tar.gz
    cd pcl-pcl-1.x.x && mkdir build && cd build
	cmake -DCMAKE_BUILD_TYPE=Release ..
	make -j2
	sudo make -j2 install


Create symlink for ROS includes

    cd /usr/local/include
    sudo ln -s pcl-1.x/pcl pcl 

  
