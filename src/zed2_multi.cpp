////////////////////////////////////////////////////////////////////////////
////
//// Copyright (c) 2021, STEREOLABS.
////
//// All rights reserved.
////
//// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
////
/////////////////////////////////////////////////////////////////////////////

#define VIDEO_MOD_AVAILABLE = 1

//// ----> Includes
#include "videocapture.hpp"

#include "ocv_display.hpp"

#include <iostream>
#include <iomanip>

// OpenCV includes
#include <opencv2/opencv.hpp>


// ROS includes
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
//  <---- Includes


// The main function
int main(int argc, char *argv[])
{
    // ----> Silence unused warning
    (void)argc;
    (void)argv;
    // <---- Silence unused warning

    // ----> ROS initialization
	ros::init(argc, argv, "zed2_multi_node");
	ros::NodeHandle nh;
	ros::Rate rate(60);
	ros::Publisher left_pub = nh.advertise<sensor_msgs::Image>("/left_image_rgb", 1000);
    ros::Publisher right_pub = nh.advertise<sensor_msgs::Image>("/right_image_rgb", 1000);
    // <---- ROS initialization

    sl_oc::VERBOSITY verbose = sl_oc::VERBOSITY::INFO;

    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::HD720;
    params.fps = sl_oc::video::FPS::FPS_60;
    params.verbose = verbose;

    // ----> Create Video Capture 0 (left)
    sl_oc::video::VideoCapture cap_0(params);
    if (!cap_0.initializeVideo(0))
    {
        std::cerr << "Cannot open camera video capture" << std::endl;
        std::cerr << "See verbosity level for more details." << std::endl;

        return EXIT_FAILURE;
    }

    int sn = cap_0.getSerialNumber();
    std::cout << "Connected to camera sn: " << sn << "[" << cap_0.getDeviceName() << "]" << std::endl;
    // <---- Create Video Capture 0 (left)

    // ----> Create Video Capture 1 (right)
    sl_oc::video::VideoCapture cap_1(params);
    if( !cap_1.initializeVideo(2) )
    {
        std::cerr << "Cannot open camera video capture" << std::endl;
        std::cerr << "See verbosity level for more details." << std::endl;

        return EXIT_FAILURE;
    }

    std::cout << "Connected to camera sn: " << cap_1.getSerialNumber() << " [" << cap_1.getDeviceName() << "]" << std::endl;
    // <---- Create Video Capture 1 (right)

    // Set video parameters
    bool autoSettingEnable = true;
    cap_0.setAutoWhiteBalance(autoSettingEnable);
    cap_0.setAECAGC(autoSettingEnable);

    cap_1.setAutoWhiteBalance(autoSettingEnable);
    cap_1.setAECAGC(autoSettingEnable);

    // Initialize ROS msgs
    sensor_msgs::ImagePtr left_msg, right_msg;

    // Infinite video grabbing loop
    while (ros::ok())
    {
        // Get last available frame
        const sl_oc::video::Frame frame_0 = cap_0.getLastFrame();
        const sl_oc::video::Frame frame_1 = cap_1.getLastFrame();

        // ----> If the frame is valid we can display it
        if (frame_0.data != nullptr && frame_1.data != nullptr)
        {

            // ----> Conversion from YUV 4:2:2 to BGR for visualization
            cv::Mat frameYUV_0 = cv::Mat( frame_0.height, frame_0.width, CV_8UC2, frame_0.data );
            cv::Mat frameBGR_0;
            cv::Mat frameYUV_1 = cv::Mat( frame_1.height, frame_1.width, CV_8UC2, frame_1.data );
            cv::Mat frameBGR_1;
            cv::cvtColor(frameYUV_0,frameBGR_0,cv::COLOR_YUV2BGR_YUYV);
            cv::cvtColor(frameYUV_1,frameBGR_1,cv::COLOR_YUV2BGR_YUYV);
            // <---- Conversion from YUV 4:2:2 to BGR for visualization

            // Show frame
            // sl_oc::tools::showImage( "Stream RGB #0", frameBGR_0, params.res  );
            // sl_oc::tools::showImage( "Stream RGB #1", frameBGR_1, params.res  );

            // Publish frame on topic
	        left_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frameBGR_0).toImageMsg();
            right_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frameBGR_1).toImageMsg();
            left_pub.publish(left_msg);
            right_pub.publish(right_msg);
        }
        // <---- If the frame is valid we can display it

        // ----> Keyboard handling
        ros::spinOnce();
        int key = cv::waitKey(5);
        if (key == 'q' || key == 'Q') // Quit
            break;
        if(key=='a' || key=='A')
        {
            autoSettingEnable = !autoSettingEnable;
            cap_0.setAutoWhiteBalance(autoSettingEnable);
            cap_0.setAECAGC(autoSettingEnable);

            cap_1.setAutoWhiteBalance(autoSettingEnable);
            cap_1.setAECAGC(autoSettingEnable);

            std::cout << "Auto GAIN/EXPOSURE and Auto White Balance: " << (autoSettingEnable?"ENABLED":"DISABLED") << std::endl;
        }
        // <---- Keyboard handling
    }

    return EXIT_SUCCESS;
}
