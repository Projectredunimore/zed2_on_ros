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

// Sample includes
#include "zed2_on_ros/calibration.hpp"
#include "zed2_on_ros/ocv_display.hpp"

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
    ros::init(argc, argv, "zed2_rect_node");
	ros::NodeHandle nh;
	ros::Publisher lraw_pub = nh.advertise<sensor_msgs::Image>("/left_image_raw", 1000);
	ros::Publisher rraw_pub = nh.advertise<sensor_msgs::Image>("/right_image_raw", 1000);
	ros::Publisher lrect_pub = nh.advertise<sensor_msgs::Image>("/left_image_rect", 1000);
	ros::Publisher rrect_pub = nh.advertise<sensor_msgs::Image>("/right_image_rect", 1000);
    // <---- ROS initialization

    sl_oc::VERBOSITY verbose = sl_oc::VERBOSITY::INFO;

    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::HD1080;
    params.fps = sl_oc::video::FPS::FPS_30;
    params.verbose = verbose;

    // ----> Create Video Capture
    sl_oc::video::VideoCapture cap(params);
    if (!cap.initializeVideo())
    {
        std::cerr << "Cannot open camera video capture" << std::endl;
        std::cerr << "See verbosity level for more details." << std::endl;

        return EXIT_FAILURE;
    }

    int sn = cap.getSerialNumber();
    std::cout << "Connected to camera sn: " << sn << "[" << cap.getDeviceName() << "]" << std::endl;
    // <---- Create Video Capture


    // ----> Retrieve calibration file from Stereolabs server
    std::string calibration_file;
    // ZED Calibration
    unsigned int serial_number = sn;
    // Download camera calibration file
    if( !sl_oc::tools::downloadCalibrationFile(serial_number, calibration_file) )
    {
        std::cerr << "Could not load calibration file from Stereolabs servers" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Calibration file found. Loading..." << std::endl;

    // ----> Frame size
    int w,h;
    cap.getFrameSize(w,h);
    // <---- Frame size

    // ----> Initialize calibration
    cv::Mat map_left_x, map_left_y;
    cv::Mat map_right_x, map_right_y;
    cv::Mat cameraMatrix_left, cameraMatrix_right;
    sl_oc::tools::initCalibration(calibration_file, cv::Size(w/2,h), map_left_x, map_left_y, map_right_x, map_right_y,
                    cameraMatrix_left, cameraMatrix_right);

    std::cout << " Camera Matrix L: \n" << cameraMatrix_left << std::endl << std::endl;
    std::cout << " Camera Matrix R: \n" << cameraMatrix_right << std::endl << std::endl;
    std::cout << "Width: " << w << ", Height: " << h << std::endl;
    // ----> Initialize calibration

    cv::Mat frameBGR, left_raw, left_rect, right_raw, right_rect;

    // Initialize ROS msgs
    sensor_msgs::ImagePtr lraw_msg, rraw_msg, lrect_msg, rrect_msg;

    uint64_t last_ts=0;


    // Infinite video grabbing loop
    while (ros::ok())
    {
        // Get last available frame
        const sl_oc::video::Frame frame = cap.getLastFrame();

        // ----> If the frame is valid we can display it
        if (frame.data != nullptr && frame.timestamp != last_ts)
        {
            last_ts = frame.timestamp;

            // ----> Conversion from YUV 4:2:2 to BGR for visualization
            cv::Mat frameYUV = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
            cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);
            // <---- Conversion from YUV 4:2:2 to BGR for visualization

            // ----> Extract left and right images from side-by-side
            left_raw = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
            right_raw = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows));
            // Display images
            // sl_oc::tools::showImage("left RAW", left_raw, params.res);
            // sl_oc::tools::showImage("right RAW", right_raw, params.res);
            // <---- Extract left and right images from side-by-side

            // ----> Apply rectification
            cv::remap(left_raw, left_rect, map_left_x, map_left_y, cv::INTER_LINEAR );
            cv::remap(right_raw, right_rect, map_right_x, map_right_y, cv::INTER_LINEAR );

            // sl_oc::tools::showImage("right RECT", right_rect, params.res);
            // sl_oc::tools::showImage("left RECT", left_rect, params.res);
            // <---- Apply rectification

            // Publish frame on topic
            lraw_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_raw).toImageMsg();
            rraw_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", right_raw).toImageMsg();
            lrect_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_rect).toImageMsg();
            rrect_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", right_rect).toImageMsg();
            lraw_pub.publish(lraw_msg);
            rraw_pub.publish(rraw_msg);
            lrect_pub.publish(lrect_msg);
            rrect_pub.publish(rrect_msg);
        }
        // <---- If the frame is valid we can display it

        // ----> Keyboard handling
        ros::spinOnce();
        int key = cv::waitKey(5);
        if (key == 'q' || key == 'Q') // Quit
            break;
        // <---- Keyboard handling
    }

    return EXIT_SUCCESS;
}
