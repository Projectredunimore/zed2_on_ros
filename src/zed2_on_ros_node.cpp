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
#include <ros/ros.h>
#include "ocv_display.hpp"

#include <iostream>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include "sensor_msgs/Image.h"

#include <cv_bridge/cv_bridge.h>
//  <---- Includes

// #define TEST_FPS 0

// The main function
int main(int argc, char *argv[])
{
    // ----> Silence unused warning
    (void)argc;
    (void)argv;
    // <---- Silence unused warning

    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::HD720;
    params.fps = sl_oc::video::FPS::FPS_60;

    // ----> Create Video Capture
    sl_oc::video::VideoCapture cap_0(params);
    if (!cap_0.initializeVideo())
    {
        std::cerr << "Cannot open camera video capture" << std::endl;
        std::cerr << "See verbosity level for more details." << std::endl;

        return EXIT_FAILURE;
    }

    std::cout << "Connected to camera sn: " << cap_0.getSerialNumber() << "[" << cap_0.getDeviceName() << "]" << std::endl;
    // <---- Create Video Capture

#ifdef TEST_FPS
    // Timestamp to check FPS
    double lastTime = static_cast<double>(getSteadyTimestamp()) / 1e9;
    // Frame timestamp to check FPS
    uint64_t lastFrameTs = 0;
#endif
	ros::init(argc, argv, "zed2_node");
	ros::NodeHandle nh;
	ros::Rate rate(60);
	ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/zed2_image", 1000);

    // Infinite video grabbing loop
    while (ros::ok())
    {
        // Get last available frame
        const sl_oc::video::Frame frame = cap_0.getLastFrame();

        // ----> If the frame is valid we can display it
        if (frame.data != nullptr)
        {
#ifdef TEST_FPS
            if (lastFrameTs != 0)
            {
                // ----> System time
                double now = static_cast<double>(getSteadyTimestamp()) / 1e9;
                double elapsed_sec = now - lastTime;
                lastTime = now;
                std::cout << "[System] Frame period: " << elapsed_sec << "sec - Freq: " << 1. / elapsed_sec << " Hz" << std::endl;
                // <---- System time

                // ----> Frame time
                double frame_dT = static_cast<double>(frame.timestamp - lastFrameTs) / 1e9;
                std::cout << "[Camera] Frame period: " << frame_dT << "sec - Freq: " << 1. / frame_dT << " Hz" << std::endl;
                // <---- Frame time
            }
            lastFrameTs = frame.timestamp;
#endif

            // ----> Conversion from YUV 4:2:2 to BGR for visualization
            cv::Mat frameYUV = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
	        cv::Mat frameBGR;
            cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);
            // <---- Conversion from YUV 4:2:2 to BGR for visualization
            // Show frame
		//sl_oc::tools::showImage( "Stream RGB", frameBGR, params.res  );

            // Publish frame on topic
	    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frameBGR).toImageMsg();
            pub.publish(msg);
        }
        // <---- If the frame is valid we can display it

        // ----> Keyboard handling
	std::cout << "ros msg published\n";
	ros::spinOnce();
	// std::cout << "spinned\n";
	// rate.sleep();
        //int key = cv::waitKey(5);
	//std::cout << "Waiting for key\n";
        //if (key == 'q' || key == 'Q') // Quit
        //    break;
        // <---- Keyboard handling
    }

    return EXIT_SUCCESS;
}
