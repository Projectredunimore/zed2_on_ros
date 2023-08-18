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

#undef HAVE_OPENCV_VIZ // Uncomment if cannot use Viz3D for point cloud rendering

#ifdef HAVE_OPENCV_VIZ
#include <opencv2/viz.hpp>
#include <opencv2/viz/viz3d.hpp>
#endif

#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

// Sample includes
#include "zed2_on_ros/calibration.hpp"
#include "zed2_on_ros/ocv_display.hpp"
#include "zed2_on_ros/stopwatch.hpp"
#include "zed2_on_ros/stereo.hpp"

// ROS includes
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/PointCloud2.h>
#include "pcl_conversions/pcl_conversions.h"
#include <std_msgs/Float32.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <camera_info_manager/camera_info_manager.h>

//  <---- Includes


    // ---> Init params (should be private) 
    int resolution_ = 1;
    std::string device_name_ = std::string("/dev/video0");
    double frame_rate_ = 30.0;
    bool show_image_ = false;
    bool use_zed_config_ = true;
    double width_, height_;
    std::string left_frame_id_ = "left_camera";
    std::string right_frame_id_ = "right_camera";
    std::string config_file_location_;
    std::string encoding_ = std::string("bgr8");
    // <---- Init params

  /**
   * @brief      Gets the camera information From Zed config.
   *
   * @param[in]  config_file         The configuration file
   * @param[in]  resolution          The resolution
   * @param[in]  left_cam_info_msg   The left camera information message
   * @param[in]  right_cam_info_msg  The right camera information message
   */
  void getZedCameraInfo(std::string config_file, int resolution, sensor_msgs::CameraInfo& left_info,
                        sensor_msgs::CameraInfo& right_info)
  {
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_file, pt);
    std::string left_str = "LEFT_CAM_";
    std::string right_str = "RIGHT_CAM_";
    std::string reso_str = "";

    switch (resolution)
    {
      case 0:
        reso_str = "2K";
        width_ = 4416;
        height_ = 1242;
        break;
      case 1:
        reso_str = "FHD";
        width_ = 3840;
        height_ = 1080;
        break;
      case 2:
        reso_str = "HD";
        width_ = 2560;
        height_ = 720;
        break;
      case 3:
        reso_str = "VGA";
        width_ = 1344;
        height_ = 376;
        break;
    }
    // left value
    double l_cx = pt.get<double>(left_str + reso_str + ".cx");
    double l_cy = pt.get<double>(left_str + reso_str + ".cy");
    double l_fx = pt.get<double>(left_str + reso_str + ".fx");
    double l_fy = pt.get<double>(left_str + reso_str + ".fy");
    double l_k1 = pt.get<double>(left_str + reso_str + ".k1");
    double l_k2 = pt.get<double>(left_str + reso_str + ".k2");
    // right value
    double r_cx = pt.get<double>(right_str + reso_str + ".cx");
    double r_cy = pt.get<double>(right_str + reso_str + ".cy");
    double r_fx = pt.get<double>(right_str + reso_str + ".fx");
    double r_fy = pt.get<double>(right_str + reso_str + ".fy");
    double r_k1 = pt.get<double>(right_str + reso_str + ".k1");
    double r_k2 = pt.get<double>(right_str + reso_str + ".k2");

    // get baseline and convert mm to m
    boost::optional<double> baselineCheck;
    double baseline = 0.0;
    // some config files have "Baseline" instead of "BaseLine", check accordingly...
    if (baselineCheck = pt.get_optional<double>("STEREO.BaseLine"))
    {
      baseline = pt.get<double>("STEREO.BaseLine") * 0.001;
    }
    else if (baselineCheck = pt.get_optional<double>("STEREO.Baseline"))
    {
      baseline = pt.get<double>("STEREO.Baseline") * 0.001;
    }
    else
    {
      throw std::runtime_error("baseline parameter not found");
    }

    // get Rx and Rz
    double rx = pt.get<double>("STEREO.RX_" + reso_str);
    double rz = pt.get<double>("STEREO.RZ_" + reso_str);
    double ry = pt.get<double>("STEREO.CV_" + reso_str);

    // assume zeros, maybe not right
    double p1 = 0, p2 = 0, k3 = 0;

    left_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    right_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    // TODO(dizeng) verify loading default zed config is still working

    // distortion parameters
    // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
    left_info.D.resize(5);
    left_info.D[0] = l_k1;
    left_info.D[1] = l_k2;
    left_info.D[2] = k3;
    left_info.D[3] = p1;
    left_info.D[4] = p2;

    right_info.D.resize(5);
    right_info.D[0] = r_k1;
    right_info.D[1] = r_k2;
    right_info.D[2] = k3;
    right_info.D[3] = p1;
    right_info.D[4] = p2;

    // Intrinsic camera matrix
    // 	[fx  0 cx]
    // K =  [ 0 fy cy]
    //	[ 0  0  1]
    left_info.K.fill(0.0);
    left_info.K[0] = l_fx;
    left_info.K[2] = l_cx;
    left_info.K[4] = l_fy;
    left_info.K[5] = l_cy;
    left_info.K[8] = 1.0;

    right_info.K.fill(0.0);
    right_info.K[0] = r_fx;
    right_info.K[2] = r_cx;
    right_info.K[4] = r_fy;
    right_info.K[5] = r_cy;
    right_info.K[8] = 1.0;

    // rectification matrix
    // Rl = R_rect, R_r = R * R_rect
    // since R is identity, Rl = Rr;
    left_info.R.fill(0.0);
    right_info.R.fill(0.0);
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << rx, ry, rz);
    cv::Mat rmat(3, 3, CV_64F);
    cv::Rodrigues(rvec, rmat);
    int id = 0;
    cv::MatIterator_<double> it, end;
    for (it = rmat.begin<double>(); it != rmat.end<double>(); ++it, id++)
    {
      left_info.R[id] = *it;
      right_info.R[id] = *it;
    }

    // Projection/camera matrix
    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    left_info.P.fill(0.0);
    left_info.P[0] = l_fx;
    left_info.P[2] = l_cx;
    left_info.P[5] = l_fy;
    left_info.P[6] = l_cy;
    left_info.P[10] = 1.0;

    right_info.P.fill(0.0);
    right_info.P[0] = r_fx;
    right_info.P[2] = r_cx;
    right_info.P[3] = (-1 * l_fx * baseline);
    right_info.P[5] = r_fy;
    right_info.P[6] = r_cy;
    right_info.P[10] = 1.0;

    left_info.width = right_info.width = width_;
    left_info.height = right_info.height = height_;

    left_info.header.frame_id = left_frame_id_;
    right_info.header.frame_id = right_frame_id_;
  }

  /**
   * @brief      { publish camera info }
   *
   * @param[in]  pub_cam_info  The pub camera information
   * @param[in]  cam_info_msg  The camera information message
   * @param[in]  now           The now
   */
  void publishCamInfo(const ros::Publisher& pub_cam_info, sensor_msgs::CameraInfo& cam_info_msg, ros::Time now)
  {
    std::cout << "publishing cma info\n";
    cam_info_msg.header.stamp = now;
    pub_cam_info.publish(cam_info_msg);
    std::cout << "cam info published" << std::endl;
  }

// #define USE_OCV_TAPI // Comment to use "normal" cv::Mat instead of CV::UMat
#define USE_HALF_SIZE_DISP // Comment to compute depth matching on full image frames

// The main function
int main(int argc, char *argv[])
{
    // ----> Silence unused warning
    (void)argc;
    (void)argv;
    // <---- Silence unused warning

    
    // ----> ROS initialization
    ros::init(argc, argv, "zed2_node");
	ros::NodeHandle nh;
	ros::Publisher depth_pub = nh.advertise<sensor_msgs::Image>("/depth_image", 1000);
	ros::Publisher left_pub = nh.advertise<sensor_msgs::Image>("/left/image_rect_color", 1000);
	ros::Publisher right_pub = nh.advertise<sensor_msgs::Image>("/right/image_rect_color", 1000);
    ros::Publisher left_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
    ros::Publisher right_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);
    ros::Publisher dist_pub = nh.advertise<std_msgs::Float32>("/distance", 1000);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1000);
    // <---- ROS initialization

    // ----> Get ROS params
    nh.getParam("resolution", resolution_);
    nh.getParam("frame_rate", frame_rate_);
    nh.getParam("config_file_location", config_file_location_);
    nh.getParam("left_frame_id", left_frame_id_);
    nh.getParam("right_frame_id", right_frame_id_);
    nh.getParam("show_image", show_image_);
    nh.getParam("use_zed_config", use_zed_config_);
    nh.getParam("device_name", device_name_);
    nh.getParam("encoding", encoding_);
    // <---- Get ROS params

    std::cout << "Config file: " << config_file_location_ << std::endl;
    std::cout << "use_zed_config: " << use_zed_config_ << std::endl;


    sl_oc::VERBOSITY verbose = sl_oc::VERBOSITY::INFO;

    // ----> Set Video parameters
    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::HD720;
    params.fps = sl_oc::video::FPS::FPS_30;
    params.verbose = verbose;
    // <---- Set Video parameters

    // ----> Create Video Capture
    sl_oc::video::VideoCapture cap(params);
    if (!cap.initializeVideo(-1))
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
    std::cout << "Calibration file path: " << calibration_file << std::endl;

    // ----> Frame size
    int w,h;
    cap.getFrameSize(w,h);
    // <---- Frame size

    // ----> Initialize calibration
    cv::Mat map_left_x, map_left_y;
    cv::Mat map_right_x, map_right_y;
    cv::Mat cameraMatrix_left, cameraMatrix_right;
    double baseline = 0;
    sl_oc::tools::initCalibration(calibration_file, cv::Size(w/2,h), map_left_x, map_left_y, map_right_x, map_right_y,
                    cameraMatrix_left, cameraMatrix_right, &baseline);

    double fx = cameraMatrix_left.at<double>(0,0);
    double fy = cameraMatrix_left.at<double>(1,1);
    double cx = cameraMatrix_left.at<double>(0,2);
    double cy = cameraMatrix_left.at<double>(1,2);

    std::cout << " Camera Matrix L: \n" << cameraMatrix_left << std::endl << std::endl;
    std::cout << " Camera Matrix R: \n" << cameraMatrix_right << std::endl << std::endl;

#ifdef USE_OCV_TAPI
    cv::UMat map_left_x_gpu = map_left_x.getUMat(cv::ACCESS_READ,cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    cv::UMat map_left_y_gpu = map_left_y.getUMat(cv::ACCESS_READ,cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    cv::UMat map_right_x_gpu = map_right_x.getUMat(cv::ACCESS_READ,cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    cv::UMat map_right_y_gpu = map_right_y.getUMat(cv::ACCESS_READ,cv::USAGE_ALLOCATE_DEVICE_MEMORY);
#endif
    // ----> Initialize calibration

    // ----> Declare OpenCV images
#ifdef USE_OCV_TAPI
    cv::UMat frameYUV;  // Full frame side-by-side in YUV 4:2:2 format
    cv::UMat frameBGR(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Full frame side-by-side in BGR format
    cv::UMat left_raw(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Left unrectified image
    cv::UMat right_raw(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Right unrectified image
    cv::UMat left_rect(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Left rectified image
    cv::UMat right_rect(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Right rectified image
    cv::UMat left_for_matcher(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Left image for the stereo matcher
    cv::UMat right_for_matcher(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Right image for the stereo matcher
    cv::UMat left_disp_half(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Half sized disparity map
    cv::UMat left_disp(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Full output disparity
    cv::UMat left_disp_float(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Final disparity map in float32
    cv::UMat left_disp_image(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Normalized and color remapped disparity map to be displayed
    cv::UMat left_depth_map(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Depth map in float32
#else
    cv::Mat frameBGR, left_raw, left_rect, right_raw, right_rect, frameYUV, left_for_matcher, right_for_matcher, left_disp_half,left_disp,left_disp_float, left_disp_vis, left_disp_image, left_depth_map;
#endif
    // <---- Declare OpenCV images

    // ----> Stereo matcher initialization
    sl_oc::tools::StereoSgbmPar stereoPar;

    //Note: you can use the tool 'zed_open_capture_depth_tune_stereo' to tune the parameters and save them to YAML
    if(!stereoPar.load())
    {
        stereoPar.save(); // Save default parameters.
    }

    cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(stereoPar.minDisparity,stereoPar.numDisparities,stereoPar.blockSize);
    left_matcher->setMinDisparity(stereoPar.minDisparity);
    left_matcher->setNumDisparities(stereoPar.numDisparities);
    left_matcher->setBlockSize(stereoPar.blockSize);
    left_matcher->setP1(stereoPar.P1);
    left_matcher->setP2(stereoPar.P2);
    left_matcher->setDisp12MaxDiff(stereoPar.disp12MaxDiff);
    left_matcher->setMode(stereoPar.mode);
    left_matcher->setPreFilterCap(stereoPar.preFilterCap);
    left_matcher->setUniquenessRatio(stereoPar.uniquenessRatio);
    left_matcher->setSpeckleWindowSize(stereoPar.speckleWindowSize);
    left_matcher->setSpeckleRange(stereoPar.speckleRange);

    stereoPar.print();
    // <---- Stereo matcher initialization


    // ----> Point Cloud
    cv::Mat cloudMat;

#ifdef HAVE_OPENCV_VIZ
    cv::viz::Viz3d pc_viewer = cv::viz::Viz3d( "Point Cloud" );
#endif
    // <---- Point Cloud

    uint64_t last_ts=0;

    // Initialize ROS msgs
    sensor_msgs::ImagePtr disp_msg;
    sensor_msgs::PointCloud2 cloud_msg;
    std_msgs::Float32 dist_msg;
    sensor_msgs::ImagePtr left_msg, right_msg;
    sensor_msgs::CameraInfo left_info, right_info;

    ROS_INFO("Try load camera calibration files");
    if (use_zed_config_)
    {
      ROS_INFO("Loading from zed calibration files");
      // get camera info from zed
      if (!calibration_file.empty())
      {
        try
        {
          getZedCameraInfo(calibration_file, resolution_, left_info, right_info);
        }
        catch (std::runtime_error& e)
        {
          ROS_INFO("Can't load camera info");
          ROS_ERROR("%s", e.what());
          throw e;
        }
      }
      else
      {
        ROS_FATAL("Please input zed config file path");
      }
    }
    else
    {
        std::cout << "No zed config \n";
        /*
      ROS_INFO("Loading from ROS calibration files");
      // here we just use camera infor manager to load info
      // get config from the left, right.yaml in config
      ros::NodeHandle left_nh("left");
      ros::NodeHandle right_nh("right");
      camera_info_manager::CameraInfoManager left_info_manager(left_nh, "camera/left",
                                                               "package://zed2_on_ros/config/left.yaml");
      left_info = left_info_manager.getCameraInfo();

      camera_info_manager::CameraInfoManager right_info_manager(right_nh, "camera/right",
                                                                "package://zed2_on_ros/config/right.yaml");
      right_info = right_info_manager.getCameraInfo();

      left_info.header.frame_id = left_frame_id_;
      right_info.header.frame_id = right_frame_id_;
      */
    }

    ROS_INFO("Got camera calibration files");

    ros::Rate r(frame_rate_);
    // Infinite video grabbing loop
    while (ros::ok())
    {
        std::cout << "start ros loop\n";
        ros::Time now = ros::Time::now();

        // Get last available frame
        const sl_oc::video::Frame frame = cap.getLastFrame();

        // ----> If the frame is valid we can display it
        if (frame.data != nullptr && frame.timestamp != last_ts)
        {
            std::cout << "frame ok \n";
            last_ts = frame.timestamp;

            // ----> Conversion from YUV 4:2:2 to BGR for visualization
#ifdef USE_OCV_TAPI
            cv::Mat frameYUV_cpu = cv::Mat( frame.height, frame.width, CV_8UC2, frame.data );
            frameYUV = frameYUV_cpu.getUMat(cv::ACCESS_READ,cv::USAGE_ALLOCATE_HOST_MEMORY);
#else
            frameYUV = cv::Mat( frame.height, frame.width, CV_8UC2, frame.data );
#endif
            cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);
            // <---- Conversion from YUV 4:2:2 to BGR for visualization

            // ----> Extract left and right images from side-by-side
            left_raw = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
            right_raw = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows));
            // <---- Extract left and right images from side-by-side
            std::cout << "left and right images extracted\n";
            // ----> Apply rectification
            sl_oc::tools::StopWatch remap_clock;
#ifdef USE_OCV_TAPI
            cv::remap(left_raw, left_rect, map_left_x_gpu, map_left_y_gpu, cv::INTER_AREA );
            cv::remap(right_raw, right_rect, map_right_x_gpu, map_right_y_gpu, cv::INTER_AREA );
#else
            cv::remap(left_raw, left_rect, map_left_x, map_left_y, cv::INTER_AREA );
            cv::remap(right_raw, right_rect, map_right_x, map_right_y, cv::INTER_AREA );
#endif
            double remap_elapsed = remap_clock.toc();
            std::stringstream remapElabInfo;
            remapElabInfo << "Rectif. processing: " << remap_elapsed << " sec - Freq: " << 1./remap_elapsed;
            // <---- Apply rectification
            std::cout << "rectified" << std::endl;

            // ----> Stereo matching
            sl_oc::tools::StopWatch stereo_clock;
            double resize_fact = 1.0;
#ifdef USE_HALF_SIZE_DISP
            resize_fact = 0.5;
            // Resize the original images to improve performances
            cv::resize(left_rect,  left_for_matcher,  cv::Size(), resize_fact, resize_fact, cv::INTER_AREA);
            cv::resize(right_rect, right_for_matcher, cv::Size(), resize_fact, resize_fact, cv::INTER_AREA);
#else
            left_for_matcher = left_rect; // No data copy
            right_for_matcher = right_rect; // No data copy
#endif
            std::cout << "resized\n";
            // Apply stereo matching
            /*
            left_matcher->compute(left_for_matcher, right_for_matcher,left_disp_half);
            std::cout << "computed" << std::endl;
            left_disp_half.convertTo(left_disp_float,CV_32FC1);
            std::cout << "converted\n";
            cv::multiply(left_disp_float,1./16.,left_disp_float); // Last 4 bits of SGBM disparity are decimal
            std::cout << "multiplky\n";
#ifdef USE_HALF_SIZE_DISP
            cv::multiply(left_disp_float,2.,left_disp_float); // Last 4 bits of SGBM disparity are decimal
            cv::Mat tmp = left_disp_float; // Required for OpenCV 3.2
            cv::resize(tmp, left_disp_float, cv::Size(), 1./resize_fact, 1./resize_fact, cv::INTER_AREA);
#else
            left_disp = left_disp_float;
#endif
            std::cout << "Stereo matching\n";

            double elapsed = stereo_clock.toc();
            std::stringstream stereoElabInfo;
            stereoElabInfo << "Stereo processing: " << elapsed << " sec - Freq: " << 1./elapsed;
            */
            // <---- Stereo matching

            // ----> Show frames
            /*
            sl_oc::tools::showImage("Right rect.", right_rect, params.res,true, remapElabInfo.str());
            sl_oc::tools::showImage("Left rect.", left_rect, params.res,true, remapElabInfo.str());
            */
            // <---- Show frames

            // Publish image frames on topic
	        left_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_rect).toImageMsg();
            right_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", right_rect).toImageMsg();
            left_pub.publish(left_msg);
            right_pub.publish(right_msg);

            std::cout << "msg published" << std::endl;

            // ----> Show disparity image
            /*
            cv::add(left_disp_float,-static_cast<double>(stereoPar.minDisparity-1),left_disp_float); // Minimum disparity offset correction
            cv::multiply(left_disp_float,1./stereoPar.numDisparities,left_disp_image,255., CV_8UC1 ); // Normalization and rescaling

            cv::applyColorMap(left_disp_image,left_disp_image,cv::COLORMAP_JET); // COLORMAP_INFERNO is better, but it's only available starting from OpenCV v4.1.0

            sl_oc::tools::showImage("Disparity", left_disp_image, params.res,true, stereoElabInfo.str());
            */
            // <---- Show disparity image

            // ----> Extract Depth map
            // The DISPARITY MAP can be now transformed in DEPTH MAP using the formula
            // depth = (f * B) / disparity
            // where 'f' is the camera focal, 'B' is the camera baseline, 'disparity' is the pixel disparity
            /*
            double num = static_cast<double>(fx*baseline);
            cv::divide(num,left_disp_float,left_depth_map);

            // float central_depth = left_depth_map.getMat(cv::ACCESS_READ).at<float>(left_depth_map.rows/2, left_depth_map.cols/2 );
            float central_depth = left_depth_map.at<float>(left_depth_map.rows/2, left_depth_map.cols/2 );
            std::cout << "Depth of the central pixel: " << central_depth << " mm" << std::endl;
            dist_msg.data = central_depth;
            dist_pub.publish(dist_msg);
            */
            // <---- Extract Depth map

            // ----> Create Point Cloud
            sl_oc::tools::StopWatch pc_clock;
            size_t buf_size = static_cast<size_t>(left_depth_map.cols * left_depth_map.rows);
            std::vector<cv::Vec3d> buffer( buf_size, cv::Vec3f::all( std::numeric_limits<float>::quiet_NaN() ) );
            // cv::Mat depth_map_cpu = left_depth_map.getMat(cv::ACCESS_READ);
            cv::Mat depth_map_cpu = left_depth_map;
            float* depth_vec = (float*)(&(depth_map_cpu.data[0]));

#pragma omp parallel for
            for(size_t idx=0; idx<buf_size;idx++ )
            {
                size_t r = idx/left_depth_map.cols;
                size_t c = idx%left_depth_map.cols;
                double depth = static_cast<double>(depth_vec[idx]);
                //std::cout << depth << " ";
                if(!isinf(depth) && depth >=0 && depth > stereoPar.minDepth_mm && depth < stereoPar.maxDepth_mm)
                {
                    buffer[idx].val[2] = depth; // Z
                    buffer[idx].val[0] = (c-cx)*depth/fx; // X
                    buffer[idx].val[1] = (r-cy)*depth/fy; // Y
                }
            }

            cloudMat = cv::Mat( left_depth_map.rows, left_depth_map.cols, CV_64FC3, &buffer[0] ).clone();

            double pc_elapsed = stereo_clock.toc();
            std::stringstream pcElabInfo;
//            pcElabInfo << "Point cloud processing: " << pc_elapsed << " sec - Freq: " << 1./pc_elapsed;
            //std::cout << pcElabInfo.str() << std::endl;
            // <---- Create Point Cloud

            std::cout << "point loud generated\n";

        
            // Convert pointcloud to ROS msg
            char pr = 100, pg = 100, pb = 100;
            pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
            // assuming cloudMat contains X,Y,Z values in consecutive columns
            for (int i = 0; i < cloudMat.cols; i++)
            {
                pcl::PointXYZ point;
                point.x = cloudMat.at<float>(0,i);
                point.y = cloudMat.at<float>(1,i);
                point.z = cloudMat.at<float>(2,i);

                // when colors need to be added
                // uint32_t rgb = static_cast<uint32_t>(pr) << 16 |
                //                static_cast<uint32_t>(pg) << 8 |
                //                static_cast<uint32_t>(pb);
                // point.rgb = *reinterpret_cast<float*>(&rgb);

                pclCloud->points.push_back(point);
            }

            std::cout << "pointcloud converted\n";

            // Publish pointcloud on topic
            pcl::toROSMsg(*pclCloud, cloud_msg);
            cloud_msg.header.frame_id = "camera_link";
            cloud_msg.header.stamp = ros::Time::now();
            cloud_pub.publish(cloud_msg);

            // Publish Depth frame on topic
            disp_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_disp_image).toImageMsg();
            depth_pub.publish(disp_msg);

            // Publish camera info msgs
            publishCamInfo(left_cam_info_pub, left_info, now);
            publishCamInfo(right_cam_info_pub, right_info, now);
            // r.sleep();
        }
        // <---- If the frame is valid we can display it

        std::cout << "frame ok\n";
        // ----> Keyboard handling
        
        ros::spinOnce();
        std::cout << "Spinning \n";
        int key = cv::waitKey(5);
        if (key == 'q' || key == 'Q') // Quit
            break;
        // <---- Keyboard handling
        std::cout << "ciaooooooooo\n";
#ifdef HAVE_OPENCV_VIZ
        // ----> Show Point Cloud
        cv::viz::WCloud cloudWidget( cloudMat, left_rect );
        cloudWidget.setRenderingProperty( cv::viz::POINT_SIZE, 1 );
        pc_viewer.showWidget( "Point Cloud", cloudWidget );
        pc_viewer.spinOnce(1);

        if(pc_viewer.wasStopped())
            break;
        // <---- Show Point Cloud
        std::cout << "shopow poitncloud\n";
#endif
    }

    return EXIT_SUCCESS;
}
