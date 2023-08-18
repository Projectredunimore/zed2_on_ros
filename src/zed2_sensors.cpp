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
#include <zed2_on_ros/sensorcapture.hpp>
#include <zed2_on_ros/sensorcapture_def.hpp>
#include "ocv_display.hpp"

#include <iostream>
#include <iomanip>
#include <unistd.h> // for usleep

// OpenCV includes
#include <opencv2/opencv.hpp>


// ROS includes
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
//  <---- Includes


// The main function
int main(int argc, char *argv[])
{
    // ----> Silence unused warning
    (void)argc;
    (void)argv;
    // <---- Silence unused warning

    // ----> ROS initialization
    ros::init(argc, argv, "zed2_imu_node");
	ros::NodeHandle nh;
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 1000);
	ros::Publisher press_pub = nh.advertise<std_msgs::Float32>("/environment/pressure", 1000);
	ros::Publisher temp_pub = nh.advertise<std_msgs::Float32>("/environment/temperature", 1000);
	ros::Publisher hum_pub = nh.advertise<std_msgs::Float32>("/environment/humidity", 1000);
    // <---- ROS initialization

    // Set the verbose level
    sl_oc::VERBOSITY verbose = sl_oc::VERBOSITY::INFO;

    // Create a SensorCapture object
    sl_oc::sensors::SensorCapture sens(verbose);

    // ----> Get a list of available camera with sensor
    std::vector<int> devs = sens.getDeviceList();

    if( devs.size()==0 )
    {
        std::cerr << "No available ZED Mini or ZED2 cameras" << std::endl;
        return EXIT_FAILURE;
    }
    // <---- Get a list of available camera with sensor

    // ----> Inizialize the sensors
    if( !sens.initializeSensors( devs[0] ) )
    {
        std::cerr << "Connection failed" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Sensor Capture connected to camera sn: " << sens.getSerialNumber() << std::endl;
    // <---- Inizialize the sensors

    // ----> Get FW version information
    uint16_t fw_maior;
    uint16_t fw_minor;

    sens.getFirmwareVersion( fw_maior, fw_minor );

    std::cout << " * Firmware version: " << std::to_string(fw_maior) << "." << std::to_string(fw_minor) << std::endl;
    // <---- Get FW version information

    // ----> Variables to calculate sensors frequencies
    uint64_t last_imu_ts = 0;
    uint64_t last_mag_ts = 0;
    uint64_t last_env_ts = 0;
    uint64_t last_cam_temp_ts = 0;
    // <---- Variables to calculate sensors frequencies

    // Initialized ROS msgs
    sensor_msgs::Imu imu_msg;
    std_msgs::Float32 press_msg, temp_msg, hum_msg;

    while(ros::ok())
    {
        // ----> Get IMU data with a timeout of 5 millisecods
        const sl_oc::sensors::data::Imu imuData = sens.getLastIMUData(500);
        if( imuData.valid == sl_oc::sensors::data::Imu::NEW_VAL  )
        {
            std::cout << "**** New IMU data ****" << std::endl;
            std::cout << " * Timestamp: " << imuData.timestamp << " nsec" << std::endl;
            if(last_imu_ts!=0)
            {
                std::cout << " * Frequency: " << 1e9/static_cast<float>(imuData.timestamp-last_imu_ts) << " Hz" << std::endl;
            }
            last_imu_ts = imuData.timestamp;
            std::cout << " * Accelerations [m/s²]: " << -imuData.aZ << " " << -imuData.aY << " " << -imuData.aX << std::endl;
            std::cout << " * Angular Velocities [°/s]: " << imuData.gX << " " << imuData.gY << " " << imuData.gZ << std::endl;

            // Publish ROS msg
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "camera_link";
            imu_msg.linear_acceleration.x = -imuData.aZ;
            imu_msg.linear_acceleration.y = -imuData.aY;
            imu_msg.linear_acceleration.z = -imuData.aX; 
            imu_msg.angular_velocity.x = imuData.gX;
            imu_msg.angular_velocity.y = imuData.gY;
            imu_msg.angular_velocity.z = imuData.gZ;
            imu_pub.publish(imu_msg);
        }
        // <---- Get IMU data with a timeout of 5 millisecods

        // ----> Get Magnetometer data with a timeout of 100 microseconds to not slow down fastest data (IMU)
        const sl_oc::sensors::data::Magnetometer magData = sens.getLastMagnetometerData(100);
        if( magData.valid == sl_oc::sensors::data::Magnetometer::NEW_VAL )
        {
            std::cout << "**** New Magnetometer data ****" << std::endl;
            std::cout << " * Timestamp: " << magData.timestamp << " nsec" << std::endl;
            if(last_mag_ts!=0)
            {
                std::cout << " * Frequency: " << 1e9/static_cast<float>(magData.timestamp-last_mag_ts) << " Hz" << std::endl;
            }
            last_mag_ts = magData.timestamp;
            std::cout << " * Magnetic field [uT]: " << magData.mX << " " << magData.mY << " " << magData.mZ << std::endl;
        }
        // <---- Get Magnetometer data with a timeout of 100 microseconds to not slow down fastest data (IMU)

        // ----> Get Environment data with a timeout of 100 microseconds to not slow down fastest data (IMU)
        const sl_oc::sensors::data::Environment envData = sens.getLastEnvironmentData(100);
        if( envData.valid == sl_oc::sensors::data::Environment::NEW_VAL )
        {
            std::cout << "**** New Environment data ****" << std::endl;
            std::cout << " * Timestamp: " << envData.timestamp << " nsec" << std::endl;
            if(last_env_ts!=0)
            {
                std::cout << " * Frequency: " << 1e9/static_cast<float>(envData.timestamp-last_env_ts) << " Hz" << std::endl;
            }
            last_env_ts = envData.timestamp;
            std::cout << " * Pressure [hPa]: " << envData.press << std::endl;
            std::cout << " * Temperature [°C]: " << envData.temp << std::endl;
            std::cout << " * Relative Humidity [%rH]: " << envData.humid << std::endl;

            // Publish ROS msgs
            press_msg.data = envData.press;
            temp_msg.data = envData.temp;
            hum_msg.data = envData.humid;
            press_pub.publish(press_msg);
            temp_pub.publish(temp_msg);
            hum_pub.publish(hum_msg);
        }
        // <---- Get Environment data with a timeout of 100 microseconds to not slow down fastest data (IMU)

        // ----> Get Temperature data with a timeout of 100 microseconds to not slow down fastest data (IMU)
        const sl_oc::sensors::data::Temperature tempData = sens.getLastCameraTemperatureData(100);
        if( tempData.valid == sl_oc::sensors::data::Temperature::NEW_VAL )
        {
            std::cout << "**** New Camera Sensors Temperature data ****" << std::endl;
            std::cout << " * Timestamp: " << tempData.timestamp << " nsec" << std::endl;
            if(last_cam_temp_ts!=0)
            {
                std::cout << " * Frequency: " << 1e9/static_cast<float>(tempData.timestamp-last_cam_temp_ts) << " Hz" << std::endl;
            }
            last_cam_temp_ts = tempData.timestamp;
            std::cout << " * Left Camera [°C]: " << tempData.temp_left << std::endl;
            std::cout << " * Right Camera [°C]: " << tempData.temp_right << std::endl;
        }
        // <---- Get Temperature data with a timeout of 100 microseconds to not slow down fastest data (IMU)
    }

    return EXIT_SUCCESS;
}
