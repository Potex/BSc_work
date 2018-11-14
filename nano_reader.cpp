#include "ros/ros.h"
#include <stdio.h>
#include "SerialPort.h"
#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <cstdlib>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
//#include <geometry_msgs/Quaternion.h>
#include <sstream>
#include <iomanip>

#define _USE_MATH_DEFINES

const double Conv = 1.000000;
float cov_base = 0.00;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "NANO_reader");
	ros::NodeHandle n;
	ros::Publisher joint = n.advertise <sensor_msgs::JointState> ("joint_states", 1000);
	ros::Publisher assn = n.advertise <sensor_msgs::Imu> ("assn", 1000);
	//ros::Publisher quat = n.advertise <geometry_msgs::Quaternion> ("quat", 1000);
	ros::Rate loop_rate(50);

	// ROS message init, and message fill up.
	sensor_msgs::Imu assn_pub;
	assn_pub.header.frame_id = "imu";

	sensor_msgs::JointState joint_pub;

	joint_pub.name.resize(3);
	joint_pub.position.resize(3);
	joint_pub.velocity.resize(3);

	joint_pub.name[0] = "front_axis_to_left_front_steer";
	joint_pub.name[1] = "front_axis_to_right_front_steer";
	joint_pub.name[2] = "rear_axis_to_left_rear_wheel";

	try
	{
		//Open serial port with defined parameters
		ROS_INFO("Opening serial connection.");
		SerialPort serial_port ("/dev/ttyUSB0");
		serial_port.Open(SerialPort::BAUD_115200,
						 SerialPort::CHAR_SIZE_8, 
						 SerialPort::PARITY_NONE, 
						 SerialPort::STOP_BITS_1, 
						 SerialPort::FLOW_CONTROL_HARD);
		
		
		// Enter the main loop
		while (ros::ok())
		{
			// Wait until the data is available
			if(serial_port.IsDataAvailable())
		    {
				// Define string to read in data
				std::string ss;
				
				// Reading in from serial port
				ss = serial_port.ReadLine();

				std::istringstream iss(ss);

				// Create a vector of strings separated by "whitespace"
				std::vector<std::string> results(std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>());
				//std::cout << results[10].c_str() << '\t' << results[11].c_str() << '\t' << results[12].c_str() << '\t' << results[13].c_str() << std::endl;

				//Print all IMU data to console as a string
				float QX = Conv*atof(results[0].c_str());
				//std::cout << QX	<< '\t';
				float QY = Conv*atof(results[1].c_str());
				//std::cout << QY	<< '\t';
				float QZ = Conv*atof(results[2].c_str());
				//std::cout << QZ	<< '\t';
				float QW = Conv*atof(results[3].c_str());
				//std::cout << QW	<< '\t';
				float GX = Conv*atof(results[4].c_str());
				//std::cout << LX	<< '\t';
				float GY = Conv*atof(results[5].c_str());
				//std::cout << LY	<< '\t';
				float GZ = Conv*atof(results[6].c_str());
				//std::cout << LZ	<< '\t';
				float LX = Conv*atof(results[7].c_str());
				//std::cout << GX	<< '\t';
				float LY = Conv*atof(results[8].c_str());
				//std::cout << GY	<< '\t';
				float LZ = Conv*atof(results[9].c_str());
				//std::cout << GZ << '\t';
				int angWheel = Conv*atoi(results[10].c_str());
				//std::cout << angWheel << '\t';
				float velo = Conv*atof(results[11].c_str());
				//std::cout << velo << '\t';
				int rpm = Conv*atoi(results[12].c_str());
				//std::cout << rpm << '\t';
				float dist = Conv*atof(results[13].c_str());
				//std::cout << dist << std::endl;

				//std::cout << angWheel << '\t' << velo << '\t' << rpm << '\t' << dist << std::endl;
				
				assn_pub.header.stamp = ros::Time::now();
				joint_pub.header.stamp = ros::Time::now();
				
				// Orientation-Quaterninon data passed to message							// IMU MESSAGE START
				assn_pub.orientation.x = QX;				
				assn_pub.orientation.y = QY;				
				assn_pub.orientation.z = QZ;
				assn_pub.orientation.w = QW;

				// Covariance matrix filled up with zeros
				std::fill( assn_pub.orientation_covariance.begin(), assn_pub.orientation_covariance.end(),  cov_base );

				// 3 axis Gyroscope data passed to message
				assn_pub.angular_velocity.x = GX;
				assn_pub.angular_velocity.y = GY;
				assn_pub.angular_velocity.z = GZ;

				// Covariance matrix filled up with zeros
				std::fill( assn_pub.angular_velocity_covariance.begin(), assn_pub.angular_velocity_covariance.end(),  cov_base );

				// 3 axis Linear acceleration data passed to message
				assn_pub.linear_acceleration.x = LX;
				assn_pub.linear_acceleration.y = LY;
				assn_pub.linear_acceleration.z = LZ;

				// Covariance matrix filled up with zeros						
				std::fill( assn_pub.linear_acceleration_covariance.begin(), assn_pub.linear_acceleration_covariance.end(),  cov_base );
				
				// Fill the JointState message											IMU MESSAGE END   -------	JOINTSTATE MESSAGE START
 				joint_pub.position[0] = (angWheel * M_PI) / 180;
				joint_pub.position[1] = (angWheel * M_PI) / 180;
				joint_pub.position[2] = dist;
				joint_pub.velocity[0] = (rpm / 9.55);
				joint_pub.velocity[1] = (rpm / 9.55);
				joint_pub.velocity[2] = (rpm / 9.55);


				// Publish messages 																JOINTSTATE MESSAGE END
				assn.publish(assn_pub);
				joint.publish(joint_pub);

			}

			ros::spinOnce();

			loop_rate.sleep();

		}
		
		// Close connection
		if(serial_port.IsOpen())
		{
			ROS_INFO("Closing serial connection.");
			serial_port.Close();
		}

		ROS_INFO("Serial node closed.");
		return 0;
	}
	catch (SerialPort::OpenFailed exception)
	{
		ROS_ERROR("Failed to open the port");
	}

	catch (SerialPort::AlreadyOpen exception)
	{
		ROS_ERROR("Port is already open");
}
}
