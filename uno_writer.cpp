#include "ros/ros.h"
#include <stdio.h>
#include <algorithm>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <cstdlib>
#include <termios.h>
#include <fcntl.h>
#include "std_msgs/String.h"
#include <sstream>
#include <iomanip>

int open_port(void);

// Declare file descriptor for the port.
int fd;

int len = 10;
std::string send ("");
float angRef = 1.80;
float torqueRef = 0.30;
int wsuc = 0;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "UNO_reader");
	ros::NodeHandle n;
	ros::Publisher uno = n.advertise<std_msgs::String> ("uno", 1000);
	// Under 40 Hz : system has much latency --> SET >= 40
	ros::Rate loop_rate(40);

	try
	{
		// Open serial port with defined parameters
		ROS_INFO("Opening serial connection.");
		open_port();
		//std::cout << fd << std::endl;
		ros::Duration(2).sleep();
		
		// Enter the main loop
		while (ros::ok())
		{
			// -------WRITING SEQUENCE ----START--------
			
			std::stringstream stream;
			stream << std::fixed << std::setprecision(2) << torqueRef << "," << angRef << "a"; 
			send = stream.str();
			wsuc = write(fd, send.c_str(), len);
			
			//std::cout << send << std::endl;
			// --------WRITING SEQUENCE -------END-------
			ros::spinOnce();
			loop_rate.sleep();
		}

	}
	catch (std::exception)
	{
		ROS_ERROR("Failed to open the port");
	}
	
close( fd );

}

// ------------PORT OPENING FUNCTION -------------------------
int open_port()
{
  	// Open port
	fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
  	if (fd == -1)
    {
   		// ERROR: Could not open port!
		std::cout << "open_port: Unable to open /dev/ttyACM0" << std::endl;
    }
  	else
	{
    	fcntl(fd, F_SETFL, FNDELAY);
		// Port opened successfully!
  		std::cout << "In Open port fd = " << fd << std::endl;
	}
	// Read the configureation of the port.
	struct termios options_2;
	tcgetattr( fd, &options_2 );
	// Set I/O speed of the port.
	cfsetispeed( &options_2, B115200 );
	cfsetospeed( &options_2, B115200 );
	
	// Enable the receiver and set local mode.
	options_2.c_cflag |= ( CLOCAL | CREAD );
	// Set the new options for the port.
	tcsetattr(fd, TCSAFLUSH, &options_2);

	// Set the Charactor size: Mask the character size bits
	options_2.c_cflag &= ~CSIZE;
	// Select 8 data bits
	options_2.c_cflag |= CS8;
  
	// Set parity - No Parity (8N1)
	options_2.c_cflag &= ~PARENB;
	options_2.c_cflag &= ~CSTOPB;
	options_2.c_cflag &= ~CSIZE;
	options_2.c_cflag |= CS8;
	
	// Disable Software Flow control
	options_2.c_iflag &= ~(IXON | IXOFF | IXANY);
	
	// Chose raw (not processed) output
	options_2.c_oflag &= ~OPOST;
	if ( tcsetattr( fd, TCSANOW, &options_2 ) == -1 ){
		// ERROR: set tcsetattr not successfull!
		std::cout << "Error with setting attributions." << std::endl;
	}
	else{
		// tcsetattr OK!
		std::cout << "Attributes settings succeeded" << std::endl;
	}
	usleep(1500);
	tcflush(fd,TCIOFLUSH);		// Flush I/O queue.
	
	return(fd);
}
// ----PORT OPENING FUNCTION -------------END------------
