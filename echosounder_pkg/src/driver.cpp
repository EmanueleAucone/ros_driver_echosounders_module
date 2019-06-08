#include "ros/ros.h"
#include <string>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <serial/serial.h>
#include <std_msgs/Empty.h>
#include "std_msgs/Float64.h"
#include <echosounder_pkg/DriverCommand.h>
#include <stdexcept> 

// Serial port global variable
serial::Serial ser;

// Publishers and subscribers
ros::Subscriber driver_sub_from_handler;
ros::Publisher driver_pub_to_handler;
ros::Publisher driver_range_pub;

// Global msg 
std_msgs::Float64 info_pub_msg;
std_msgs::Bool pub_handler_msg;

// Flags
bool flag_data = false;
bool sensor_on = false;

// Global variables
std::string driver, port, mode;
int N;

// Callback to receive start/stop command from handler
void setupCallback(const echosounder_pkg::DriverCommand::ConstPtr& sub_msg)
{
	if(sub_msg->flag)
	{
		std::cout << "\r\n\n\n\033[32m\033[1mDriver enabled to receive data! \033[0m" << std::endl;
		flag_data = true;
		mode = sub_msg->mode;
		N = sub_msg->n;
	}
	else if(!sub_msg->flag)
		flag_data = false;	
}

// Enable communication with the serial port
int enableCommunication()
{
	try
	{
		ser.setPort(port);
		ser.setBaudrate(9600);
		serial::Timeout tout = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(tout);
		if(!ser.isOpen())
			ser.open();
		return 1;
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}

	if(ser.isOpen())
	{
		ROS_INFO_STREAM("Serial Port initialized");
		return 1;
	}
	else
		return -1;
}

// Interrogate the sensor through serial port and acquire measurements
void readData()
{
	if(ser.available())
	{		
		ROS_INFO_STREAM("Reading from serial port");
		std_msgs::String result;
		// Read data from the sensor
		result.data = ser.read(ser.available());

		// echosounder is set to 3P2 mode (es. 025.50m)
		try
		{
		  	info_pub_msg.data = std::stod(result.data.substr(1,6));
		}
		
		catch(const std::invalid_argument& )
		{
			std::cout << "Read value not correct." << std::endl;
		  
		}
		catch(const std::out_of_range&)
		{
		  	std::cout << "Out of range exception." << std::endl;
		}
		  
		// Publish and print range values
		driver_range_pub.publish(info_pub_msg);
		
		if(info_pub_msg.data >= 50.00)
			std::cout << "Out-of-range: " << info_pub_msg.data << " meters" << std::endl;
		else
			std::cout << "Echosounder range data: " << info_pub_msg.data << " meters" << std::endl;
	}
}

// Main function
int main(int argc, char **argv)
{
	ros::init(argc, argv, "driver");
	ros::NodeHandle nh("~");

	// Initialize driver parameters
	nh.getParam("driver_name", driver);
	if(nh.getParam("serial_port", port))
		std::cout << "Found parameter: " << port << std::endl;
   	else
   	{
		// default value if not found
		port = "/dev/ttyS1";
		std::cout << "Parameter not found" << std::endl;
	}	
	
	ros::Rate loop(100);

	// err equal to 1 if the communication is established correctly
	int err = enableCommunication();

	// Subscribe to receive ON/OFF command from handler
	driver_sub_from_handler = nh.subscribe("/handler/command/" + driver, 10, &setupCallback);
	// Publish acknowledgement to handler
	driver_pub_to_handler = nh.advertise<std_msgs::Bool>("/" + driver + "/acknowledgement", 10);
	// Publish range values
	driver_range_pub = nh.advertise<std_msgs::Float64>("/" + driver + "/info/range", 10);
	
	while(ros::ok())
	{	
		int count = 0;
		if(flag_data)
		{
			if(err == 1)
			{
				// Written only the first time the echosounder is enabled
				std::cout << "Enabling echosounder!" << std::endl;
				err++;
		
				// Send back acknowledgement to the handler
				pub_handler_msg.data = true;
				driver_pub_to_handler.publish(pub_handler_msg);
				sleep(0.05);

				sensor_on = true;
			}
			
			// Acquistion mode set to seconds
			if(mode == "sec")
			{
				// Write 'Z' character on the serial port to interrogate the sensor
				ser.flush();
				ser.write("Z");

				// Wait for the availability of the port
				while(!ser.available())
					sleep(0.01);
	
				// Read range data from sensor
				readData();

				// Delay added for the security of the port buffer (a value less than 1 could induce measurement overlap in the port)
				sleep(1);	
			}
			// Acquistion mode set to samples
			else if(mode == "acq")
			{
				// Read N samples
				while(count < N)
				{
					// Write 'Z' character on the serial port to interrogate the sensor
					ser.flush();
					ser.write("Z");

					// Wait for the availability of the port
					while(!ser.available())
						sleep(0.01);
	
					// Read range data from sensor
					readData();

					// Delay added for the security of the port buffer (a value less than 1 could induce measurement overlap in the port)
					sleep(1);
	
					count++;
				}
			}
		}
		else
		{
			if(sensor_on)
			{
				// Send acknowledgement
				pub_handler_msg.data = false;
				driver_pub_to_handler.publish(pub_handler_msg);
				// Restore initial conditions to make the driver ready for next enabling
				err = 1;
				sensor_on = false;
			}
		}
		// Listen to the handler command  callback
		ros::spinOnce();	
		loop.sleep();       
	}
	return 0;
}
