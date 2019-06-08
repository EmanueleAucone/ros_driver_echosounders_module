#include "ros/ros.h"
#include <string>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include <math.h>
#include <echosounder_pkg/EchoCommand.h>
#include <echosounder_pkg/DriverCommand.h>
#include <echosounder_pkg/SeaBottom.h>

// Publishers and subscribers
ros::Subscriber handler_sub_from_driver1;
ros::Publisher handler_pub_to_driver1;
ros::Subscriber handler_sub_from_driver2;
ros::Publisher handler_pub_to_driver2;
ros::Subscriber handler_high_level_command;
ros::Subscriber handler_sub_range1;
ros::Subscriber handler_sub_range2;
ros::Publisher handler_pub_slope;

// Command to the driver
echosounder_pkg::DriverCommand pub_msg;

// Flags
bool flag_ack = false;
bool completed = false;
bool cmd_received = false;

// Variables for the sea bottom parameters computation
float THETA;	
float range1,range2;
echosounder_pkg::SeaBottom sea_msg;

// Global variable obtained reading high level command
struct command {
	std::string echo1;
	std::string mode1;
	int N1;
	float K1;
	std::string echo2;
	std::string mode2;
	int N2;
	float K2;
	bool repeat;
} ;
command cmd;

// Callback to receive acknoledgement from the drivers
void handlerCallback(const std_msgs::Bool& sub_msg)
{
	if(sub_msg.data)
	{
		flag_ack = true;
		std::cout << "\r\n\n\n\033[32m\033[1mAcknowledgement received: the driver is acquiring from the sensor! \033[0m" << std::endl;
	}
	else
	{
		std::cout << "\r\n\n\n\033[32m\033[1mAcknowledgement received: the driver is not acquiring from the sensor! \033[0m" << std::endl;
		flag_ack = false;
	}
}

// Callback to receive range values from driver1
void rangeCallback1(const std_msgs::Float64& sub_msg)
{
	range1=sub_msg.data;
}

// Callback to receive range values from driver2
void rangeCallback2(const std_msgs::Float64& sub_msg){
	range2=sub_msg.data;
}

// Callabck to receive high level command to start the operation functionalities
void commandCallback(const echosounder_pkg::EchoCommand::ConstPtr& cmd_msg)
{
	//read from the high level command msg and check if the parameters are set correctly
	if(cmd_msg->echo1 == "echo1" || cmd_msg->echo1 == "echo2")
		cmd.echo1 = cmd_msg->echo1;
	else
	{
		std::cout << "Error while inserting primary echosounder name..." << std::endl;
		return;
	}
	if(cmd_msg->mode1 == "sec" || cmd_msg->mode1 == "acq")
		cmd.mode1 = cmd_msg->mode1;
	else
	{
		std::cout << "Error while inserting primary echosounder acquisition mode..." << std::endl;
		return;
	}
	if(cmd_msg->N1 >= 0)
		cmd.N1 = cmd_msg->N1;
	else
	{
		std::cout << "Error while inserting primary echosounder acquisition time..." << std::endl;
		return;
	}
	if(cmd_msg->K1 >= 0)
		cmd.K1 = cmd_msg->K1;
	else
	{
		std::cout << "Error while inserting primary echosounder time delay..." << std::endl;
		return;
	}
	if(cmd_msg->echo2 == "echo1" || cmd_msg->echo2 == "echo2")
		cmd.echo2 = cmd_msg->echo2;
	else
	{
		std::cout << "Error while inserting secondary echosounder time delay..." << std::endl;
		return;
	}
	if(cmd_msg->mode2 == "sec" || cmd_msg->mode2 == "acq")
		cmd.mode2 = cmd_msg->mode2;
	else
	{
		std::cout << "Error while inserting secondary echosounder acquisition mode..." << std::endl;
		return;
	}
	if(cmd_msg->N2 >= 0)
		cmd.N2 = cmd_msg->N2;
	else
	{
		std::cout << "Error while inserting secondary echosounder acquisition time..." << std::endl;
		return;
	}
	if(cmd_msg->K2 >= 0)
		cmd.K2 = cmd_msg->K2;
	else
	{
		std::cout << "Error while inserting secondary echosounder time delay..." << std::endl;
		return;
	}
	if(cmd.repeat == true || cmd.repeat == false)
		cmd.repeat = cmd_msg->repeat;
	else
	{
		std::cout << "Error while inserting repeating mode..." << std::endl;
		return;
	}
	
	std::cout << "\r\n\n\n\033[32m\033[1mHigh Level Command received!\033[0m" << std::endl;
	cmd_received = true;
}

// Enable the generic driver in order to acquire data
void enableDriver(ros::Publisher handler_pub, int i, std::string mode, int N)
{
	// Driver enabled only if the number of seconds/samples is greater than 0
	if(N > 0)
	{
		if(mode == "sec")
		{	
			// Acquisition functionality for N seconds
			std::cout << "\r\n\n\n\033[32m\033[1mStart driver \033[0m" << i << std::endl;
			pub_msg.flag = true;
			pub_msg.mode = mode;
			pub_msg.n = N;
			handler_pub.publish(pub_msg);

			// Wait for acknowledgement
			while(!flag_ack)         
				ros::spinOnce();

			std::cout << "Acquisition for " << N << " seconds!" << std::endl;
			// The handler sleeps and the driver acquire measurements
			sleep(N);
		}
		else if(mode == "acq")
		{
			// Acquisition functionality for N samples
			std::cout << "\r\n\n\n\033[32m\033[1mStart driver \033[0m" << i << std::endl;
			pub_msg.flag = true;
			pub_msg.mode = mode;
			pub_msg.n = N;
			handler_pub.publish(pub_msg);

			// Wait for acknowledgement
			while(!flag_ack)         
				ros::spinOnce();

			std::cout << "Acquisition of " << N << " samples!" << std::endl;
			// the driver itself handles the correct number of acquisition
		}

		// Disable the driver
		pub_msg.flag = false;
		handler_pub.publish(pub_msg);
		
		// Wait for acknowledgement
		while(flag_ack)         
			ros::spinOnce();
	
		std::cout << "\r\n\n\n\033[32m\033[1mStop driver \033[0m" << i << std::endl;
	}
}

// Main function
int main(int argc, char **argv)
{
	ros::init(argc, argv, "handler");
	ros::NodeHandle nh("~");

	// Read from the launch file the geometric parameter
	if(nh.getParam("theta", THETA))
		ROS_INFO("Found parameter: %s", THETA);
	THETA = (M_PI/180)*THETA;	// from deg to rad

	ros::Rate loop(100);

	// Subscribe to the callback to receive ON/OFF command from drivers
	handler_sub_from_driver1 = nh.subscribe("/driver1/acknowledgement", 1, &handlerCallback);
	handler_sub_from_driver2 = nh.subscribe("/driver2/acknowledgement", 1, &handlerCallback);

	// Subscribe to the callabck to receive high level command
	handler_high_level_command = nh.subscribe("/handler/high_level_command", 1, &commandCallback);

	// Subscribe to range related topics in order to compute sea-bottom slope
	handler_sub_range1= nh.subscribe("/driver1/info/range", 1 ,&rangeCallback1);
	handler_sub_range2= nh.subscribe("/driver2/info/range", 1 ,&rangeCallback2);
	
	//Publish slope computation parameters on a specific topic
	handler_pub_slope = nh.advertise<echosounder_pkg::SeaBottom>("/handler/sea_bottom", 10);
	
	// Publish commands to the drivers
	handler_pub_to_driver1 = nh.advertise<echosounder_pkg::DriverCommand>("/handler/command/driver1", 10);
	handler_pub_to_driver2 = nh.advertise<echosounder_pkg::DriverCommand>("/handler/command/driver2", 10);
		
	std::cout << "\r\n\n\n\033[32m\033[1mSend high level command to start! \033[0m" << std::endl;
	sleep(1);

	while(ros::ok())
	{
		if(cmd_received == true)
		{
			cmd_received = false;
			// Handling of the different scenarios
			do
			{
				if(cmd.echo1 == "echo1" && cmd.echo2 == "echo2")
				{
					// Enable echo 1
					enableDriver(handler_pub_to_driver1, 1, cmd.mode1, cmd.N1);
					if(cmd.N1 > 0 && cmd.K1 > 0)
					{
						// Wait for K1 seconds if the delay is set by the high level command
						std::cout << "Wait for " << cmd.K1 << "seconds" << std::endl;
						sleep(cmd.K1);
					}
					// Enable echo 2
					enableDriver(handler_pub_to_driver2, 2, cmd.mode2, cmd.N2);
					if(cmd.N2 > 0 && cmd.K2 > 0)
					{
						// Wait for K2 seconds if the delay is set by the high level command
						std::cout << "Wait for " << cmd.K2 << "seconds" << std::endl;
						sleep(cmd.K2);
					}
					
					// Compute sea-bottom profile if the number of acquistions was respectively 1 and 1
					if(cmd.N1 == 1 && cmd.N2 == 1)
					{						
						// Receive range values
						ros::spinOnce();  
						// Slope computation
						sea_msg.sigma = (180/M_PI)*atan2(range2*cos(THETA)-range1,range2*sin(THETA));
						// Altitude computation
						sea_msg.h= range1*cos((M_PI/180)*sea_msg.sigma);

						handler_pub_slope.publish(sea_msg);
					 
						// Print sea-bottom parameters on terminal for a direct visualization of them
						std::cout << "Sea-bottom slope: "    << sea_msg.sigma << std::endl;
						std::cout << "Sea-bottom altitude: " << sea_msg.h << std::endl;
					}
				}
				else if(cmd.echo1 == "echo2" && cmd.echo2 == "echo1")
				{
					// Enable echo 2
					enableDriver(handler_pub_to_driver2, 2, cmd.mode1, cmd.N1);
					if(cmd.N1 > 0 && cmd.K1 > 0)
					{
						// Wait for K1 seconds if the delay is set by the high level command
						std::cout << "Wait for " << cmd.K1 << "seconds" << std::endl;
						sleep(cmd.K1);
					}
					// Enable echo 1
					enableDriver(handler_pub_to_driver1, 1, cmd.mode2, cmd.N2);
					if(cmd.N2 > 0 && cmd.K2 > 0)
					{
						// Wait for K2 seconds if the delay is set by the high level command
						std::cout << "Wait for " << cmd.K2 << "seconds" << std::endl;
						sleep(cmd.K2);
					}

					// Compute sea-bottom profile if the number of acquistions was respectively 1 and 1
					if(cmd.N1 == 1 && cmd.N2 == 1)
					{
						// Receive range values
						ros::spinOnce();  
						// Slope computation
						sea_msg.sigma = (180/M_PI)*atan2(range1*cos(THETA)-range2,range1*sin(THETA));
						// Altitude computation
						sea_msg.h= range2*cos((M_PI/180)*sea_msg.sigma);
						
						handler_pub_slope.publish(sea_msg);
					 
						// Print sea-bottom parameters on terminal for a direct visualization of them
						std::cout << "Sea-bottom slope: "    << sea_msg.sigma << std::endl;
					  	std::cout << "Sea-bottom altitude: " << sea_msg.h << std::endl;
					  
					}
				}
				ros::spinOnce();
				loop.sleep();	
			} while(cmd.repeat);
		}
		// Listen to high level command callback
		ros::spinOnce();
		loop.sleep();
	}
	return 0;
}
