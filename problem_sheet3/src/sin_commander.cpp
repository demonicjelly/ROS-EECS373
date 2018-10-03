#include<ros/ros.h> 
#include<std_msgs/Float64.h> 

#include<problem_sheet3/SinMsgAction.h>

#include<problem_sheet3/sinactionserver.h>

/*
int g_amplitude;
int g_frequency;
*/
/*
bool callback(problem_sheet2::SinWaveMsgRequest& request, problem_sheet2::SinWaveMsgResponse& response)
{
	ROS_INFO("callback activated");
	
	g_amplitude = request.amplitude;
	g_frequency =request.frequency;
	
	response.recieved = true;
	
	//ROS_INFO("TEST FOR CALLBACK");

	return true;
}
*/



int main(int argc, char **argv) {
	ros::init(argc, argv, "sin_comm"); //name this node  
	ros::NodeHandle nh; // node handle 

	ROS_INFO("instantiating the demo action server: ");

	SinActionServer as_object;

/*
//Set up publisher
	ros::Publisher vel_cmd_publisher = nh.advertise<std_msgs::Float64>("vel_cmd", 1);

//Define variables used in code and message sin_vel to be published
	std_msgs::Float64 sin_vel;

	double amplitude;
	double frequency;

	double velocity;

	double dt = 0.01;
	double sample_rate = 1.0/dt;
	double t = 0;

	ros::Rate naptime(sample_rate);
*/

/*
//Ask for user inputs
	ROS_INFO("Input Amplitude");
	std::cin >> amplitude;

	ROS_INFO("Input Frequency");
	std::cin >> frequency;
*/


	

	while(ros::ok()){

	ros::spinOnce();
/*
	//Calculate sinusoidal velocities
	velocity = g_amplitude * (2*3.14*g_frequency) * cos(2*3.14*g_frequency*t); 

		sin_vel.data = velocity; //put sin_vel double into the Float64 mesage to be published
		vel_cmd_publisher.publish(sin_vel); //publish message
		t+=dt;	 //increment time
		naptime.sleep(); 
*/
    }
}
