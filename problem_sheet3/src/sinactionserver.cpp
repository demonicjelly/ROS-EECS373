#include<problem_sheet3/sinactionserver.h>

SinActionServer::SinActionServer() :
	as_(nh, "sin_action", boost::bind(&SinActionServer::executeCB, this, _1), false)
	{
		ROS_INFO("Initialising");
		
		goal_.amplitude = 0;
		goal_.frequency = 0;
		goal_.cycles = 0;

		result_.output = 0;
	
		feedback_.fdbk = 0;


		as_.start();
	}

void SinActionServer::executeCB(const actionlib::SimpleActionServer<problem_sheet3::SinMsgAction>::GoalConstPtr& goal){

	ROS_INFO("Entering Callback");

	double amplitude = goal->amplitude;
	double frequency = goal->frequency;
	double cycles = goal-> cycles;	
	
	problem_sheet3::SinMsgResult result;
	
	ros::NodeHandle nh; // node handle 

//Set up publisher
	ros::Publisher vel_cmd_publisher = nh.advertise<std_msgs::Float64>("vel_cmd", 1);


//Define variables used in code and message sin_vel to be published
	std_msgs::Float64 sin_vel;

	double velocity;

	double dt = 0.01;
	double sample_rate = 1.0/dt;
	double t = 0;
	double period = 1/frequency;

	ros::Rate naptime(sample_rate);
	
	while(ros::ok()){
		
		if(cycles == 0){
			amplitude = 0;
			frequency = 0;
			velocity = 0; 
			sin_vel.data = velocity;
			vel_cmd_publisher.publish(sin_vel); //publish message

			result.output = true;
			as_.setSucceeded(result_);
			naptime.sleep();
			break;
		}
		
		//Calculate sinusoidal velocities
		velocity = amplitude * (2*3.14*frequency) * cos(2*3.14*frequency*t); 


		sin_vel.data = velocity; //put sin_vel double into the Float64 mesage to be published
		
		if(t >= period)
		{
			cycles--;
			t=0;
		}

		vel_cmd_publisher.publish(sin_vel); //publish message
		t+=dt;	 //increment time
		
		naptime.sleep(); 
    }
    
}


