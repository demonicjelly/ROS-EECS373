#include<ros/ros.h> 
#include<std_msgs/Float64.h> 

#include<problem_sheet3/SinMsg.h>

//using namespace std;

void doneCb(const actionlib::SimpleClientFoalState& state, const sinactionserver::sinWaveMsgResultConstPtr& result){
	//THIS ISNT ESSENTIAL, can be called when goal completes 
	
}



int main(int argc, char **argv) {
	ros::init(argc, argv, "sin_srv_client"); //name this node  
	ros::NodeHandle nh; // node handle 
	
	int g_count = 0;
	
	actionlib::SimpleActionClient<problem_sheet3::sinWaveMsgAction> action_client("sin_action", true);
	
	// attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    
	if (!server_exists) 
	{
        ROS_WARN("could not connect to server; halting");
        return 0; // bail out; optionally, could print a warning message and retry
    }
	
    ROS_INFO("connected to action server"); // if here, then we connected to the server;
	
	int amplitude;
	int frequency;
	int cycles
	
	while (ros::ok()){
		std::cout << std::endl;
		std::cout << "Enter the desired sin wave amplitude: ";
		std::cin >> amplitude;

		std::cout << std::endl << "Enter the desired sin wave frequency: ";
		std::cin >> frequency;
		
		std::cout << std::endl << "Enter the number of cycles: ";
		std::cin >> cycles;
		
		goal.amplitude = amplitude;
		goal.frequency = frequency;
		goal.cycles = cycles;
		
		
		action_client.sendGoal(goal, &doneCb);
		
		ros::Time begin = ros::Time::now();

        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (result){
            //if here, then server returned a result to us
            
            ros::Time end = ros::Time::now();
            
            double timer = end.toSec() - begin.toSec();
            
            ROS_INFO("Server returned result after %d seconds", timer);
            
        }
		

	}
	return 0;
}
