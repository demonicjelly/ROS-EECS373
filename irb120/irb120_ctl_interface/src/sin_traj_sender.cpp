//sin_traj_sender.cpp 
//
//A couple references:
//test_traj_sin_sender_irb120.cpp: wsn, Oct 2018
//test_traj_sender_irb120.cpp: wsn, Dec 2017


//Include various libraries
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <cmath>

#include <ros/ros.h> 

//And include some message types
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <trajectory_msgs/JointTrajectory.h>
//#include <trajectory_msgs/JointTrajectoryPoint.h>


int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "sin_traj_sender"); //node name

    ros::NodeHandle nh; // create a node handle
    
    ros::Publisher joint_publish = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);  
    
    //Trajectory messages
    trajectory_msgs::JointTrajectory new_trajectory;
    trajectory_msgs::JointTrajectoryPoint trajectory_point_home; 
    trajectory_msgs::JointTrajectoryPoint trajectory_points;

    
    new_trajectory.points.clear(); //fill in all the joint names
    new_trajectory.joint_names.push_back("joint1");
    new_trajectory.joint_names.push_back("joint2");
    new_trajectory.joint_names.push_back("joint3");
    new_trajectory.joint_names.push_back("joint4");
    new_trajectory.joint_names.push_back("joint5");
    new_trajectory.joint_names.push_back("joint6");    

    ros::Rate naptime(1.0); //1Hz update rate
    

    // clear any previous trajectory positions
    trajectory_point_home.positions.clear();
    trajectory_points.positions.clear();    
    
    
    //Set home point to 0 and initisialise other trajectory point to 0
    for (int ijnt=0;ijnt<6;ijnt++) {
        trajectory_point_home.positions.push_back(0.0); 
        trajectory_points.positions.push_back(0.0); 

    }
   
    trajectory_point_home.time_from_start = ros::Duration(1.0); //set 1 second arrival time
    
    //Set home position as first point in the trajectory message.
    new_trajectory.points.clear();
    new_trajectory.header.stamp = ros::Time::now();
    new_trajectory.points.push_back(trajectory_point_home); 
 
    ROS_INFO("Moving to home position: ");
    int npts = new_trajectory.points.size();
    ROS_INFO("Initial trajectory points = %d",npts);
    for (int i=0;i<2;i++) //Send multiple times and with a sleep period to ensure contact with subscriber
    {
        joint_publish.publish(new_trajectory);
        ros::spinOnce();
        naptime.sleep();
    }
    double a1 = 1.0; //Individual amplitudes for each joint
    double a2 = 0.6;
    double a3 = 0.6;
    double a4 = 0.6;
    double a5 = 0.7;
    double a6 = 0.5;
    double freq = 1.0;
    double dt = 0.1; //Time step
    double Tfinal = 20.0; //Run for this amount of time
    double t=0;		//initliase time to 0
    int n_traj_pts = Tfinal/dt; //Number of trajectory points from the time and time steps.
    
    new_trajectory.points.clear(); //Clear trajectory points message

    for (int ipt =0;ipt<n_traj_pts;ipt++)//Iterate for the number fo trajectory points there are
    {     
	//Calulate trajectory points using sin function
	trajectory_points.positions[0] = -a1*sin(freq*t);
	trajectory_points.positions[1] = abs(a2*sin(freq*t));
	trajectory_points.positions[2] = a3*sin(freq*t)*sin(freq*t);
	trajectory_points.positions[3] = -a4*sin(freq*t);
	trajectory_points.positions[4] = -a5*sin(freq*t);
	trajectory_points.positions[5] = -a6*sin(freq*t);

	//increment time and calcule time taken
	trajectory_points.time_from_start = ros::Duration(t); 
	t+=dt;
                 
	//Add trajectories to the trajectory message
	new_trajectory.points.push_back(trajectory_points);   
    }

	//Add time stamp to trajectory message
    new_trajectory.header.stamp = ros::Time::now();


     npts = new_trajectory.points.size(); //Find number of trajectory points
     int njnts = new_trajectory.points[0].positions.size(); //Find number of joints
    ROS_INFO("sending %d trajectory points , each with %d joints ",npts,njnts); 
    joint_publish.publish(new_trajectory); //Publish trajectories

    //Run naptime multiple times to ensure trajectories are published
    for (int i=0;i<3;i++) 
    {
	naptime.sleep();
    }

    return 0;
}

