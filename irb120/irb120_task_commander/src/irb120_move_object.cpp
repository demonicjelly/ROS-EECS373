//irb120_move_object.cpp

//this version is a variation on irb120_reactive_task_commander, but allows the object to
//be moved to a desired location

#include <ros/ros.h>

#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace std; // avoids having to say: std::string, std::cout, etc
#include <irb120_fk_ik/irb120_kinematics.h>  //access to forward and inverse kinematics
#include <fk_ik_virtual/fk_ik_virtual.h> //defines the base class with virtual fncs
// this is useful to keep the motion planner generic
#include "robot_specific_fk_ik_mappings.h" //these two files are needed to provide robot-specific info to generic planner
#include "robot_specific_names.h"

#include <generic_cartesian_planner/generic_cartesian_planner.h>
#include <cartesian_interpolator/cartesian_interpolator.h>

#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include<sensor_msgs/JointState.h>

//add these to use the "magic" object finder action server
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <magic_object_finder/magicObjectFinderAction.h>

//the following will be useful when need tool transforms
//#include <tf/transform_listener.h>
//#include <xform_utils/xform_utils.h>

//XformUtils xformUtils; //handy conversion utilities--but don't need these yet



//some magic numbers--place at the top of the program
std::vector<double> g_planner_joint_weights{3, 3, 2, 1, 1, 0.5}; //specify weights to use for planner optimization

//another magic value: hard-coded name of object of interest
string g_object_name("gear_part_ariac");  //hard-coded object name; edit this for different objects
int g_found_object_code; //global to communicate between callback and main: true if named object was found
geometry_msgs::PoseStamped g_perceived_object_pose; //global to communicate between callback and main: pose  of found object

ros::Publisher *g_pose_publisher; //make this global so callback can access it--for displaying object frames in rviz

CartTrajPlanner *pCartTrajPlanner; //does  not  have to be global, unless needed by other functions

//arm pose in joint space; the only reason this is global is that it will be useful, in the future, for it
//to be updated by a subscriber to joint_states
Eigen::VectorXd g_q_vec_arm_Xd;

//a utility for debugging: displays affines (origins only) from an std::vector of affines
void print_affines(std::vector<Eigen::Affine3d> affine_path) {
    int npts = affine_path.size();
    ROS_INFO("affine path, origins only: ");
    for (int i = 0; i < npts; i++) {
        cout << affine_path[i].translation().transpose() << endl;
    }
}

//a utility for debugging: displays a trajectory message
void print_traj(trajectory_msgs::JointTrajectory des_trajectory) {
    int npts = des_trajectory.points.size();
    int njnts = des_trajectory.points[0].positions.size();
    //Eigen::VectorXd jspace_pt;
    //jspace_pt.resize(njnts);
    ROS_INFO("traj points: ");
    for (int i = 0; i < npts; i++) {
        //jspace_pt=optimal_path[i];
        for (int j = 0; j < njnts; j++) {
            cout << (des_trajectory.points[i]).positions[j] << ", ";
        }
        cout << "; t = " << des_trajectory.points[i].time_from_start.toSec() << endl;

    }
}

//this callback function receives a result from the magic object finder action server
//it sets g_found_object_code to true or false, depending on whether the  object was found
//if the object was found, then components of g_perceived_object_pose are filled in
void objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
        const magic_object_finder::magicObjectFinderResultConstPtr& result) {
    ROS_INFO(" objectFinderDoneCb: server responded with state [%s]", state.toString().c_str());
    g_found_object_code=result->found_object_code;
    ROS_INFO("got object code response = %d; ",g_found_object_code);
    if (g_found_object_code==magic_object_finder::magicObjectFinderResult::OBJECT_NOT_FOUND) {
        ROS_WARN("object-finder responded: object not found");
    }
    else if (g_found_object_code==magic_object_finder::magicObjectFinderResult::OBJECT_FOUND) {
        ROS_INFO("found object!");
         g_perceived_object_pose= result->object_pose;
         ROS_INFO("got pose x,y,z = %f, %f, %f",g_perceived_object_pose.pose.position.x,
                 g_perceived_object_pose.pose.position.y,
                 g_perceived_object_pose.pose.position.z);

         ROS_INFO("got quaternion x,y,z, w = %f, %f, %f, %f",g_perceived_object_pose.pose.orientation.x,
                 g_perceived_object_pose.pose.orientation.y,
                 g_perceived_object_pose.pose.orientation.z,
                 g_perceived_object_pose.pose.orientation.w);
         g_pose_publisher->publish(g_perceived_object_pose);  //this is to enable display of pose of found object in rviz
    }
    else {
        ROS_WARN("object not found!");
    }
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "task_commander"); // name this node 
    ros::NodeHandle nh; //standard ros node handle   
    Eigen::Affine3d start_flange_affine, goal_flange_affine; //specify start and goal in Cartesian coords
    std::vector<Eigen::VectorXd> optimal_path; //a path in joint space is a sequence of 6-DOF joint-angle specifications
    trajectory_msgs::JointTrajectory new_trajectory; //will package trajectory messages here


    //set up an action client to query object poses using the magic object finder
    actionlib::SimpleActionClient<magic_object_finder::magicObjectFinderAction> object_finder_ac("object_finder_action_service", true);
    bool finished_before_timeout=false; 
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_finder_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_finder action server"); // if here, then we connected to the server; 
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true); 
    g_pose_publisher = &pose_publisher;
    magic_object_finder::magicObjectFinderGoal object_finder_goal; //instantiate goal message to communicate with magic_object_finder
       
    
    //the following is an std::vector of affines.  It describes a path in Cartesian coords, including orientations
    //not needed yet; is constructed inside the generic planner by interpolation
    //std::vector<Eigen::Affine3d> affine_path; 
    Eigen::Matrix3d R_down; //define an orientation corresponding to toolflange pointing down
    Eigen::Vector3d x_axis, y_axis, z_axis, flange_origin;
    z_axis << 0, 0, -1; //points flange down
    x_axis << -1, 0, 0; //arbitrary
    y_axis = z_axis.cross(x_axis); //construct y-axis consistent with right-hand coordinate frame
    R_down.col(0) = x_axis;
    R_down.col(1) = y_axis;
    R_down.col(2) = z_axis;
    flange_origin << 0.2, 0, 0.01;  //SHOULD GET FIXED: hard-coded pose can result in ugly/dangerous motion
    int nsteps = 100; //will need to specify how many interpolation points in Cartesian path; this is pretty coarse
    double arrival_time = 5.0; //will  need to specify arrival time for a Cartesian path

    //for this next line, I apparently did something wrong.  I should not have to  instantiate a cartesianInterpolator,
    //since the generic planner instantiates one.  But I get a compiler error.  Hmm...  Workaround.
    CartesianInterpolator cartesianInterpolator;

    g_q_vec_arm_Xd.resize(NJNTS); //generic vector resized to actual robot number of joints
    g_q_vec_arm_Xd << 0, 0, 0, 0, 0, 0; //assumes arm starts in this pose; better would be  to subscribe to joint_states to get actual angles

    //our irb120 control  interface uses this topic to receive trajectories
    ros::Publisher traj_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);

    //somewhat odd construction: a pointer to an object of type CartTrajPlanner, with arguments provided
    //that are pointers to forward and inverse kinematic functions.  This is to keep the planner generic,
    //and defer WHICH robot FK and IK to use until run time; Uses virtual functions for this.
    pCartTrajPlanner = new CartTrajPlanner(pIKSolver, pFwdSolver, njnts);
    //the planner needs to define penalty weights to optimize a path
    pCartTrajPlanner->set_jspace_planner_weights(g_planner_joint_weights);
    //to fill out a trajectory, need to provide the joint names; these are contained in a robot-specific header file
    pCartTrajPlanner->set_joint_names(g_jnt_names);


    optimal_path.clear(); //reset this std::vector before  each use, else  will have old values persisting
    optimal_path.push_back(g_q_vec_arm_Xd); //start from current pose
    optimal_path.push_back(g_q_vec_arm_Xd); // go from current pose to current pose--not very useful; but can "warm up" control 
    //publish/subscribe interface
    arrival_time = 1; //move should require zero time, but provide something small

    //function call from library (Class) CartTrajPlanner: converts a joint-space path to a joint-space trajectory
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    print_traj(new_trajectory); //display for  debug

    traj_publisher.publish(new_trajectory); //publish the trajectory; 
    ros::Duration(1).sleep();

    //example to show how to use forward kinematics from the class pointers provided 
    start_flange_affine = pFwdSolver->fwd_kin_solve(g_q_vec_arm_Xd);
    //display the flange affine corresponding to the specfied arm angles
    ROS_INFO_STREAM("fwd soln: origin = " << start_flange_affine.translation().transpose() << endl);
    ROS_INFO_STREAM("fwd soln: orientation: " << endl << start_flange_affine.linear() << endl);
    //xxxxxxxxxx   done with inquiry.  If here, then part pose is in g_perceived_object_pose.  Use it to compute robot motion    
    
    
    goal_flange_affine.linear() = R_down; //set the  goal orientation for flange to point down; will not need to change this for now


	//Set goal destination for object
	double des_x = 0.3;
	double des_y = 0.3;

	//Initialise variables 
	double offset_x = 0;
	double offset_y = 0;
	double offset = 0.065;


	while(ros::ok())
	{
		//Getting the pose of the object using the magic object finder.
		//specify the part name, send it in the goal message, wait for and interpret the result
		object_finder_goal.object_name = g_object_name.c_str(); //convert string object to old C-style string data
		object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); //request object finding via action server
		    
		finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0)); //wait for a max time for response
		//NOTE: could do something else here (if useful) while waiting for response from action server
		if (!finished_before_timeout) {
		        ROS_WARN("giving up waiting on result "); //this should not happen; should get result of found or not-found
		        return 1;
		    }
		//check the result code to see if object was found or not
		if (g_found_object_code == magic_object_finder::magicObjectFinderResult::OBJECT_FOUND)   {
		    ROS_INFO("found object!");
		}    
		else {
		    ROS_WARN("object not found!  Quitting");
		    return 1;
		}
		//Object has been found and pose is in g_perceived_object_pose 
		
		goal_flange_affine.linear() = R_down; //set the  goal orientation for flange to point down; will not need to change this for now

	//////////////////////////////////////////////////////////////////////////////////////////////
	//Test if object is in the desired postion 
	//////////////////////////////////////////////////////////////////////////////////////////////
	
		//Calculate error values of the object from the desired position
		double error_x = des_x - g_perceived_object_pose.pose.position.x;  
		double error_y = des_y - g_perceived_object_pose.pose.position.y;
	
		//If the error is small (0.01) in both x and y. 
		//The precision can be increased by deacreasing the values in the if conditions
		if(error_x < 0.01 && error_x > -0.01){ 
			if(error_y < 0.01 && error_y > -0.01){				
				//Move the flange to a default position above the object
				flange_origin << g_perceived_object_pose.pose.position.x, g_perceived_object_pose.pose.position.y, 0.3; 	  			
				goal_flange_affine.translation() = flange_origin;
				ROS_INFO_STREAM("move to flange origin= " << goal_flange_affine.translation().transpose() << endl);
				g_q_vec_arm_Xd = optimal_path.back(); //start from the joint-space pose that ended the prior plan
				//convert move to an optimal joint-space path:
				optimal_path.clear();
				nsteps = 100; //lots of points for smooth motion

				if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
						nsteps, optimal_path)) {
					ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
					return 1;
				}

				pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
				print_traj(new_trajectory);
				traj_publisher.publish(new_trajectory); //publish the trajectory
				ros::Duration(arrival_time).sleep(); //wait for the motion
			 
				ROS_INFO("done with last trajectory");
				//Once the flange has moved to the position above the object, break the while(ros:ok()) loop
				break;
			}
		}
	
	
	///////////////////////////////////////////////////////////////////////////////////////
	//move it to the correct position above the object (higher z) with correct offset 
	///////////////////////////////////////////////////////////////////////////////////////
	
	//Using if statements to set appropriate x and y offsets depending
	//on the position of the object in relation the the desired position
	//As the while loop iterates it will run through the if statements until all are fulfilled and the loop is broken
		if(g_perceived_object_pose.pose.position.x > des_x + 0.01)
		{
			offset_x = offset;
			offset_y = 0;
		}
		else if(g_perceived_object_pose.pose.position.x < des_x - 0.01)
		{
			offset_x = -offset;
			offset_y = 0; 
		}
		else if(g_perceived_object_pose.pose.position.y > des_y + 0.01){
			offset_x = 0;
			offset_y = offset; 
		}
		else if(g_perceived_object_pose.pose.position.y < des_y - 0.01){
			offset_x = 0;
			offset_y = -offset;
		}
	
	flange_origin << g_perceived_object_pose.pose.position.x + offset_x, g_perceived_object_pose.pose.position.y+offset_y, 0.3; 	   //Set the desired flange position 

	goal_flange_affine.translation() = flange_origin; //make this part of the flange  affine description
		ROS_INFO_STREAM("move to flange origin= " << goal_flange_affine.translation().transpose() << endl);
		ROS_INFO_STREAM("with orientation: " << endl << goal_flange_affine.linear() << endl);

		//interpolate from start pose to goal pose with this many samples along Cartesian path
		nsteps = 100; //arbitrary; tune me

		//compute an optimal joint-space path:
		optimal_path.clear();
		//planner will return "false" if unsuccessful; should add error handling
		//successful result will be a joint-space path in optimal_path
		if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
		        nsteps, optimal_path)) {
		    ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
		    return 1;
		}
		//if here, have a viable joint-space path; convert it to a trajectory:
		//choose arrival time--to  be  tuned
		arrival_time = 3.0;
		pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
		print_traj(new_trajectory);
		traj_publisher.publish(new_trajectory); //publish the trajectory--this should move  the robot
		ros::Duration(arrival_time).sleep(); //wait for the motion to complete (dead reckoning)
		ROS_INFO("done with first trajectory");
		
	//////////////////////////////////////////////////////////////////////////////////////////	
	//go to pose with offset from object; same x and y, but lower z value for flange origin
	//////////////////////////////////////////////////////////////////////////////////////////
	
	flange_origin << g_perceived_object_pose.pose.position.x + offset_x, g_perceived_object_pose.pose.position.y+offset_y, 0.005; 	    //The only change in position is the z value which is now 0.005

		goal_flange_affine.translation() = flange_origin;
		ROS_INFO_STREAM("move to flange origin= " << goal_flange_affine.translation().transpose() << endl);
		g_q_vec_arm_Xd = optimal_path.back(); //extract the last joint-space pose from the  plan, so can use it for start of next plan
		// better would be to get resulting joint-space values from joint_states
		//compute an optimal Cartesian motion in joint space from current joint-space pose to desired Cartesian pose
		optimal_path.clear();
		nsteps = 100; //tune me
		//compute the plan, to be returned in optimal_path
		if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
		        nsteps, optimal_path)) {
		    ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
		    return 1;
		}
		//convert the path to a trajectory (adds joint-space names,  arrival times, etc)
		pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
		print_traj(new_trajectory);
		traj_publisher.publish(new_trajectory); //publish the trajectory
		ros::Duration(arrival_time).sleep(); //wait for the motion
		ROS_INFO("done with second trajectory");
		
	//////////////////////////////////////////////////////////////////////////////////////////////
	//Push the object towards the desired location by moving the flange
	//////////////////////////////////////////////////////////////////////////////////////////////	
		
		//Usin the same if conditions as before, now set the flange origin to the
		//desired location of the object (minus offset)
		if(g_perceived_object_pose.pose.position.x > des_x + 0.01)
		{
			flange_origin << des_x + offset_x, g_perceived_object_pose.pose.position.y, 0.005;
		}
		else if(g_perceived_object_pose.pose.position.x < des_x - 0.01)
		{
			flange_origin << des_x + offset_x, g_perceived_object_pose.pose.position.y, 0.005;
		}
		else if(g_perceived_object_pose.pose.position.y > des_y + 0.01){
			flange_origin << g_perceived_object_pose.pose.position.x, des_y + offset_y, 0.005; 
		}
		else if(g_perceived_object_pose.pose.position.y < des_y - 0.01){
			flange_origin << g_perceived_object_pose.pose.position.x, des_y + offset_y, 0.005; 
		}
	
		goal_flange_affine.translation() = flange_origin;
		ROS_INFO_STREAM("move to flange origin: " << goal_flange_affine.translation().transpose() << endl);
		g_q_vec_arm_Xd = optimal_path.back(); //start from the joint-space pose that ended the prior plan
		//convert move to an optimal joint-space path:
		optimal_path.clear();
		nsteps = 100;

		if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(g_q_vec_arm_Xd, goal_flange_affine,
		        nsteps, optimal_path)) {
		    ROS_WARN("no feasible IK path for specified Cartesian motion; quitting");
		    return 1;
		}

		pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
		print_traj(new_trajectory);
		traj_publisher.publish(new_trajectory); //publish the trajectory
		ros::Duration(arrival_time).sleep(); //wait for the motion
		ROS_INFO("done with third trajectory");

	}//end of while loop

}




