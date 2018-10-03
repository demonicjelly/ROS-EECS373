#ifndef SIN_ACTION_SERVER_H_
#define SIN_ACION_SERVER_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include<ros/ros.h> 
#include<std_msgs/Float64.h> 
#include <actionlib/server/simple_action_server.h>

#include<problem_sheet3/SinMsgAction.h>

class SinActionServer
{

private:

	ros:: NodeHandle nh;

	actionlib:: SimpleActionServer<problem_sheet3::SinMsgAction> as_;

	problem_sheet3::SinMsgGoal goal_;
	problem_sheet3::SinMsgResult result_;
	problem_sheet3::SinMsgFeedback feedback_;


public:

	SinActionServer();

	~SinActionServer(void){
	}

	void executeCB(const actionlib::SimpleActionServer<problem_sheet3::SinMsgAction>::GoalConstPtr& goal);

};

#endif
