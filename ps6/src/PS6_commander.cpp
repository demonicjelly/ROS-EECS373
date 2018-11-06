#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/DroneControl.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <iostream>
#include <string>

using namespace std;

bool g_take_new_snapshot = false;
osrf_gear::LogicalCameraImage g_cam1_data;

void cam2CB(const osrf_gear::LogicalCameraImage& message_holder) {
    if (g_take_new_snapshot) {
        ROS_INFO_STREAM("image from cam1: " << message_holder << endl);
		ROS_INFO_STREAM("image from cam1: " << message_holder.models.size() << endl);
        g_cam1_data = message_holder;
		ROS_INFO_STREAM("g_cam1_data: " << g_cam1_data.models.size() << endl);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ps6");
    ros::NodeHandle n;
    ros::ServiceClient startup_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    std_srvs::Trigger startup_srv;
    ros::ServiceClient conveyor_client = n.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    osrf_gear::ConveyorBeltControl conveyor_srv;
    ros::ServiceClient drone_client = n.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    osrf_gear::DroneControl drone_srv;

    ros::Subscriber cam2_subscriber = n.subscribe("/ariac/logical_camera_2", 1, cam2CB);

    startup_srv.response.success = false;
    while (!startup_srv.response.success) {
        ROS_WARN("not successful starting up yet...");
        startup_client.call(startup_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("got success response from startup service");

    conveyor_srv.request.power = 100.0;
    conveyor_srv.response.success = false;
    while (!conveyor_srv.response.success) {
        ROS_WARN("not successful starting conveyor yet...");
        conveyor_client.call(conveyor_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("I see a box");
    
    ROS_INFO("got success response from conveyor service");

	//FIND A BOX AND STOP THE CONVEYOR WHEN THE BOX IS FOUND
	bool box_found = false;
	bool box_in_position = false;
	while (!box_found) {	//Take a snapshot to see if the box is under the camera
		g_take_new_snapshot = true;
		while (g_cam1_data.models.size() < 1) {    //keep iterating until the box is found
		    ros::spinOnce();
		    ros::Duration(0.5).sleep();
		}
		ROS_INFO("IT HAS ARRIVED");
		box_found = true;
		while (!box_in_position) {	//While the box is not in position under the camera keep taking screenshots
			ros::spinOnce();
			ros::Duration(0.5).sleep();
			if (g_cam1_data.models[0].pose.position.z < 0.1 && g_cam1_data.models[0].pose.position.z > -0.1) {
				//if the box is under the camera/within (-0.1, 0.1) of the camera frame
				ROS_INFO("THE BOX HAS ARRIIIVVEDDD");
				box_in_position = true;
				conveyor_srv.request.power = 0.0; //stop conveyor
				conveyor_srv.response.success = false;
				while (!conveyor_srv.response.success) {
					ROS_WARN("not successful stopping conveyor yet...");
					conveyor_client.call(conveyor_srv);
					ros::Duration(0.5).sleep();
				}
				ros::Duration(3.0).sleep(); //Hold the box under the camera for 3 seconds
				conveyor_srv.request.power = 100.0; //restart conveyor
				conveyor_srv.response.success = false;
				while (!conveyor_srv.response.success) {
					ROS_WARN("not successful starting conveyor yet...");
					conveyor_client.call(conveyor_srv);
					ros::Duration(0.5).sleep();
				}
			}
		}
		//no need to take any more snapshots as the box has been and gone from under the camera
		g_take_new_snapshot = false;
	}
    drone_srv.request.shipment_type = "thechosenbox";
    drone_srv.response.success = false;
    while (!drone_srv.response.success) {
        ROS_WARN("not successful starting drone yet...");
        drone_client.call(drone_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("got success response from drone service");
}

