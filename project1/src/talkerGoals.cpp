// This goal talker will work as the publisher

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Point.h"
#include <iostream>

int main(int argc, char **argv)

{
	ros::init(argc, argv,"talkerGoals");
	ros::NodeHandle node_obj("~"); //= ros::NodeHandle("sdf");

	// Firstly, we get the goal params of the robot
	double X_GOAL, Y_GOAL, T_GOAL;
	node_obj.getParam("X_GOAL", X_GOAL);
	node_obj.getParam("Y_GOAL", Y_GOAL);
	node_obj.getParam("T_GOAL", T_GOAL);


	// Now, we send the data of the current goal to the robots
	ros::Publisher goal_publisher = node_obj.advertise<geometry_msgs::Point>("mapGoal",10);

	// Next, we stabish the rate on movement
	ros::Rate rate(1.0 / T_GOAL);

	// Finally, we publish the goal in a loop until the robot reaches it
	while (ros::ok()) {
        geometry_msgs::Point goal;
        goal.x = X_GOAL;
        goal.y = Y_GOAL;
        goal.z = 0.0;
        goal_publisher.publish(goal);
        ros::spinOnce();
        rate.sleep();
    }

	return 0;
}
