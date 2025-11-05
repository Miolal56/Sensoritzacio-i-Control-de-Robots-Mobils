#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <tf2/utils.h>
#include <iostream>

class MoveRobot {
private:
    ros::NodeHandle nh;//("~");
    ros::Publisher vel_pub;
    ros::Subscriber odom_sub, laser_sub, goal_sub;

    double x, y, yaw; // Posición del robot
    geometry_msgs::Point goal;
	sensor_msgs::LaserScan::ConstPtr laser;
    std::vector<float> laser_ranges;

public:
	MoveRobot(int id, int leader) {
		std::stringstream ss;
		ss << "robot_" << id;
		std::string base = ss.str();
        vel_pub = nh.advertise<geometry_msgs::Twist>(base + "/cmd_vel", 10);
        odom_sub = nh.subscribe(base + "/odom", 10, &MoveRobot::odomCallback, this);
        laser_sub = nh.subscribe(base + "/base_scan_1", 10, &MoveRobot::laserCallback, this);
		if(leader == -1)
        	goal_sub = nh.subscribe("talkerGoals/mapGoal", 10, &MoveRobot::goalCallback, this);
		else{
			std::stringstream ss2;
			ss2 << "robot_" << leader;
			std::string follow = ss2.str();
        	goal_sub = nh.subscribe(follow + "/odom", 10, &MoveRobot::goalLeaderCallback, this);
		}
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;
        yaw = tf2::getYaw(msg->pose.pose.orientation);
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
		laser_ranges = msg->ranges;
		laser = msg;
		ROS_INFO_THROTTLE(2.0, "Laser topic: %s  size=%zu  inc=%.5f",
		                  msg->header.frame_id.c_str(),
		                  laser_ranges.size(),
		                  msg->angle_increment);
	}

    void goalCallback(const geometry_msgs::Point::ConstPtr& msg) {
        goal = *msg;
    }

    void goalLeaderCallback(const nav_msgs::Odometry::ConstPtr& msg) {
		goal = msg->pose.pose.position;
    }

    void moveLoop(int ALGOR) {

        ros::Rate rate(10);

		bool bucle = true;
        while (ros::ok() && bucle) {
			if(ALGOR == 1){
				bucle = simple();
			}
			if(ALGOR == 2){
				bucle = potencial();
			}
            ros::spinOnce();
            rate.sleep();
        }
    }

	bool simple(){
		geometry_msgs::Twist cmd;

		if (laser_ranges.empty() || laser_ranges.size() < 3) {
		    ROS_WARN_THROTTLE(1.0, "Simple: no enougth laser data (%d)", (int)laser_ranges.size());
		    return true;
		}

		if (!std::isfinite(goal.x) || !std::isfinite(goal.y)) {
			ROS_WARN_THROTTLE(1.0, "Simple: invalid goal values (%.2f, %.2f)", goal.x, goal.y);
			return true;
		}

		// --- Parameters ---
		double min_obstacle_dist = 1.0; // m, safety distance
		double linear_speed = 0.5; // m/s (slower = more stable avoidance)

		// Distance to goal
		double dx = goal.x - x;
        double dy = goal.y - y;
        double dist_to_goal = hypot(dx, dy);

		// If we already arrived, we stop moving
		if (dist_to_goal < 0.3) {
			ROS_INFO_THROTTLE(1.0, "Goal reached!");
			cmd.linear.x = 0.0;
			cmd.angular.z = 0.0;
			vel_pub.publish(cmd);
			return false;
		}

		// Angle error from current location to the goal
		double angle_error = atan2(sin(atan2(dy, dx) - yaw), cos(atan2(dy, dx) - yaw));

		// Detect if there's an obstacle close to the robot
		// We divide all robot's lasers into three groups
		int third = laser_ranges.size() / 3;

		// The first group are the right ones
		double right_min = *std::min_element(laser_ranges.begin(), laser_ranges.begin() + third);
		// The second group are the middle ones
		double front_min = *std::min_element(laser_ranges.begin() + third, laser_ranges.begin() + 2*third);
		// The third group are the left ones
		double left_min = *std::min_element(laser_ranges.begin() + 2*third, laser_ranges.end());

		// Now let's start the obstacle detection on the front
		if(front_min < min_obstacle_dist){
			// If there's an object too close from the front we need to stop going forward
			cmd.linear.x = 0.0;

			// And rotate the robot to the side with less obtacles
			if (left_min > right_min) cmd.angular.z = 1; // turn left
			else cmd.angular.z = -1; // turn right

		} else {

			// If any obstacle is detected in front of the robot, it continues forward
			cmd.linear.x = linear_speed;

			// After that, let's detect if there's an obtacle on the left or right
 			// If there's an obstacle too close from the left or right
			if(left_min < min_obstacle_dist || right_min < min_obstacle_dist){
				cmd.angular.z = 0;
			} else {
				cmd.angular.z = angle_error;
			}
		}
		
		// Finally, we publish the calculated movement data
		vel_pub.publish(cmd);
		return true;
	}

	template <typename T>
	T clamp(T value, T low, T high) {
		return std::max(low, std::min(value, high));
	}

	bool potencial(){
		geometry_msgs::Twist cmd;
		double weight1, weight2;

		if (laser_ranges.empty()) {
		    ROS_WARN_THROTTLE(1.0, "Simple: no enougth laser data (%d)", (int)laser_ranges.size());
		    return true;
		}

		if (!std::isfinite(goal.x) || !std::isfinite(goal.y)) {
			ROS_WARN_THROTTLE(1.0, "Simple: invalid goal values (%.2f, %.2f)", goal.x, goal.y);
			return true;
		}

		// First we calculate the go to goal vector
		std::pair<bool, geometry_msgs::Twist> go_to_goal = goal_vector();

		// If we have reached the goal, we stop
		if(!go_to_goal.first){
		    cmd.linear.x = 0;
		    cmd.angular.z = 0;
		    vel_pub.publish(cmd);
		    return false;
		}

		// We also calculate the obstacle avoidance vector
		std::pair<double, geometry_msgs::Twist> aviod_obstacles = obstacle_vector();

		// Time to calculate the weight for the go to goal vector
		weight2 = aviod_obstacles.first;
		weight1 = 1.0 - weight2;

		// Finally we calculate the command vector
		cmd.linear.x = weight1 * go_to_goal.second.linear.x + weight2 * aviod_obstacles.second.linear.x;
		cmd.angular.z = weight1 * go_to_goal.second.angular.z + weight2 * aviod_obstacles.second.angular.z;

		// Clamp linear velocity to prevent backward motion
		if (cmd.linear.x < 0.0)
		    cmd.linear.x = 0.0;

		// Add small dead zone to avoid jittering near zero
		if (fabs(cmd.linear.x) < 0.05)
		    cmd.linear.x = 0.0;

		// Finally, we publish the calculated movement data
		vel_pub.publish(cmd);
		return true;
	}

	std::pair<bool, geometry_msgs::Twist> goal_vector(){
		geometry_msgs::Twist cmd;

		double dx = goal.x - x;
		double dy = goal.y - y;
		double dist_to_goal = hypot(dx, dy);

		// If we already arrived, we stop moving
		if (dist_to_goal < 0.3 && dx < 1 && dy < 1) {
		    ROS_INFO_THROTTLE(1.0, "Goal reached!");
		    cmd.linear.x = 0.0;
		    cmd.angular.z = 0.0;
		    return std::make_pair(false, cmd);
		}

		// Limit forward command to a reasonable constant speed
		cmd.linear.x = std::min(dist_to_goal, 0.5);  
		cmd.angular.z = atan2(sin(atan2(dy, dx) - yaw), cos(atan2(dy, dx) - yaw));
		return std::make_pair(true, cmd);
	}

	std::pair<double, geometry_msgs::Twist> obstacle_vector(){
		geometry_msgs::Twist cmd;

		if (laser_ranges.empty() || laser_ranges.size() < 3) {
		    ROS_WARN_THROTTLE(1.0, "Potential: not enough laser data (%d)", (int)laser_ranges.size());
		    return std::make_pair(0, cmd);
		}

		if (!std::isfinite(goal.x) || !std::isfinite(goal.y)) {
		    ROS_WARN_THROTTLE(1.0, "Potential: invalid goal values (%.2f, %.2f)", goal.x, goal.y);
		    return std::make_pair(0, cmd);
		}

		// --- Parameters ---
		double critical_distance = 0.3; 
		double max_laser_distance = laser->range_max;
		double linear_speed = 0.5;
		double K_REP = 1.0;
		double influence_distance = 1.5; // fixed influence range

		// These will hold the combined obstacle vector
		double force_x = 0.0;
		double force_y = 0.0;
		double min_dist = max_laser_distance;

		// Iterate through all laser readings
		for (size_t i = 0; i < laser_ranges.size(); ++i) {
		    double d = laser_ranges[i];
		    if (std::isnan(d) || std::isinf(d)) continue;
		    if (d > max_laser_distance) continue;

		    double angle = laser->angle_min + i * laser->angle_increment;
		    if (d < min_dist) min_dist = d;

		    if (d < influence_distance) {
		        double strength = K_REP * (1.0 / d - 1.0 / influence_distance) / (d * d);
		        force_x += -strength * cos(angle);
		        force_y += -strength * sin(angle);
		    }
		}

		// Normalize or limit vector strength
		double mag = hypot(force_x, force_y);
		if (mag > 1.0) {
		    force_x /= mag;
		    force_y /= mag;
		}

		// Clamp linear speed to non-negative (no backward)
		cmd.linear.x = std::max(0.0, linear_speed * (1.0 - mag));

		// Limit angular speed to avoid over-oscillation
		cmd.angular.z = clamp(atan2(force_y, force_x), -1.0, 1.0);

		// Calculate obstacle weight
		double weight = 0.0;
		if (min_dist < influence_distance)
		    weight = std::min(1.0, (influence_distance - min_dist) / (influence_distance - critical_distance));

		// Smooth weight change to reduce twitching
		static double prev_weight = 0.0;
		double alpha = 0.3;
		weight = alpha * weight + (1 - alpha) * prev_weight;
		prev_weight = weight;

		return std::make_pair(weight, cmd);
	}
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "moveRobot");
	ros::NodeHandle node_obj("~");
	int ALGOR, ID_ROBOT, LEADER;
	node_obj.getParam("ALGOR", ALGOR);
	node_obj.getParam("LEADER", LEADER);
	node_obj.getParam("ID_ROBOT", ID_ROBOT);
    MoveRobot robot(ID_ROBOT, LEADER);
    robot.moveLoop(ALGOR);
}
