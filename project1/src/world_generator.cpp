#include "ros/ros.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ros/package.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "world_generator");
    ros::NodeHandle nh("~");

    double X_GOAL, Y_GOAL;
    nh.getParam("/talkerGoals/X_GOAL", X_GOAL);
    nh.getParam("/talkerGoals/Y_GOAL", Y_GOAL);

	std::string pkg_path = ros::package::getPath("project1");
    std::string template_path = pkg_path + "/worlds/mundo_template.world";
    std::string output_path = pkg_path + "/worlds/mundo.world";

    std::ifstream in(template_path.c_str());
    if (!in.is_open()) {
        ROS_ERROR("Cannot open template world: %s", template_path.c_str());
        return 1;
    }

    std::ostringstream buffer;
    buffer << in.rdbuf();
    in.close();

    std::ostringstream zone;
    zone << "floorplan2\n(\n"
         << "  name \"goal_zone\"\n"
         << "  size [0.5 0.5 0.0]\n"
         << "  pose [" << X_GOAL << " " << Y_GOAL << " 0.1 0.0]\n"
         << "  color \"green\"\n"
         << ")\n";

	std::ostringstream robots;
	std::vector<std::string> Colour;

	Colour.push_back("blue");
	Colour.push_back("yellow");
	Colour.push_back("orange");
	Colour.push_back("cyan");
	Colour.push_back("magenta");
	Colour.push_back("pink");
	Colour.push_back("green");
	Colour.push_back("red");
	Colour.push_back("black");
	Colour.push_back("white");

	int num_robots;
    nh.param("/NUM_ROBOTS", num_robots, 1);
    double POS_X, POS_Y;
	for(int ID = 0; ID < num_robots; ++ID){
        std::stringstream ns;
        ns << "/moveRobot" << ID;

        nh.param(ns.str() + "/X_START", POS_X, 0.000);
        nh.param(ns.str() + "/Y_START", POS_Y, 0.000);

		robots	<< "pioneer2dx\n(\n"
	  			<< "  name \" robot_" << ID << "\"\n"
	  			<< "  pose [ " << POS_X << " " << POS_Y << " 0.000 0.000 ]\n"
	 		 	<< "  color \"" + Colour[ID%Colour.size()] << "\"\n"
	  			<< "  sicklaser\n  (\n"
				<< "    ctrl \"lasernoise\"\n"
				<< "    alwayson 1\n  )\n)\n";
	}

    std::string content = buffer.str();

    size_t pos = content.find("#GOAL_ZONE#");
    if (pos != std::string::npos)
        content.replace(pos, 11, zone.str());
	else
		content.append("\n" + zone.str());

    pos = content.find("#ROBOTS#");
    if (pos != std::string::npos)
        content.replace(pos, 11, robots.str());
	else
		content.append("\n" + robots.str());

    std::ofstream out(output_path.c_str());

    out << content;
    out.close();

    ROS_INFO("Generated world at %s with goal zone at (%.2f, %.2f) and its %d robots", output_path.c_str(), X_GOAL, Y_GOAL, num_robots);
    return 0;
}
