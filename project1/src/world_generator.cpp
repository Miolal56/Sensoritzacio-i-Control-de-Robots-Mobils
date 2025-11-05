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

    std::string content = buffer.str();
    size_t pos = content.find("#GOAL_ZONE#");
    if (pos != std::string::npos)
        content.replace(pos, 11, zone.str());
	else
		content.append("\n" + zone.str());

    std::ofstream out(output_path.c_str());
    out << content;
    out.close();

    ROS_INFO("Generated world at %s with goal zone at (%.2f, %.2f)", output_path.c_str(), X_GOAL, Y_GOAL);
    return 0;
}
