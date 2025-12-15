#include <ros/ros.h>
#include <user_demo/mission_node.hpp>

using namespace mission;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission_node");
    MissionNode mission_node;
    mission_node.initMissionNode();
    ros::spin();
    return 0; 
}