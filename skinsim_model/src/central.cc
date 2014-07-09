#include "ros/ros.h"

#include "skinsim_msgs/tactileData.h"

void TactCallback(const skinsim_msgs::tactileData::ConstPtr& _msg)
{
	double sens_force = 0;
	for (unsigned int i = 0; i < _msg->force.size(); ++i)
	{
	  sens_force = sens_force + _msg->force[i];
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "central");
	ros::NodeHandle n;
      	//ros::Subscriber sub = n.subscribe("/gazebo/chatter", 1000, chatterCallback);
        ros::Subscriber sub = n.subscribe("tacData",1,TactCallback);
        ros::spin();
	return 0;
}
