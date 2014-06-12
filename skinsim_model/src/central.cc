#include "ros/ros.h"

#include "skinsim_msgs/tactileData.h"

void tactCallback(const skinsim_msgs::tactileData::ConstPtr& msg)
{

	double sens_force = 0;
	for (unsigned int i = 0; i < msg->force.size(); ++i)
	{
	  sens_force = sens_force + msg->force[i];
	}

        //std::cout<<sens_force<<"\n";
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "central");
	ros::NodeHandle n;
      	//ros::Subscriber sub = n.subscribe("/gazebo/chatter", 1000, chatterCallback);
        ros::Subscriber sub = n.subscribe("tacData",1,tactCallback);
        ros::spin();
	return 0;
}
