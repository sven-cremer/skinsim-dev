#include <gazebo/transport/TransportIface.hh>
#include <gazebo/msgs/msgs.hh>

#include <iostream>

double disp_t = 1.90;
double array[49] = {0};

/////////////////////////////////////////////////
// Function is called everytime a message is received.
typedef const boost::shared_ptr<const gazebo::msgs::Vector3d> VectorThreeDPtr;
void cb(VectorThreeDPtr &_msg)
{
  int n;
  //int *p = array;
  // Dump the message contents to stdout.
  //std::cout << "x : " <<_msg->x() << "\n";
  //std::cout << "y : " <<_msg->y() << "\n";
  //std::cout << "z : " <<_msg->z() << "\n";
  //std::cout << "display_t: "<<disp_t <<"\n";
  n = _msg->y();
  array[n]=_msg->z();
  if(_msg->x()>disp_t)
  {
    double sum = 0;
    for(int i = 0;i<49; i++)
    {
      sum = sum+array[i];
    }
    std::cout << disp_t<<","<<sum<<"\n";
    disp_t = disp_t + 2;
  }
}


/////////////////////////////////////////////////
int main()
{
  // Initialize transport
  gazebo::transport::init();

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/force_sensor_info", cb);



  // Start transport
  gazebo::transport::run();

  

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  

  // Make sure to shut everything down.
  gazebo::transport::fini();
}
