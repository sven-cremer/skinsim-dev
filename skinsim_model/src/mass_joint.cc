#include "ros/ros.h"

#include "gazebo/common/CommonIface.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/gazebo.hh"

#include "skinsim_msgs/inputData.h"

namespace gazebo
{
  class MassJoint : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      this->ros_node = new ros::NodeHandle("~");
      this->model = _model;
      this->joint = this->model->GetJoint("my_mass_joint");
      this->input_pub = this->ros_node->advertise<skinsim_msgs::inputData>("inputData",1);
      action_t = 3.0;
      a = 1;
      force = 0.0;
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MassJoint::OnUpdate, this));
      
    }

    public: void OnUpdate()
    {
      // Apply a small force to the model.
      skinsim_msgs::inputData inputData;
      double current_time = this->model->GetWorld()->GetSimTime().Double();

      this->joint->SetForce(0, force);

     double current_force = this->joint->GetForce(0);

      std::cout<< "current force: "<<current_force<<"\n";
      inputData.input_force = current_force;
      inputData.time = current_time;

      if(current_time > action_t)
      {
        switch(a)
        {
          case 1:
            force = -0.14;
            break;
          case 2:
            force = -1.80;
            break;
          case 3:
            force = -1.0;
            break;
          case 4:
            force = -3.25;
            break;
          default:
            force = -2.50;
            a = 0;
          }
        a++;
        action_t = action_t + 2;
      }
      input_pub.publish(inputData); 
    }
private:
    physics::JointPtr joint;
    physics::ModelPtr model;  
    event::ConnectionPtr updateConnection;
    ros::NodeHandle* ros_node;
    double action_t;
    double force;
    int a;
    ros::Publisher input_pub;


  };


  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MassJoint)
}
