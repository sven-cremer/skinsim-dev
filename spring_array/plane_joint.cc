#include "gazebo/common/CommonIface.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/gazebo.hh"

#include "ros/ros.h"

namespace gazebo
{
  class Plane_Joint : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      
      // ROS Nodehandle
      this->ros_node = new ros::NodeHandle("~");

      std::string para_plane_spring = "/plane_spring";
      std::string para_plane_damper = "/plane_damper";

      if (!this->ros_node->getParam(para_plane_spring, plane_spring)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_plane_spring.c_str()); }
      if (!this->ros_node->getParam(para_plane_damper, plane_damper)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_plane_damper.c_str()); }

      this->model_ = _model;
      this->joint_ = this->model_->GetJoint("plane_joint");

      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Plane_Joint::UpdateJoint, this));
    }
    public: void UpdateJoint()
    {
      double rest_angle = 0;
      double stiffness = 24;
      double damp_coefficient = 10;
      double current_angle = this->joint_->GetAngle(0).Radian();
      double current_force = this->joint_->GetForce(0);
      double current_velocity = this->joint_->GetVelocity(0);
      double current_time = this->model_->GetWorld()->GetSimTime().Double();
      this->joint_->SetForce(0, (rest_angle - current_angle)*stiffness-damp_coefficient*current_velocity);
      //std::cout << name<<" Current force: "<<current_force<<"\n";

    }

    physics::JointPtr joint_;
    physics::ModelPtr model_;  
    event::ConnectionPtr update_connection_;

    // ROS
    ros::NodeHandle* ros_node;

    // Parameters
    double plane_spring ;
    double plane_damper ;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Plane_Joint)
}
