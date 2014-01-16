#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo.hh"

namespace gazebo
{
  class Tactile_Joint : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      
      // init joints, hardcoded for robot
	  this->jointNames.push_back("tactile_joint_1" );
	  this->jointNames.push_back("tactile_joint_2" );
	  this->jointNames.push_back("tactile_joint_3" );
	  this->jointNames.push_back("tactile_joint_4" );
	  this->jointNames.push_back("tactile_joint_5" );
	  this->jointNames.push_back("tactile_joint_6" );
	  this->jointNames.push_back("tactile_joint_7" );
	  this->jointNames.push_back("tactile_joint_8" );
	  this->jointNames.push_back("tactile_joint_9" );
	  this->jointNames.push_back("tactile_joint_10");
	  this->jointNames.push_back("tactile_joint_11");
	  this->jointNames.push_back("tactile_joint_12");
	  this->jointNames.push_back("tactile_joint_13");
	  this->jointNames.push_back("tactile_joint_14");
	  this->jointNames.push_back("tactile_joint_15");
	  this->jointNames.push_back("tactile_joint_16");
	  this->jointNames.push_back("tactile_joint_17");
	  this->jointNames.push_back("tactile_joint_18");
	  this->jointNames.push_back("tactile_joint_19");
	  this->jointNames.push_back("tactile_joint_20");
	  this->jointNames.push_back("tactile_joint_21");
	  this->jointNames.push_back("tactile_joint_22");
	  this->jointNames.push_back("tactile_joint_23");
	  this->jointNames.push_back("tactile_joint_24");
	  this->jointNames.push_back("tactile_joint_25");
	  this->jointNames.push_back("tactile_joint_26");
	  this->jointNames.push_back("tactile_joint_27");
	  this->jointNames.push_back("tactile_joint_28");
	  this->jointNames.push_back("tactile_joint_29");
	  this->jointNames.push_back("tactile_joint_30");
	  this->jointNames.push_back("tactile_joint_31");
	  this->jointNames.push_back("tactile_joint_32");
	  this->jointNames.push_back("tactile_joint_33");
	  this->jointNames.push_back("tactile_joint_34");
	  this->jointNames.push_back("tactile_joint_35");
	  this->jointNames.push_back("tactile_joint_36");
	  this->jointNames.push_back("tactile_joint_37");
	  this->jointNames.push_back("tactile_joint_38");
	  this->jointNames.push_back("tactile_joint_39");
	  this->jointNames.push_back("tactile_joint_40");
	  this->jointNames.push_back("tactile_joint_41");
	  this->jointNames.push_back("tactile_joint_42");
	  this->jointNames.push_back("tactile_joint_43");
	  this->jointNames.push_back("tactile_joint_44");
	  this->jointNames.push_back("tactile_joint_45");
	  this->jointNames.push_back("tactile_joint_46");
	  this->jointNames.push_back("tactile_joint_47");
	  this->jointNames.push_back("tactile_joint_48");
	  this->jointNames.push_back("tactile_joint_49");

      this->model_ = _model;

      // get pointers to joints from gazebo
		this->joints.resize(this->jointNames.size());
		for (unsigned int i = 0; i < this->joints.size(); ++i)
		{
		  this->joints[i] = this->model_->GetJoint(this->jointNames[i]);
		  if (!this->joints[i])
		  {
			//ROS_ERROR("SkinSim robot expected joint[%s] not present, plugin not loaded",
			//  this->jointNames[i].c_str());
			return;
		  }
		}

     // Create a new transport node
     transport::NodePtr node(new transport::Node());
 
     // Create a publisher on the ~/factory topic
     pub = node->Advertise<msgs::Vector3d>("~/force_sensor_info");

     // Initialize the node with the Model name
     node->Init(model_->GetName());


      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Tactile_Joint::UpdateJoint, this));
    }
    
    public: void UpdateJoint()
    {
      double rest_angle = 0;
      double stiffness = 4;
      double permittivity = 1;
      double damp_coefficient = 0.35;
      double current_angle    = 0;
      double current_force    = 0;
      double current_velocity = 0;
      double current_voltage = 0;
      double capacitor_area = 0.25;
      double capacitor_depth = 0.4;
      double current_time = this->model_->GetWorld()->GetSimTime().Double();
      math::Vector3 vect;
      for (unsigned int i = 0; i < this->joints.size(); ++i)
	  {
		  current_angle = this->joints[i]->GetAngle(0).Radian();
		  current_force = this->joints[i]->GetForce(0);
		  current_velocity = this->joints[i]->GetVelocity(0);
 
		  current_voltage = sqrt(2*current_force*pow((capacitor_depth + current_angle),2)/(permittivity*capacitor_area));
		  // This sets the mass-spring-damper dynamics, currently only spring and damper
		  this->joints[i]->SetForce(0, (rest_angle - current_angle)*stiffness-damp_coefficient*current_velocity);
		  vect.x = current_time;
		  vect.y = i;
		  vect.z = current_force;
		  msgs::Set(&msg, vect);
		  pub->Publish(msg);
	  }
  
      // FIXME publish a vector of vectors ... Fixed See above


      
    }
    
    /// \brief keep a list of hard coded joint names.
	std::vector<std::string> jointNames;
    //physics::JointPtr joint_;

	/// \brief Internal list of pointers to Joints
	physics::Joint_V joints;

	physics::ModelPtr model_;
    event::ConnectionPtr update_connection_;
    msgs::Vector3d msg;
    transport::PublisherPtr pub;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Tactile_Joint)
}
