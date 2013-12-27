#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  class Spring_Joint : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      
      // init joints, hardcoded for robot
	  this->jointNames.push_back("joint_1" );
	  this->jointNames.push_back("joint_2" );
	  this->jointNames.push_back("joint_3" );
	  this->jointNames.push_back("joint_4" );
	  this->jointNames.push_back("joint_5" );
	  this->jointNames.push_back("joint_6" );
	  this->jointNames.push_back("joint_7" );
	  this->jointNames.push_back("joint_8" );
	  this->jointNames.push_back("joint_9" );
	  this->jointNames.push_back("joint_10");
	  this->jointNames.push_back("joint_11");
	  this->jointNames.push_back("joint_12");
	  this->jointNames.push_back("joint_13");
	  this->jointNames.push_back("joint_14");
	  this->jointNames.push_back("joint_15");
	  this->jointNames.push_back("joint_16");
	  this->jointNames.push_back("joint_17");
	  this->jointNames.push_back("joint_18");
	  this->jointNames.push_back("joint_19");
	  this->jointNames.push_back("joint_20");
	  this->jointNames.push_back("joint_21");
	  this->jointNames.push_back("joint_22");
	  this->jointNames.push_back("joint_23");
	  this->jointNames.push_back("joint_24");
	  this->jointNames.push_back("joint_25");
	  this->jointNames.push_back("joint_26");
	  this->jointNames.push_back("joint_27");
	  this->jointNames.push_back("joint_28");
	  this->jointNames.push_back("joint_29");
	  this->jointNames.push_back("joint_30");
	  this->jointNames.push_back("joint_31");
	  this->jointNames.push_back("joint_32");
	  this->jointNames.push_back("joint_33");
	  this->jointNames.push_back("joint_34");
	  this->jointNames.push_back("joint_35");
	  this->jointNames.push_back("joint_36");
	  this->jointNames.push_back("joint_37");
	  this->jointNames.push_back("joint_38");
	  this->jointNames.push_back("joint_39");
	  this->jointNames.push_back("joint_40");
	  this->jointNames.push_back("joint_41");
	  this->jointNames.push_back("joint_42");
	  this->jointNames.push_back("joint_43");
	  this->jointNames.push_back("joint_44");
	  this->jointNames.push_back("joint_45");
	  this->jointNames.push_back("joint_46");
	  this->jointNames.push_back("joint_47");
	  this->jointNames.push_back("joint_48");
	  this->jointNames.push_back("joint_49");

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
        boost::bind(&Spring_Joint::UpdateJoint, this));
    }
    
    public: void UpdateJoint()
    {
      double rest_angle = 0;
      double stiffness = 2;
      double damp_coefficient = 0.45;
      double current_angle    = 0;
      double current_force    = 0;
      double current_velocity = 0;
      
      double current_time = this->model_->GetWorld()->GetSimTime().Double();
      math::Vector3 vect;
      for (unsigned int i = 0; i < this->joints.size(); ++i)
	  {
		  current_angle = this->joints[i]->GetAngle(0).Radian();
		  current_force = this->joints[i]->GetForce(0);
		  current_velocity = this->joints[i]->GetVelocity(0);
		  
		  // This sets the mass-spring-damper dynamics, currently only spring and damper
		  this->joints[i]->SetForce(0, (rest_angle - current_angle)*stiffness-damp_coefficient*current_velocity);
		  vect.x = current_time;
		  vect.y = i;
		  vect.z = current_force;
		  msgs::Set(&msg, vect);
		  pub->Publish(msg);
	  }
  
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
  GZ_REGISTER_MODEL_PLUGIN(Spring_Joint)
}
