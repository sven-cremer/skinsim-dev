/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, UT Arlington
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of UT Arlington nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * plunger.cpp
 *
 *  Created on: Aug 6, 2016
 *      Author: Sven Cremer
 */

#include <SkinSim/plunger.h>

namespace gazebo
{
// Register plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Plunger);

//////////////////////////////////////////////////////////////////////////
// Constructor
Plunger::Plunger()
{
	this->ros_connections_ = 0;
}

//////////////////////////////////////////////////////////////////////////
// Destructor
Plunger::~Plunger()
{
	event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
	// Custom Callback Queue
	this->ros_queue_.clear();
	this->ros_queue_.disable();
	this->ros_node_->shutdown();
	this->callback_ros_queue_thread_.join();
	delete this->ros_node_;
}

//////////////////////////////////////////////////////////////////////////
// Load the controller
void Plunger::Load( physics::ModelPtr _model, sdf::ElementPtr _sdf )
{

	// Save pointers and names
	this->model_ = _model;
	this->world_ = this->model_->GetWorld();

	this->joint_pid_ = this->model_->GetJointController();

	this->model_name_ = model_->GetName();
	this->topic_name_ = "plunger_data";
	this->joint_name_ = "plunger_joint"; //model_name_+"_joint";

	// Set default values
	this->ros_namespace_ = "skinsim/";
	this->update_rate_   = 0;
	this->Kp_            = 0.0;
	this->Kd_            = 0.0;
	this->Kv_            = 0.0;

	// Load parameters from model SDF file
	if (_sdf->HasElement("rosNamespace"))
		this->ros_namespace_ = _sdf->GetElement("rosNamespace")->Get<std::string>() + "/";

	if (_sdf->HasElement("updateRate"))
		this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

	if (_sdf->HasElement("Kp"))
		this->Kp_ = _sdf->GetElement("Kp")->Get<double>();
	if (_sdf->HasElement("Kd"))
		this->Kd_ = _sdf->GetElement("Kd")->Get<double>();
	if (_sdf->HasElement("Kv"))
		this->Kv_ = _sdf->GetElement("Kv")->Get<double>();

	double JointPgain_ = 5.0;
	if (_sdf->HasElement("JointPgain"))
		JointPgain_ = _sdf->GetElement("JointPgain")->Get<double>();
	double JointIgain_ = 0.01;
	if (_sdf->HasElement("JointIgain"))
		JointIgain_ = _sdf->GetElement("JointIgain")->Get<double>();
	double JointDgain_ = 0.1;
	if (_sdf->HasElement("JointDgain"))
		JointDgain_ = _sdf->GetElement("JointDgain")->Get<double>();

	// Get pointers to joint from Gazebo
	this->joint_ = this->model_->GetJoint(this->joint_name_);
	if (!this->joint_)
	{
		std::cerr<<"Plunger joint ["<<this->joint_name_.c_str()<<"] not present, plugin not loaded \n";
		return;
	}
	this->axis_global_ = this->joint_->GetGlobalAxis(0);
	this->axis_local_  = this->joint_->GetLocalAxis(0);

	// Make sure the ROS node for Gazebo has already been initialized
	if (!ros::isInitialized())
	{
		this->pub_to_ros_ = false;

		std::cout<<"Warning: The ROS node for Gazebo has not been initialized! \n"
				<<"Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package \n";
	}
	else
	{
		this->pub_to_ros_ = true;

		this->ros_node_ = new ros::NodeHandle(this->ros_namespace_);

		// ROS topics
		ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<skinsim_ros_msgs::PlungerData>(
				this->topic_name_,1,
				boost::bind( &Plunger::RosConnect,this),
				boost::bind( &Plunger::RosDisconnect,this), ros::VoidPtr(), &this->ros_queue_);
		this->ros_pub_ = this->ros_node_->advertise(ao);

		// ROS subscribers
		this->ros_sub_fb_ = this->ros_node_->subscribe("/skinsim/force_feedback", 1, &Plunger::ForceFeedbackCB, this);

		// ROS services
		this->ros_srv_ 			= 	this->ros_node_->advertiseService("set_controller", &Plunger::serviceCB, this);
		this->ros_srv_gains_ 	= 	this->ros_node_->advertiseService("set_PIDGains", &Plunger::serviceSetPIDCB, this);

		// Custom Callback Queue
		this->callback_ros_queue_thread_ = boost::thread( boost::bind( &Plunger::RosQueueThread,this ) );
	}

	// Update joints at every simulation iteration
	this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&Plunger::UpdateJoints, this));

	// Controller
	this->position_desired_ = 0.0;
	this->velocity_desired_ = 0.0;
	this->force_desired_    = 0.0;
	this->force_prev_       = 0.0;
	this->feedback_type_.selected   = skinsim_ros_msgs::FeedbackType::TACTILE_APPLIED;
	this->controller_type_.selected = skinsim_ros_msgs::ControllerType::DIRECT;

	// P-controller
	this->pid_ = common::PID(JointPgain_, JointIgain_, JointDgain_, 15, -15);

	// Apply the P-controller to the joint.
	//this->joint_pid_->SetPositionPID(this->joint_->GetScopedName(), this->pid_);

	// Debug
	std::cout<<"> Model name: "<<model_name_<<"\n";
	/*
	std::cout<<"> Joint scoped name: "<<this->joint_->GetScopedName()<<"\n";
	std::cout<<"> Joint PID: "<<JointPgain_<<", "<<JointIgain_<<", "<<JointDgain_<<"\n";
	std::cout<<"> Kp: "<<Kp_<<"\tKd: "<<Kd_<<"\n";
	for(int i=0;i<3;i++)
	{
		std::cout<<"> Local axis  ["<<i<<"]: "<<this->joint_->GetLocalAxis(i)<<"\n";
		std::cout<<"> Global axis ["<<i<<"]: "<<this->joint_->GetGlobalAxis(i)<<"\n";
	}
	*/

	this->joint_->SetStiffnessDamping(0,0,0,0);	// A position change does not result in a force

	// Contact sensor

	// Get the contact sensor
	sensors::SensorPtr _sensor = sensors::get_sensor("plunger_sensor");
	this->contact_sensor_ptr_  = boost::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

	// Make sure the parent sensor is valid
	if (!this->contact_sensor_ptr_)
	{
		std::cerr << "Error: ContactPlugin requires a ContactSensor.\n";
	}

	// Connect to the sensor update event
	this->update_contact_connection_ = this->contact_sensor_ptr_->ConnectUpdated(
			boost::bind(&Plunger::OnContactUpdate, this));

	// Make sure the sensor is active
	this->contact_sensor_ptr_->SetActive(false);

	this->effort_ = 0.0;
}

//////////////////////////////////////////////////////////////////////////
// Update the controller
void Plunger::UpdateJoints()
{
	// Compute time step for PID
	common::Time cur_time  = this->world_->GetSimTime();
	common::Time step_time = cur_time - this->last_time_;

	// Rate control
	//	if (this->update_rate_ > 0 && (cur_time-this->last_time_).Double() < (1.0/this->update_rate_))
	//		return;
	if(controller_type_.selected == skinsim_ros_msgs::ControllerType::DIGITAL_PID)	// Check Ts
	{
		if( (cur_time - this->last_time_).Double() < Ts )
		{
			this->joint_->SetForce(0, -this->effort_);	// Send old value
			return;
		}
	}

	this->position_current_ = this->joint_->GetAngle(0).Radian();
	this->velocity_current_ = this->joint_->GetVelocity(0);
	this->wrench_ 			= this->joint_->GetForceTorque(0);

	this->force_  = this->wrench_.body2Force;
	this->torque_ = this->wrench_.body2Torque;

	// Select feedback
	switch(this->feedback_type_.selected)
	{
	// Plunger
	case skinsim_ros_msgs::FeedbackType::PLUNGER_LOAD_CELL:
	{
		this->force_current_ = -this->force_.Dot(this->axis_global_);
		break;
	}
	// Tactile applied
	case skinsim_ros_msgs::FeedbackType::TACTILE_APPLIED:
	{
		 this->force_current_ = msg_fb_.force_applied;		// Force applied > 0
		 break;
	}
	// Tactile sensed
	case skinsim_ros_msgs::FeedbackType::TACTILE_SENSED:
	{
		 this->force_current_ = -msg_fb_.force_sensed;		// Force sensed < 0
		 break;
	}
	// Default
	default:
		std::cerr<<"Feedback type not implemented.\n";
		break;
	}

	this->force_dot_ = (force_current_-force_prev_)/step_time.Double();

	// FIXME Controller dynamics
	//position_desired_ = Kp_*(force_desired_ - force_current_) - Kd_*force_dot_;

	//	if(!this->joint_->SetPosition(0, position_desired_))
//	{
//		std::cout<<"Plunger: SetPosition failed!\n";
//	}
	//this->joint_pid_->SetPositionTarget(this->joint_->GetScopedName(),position_desired_);
//	this->joint_pid_->Update();

	// Select controller
	switch(this->controller_type_.selected)
	{
	// Set force directly in Gazebo
	case skinsim_ros_msgs::ControllerType::DIRECT:
	{
		this->effort_ = this->force_desired_;
		this->joint_->SetForce(0, this->effort_);
		break;
	}
	// Force-Based Explicit Force control
	case skinsim_ros_msgs::ControllerType::FORCE_BASED_FORCE_CONTROL:
	{
		if(num_contacts_>0)
		{
			this->effort_ = force_desired_ + Kp_*(force_desired_ - force_current_) - Kv_*velocity_current_;
			//this->effort_ = force_desired_ + Kp_*(force_desired_ - force_current_) - Kd_*force_dot_;
			this->joint_->SetForce(0, -this->effort_);
		}
		else
		{
			this->joint_->SetVelocity (0, velocity_desired_);
		}
		break;
	}
	// Position-Based Explicit Force control
	case skinsim_ros_msgs::ControllerType::POSITION_BASED_FORCE_CONTROL:
	{
		position_desired_ = Kp_*(force_desired_ - force_current_) - Kd_*force_dot_;
		double postion_error = position_current_ - position_desired_;
		this->effort_ = this->pid_.Update(postion_error, step_time);
		this->joint_->SetForce(0, -this->effort_);
		break;
	}
	// Digital PID control
	case skinsim_ros_msgs::ControllerType::DIGITAL_PID:
	{
		if(num_contacts_>0)
		{
			// Update variables
			e_(2)=e_(1);
			e_(1)=e_(0);
			u_(2)=u_(1);
			u_(1)=u_(0);

			// Compute new error
			e_(0) = force_desired_ - force_current_;	// Reference minus measured force

			// Compute new control
			u_(0) = -ku_(1)*u_(1) - ku_(2)*u_(2) + ke_(0)*e_(0) + ke_(1)*e_(1) + ke_(2)*e_(2);

			// Apply limits
			double umax =  15;	// TODO
			double umin = -15;	// TODO
			if (u_(0) > umax)
				u_(0) = umax;
			if (u_(0) < umin)
				u_(0) = umin;

			effort_ = u_(0);
			this->joint_->SetForce(0, -this->effort_);
		}
		else
		{
			this->joint_->SetVelocity (0, velocity_desired_);
		}
		break;
	}
	// Impedance control
	case skinsim_ros_msgs::ControllerType::IMPEDANCE_CONTROL:
	// Default
	default:
		std::cerr<<"Controller type not implemented.\n";
		break;
	}

	// Debug
	//std::cout<<"F cur: "<<force_current_<<" \tEffort: "<<effort_<<" \tF dot: "<<force_dot_;//<<" \t<-\t"<<force_<<"\n";
	//std::cout<<"\tPos cur: "<<position_current_<<"\tPos des: "<<position_desired_<<"\n";

	// Decide whether to publish
	if (this->ros_connections_ > 0 && pub_to_ros_)
	{
		this->lock_.lock();
		// Copy data into ROS message
		this->msg_data_.effort    = this->effort_;
		this->msg_data_.force     = this->force_current_;
		this->msg_data_.force_dot = this->force_dot_;
		this->msg_data_.position  = this->position_current_;
		this->msg_data_.velocity  = this->velocity_current_;
		// Publish data
		this->ros_pub_.publish(this->msg_data_);
		this->lock_.unlock();
	}

	// Save last time stamp
	this->last_time_ = cur_time;

	// Save force
	force_prev_ = force_current_;
}

//////////////////////////////////////////////////////////////////////////
// Callback function when subscriber connects
bool Plunger::serviceCB(skinsim_ros_msgs::SetController::Request& req, skinsim_ros_msgs::SetController::Response& res)
{
	// Store data
	this->lock_.lock();
	this->controller_type_.selected = req.type.selected;
	this->feedback_type_  .selected = req.fb.selected;

	switch(this->controller_type_.selected)
	{
	case skinsim_ros_msgs::ControllerType::DIRECT:
	case skinsim_ros_msgs::ControllerType::FORCE_BASED_FORCE_CONTROL:
	{
		this->force_desired_    = req.f_des;
		this->position_desired_ = req.x_des;
		this->velocity_desired_	= req.v_des;
		this->Kp_               = req.Kp;
		this->Ki_               = req.Ki;
		this->Kd_               = req.Kd;
		this->Kv_               = req.Kv;
		std::cout<<"Force Based Force Controller Initiated"<<std::endl;
		res.success = true;
		break;
	}
	case skinsim_ros_msgs::ControllerType::POSITION_BASED_FORCE_CONTROL:
	{
		this->force_desired_    = req.f_des;
		this->position_desired_ = req.x_des;
		this->velocity_desired_	= req.v_des;
		this->Kp_               = req.Kp;
		this->Ki_               = req.Ki;
		this->Kd_               = req.Kd;
		this->Kv_               = req.Kv;
		std::cout<<"Position Based Force Controller Initiated"<<std::endl;
		res.success = true;
		break;
	}
	case skinsim_ros_msgs::ControllerType::DIGITAL_PID:
	{
		this->force_desired_    = req.f_des;
		this->position_desired_ = req.x_des;
		this->velocity_desired_	= req.v_des;
		this->Kp_               = req.Kp;
		this->Ki_               = req.Ki;
		this->Kd_               = req.Kd;
		N =100;			// TODO pass these values
		Ts = 0.001;

		a_(0) = (1+N*Ts);
		a_(1) = -(2 + N*Ts);
		a_(2) = 1;
		b_(0) = Kp_*(1+N*Ts) + Ki_*Ts*(1+N*Ts) + Kd_*N;
		b_(1) = -(Kp_*(2+N*Ts) + Ki_*Ts + 2*Kd_*N);
		b_(2) = Kp_ + Kd_*N;
		ku_(1) = a_(1)/a_(0);
		ku_(2) = a_(2)/a_(0);
		ke_(0) = b_(0)/a_(0);
		ke_(1) = b_(1)/a_(0);
		ke_(2) = b_(2)/a_(0);

		res.success = true;
		break;
	}
	default:
		std::cout<<"Plunger controller not implemented!\n";
		res.success = false;
		break;
	}
	this->lock_.unlock();
	return true;
}

//////////////////////////////////////////////////////////////////////////
// Callback function when subscriber connects
bool Plunger::serviceSetPIDCB(skinsim_ros_msgs::SetPIDGains::Request& req, skinsim_ros_msgs::SetPIDGains::Response& res)
{
	// Store data
	this->lock_.lock();

	this->Kp_ = req.Kp;
	this->Ki_ = req.Ki;
	this->Kd_ = req.Kd;

	if(controller_type_.selected == skinsim_ros_msgs::ControllerType::DIGITAL_PID)
	{
		N =100;			// TODO pass these values
		Ts = 0.001;

		a_(0) = (1+N*Ts);
		a_(1) = -(2 + N*Ts);
		a_(2) = 1;
		b_(0) = Kp_*(1+N*Ts) + Ki_*Ts*(1+N*Ts) + Kd_*N;
		b_(1) = -(Kp_*(2+N*Ts) + Ki_*Ts + 2*Kd_*N);
		b_(2) = Kp_ + Kd_*N;
		ku_(1) = a_(1)/a_(0);
		ku_(2) = a_(2)/a_(0);
		ke_(0) = b_(0)/a_(0);
		ke_(1) = b_(1)/a_(0);
		ke_(2) = b_(2)/a_(0);
	}

	res.success = true;

	this->lock_.unlock();
	return true;
}

//////////////////////////////////////////////////////////////////////////
// Callback function for force feedback subscriber
void Plunger::ForceFeedbackCB(const skinsim_ros_msgs::ForceFeedback::ConstPtr& _msg)
{
	this->lock_.lock();
	msg_fb_.force_applied = _msg->force_applied;
	msg_fb_.force_sensed  = _msg->force_sensed;
	num_contacts_         = _msg->contacts;
	this->lock_.unlock();
}

//////////////////////////////////////////////////////////////////////////
// Callback function when subscriber connects
void Plunger::RosConnect()
{
	this->ros_connections_++;
}

//////////////////////////////////////////////////////////////////////////
// Callback function when subscriber disconnects
void Plunger::RosDisconnect()
{
	this->ros_connections_--;
}

//////////////////////////////////////////////////////////////////////////
// Callback queue thread that processes messages
void Plunger::RosQueueThread()
{
	static const double timeout = 0.01;

	while (this->ros_node_->ok())
	{
		this->ros_queue_.callAvailable(ros::WallDuration(timeout));
	}
}
//////////////////////////////////////////////////////////////////////////
// Callback that receives the contact sensor's update signal.
void Plunger::OnContactUpdate()
{
	// Get all the contacts.
//	msgs::Contacts contacts;
//	contacts = this->contact_sensor_ptr_->GetContacts();
	num_contacts_ = this->contact_sensor_ptr_->GetContacts().contact_size();
//	std::cout<<num_contacts_<<", "<<this->contact_sensor_ptr_->GetCollisionContactCount("plunger::plunger_link::plunger_collision")<<"\n";
}

} // namespace

