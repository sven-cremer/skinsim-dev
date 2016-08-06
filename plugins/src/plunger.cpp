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
	this->topic_name_ = model_name_;
	this->joint_name_ = "plunger_joint"; //model_name_+"_joint";

	// Set default values
	this->ros_namespace_ = "skinsim/";
	this->update_rate_   = 0;
	this->Kp_            = 0.1;
	this->Kd_            = 0.01;

	// Load parameters from model SDF file
	if (_sdf->HasElement("rosNamespace"))
		this->ros_namespace_ = _sdf->GetElement("rosNamespace")->Get<std::string>() + "/";

	if (_sdf->HasElement("updateRate"))
		this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

	if (_sdf->HasElement("Kp"))
		this->Kp_ = _sdf->GetElement("Kp")->Get<double>();

	if (_sdf->HasElement("Kd"))
		this->Kd_ = _sdf->GetElement("Kd")->Get<double>();

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

		// Custom Callback Queue
		ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<geometry_msgs::WrenchStamped>(
				this->topic_name_,1,
				boost::bind( &Plunger::RosConnect,this),
				boost::bind( &Plunger::RosDisconnect,this), ros::VoidPtr(), &this->ros_queue_);
		this->ros_pub_ = this->ros_node_->advertise(ao);

		msg_wrench_.header.frame_id = "world";

		// Custom Callback Queue
		this->callback_ros_queue_thread_ = boost::thread( boost::bind( &Plunger::RosQueueThread,this ) );
	}

	// Update joints at every simulation iteration
	this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&Plunger::UpdateJoints, this));

	// Controller
	this->force_desired_ = -2.5;
	this->force_prev_    = 0.0;

	// P-controller
	this->pid_ = common::PID(JointPgain_, JointIgain_, JointDgain_, 15, -15);

	// Apply the P-controller to the joint.
	//this->joint_pid_->SetPositionPID(this->joint_->GetScopedName(), this->pid_);

	// Debug
	std::cout<<"> Model name: "<<model_name_<<"\n";
	std::cout<<"> Joint scoped name: "<<this->joint_->GetScopedName()<<"\n";
	std::cout<<"> Joint PID: "<<JointPgain_<<", "<<JointIgain_<<", "<<JointDgain_<<"\n";
	std::cout<<"> Kp: "<<Kp_<<"\tKd: "<<Kd_<<"\n";
	for(int i=0;i<3;i++)
	{
		std::cout<<"> Local axis  ["<<i<<"]: "<<this->joint_->GetLocalAxis(i)<<"\n";
		std::cout<<"> Global axis ["<<i<<"]: "<<this->joint_->GetGlobalAxis(i)<<"\n";
	}
	//this->joint_->SetStiffnessDamping(0,122,1.1,0);
	this->joint_->SetStiffnessDamping(0,0,0,0);
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

	this->position_current_ = this->joint_->GetAngle(0).Radian();
	this->velocity_ 		= this->joint_->GetVelocity(0);
	this->wrench_ 			= this->joint_->GetForceTorque(0);

	this->force_  = this->wrench_.body2Force;
	this->torque_ = this->wrench_.body2Torque;

	this->force_current_ = this->force_.Dot(this->axis_global_);

	double force_dot_ = (force_current_-force_prev_)/step_time.Double();

	// FIXME Controller dynamics
	position_desired_ = Kp_*(force_desired_ - force_current_) - Kd_*force_dot_;
	force_prev_ = force_current_;
	//	if(!this->joint_->SetPosition(0, position_desired_))
//	{
//		std::cout<<"Plunger: SetPosition failed!\n";
//	}
	//this->joint_pid_->SetPositionTarget(this->joint_->GetScopedName(),position_desired_);
//	this->joint_pid_->Update();

	// common::PID
	double postion_error = position_current_ - position_desired_;
	double effort = this->pid_.Update(postion_error, step_time);
	this->joint_->SetForce(0, effort);

	// Debug
	std::cout<<"F cur: "<<force_current_<<" \tF dot: "<<force_dot_<<" \tEffort: "<<effort;//<<" \t<-\t"<<force_<<"\n";
	std::cout<<"\tPos cur: "<<position_current_<<"\tPos des: "<<position_desired_<<"\n";

	// Decide whether to publish
	if (this->ros_connections_ == 0 || !pub_to_ros_)
		return;

	this->lock_.lock();
	// Copy data into ROS message
	this->msg_wrench_.header.stamp.sec  = (this->world_->GetSimTime()).sec;
	this->msg_wrench_.header.stamp.nsec = (this->world_->GetSimTime()).nsec;
	this->msg_wrench_.wrench.force.x  = force_ .x;
	this->msg_wrench_.wrench.force.y  = force_ .y;
	this->msg_wrench_.wrench.force.z  = force_ .z;
	this->msg_wrench_.wrench.torque.x = torque_.x;
	this->msg_wrench_.wrench.torque.y = torque_.y;
	this->msg_wrench_.wrench.torque.z = torque_.z;
	// Publish data
	this->ros_pub_.publish(this->msg_wrench_);
	this->lock_.unlock();

	// Save last time stamp
	this->last_time_ = cur_time;
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

}

