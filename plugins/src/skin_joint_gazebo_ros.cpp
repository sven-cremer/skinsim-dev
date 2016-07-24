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
 * skin_joint_gazebo_ros.cpp
 *
 *  Created on: Jul 5, 2016
 *      Author: Sven Cremer
 */

#include <SkinSim/skin_joint_gazebo_ros.h>

namespace gazebo
{
// Register plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SkinJointGazeboRos);

//////////////////////////////////////////////////////////////////////////
// Constructor
SkinJointGazeboRos::SkinJointGazeboRos()
{
	this->ros_connections_ = 0;
}

//////////////////////////////////////////////////////////////////////////
// Destructor
SkinJointGazeboRos::~SkinJointGazeboRos()
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
void SkinJointGazeboRos::Load( physics::ModelPtr _model, sdf::ElementPtr _sdf )
{
	// Save pointers and names
	this->model_ = _model;
	this->world_ = this->model_->GetWorld();

	this->model_name_ = model_->GetName();
	this->topic_name_ = model_name_;

	// Set default values
	this->ros_namespace_ = "skinsim/";
	this->update_rate_   = 0;
	this->mass_          = 0.0;
    this->sping_         = 122.24;
    this->damper_        = 1.83;

	// Load parameters from model SDF file
	if (_sdf->HasElement("rosNamespace"))
		this->ros_namespace_ = _sdf->GetElement("rosNamespace")->Get<std::string>() + "/";

	if (_sdf->HasElement("updateRate"))
		this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

	if (_sdf->HasElement("mass"))
		this->mass_ = _sdf->GetElement("mass")->Get<double>();

	if (_sdf->HasElement("spring"))
		this->sping_ = _sdf->GetElement("spring")->Get<double>();

	if (_sdf->HasElement("damping"))
		this->damper_ = _sdf->GetElement("damping")->Get<double>();

	// Load joint names from file
	std::string fullname;
	getModelConfigPath( fullname, _sdf );
	fullname = fullname + std::string("/joint_names.yaml");
	input_file_.open(fullname.c_str());
	std::cout<<"Loading file: "<<fullname<<"\n";
	YAML::Node doc;
	doc = YAML::LoadAll(input_file_);
	for(std::size_t i=0;i<doc[0].size();i++)
	{
		this->joint_names_.push_back( doc[0][i]["Joint"].as<std::string>() );
		//std::cout << this->joint_names_[i]<< std::endl;
	}
	input_file_.close();

	// Get pointers to joints from Gazebo
	this->num_joints_ = this->joint_names_.size();
	this->joints_.resize(this->num_joints_);
	for (unsigned int i = 0; i < this->joints_.size(); ++i)
	{
		this->joints_[i] = this->model_->GetJoint(this->joint_names_[i]);
		if (!this->joints_[i])
		{
			//ROS_ERROR("SkinSim robot expected joint[%s] not present, plugin not loaded", this->jointNames[i].c_str());
			std::cerr<<"SkinSim robot expected joint ["<<this->joint_names_[i].c_str()<<"] not present, plugin not loaded \n";
			return;
		}
	}

//    // Create a new Gazbeo transport node
//    transport::NodePtr node(new transport::Node());
//
//    // Initialize the node with the Model name
//    node->Init("test123");


	// Make sure the ROS node for Gazebo has already been initialized
	if (!ros::isInitialized())
	{
		this->pub_to_ros_ = false;

		std::cout<<"Warning: The ROS node for Gazebo has not been initialized! \n"
				 <<"Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package \n";
//		int argc = 0;
//		char **argv = NULL;
//		ros::init(argc, argv, "gazebo_ros_client", ros::init_options::NoSigintHandler);
	}
	else
	{
		this->pub_to_ros_ = true;

		this->ros_node_ = new ros::NodeHandle(this->ros_namespace_);

		// Custom Callback Queue
		ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<skinsim_ros_msgs::Joint1DArray>(
				this->topic_name_,1,
				boost::bind( &SkinJointGazeboRos::RosConnect,this),
				boost::bind( &SkinJointGazeboRos::RosDisconnect,this), ros::VoidPtr(), &this->ros_queue_);
		this->ros_pub_ = this->ros_node_->advertise(ao);

		// Custom Callback Queue
		this->callback_ros_queue_thread_ = boost::thread( boost::bind( &SkinJointGazeboRos::RosQueueThread,this ) );

		this->skin_msg_.dataArray.resize(this->num_joints_);
	}

	// Update joints at every simulation iteration
	this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&SkinJointGazeboRos::UpdateJoints, this));

	// Debug
	std::cout<<"> Model name: "<<model_name_<<"\n";
	std::cout<<"> Model child count: "<<model_->GetChildCount()<<"\n";
	std::cout<<"> Model joint count: "<<model_->GetJointCount()<<"\n";
	std::cout<<"> Num joint names: "<<num_joints_<<"\n";

	std::cout<<"> rosNamespace: "<<this->ros_namespace_<<"\n";
	std::cout<<"> updateRate:   "<<this->update_rate_<<"\n";
	std::cout<<"> mass:         "<<this->mass_<<"\n";
	std::cout<<"> spring:       "<<this->sping_<<"\n";
	std::cout<<"> damping:      "<<this->damper_<<"\n";

}

//////////////////////////////////////////////////////////////////////////
// Update the controller
void SkinJointGazeboRos::UpdateJoints()
{
	// TODO make parameters
	double rest_angle = 0.0;
	double current_angle = 0;
	double current_force = 0;
	double current_velocity = 0;
	double sens_force = 0;

	common::Time cur_time = this->world_->GetSimTime();

	// Rate control
//	if (this->update_rate_ > 0 && (cur_time-this->last_time_).Double() < (1.0/this->update_rate_))
//		return;

    for (unsigned int i = 0; i < this->joints_.size(); ++i)
    {
      current_angle = this->joints_[i]->GetAngle(0).Radian();
      current_velocity = this->joints_[i]->GetVelocity(0);

      // This sets the mass-spring-damper dynamics, currently only spring and damper
      this->joints_[i]->SetForce(0, (rest_angle - current_angle) * sping_ - damper_ * current_velocity);
    }

    // Decide whether to publish
	if (this->ros_connections_ == 0 || !pub_to_ros_)
		return;

	this->lock_.lock();

	// TODO: Copy data into ROS message
//	this->skin_msg_.header.stamp.sec = (this->world_->GetSimTime()).sec;
//	this->skin_msg_.header.stamp.nsec = (this->world_->GetSimTime()).nsec;
//	this->skin_msg_.time = (this->world_->GetSimTime()).Double();

	for(int i=0;i<this->joint_names_.size();i++)
	{
		this->skin_msg_.dataArray[i].position = this->joints_[i]->GetAngle(0).Radian();

		this->skin_msg_.dataArray[i].velocity = this->joints_[i]->GetVelocity(0);

		this->skin_msg_.dataArray[i].force = this->joints_[i]->GetForce(0);
	}

	this->ros_pub_.publish(this->skin_msg_);
	this->lock_.unlock();

	// save last time stamp
	this->last_time_ = cur_time;
}

//////////////////////////////////////////////////////////////////////////
// Callback function when subscriber connects
void SkinJointGazeboRos::RosConnect()
{
	this->ros_connections_++;
}

//////////////////////////////////////////////////////////////////////////
// Callback function when subscriber disconnects
void SkinJointGazeboRos::RosDisconnect()
{
	this->ros_connections_--;
}

//////////////////////////////////////////////////////////////////////////
// Callback queue thread that processes messages
void SkinJointGazeboRos::RosQueueThread()
{
	static const double timeout = 0.01;

	while (this->ros_node_->ok())
	{
		this->ros_queue_.callAvailable(ros::WallDuration(timeout));
	}
}

}
