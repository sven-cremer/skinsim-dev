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
	this->contact_mgr_ = this->world_->GetPhysicsEngine()->GetContactManager();

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

	// Store collision names
	for (unsigned int i = 0; i < this->num_joints_; ++i)
	{
		std::string link = this->joints_[i]->GetChild()->GetName();
		std::string name  = link + "_collision";
		collision_names_.push_back(name);
		collision_index_.push_back(false);
	}
//	past_collisions_window_ = 3;
//	past_collisions_.resize(num_joints_,past_collisions_window_);
//	past_collisions_.setZero();
//	past_forces_.resize(num_joints_,1);
//	past_forces_.setZero();

	f_app_     .resize(num_joints_);
	f_sen_     .resize(num_joints_);
	f_app_prev_.resize(num_joints_);
	f_sen_prev_.resize(num_joints_);
	f_app_     .setZero();
	f_sen_     .setZero();
	f_app_prev_.setZero();
	f_sen_prev_.setZero();

	// Compute distances
	math::Pose current;
	math::Pose target;

	for (int i = 0; i < this->joints_.size(); ++i)
	{
		// Select  joint
		current = this->joints_[i]->GetChild()->GetInitialRelativePose ();

		Distances tmp;
		tmp.index.resize( this->joints_.size() );
		tmp.distance.resize( this->joints_.size() );		// Includes distance to itself

		// Compute distances
		for (int j = 0; j < this->joints_.size(); ++j)
		{
				target = this->joints_[j]->GetChild()->GetInitialRelativePose ();
				tmp.distance[j] = current.pos.Distance(target.pos);
				tmp.index[j] = j;
		}

		// Sort values
		std::sort(tmp.index.begin(), tmp.index.end(), boost::bind(indexSort, _1, _2, tmp.distance));
		std::sort(tmp.distance.begin(), tmp.distance.end() );

		// Store result
		layout.push_back(tmp);
	}
	// Print results
//	for (int i = 0; i < this->joints_.size(); ++i)
//	{
//		std::cout<<this->joint_names_[i]<<"\n";
//		for (int j = 0; j < this->joints_.size(); ++j)
//		{
//			std::cout<<"  "<<joint_names_[ layout[i].index[j] ] << ": "<<layout[i].distance[j]<<"\n";
//		}
//	}


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

		// ROS topics
		ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<skinsim_ros_msgs::Joint1DArray>(
				this->topic_name_,1,
				boost::bind( &SkinJointGazeboRos::RosConnect,this),
				boost::bind( &SkinJointGazeboRos::RosDisconnect,this), ros::VoidPtr(), &this->ros_queue_);
		this->ros_pub_joint_ = this->ros_node_->advertise(ao);

		ros::AdvertiseOptions ao2 = ros::AdvertiseOptions::create<skinsim_ros_msgs::PointArray>(
				"layout",1,
				boost::bind( &SkinJointGazeboRos::emptyCB,this),
				boost::bind( &SkinJointGazeboRos::emptyCB,this), ros::VoidPtr(), &this->ros_queue_);		// TODO check if it is OK to use the same ros queue
		this->ros_pub_layout_ = this->ros_node_->advertise(ao2);

		ros::AdvertiseOptions ao3 = ros::AdvertiseOptions::create<visualization_msgs::MarkerArray>(
				"rviz",1,
				boost::bind( &SkinJointGazeboRos::emptyCB,this),
				boost::bind( &SkinJointGazeboRos::emptyCB,this), ros::VoidPtr(), &this->ros_queue_);		// TODO check if it is OK to use the same ros queue
		this->ros_pub_rviz_ = this->ros_node_->advertise(ao3);

		// ROS services
		this->ros_srv_ = this->ros_node_->advertiseService("publish_layout", &SkinJointGazeboRos::serviceCB, this);

		// Custom Callback Queue
		this->callback_ros_queue_thread_ = boost::thread( boost::bind( &SkinJointGazeboRos::RosQueueThread,this ) );

		this->joint_msg_.dataArray.resize(this->num_joints_);
	}

	// Update joints at every simulation iteration
	this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&SkinJointGazeboRos::UpdateJoints, this));

	// Debug
	std::cout<<"> Model name: "<<model_name_<<"\n";
	std::cout<<"> Model child count: "<<model_->GetChildCount()<<"\n";
	std::cout<<"> Model joint count: "<<model_->GetJointCount()<<"\n";
	std::cout<<"> Joint names count: "<<num_joints_<<"\n";

	std::cout<<"> rosNamespace: "<<this->ros_namespace_<<"\n";
	std::cout<<"> updateRate:   "<<this->update_rate_<<"\n";
	std::cout<<"> mass:         "<<this->mass_<<"\n";
	std::cout<<"> spring:       "<<this->sping_<<"\n";
	std::cout<<"> damping:      "<<this->damper_<<"\n";

	std::cout<<"GetInitialAnchorPose: "<<this->joints_[5]->GetInitialAnchorPose()<<"\n";
	std::cout<<"GetWorldPose: "<<this->model_->GetWorldPose()<<"\n";
	std::string link_name = "patch_0_spring_0";
	if(this->model_->GetChildLink(link_name))
	{
		std::cout<<"1: "<< this->model_->GetChildLink(link_name)->GetInitialRelativePose () <<"\n";
		std::cout<<"1: "<< this->joints_[0]->GetChild()->GetInitialRelativePose () <<"\n";
		std::cout<<"2: "<< this->model_->GetChildLink(link_name)->GetRelativePose ()        <<"\n";
		std::cout<<"3: "<< this->model_->GetChildLink(link_name)->GetWorldCoGPose ()        <<"\n";
		std::cout<<"4: "<< this->model_->GetChildLink(link_name)->GetWorldPose ()           <<"\n";
		std::cout<<"Collision property: "<<this->joints_[0]->GetChild()->GetCollisions()[0]->GetName()<<"\n";
	}
	std::cout << std::hex;
	std::cout << "Get type: " << this->joints_[0]->GetType() << "\n";
	std::cout << "physics::Base::HINGE_JOINT:  " << physics::Base::HINGE_JOINT <<"\n";
	std::cout << "physics::Base::SLIDER_JOINT: " << physics::Base::SLIDER_JOINT <<"\n";
	std::cout << std::dec;
	std::cout << "Has Hinge joint? " << this->joints_[0]->HasType(physics::Base::HINGE_JOINT) << "\n";
	std::cout << "Has Slider joint? " << this->joints_[0]->HasType(physics::Base::SLIDER_JOINT) << "\n";
	std::cout << this->joints_[0]->GetLocalAxis(0) <<"\n";
	std::cout << this->joints_[0]->GetForce(0) <<"\n";

	// RVIZ marker
	visualization_msgs::Marker m_vizMarker;
	m_vizMarker.header.frame_id = "world";

	m_vizMarker.ns = "skinsim";

	m_vizMarker.type = visualization_msgs::Marker::ARROW;
	m_vizMarker.action = visualization_msgs::Marker::ADD;

	m_vizMarker.pose.orientation.w = 0.70711;
	m_vizMarker.pose.orientation.x = 0;
	m_vizMarker.pose.orientation.y = 0.70711;
	m_vizMarker.pose.orientation.z = 0;

	m_vizMarker.scale.y = 0.01;
	m_vizMarker.scale.z = 0.01;

	m_vizMarker.color.a = 1.0;
	m_vizMarker.color.b = 0.0;

	m_vizMarker.header.stamp = ros::Time::now();

	for(int i=0;i<this->joint_names_.size();i++)
	{
		m_vizMarker.id = i;
		math::Pose p = this->joints_[i]->GetChild()->GetInitialRelativePose();

		m_vizMarker.pose.position.x = p.pos.x*4;	// Increase spacing
		m_vizMarker.pose.position.y = p.pos.y*4;
		m_vizMarker.pose.position.z = p.pos.z;

		msg_rviz_.markers.push_back(m_vizMarker);
	}

	// Set MSD parameters for Gazebo
	for(int i=0;i<this->joint_names_.size();i++)
	{
		//this->joints_[i]->SetStiffnessDamping(0,122,1.1,0);
		this->joints_[i]->SetStiffnessDamping(0,0,0,0);			// TODO is this necessary?
	}

	// Digital filter
	int order = 2;
	a_filt.resize(order+1);
	b_filt.resize(order+1);
	a_filt<<1,-1.561,0.6414;
	b_filt<<0.0201,0.0402,0.0201;
	for(int i=0;i<num_joints_;i++)
	{
		ice::digitalFilter tmp;
		if(!tmp.init(order, true, b_filt, a_filt))
		{
			std::cerr<<"Failed to init digital filter!\n";\
		}
		digitalFilters.push_back(tmp);
	}
	useDigitalFilter = false;
}

//////////////////////////////////////////////////////////////////////////
// Update the controller
void SkinJointGazeboRos::UpdateJoints()
{
	// TODO make parameters
	double rest_angle = 0.0;
	double current_angle = 0;
	//double current_force = 0;
	double current_velocity = 0;
	//double force = 0;
	double force_dist =0;

	common::Time cur_time = this->world_->GetSimTime();

	// Rate control
//	if (this->update_rate_ > 0 && (cur_time-this->last_time_).Double() < (1.0/this->update_rate_))
//		return;

	// Initialize collision variables
//	past_collisions_.block(0,0,num_joints_,past_collisions_window_-1) = past_collisions_.block(0,1,num_joints_,past_collisions_window_-1);	// Shift left
	for (unsigned int i = 0; i < num_joints_; ++i)
	{
//		past_collisions_(i,past_collisions_window_-1) = 0;
		collision_index_[i] = false;
	}

	// Check for contacts
	// FIXME: If there are no subscribers to ~/physics/contacts, then the return value will be NULL.
	std::vector<physics::Contact*> contacts_ = contact_mgr_->GetContacts();
    for (unsigned int i = 0; i < contact_mgr_->GetContactCount(); ++i)	// Note: do not iterate from 0 to .size()
    {
    	//contacts_[i]->collision1->GetLink()->GetName()

    	std::string name = contacts_[i]->collision1->GetName();
    	int idx = std::find(collision_names_.begin(), collision_names_.end(), name) - collision_names_.begin();

    	if (idx < collision_names_.size()) // Name found
    	{
    		collision_index_[idx] = true;
    		//past_collisions_(idx,past_collisions_window_-1) = 1;
    	}
    	else	// Name not found
    	{
    		name = contacts_[i]->collision2->GetName();
    		idx = std::find(collision_names_.begin(), collision_names_.end(), name) - collision_names_.begin();

    		if (idx < collision_names_.size()) // Name found
        	{
    			collision_index_[idx] = true;
    			//past_collisions_(idx,past_collisions_window_-1) = 1;
        	}
    	}
    }
    // Filter contacts
//	for (unsigned int i = 0; i < num_joints_; ++i)
//	{
//		if( past_collisions_.row(i).sum() >= 2)		// Physics engine screwed up, use old values instead
//			collision_index_[i] = true;
//	}

	// Compute applied force
	for (unsigned int i = 0; i < this->num_joints_; ++i)
	{
			current_angle = this->joints_[i]->GetAngle(0).Radian();
			current_velocity = this->joints_[i]->GetVelocity(0);

			// This sets the mass-spring-damper dynamics, currently only spring and damper
			f_app_(i) = (rest_angle - current_angle) * sping_ - damper_ * current_velocity;
	}
	// Apply low-pass filter	TODO filter velocity instead?
	if(useDigitalFilter)
	{
		for (unsigned int i = 0; i < this->num_joints_; ++i)
        {
			f_app_prev_(i) = digitalFilters[i].getNextFilteredValue(f_app_(i));
        }
	}
	// Set Dynamics
	for (unsigned int i = 0; i < this->num_joints_; ++i)
	{
		this->joints_[i]->SetForce(0, f_app_(i));

//		if( fabs(force - this->joints_[ i ]->GetForce(0) ) > 0.00001 )
//			std::cout<<"["<<(this->world_->GetSimTime()).Double()<<"] "<<"Force "<<i<<": "<<force<<" -> "<<this->joints_[ i ]->GetForce(0)<<"\n";

	}
	// Compute sensed force
	f_sen_.setZero();
	for (unsigned int i = 0; i < this->num_joints_; ++i)
	{
		if(collision_index_[i])
		{
			f_sen_(i) = -0.9*f_app_(i);	// TODO scale force

			// Force distribution
			for (int j = 1; j < this->num_joints_; ++j)	// skip first
			{
				if(!collision_index_[ layout[i].index[j] ])
				{
					// Apply force distribution model
					force_dist = -f_app_(i) * exp(- 1.0 * pow(layout[i].distance[j]/0.01,2) );

					f_sen_( layout[i].index[j] ) += force_dist;

					if( fabs(force_dist)<0.00001 )	// Stop once force becomes negligible
						break;

//					if( fabs(force_dist - this->joints_[ layout[i].index[j] ]->GetForce(0) ) > 0.00001 )
//						std::cout<<"["<<(this->world_->GetSimTime()).Double()<<"] "<<"Dist "<<i<<" ("<<j<<"): "<<force_dist<<" -> "<<this->joints_[ layout[i].index[j] ]->GetForce(0)<<"\n";

				}
			}
		}
	}

    // Publish RVIZ marker
	this->lock_.lock();
	ros::Time t_stamp = ros::Time::now();
	for(int i=0;i<this->joint_names_.size();i++)
	{
		msg_rviz_.markers[i].header.stamp = t_stamp;
		msg_rviz_.markers[i].pose.position.z = this->joints_[i]->GetAngle(0).Radian();

		msg_rviz_.markers[ i ].scale.x = f_sen_(i); // this->joints_[ i ]->GetForce(0);
		//msg_rviz_.markers[i].scale.x = -this->joints_[i]->GetForce(0);	// Get external force set by user
																		// TODO not reliable so keep track of forces instead
		if(collision_index_[i])
		{
			msg_rviz_.markers[i].color.r = 1.0;
			msg_rviz_.markers[i].color.g = 0.0;
		}
		else
		{
			msg_rviz_.markers[i].color.r = 0.0;
			msg_rviz_.markers[i].color.g = 1.0;
		}
	}
	this->ros_pub_rviz_ .publish(this->msg_rviz_);
	this->lock_.unlock();


    // Decide whether to publish
	if (this->ros_connections_ == 0 || !pub_to_ros_)
		return;

	this->lock_.lock();
	// TODO: Copy data into ROS message
//	this->skin_msg_.header.stamp.sec = (this->world_->GetSimTime()).sec;
//	this->skin_msg_.header.stamp.nsec = (this->world_->GetSimTime()).nsec;
//	this->skin_msg_.time = (this->world_->GetSimTime()).Double();

	for(int i=0;i<this->num_joints_;i++)
	{
		this->joint_msg_.dataArray[i].position = this->joints_[i]->GetAngle(0).Radian();

		this->joint_msg_.dataArray[i].velocity = this->joints_[i]->GetVelocity(0);

		this->joint_msg_.dataArray[i].force = msg_rviz_.markers[ i ].scale.x; //this->joints_[i]->GetForce(0);
//		physics::JointWrench wrench = this->joints_[i]->GetForceTorque(0);	// Get internal + external forces
//		math::Vector3 force = wrench.body2Force;
//		//this->joint_msg_.dataArray[i].force = force.GetLength();
//		std::cout<<force<<"\t"<<	this->joints_[i]->GetForce(0)*this->joints_[i]->GetLocalAxis(0) <<"\n";

		//this->joint_msg_.dataArray[i].collisions = this->contact_mgr_->GetContactCount();
		if(collision_index_[i])
			this->joint_msg_.dataArray[i].collisions = 1;
		else
			this->joint_msg_.dataArray[i].collisions = 0;
	}

	this->ros_pub_joint_.publish(this->joint_msg_);
	this->lock_.unlock();

	// save last time stamp
	this->last_time_ = cur_time;
}

//////////////////////////////////////////////////////////////////////////
// Callback function when subscriber connects
bool SkinJointGazeboRos::serviceCB(skinsim_ros_msgs::Empty::Request& req, skinsim_ros_msgs::Empty::Response& res)
{
	math::Pose p;

	this->lock_.lock();

	// Copy position data into ROS message
	layout_msg_.data.resize(this->num_joints_);

	for(int i=0;i<this->num_joints_;i++)
	{
		p = this->joints_[i]->GetChild()->GetInitialRelativePose();

		layout_msg_.data[i].x = p.pos.x;
		layout_msg_.data[i].y = p.pos.y;
		layout_msg_.data[i].z = p.pos.z;
	}

	this->ros_pub_layout_.publish(this->layout_msg_);

	this->lock_.unlock();

	return true;
}

//////////////////////////////////////////////////////////////////////////
// Empty callback function
void SkinJointGazeboRos::emptyCB()
{
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
