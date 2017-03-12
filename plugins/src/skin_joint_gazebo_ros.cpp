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
	this->ros_connections_joint_   = 0;
	this->ros_connections_rviz_    = 0;
	this->ros_connections_tactile_ = 0;
	this->ros_connections_fb_      = 0;
	this->ros_connections_COP_     = 0;
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
    this->spread_scaling = 1.0;
    this->spread_sigma   = 0.01;
    this->sigma_         = 0.0;
    this->mu_            = 0.0;
    this->delay_         = 0.0;

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

	if (_sdf->HasElement("spreadScaling"))
		this->spread_scaling = _sdf->GetElement("spreadScaling")->Get<double>();

	if (_sdf->HasElement("spreadSigma"))
		this->spread_sigma = _sdf->GetElement("spreadSigma")->Get<double>();

	if (_sdf->HasElement("noiseSigma"))
			this->sigma_ = _sdf->GetElement("noiseSigma")->Get<double>();

	if (_sdf->HasElement("noiseMu"))
			this->mu_ = _sdf->GetElement("noiseMu")->Get<double>();

	if (_sdf->HasElement("delay"))
			this->delay_ = _sdf->GetElement("delay")->Get<double>();

	if (_sdf->HasElement("noiseAmplitude"))
				this->noiseAmplitude_ = _sdf->GetElement("noiseAmplitude")->Get<double>();

	// Compute force spread parameters
	this->spread_variance = spread_sigma*spread_sigma;
    //this->K_spread = spread_scaling / sqrt( 2*spread_variance*M_PI );

	// Load joint names from file
	std::string fullname;
	getModelConfigPath( fullname, _sdf );
	fullname = fullname + std::string("/joint_names.yaml");
	input_file_.open(fullname.c_str());
	std::cout<<"> Loading file: "<<fullname<<"\n";
	YAML::Node doc;
	doc = YAML::LoadAll(input_file_);
	for(std::size_t i=0;i<doc[0].size();i++)
	{
		this->joint_names_.push_back( doc[0][i].as<std::string>() );
		//std::cout << this->joint_names_[i]<< std::endl;
	}
	input_file_.close();
	this->num_joints_ = this->joint_names_.size();

	// Get pointers to joints from Gazebo
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

	// Load tactile sensors
	getModelConfigPath( fullname, _sdf );
	fullname = fullname + std::string("/tactile_id.yaml");
	input_file_.open(fullname.c_str());
	std::cout<<"> Loading file: "<<fullname<<"\n";
	YAML::Node node;
	node = YAML::LoadAll(input_file_);
	for (unsigned int i = 0; i < this->num_joints_; ++i)
	{
		if( node[0][ joint_names_[i] ] )
		{
			bool sensor_already_added = false;
			int index = node[0][ joint_names_[i] ].as<int>();
			for (unsigned int j = 0; j < sensors_.size(); ++j)
			{
				if( sensors_[j].index == index )
				{
					sensors_[j].joint_names.push_back( joint_names_[i] );
					sensors_[j].joint_index.push_back( i );
					sensor_already_added = true;
					break;
				}
			}
			if(!sensor_already_added)
			{
				Tactile tmp;
				tmp.index = index;
				tmp.joint_names.push_back( joint_names_[i] );
				tmp.joint_index.push_back( i );
				sensors_.push_back(tmp);
			}
		}
	}
	input_file_.close();
	// Compute tactile sensor positions
	for (unsigned int i = 0; i < sensors_.size(); ++i)
	{
		//std::cout<<"Tactile "<<sensors_[i].index<<": ";
		sensors_[i].position.Set(0, 0, 0);
		for (unsigned int j = 0; j < sensors_[i].joint_names.size(); ++j)
		{
			//std::cout<<sensors_[i].joint_names[j]<<" ("<<sensors_[i].joint_index[j]<<"), ";
			int index = sensors_[i].joint_index[j];
			target = this->joints_[index]->GetChild()->GetInitialRelativePose ();
			sensors_[i].position += target.pos;
		}
		sensors_[i].position.x = sensors_[i].position.x / sensors_[i].joint_names.size();
		sensors_[i].position.y = sensors_[i].position.y / sensors_[i].joint_names.size();
		//std::cout<<"("<<sensors_[i].position<<")\n";
	}
	this->num_sensors_ = this->sensors_.size();

	// Compute and save initial skin layout into ROS message
	msg_layout_elements_.data.resize(this->num_joints_);
	for( int i=0; i < this->num_joints_; i++ )
	{
		math::Pose p = this->joints_[i]->GetChild()->GetInitialRelativePose();

		msg_layout_elements_.data[i].x = p.pos.x;
		msg_layout_elements_.data[i].y = p.pos.y;
		msg_layout_elements_.data[i].z = p.pos.z;
	}
	msg_layout_sensors_.data.resize(this->num_sensors_);
	for( int i = 0; i < this->num_sensors_; ++i )
	{
		msg_layout_sensors_.data[i].x = sensors_[i].position.x;
		msg_layout_sensors_.data[i].y = sensors_[i].position.y;
		msg_layout_sensors_.data[i].z = sensors_[i].position.z;
	}

	/* Init filters
	std::vector<digitalFilter> filters_;
	digitalFilter temp;
	temp.init(..)
	for( < num_sensors)
	filters_.push_back(temp)
	*/
	int num_coeff;
	Eigen::VectorXd a_filt;			// Filter coefficients (denominator)
	Eigen::VectorXd b_filt;			// Filter coefficients (numerator)

	a_filt.resize(2);
	b_filt.resize(2);

	a_filt << 1.0, 0.2679;
	b_filt << 0.634, 0.634;

	std::cout<<"Loaded /a_filt: \n"<<a_filt<<"\n---\n";
	std::cout<<"Loaded /b_filt: \n"<<b_filt<<"\n---\n";

	num_coeff = a_filt.size();
	int order = num_coeff-1;
	std::cout<<"Filter order: "<<order<<"\n";

	ice::digitalFilter tmp;
	if(!tmp.init(order, true, b_filt, a_filt))
	{
		ROS_ERROR("Failed to init digital filter!");
	}
	for(int i=0;i<this->num_sensors_;i++)
	{

		digitalFilters.push_back(tmp);
	}

	// Center of Pressure (COP)
	this->cop_force_ .resize(this->num_sensors_);
	this->cop_x_     .resize(this->num_sensors_);
	this->cop_y_     .resize(this->num_sensors_);
	this->seed       = 1;								//TODO randomize seed

	// Make sure the ROS node for Gazebo has already been initialized
	if (!ros::isInitialized())
	{
		this->pub_to_ros_ = false;

		std::cerr<<" Warning: The ROS node for Gazebo has not been initialized! \n"
				 <<"          Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package \n";
	}
	// Advertise ROS topics and services
	else
	{
		this->pub_to_ros_ = true;

		this->ros_node_ = new ros::NodeHandle(this->ros_namespace_);

		// ROS topics
		ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<skinsim_ros_msgs::Joint1DArray>(
				"joint_data",1,
				boost::bind( &SkinJointGazeboRos::RosConnectJoint,this),
				boost::bind( &SkinJointGazeboRos::RosDisconnectJoint,this), ros::VoidPtr(), &this->ros_queue_);
		this->ros_pub_joint_ = this->ros_node_->advertise(ao);

		ros::AdvertiseOptions ao2 = ros::AdvertiseOptions::create<skinsim_ros_msgs::PointArray>(
				"layout",1,
				boost::bind( &SkinJointGazeboRos::emptyCB,this),
				boost::bind( &SkinJointGazeboRos::emptyCB,this), ros::VoidPtr(), &this->ros_queue_);
		this->ros_pub_layout_ = this->ros_node_->advertise(ao2);

		ros::AdvertiseOptions ao3 = ros::AdvertiseOptions::create<visualization_msgs::MarkerArray>(
				"rviz",1,
				boost::bind( &SkinJointGazeboRos::RosConnectRviz,this),
				boost::bind( &SkinJointGazeboRos::RosDisconnectRviz,this), ros::VoidPtr(), &this->ros_queue_);
		this->ros_pub_rviz_ = this->ros_node_->advertise(ao3);

		ros::AdvertiseOptions ao4 = ros::AdvertiseOptions::create<skinsim_ros_msgs::TactileData>(
				"tactile_data",1,
				boost::bind( &SkinJointGazeboRos::RosConnectTactile,this),
				boost::bind( &SkinJointGazeboRos::RosDisconnectTactile,this), ros::VoidPtr(), &this->ros_queue_);		// TODO check if it is OK to use the same ros queue
		this->ros_pub_tactile_ = this->ros_node_->advertise(ao4);

		ros::AdvertiseOptions ao5 = ros::AdvertiseOptions::create<skinsim_ros_msgs::ForceFeedback>(
				"force_feedback",1,
				boost::bind( &SkinJointGazeboRos::RosConnectFb,this),
				boost::bind( &SkinJointGazeboRos::RosDisconnectFb,this), ros::VoidPtr(), &this->ros_queue_);
		this->ros_pub_fb_ = this->ros_node_->advertise(ao5);

		ros::AdvertiseOptions ao6 = ros::AdvertiseOptions::create<skinsim_ros_msgs::CenterOfPressure>(
				"center_of_pressure",1,
				boost::bind( &SkinJointGazeboRos::RosConnectCOP,this),
				boost::bind( &SkinJointGazeboRos::RosDisconnectCOP,this), ros::VoidPtr(), &this->ros_queue_);
		this->ros_pub_COP_ = this->ros_node_->advertise(ao6);

		// ROS services
		this->ros_srv_ = this->ros_node_->advertiseService("publish_layout", &SkinJointGazeboRos::serviceCB, this);

		// Custom Callback Queue
		this->callback_ros_queue_thread_ = boost::thread( boost::bind( &SkinJointGazeboRos::RosQueueThread,this ) );

		// ROS message initialization
		this->msg_joint_.dataArray.resize(this->num_joints_);
		this->msg_tactile_.data.resize(this->num_sensors_);

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
	}

	// Update joints at every simulation iteration
	this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&SkinJointGazeboRos::UpdateJoints, this));

	// Debug
	std::cout<<"> Model name: "<<model_name_<<"\n";
	/*
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
	*/

	// Set MSD parameters for Gazebo
	this->rest_angle_ = 0.0;
	for(int i=0;i<this->joint_names_.size();i++)
	{
		//this->joints_[i]->SetStiffnessDamping(0,122,1.1,0);
		this->joints_[i]->SetStiffnessDamping(0,this->sping_,this->damper_,this->rest_angle_);			// Use ODE to simulate MSD (this is slightly faster)
	}																									// Note that this->joints_[i]->GetForce(0) measures externally applied forces, so this will be zero!
}

//////////////////////////////////////////////////////////////////////////
// Update the controller
void SkinJointGazeboRos::UpdateJoints()
{

	common::Time cur_time = this->world_->GetSimTime();

	// Rate control
//	if (this->update_rate_ > 0 && (cur_time-this->last_time_).Double() < (1.0/this->update_rate_))
//		return;

	// Initialize collision variables
	for (unsigned int i = 0; i < num_joints_; ++i)
	{
		collision_index_[i] = false;
	}
	num_contacts_ = 0;

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
    		num_contacts_++;
    	}
    	else	// Name not found
    	{
    		name = contacts_[i]->collision2->GetName();
    		idx = std::find(collision_names_.begin(), collision_names_.end(), name) - collision_names_.begin();

    		if (idx < collision_names_.size()) // Name found
        	{
    			collision_index_[idx] = true;
    			num_contacts_++;
        	}
    	}
    }

	// Compute applied force
    double current_velocity = 0;
    double current_angle = 0;
	for (unsigned int i = 0; i < this->num_joints_; ++i)
	{
			current_angle = this->joints_[i]->GetAngle(0).Radian();
			current_velocity = this->joints_[i]->GetVelocity(0);

			// This sets the mass-spring-damper dynamics, currently only spring and damper
			f_app_(i) = (rest_angle_ - current_angle) * sping_ - damper_ * current_velocity;	// This is the same value that ODE computes, TODO could maybe get this from GetForceTorque()

//			physics::JointWrench wrench = this->joints_[i]->GetForceTorque(0);	// Get internal + external forces
//			math::Vector3 force = wrench.body2Force;
//			std::cout<<force<<"\t"<<	this->joints_[i]->GetForce(0)*this->joints_[i]->GetLocalAxis(0) <<"\n";
	}

	// Set Dynamics
	/*
	for (unsigned int i = 0; i < this->num_joints_; ++i)
	{
		this->joints_[i]->SetForce(0, f_app_(i));

		//if( fabs(f_app_(i) - this->joints_[ i ]->GetForce(0) ) > 0.00001 )	// This works!
		//	std::cout<<"["<<(this->world_->GetSimTime()).Double()<<"] "<<"Force "<<i<<": "<<f_app_(i)<<" -> "<<this->joints_[ i ]->GetForce(0)<<"\n";
	}
	*/

	// Sensed Force: compute force distribution
	double force_dist = 0;
	f_sen_.setZero();
	for (unsigned int i = 0; i < this->num_joints_; ++i)
	{
		// Force distribution model
		if(collision_index_[i])
		{
			for (int j = 0; j < this->num_joints_; ++j)	// First entry has 0 distance, i.e. i == layout[i].index[0]
			{
				// Apply force distribution model
				force_dist = -f_app_(i) * spread_scaling * exp( -pow(layout[i].distance[j],2) / (2*spread_variance) );

				f_sen_( layout[i].index[j] ) += force_dist;

				if( fabs(force_dist)<0.00001 )	// Stop once force becomes negligible
					break;

//				if( fabs(force_dist - this->joints_[ layout[i].index[j] ]->GetForce(0) ) > 0.00001 )
//					std::cout<<"["<<(this->world_->GetSimTime()).Double()<<"] "<<"Dist "<<i<<" ("<<j<<"): "<<force_dist<<" -> "<<this->joints_[ layout[i].index[j] ]->GetForce(0)<<"\n";

			}
		}
	}

	// Sensed Force: add noise
	if(false)	// TODO add variable
	{
		for (unsigned int i = 0; i < this->num_joints_; ++i)
		{
			f_sen_(i) += CalulateNoise(this->mu_,this->sigma_);		// Added SNR
		}
	}

	// Sensed Force: add time delay
	if(this->delay_ > 0)
	{
		// TODO use buffer
	}

	// Compute tactile data
	for (unsigned int i = 0; i < sensors_.size(); ++i)
	{
		sensors_[i].force_applied = 0;
		sensors_[i].force_sensed  = 0;
		for (unsigned int j = 0; j < sensors_[i].joint_index.size(); ++j)
		{
			int index = sensors_[i].joint_index[j];
			//std::cout<<index<<": "<<f_app_(index)<<" N,";
			sensors_[i].force_applied += f_app_(index);
			sensors_[i].force_sensed  += f_sen_(index);
		}
		//sensors_[i].force_sensed  += (this->noiseAmplitude_ * (CalulateNoise(this->mu_,this->sigma_)/sqrt(sensors_[i].joint_index.size())));
		//std::cout<<"\nSensor "<<i<<":\tf_app="<<sensors_[i].force_applied<<"\tf_sen="<<sensors_[i].force_sensed<<"\n";
	}

	// Filter sensed force
	if(true)
	{
		for (unsigned int i = 0; i < this->num_sensors_; ++i)
		{
			sensors_[i].force_sensed = digitalFilters[i].getNextFilteredValue(sensors_[i].force_sensed);
		}
	}

    // Decide whether to publish
//	if (!pub_to_ros_)
//		return;

	// Publish joint data
	if(this->ros_connections_joint_ > 0 )
	{
		this->lock_.lock();
		// Copy data into ROS message
		for(int i=0;i<this->num_joints_;i++)
		{
			this->msg_joint_.dataArray[i].position = this->joints_[i]->GetAngle(0).Radian();

			this->msg_joint_.dataArray[i].velocity = this->joints_[i]->GetVelocity(0);

			this->msg_joint_.dataArray[i].force_applied = f_app_(i);
			this->msg_joint_.dataArray[i].force_sensed  = f_sen_(i);

			if(collision_index_[i])
				this->msg_joint_.dataArray[i].contact = 1;
			else
				this->msg_joint_.dataArray[i].contact = 0;
		}

		this->ros_pub_joint_.publish(this->msg_joint_);
		this->lock_.unlock();
	}

	// Publish RVIZ marker
	if(this->ros_connections_rviz_ > 0 )
	{
		this->lock_.lock();
		ros::Time t_stamp = ros::Time::now();
		for(int i=0;i<this->joint_names_.size();i++)
		{
			msg_rviz_.markers[i].header.stamp    = t_stamp;
			msg_rviz_.markers[i].pose.position.z = this->joints_[i]->GetAngle(0).Radian();
			msg_rviz_.markers[i].scale.x         = f_sen_(i);

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
	}


	// Publish Tactile data
	if(this->ros_connections_tactile_ > 0 )
	{
		this->lock_.lock();
		// Copy data into ROS message
		for(int i=0;i<this->num_sensors_;i++)
		{
			//msg_tactile_.data[i] = sensors_[i].force_sensed;
			msg_tactile_.data[i] = sensors_[i].force_applied;
		}
		this->ros_pub_tactile_.publish(this->msg_tactile_);
		this->lock_.unlock();
	}

	// Publish force feedback
	if(this->ros_connections_fb_ > 0 )
	{
		this->lock_.lock();

		// Copy data into ROS message
		msg_fb_.force_applied = f_app_.sum();	// Real force applied by plunger
		msg_fb_.force_sensed  = 0.0; 			//f_sen_.sum();
		for(int i=0;i<this->num_sensors_;i++)	// Sum of tactile data
		{
			msg_fb_.force_sensed += sensors_[i].force_sensed;
		}

		msg_fb_.force_sensed *= -1;	// FIXME temporary scaling factor

		msg_fb_.contacts      = num_contacts_;

		this->ros_pub_fb_.publish(this->msg_fb_);
		this->lock_.unlock();
	}

	// Publish Center of Pressure (COP)
	if(this->ros_connections_COP_ > 0 )
	{
		FindCOP();

		this->lock_.lock();
		this->ros_pub_COP_.publish(this->msg_COP_);
		this->lock_.unlock();
	}

	// Save last time stamp
	this->last_time_ = cur_time;
}

//////////////////////////////////////////////////////////////////////////
// Calculate Noise
double SkinJointGazeboRos::CalulateNoise(double mu, double sigma)
{
	this->seed ++;

	// using Box-Muller transform to generate two independent standard
	// normally disbributed normal variables see wikipedia

	// normalized uniform random variable
	double U = static_cast<double>(rand_r(&this->seed)) /
			 static_cast<double>(RAND_MAX);

	// normalized uniform random variable
	double V = static_cast<double>(rand_r(&this->seed)) /
			 static_cast<double>(RAND_MAX);

	double X = sqrt(-2.0 * ::log(U)) * cos(2.0*M_PI * V);
	// double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

	// there are 2 indep. vars, we'll just use X
	// scale to our mu and sigma
	X = sigma * X + mu;
	return X;
}

//////////////////////////////////////////////////////////////////////////
// Calculate Center of Pressure
void SkinJointGazeboRos::FindCOP()
{
	msg_COP_.force_magnitude = 0.0;
	msg_COP_.x = 0.0;
	msg_COP_.y = 0.0;

	// Check if a force has been applied
	if(fabs( f_sen_.sum() ) == 0 )
	{
		return;
	}

	double force;

	for (int i = 0; i < sensors_.size(); ++i)
	{
		force = -sensors_[i].force_sensed; // TODO check sign

		msg_COP_.force_magnitude += force;
		msg_COP_.x += sensors_[i].position.x * force;
		msg_COP_.y += sensors_[i].position.y * force;
	}

	// Normalize
	if(msg_COP_.force_magnitude != 0)
	{
		msg_COP_.x           = msg_COP_.x/msg_COP_.force_magnitude;
		msg_COP_.y           = msg_COP_.y/msg_COP_.force_magnitude;
	}

	/*
	math::Pose p;
	int index;

	// Compute COP of each taxel i
	for (int i = 0; i < sensors_.size(); ++i)
	{
		TactileCOP cop;
		// Loop over elements j inside taxel i
		for (int j = 0; j < sensors_[i].joint_index.size(); ++j)
		{
			// Get skin element info
			index = sensors_[i].joint_index[j];
			p     = this->joints_[index]->GetChild()->GetInitialRelativePose();
			// Update COP variables
			cop.force += -f_sen_(index);			// TODO check sign
			cop.x     += p.pos.x * f_sen_(index);
			cop.y     += p.pos.y * f_sen_(index);
		}
		// Normalize COP
		if(cop.force != 0 )
		{
			cop.x = cop.x/cop.force;
			cop.y = cop.y/cop.force;
		}
		// Save results
		cop_force_(i) = cop.force;	// TODO this is already stored in the tactile data
		cop_x_(i)     = cop.x;
		cop_y_(i)     = cop.y;
	}

	// Compute COP of entire array
	msg_COP_.force_magnitude = cop_force_.sum();
	msg_COP_.x               = (cop_x_.cwiseProduct(cop_force_)).sum();
	msg_COP_.y               = (cop_y_.cwiseProduct(cop_force_)).sum();
	*/
}

//////////////////////////////////////////////////////////////////////////
// Callback function when subscriber connects
bool SkinJointGazeboRos::serviceCB(skinsim_ros_msgs::GetLayout::Request& req, skinsim_ros_msgs::GetLayout::Response& res)
{
	res.success = false;

	this->lock_.lock();

	switch(req.selected)
	{
	// Get position of each skin element
	case skinsim_ros_msgs::GetLayout::Request::ELEMENTS:
	{
		/*
		// Copy position data into ROS message
		msg_layout_.data.resize(this->num_joints_);

		for( int i=0; i < this->num_joints_; i++ )
		{
			math::Pose p = this->joints_[i]->GetChild()->GetInitialRelativePose();

			msg_layout_.data[i].x = p.pos.x;
			msg_layout_.data[i].y = p.pos.y;
			msg_layout_.data[i].z = p.pos.z;
		}
		*/
		this->ros_pub_layout_.publish(this->msg_layout_elements_);
		res.success = true;
		break;
	}
	// Get position of each tactile sensor
	case skinsim_ros_msgs::GetLayout::Request::SENSORS:
	{
		/*
		// Copy position data into ROS message
		msg_layout_.data.resize(this->num_sensors_);

		for( int i = 0; i < this->num_sensors_; ++i )
		{
			msg_layout_.data[i].x = sensors_[i].position.x;
			msg_layout_.data[i].y = sensors_[i].position.y;
			msg_layout_.data[i].z = sensors_[i].position.z;
		}
		*/
		this->ros_pub_layout_.publish(this->msg_layout_sensors_);
		res.success = true;
		break;
	}
	// Default
	default:
		std::cout<<"Layout not found.\n";
		break;
	}
	//this->ros_pub_layout_.publish(this->msg_layout_);

	this->lock_.unlock();

	return true;
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
//////////////////////////////////////////////////////////////////////////
// Empty callback function
void SkinJointGazeboRos::emptyCB()
{
}
//////////////////////////////////////////////////////////////////////////
// Callback function when subscriber connects
void SkinJointGazeboRos::RosConnectJoint()
{
	this->ros_connections_joint_++;
}
void SkinJointGazeboRos::RosConnectRviz()
{
	this->ros_connections_rviz_++;
}
void SkinJointGazeboRos::RosConnectTactile()
{
	this->ros_connections_tactile_++;
}
void SkinJointGazeboRos::RosConnectFb()
{
	this->ros_connections_fb_++;
}

void SkinJointGazeboRos::RosConnectCOP()
{
	this->ros_connections_COP_++;
}

//////////////////////////////////////////////////////////////////////////
// Callback function when subscriber disconnects
void SkinJointGazeboRos::RosDisconnectJoint()
{
	this->ros_connections_joint_--;
}
void SkinJointGazeboRos::RosDisconnectRviz()
{
	this->ros_connections_rviz_--;
}
void SkinJointGazeboRos::RosDisconnectTactile()
{
	this->ros_connections_tactile_--;
}
void SkinJointGazeboRos::RosDisconnectFb()
{
	this->ros_connections_fb_--;
}

void SkinJointGazeboRos::RosDisconnectCOP()
{
	this->ros_connections_COP_--;
}


} // namespace
