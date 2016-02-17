/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, UT Arlington
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

/* Author: Sumit Kumar Das
 *
 * SkinJointForceDistributionPlugin.cc
 *  Created on: Nov 11, 2015
 */

#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <list>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>

#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>

#include "SkinSim/ModelPath.hh"

namespace gazebo
{
	class SkinJointForceDistributionPlugin : public ModelPlugin
	{

	public:

		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{

			std::string fullname;
			getModelConfigPath( fullname, _sdf );
			fullname 				    	=	fullname + std::string("/joint_names.yaml");
			input_file_.open(fullname.c_str());

			// Open the yaml file to get joint names
			std::cout<<"Loading file: "<<fullname<<"\n";
			YAML::Node doc;
			doc 							= 	YAML::LoadAll(input_file_);
			for(std::size_t i=0;i<doc[0].size();i++)
			{
				this->joint_names_.push_back("spring_" + doc[0][i]["Joint"].as<std::string>());						// FIXME overwrites previous data
			}
			input_file_.close();


			// get pointers to joints from Gazebo
			this->model_ 					= 	_model;
			this->joints_.resize(this->model_->GetJointCount());
			this->joints_ 					= 	this->model_->GetJoints();

			// Create a new transport node
			transport::NodePtr node(new transport::Node());

			// Initialize the node with the Model name
			node->Init(model_->GetName());

			// Define Callback function
			this->update_connection_ 		= 	event::Events::ConnectWorldUpdateBegin(boost::bind(&SkinJointForceDistributionPlugin::UpdateJoint, this));
		}

	public:
		void OnUpdate()
		{
		}

		void UpdateJoint()
		{
			double rest_angle 		 		= 	0;
			double current_angle 			= 	0;
			double current_force 	 		= 	0;
			double current_velocity  		= 	0;
			double sens_force 				= 	0;
			int    counter_spring 			=	0;

			this->spring_ 					=	1.24 ;
			this->damper_ 				    =	1.83  ;

			double current_time 			= 	this->model_->GetWorld()->GetSimTime().Double();
			if (this->start)
			{
				this->initState.resize(this->joints_.size());
				for (unsigned int i = 0; i < this->joints_.size(); ++i)
				{
					this->initState[i]   	= 	this->joints_[i]->GetAngle(0).Radian();
				}
				this->start = false;
			}
			for (unsigned int i = 0; i < this->joints_.size(); ++i)
			{
				current_angle 	        = 	this->joints_[i]->GetAngle(0).Radian();
				current_velocity 	    = 	this->joints_[i]->GetVelocity(0);
				if (this->joints_[i]->GetName() != "plane_joint")
				{
					if (fabs(this->initState[i] - current_angle) > .02)
					{
//						counter_spring 	+= 	1;
//						std::cout<<1<< " ";
						std::cout<<round(this->joints_[i]->GetForce(0) * 100)<< " ";
					}
					else
					{
//						std::cout<<0<< " ";
						std::cout<<round(this->joints_[i]->GetForce(0)* 100)<< " ";
					}
					if (i % 15 == 0)
						std::cout<<std::endl;
				}

				// This sets the mass-spring-damper dynamics, currently only spring and damper
				this->joints_[i]->SetForce(0, ((rest_angle - current_angle) * spring_) - (damper_ * current_velocity));
			}
			std::cout<<std::endl<<std::endl;
		}

	private:

		std::vector<std::string> joint_names_;
		physics::Joint_V joints_;
		physics::ModelPtr model_;
		event::ConnectionPtr update_connection_;
		YAML::Node    doc_;
		std::ifstream input_file_;

		// Parameters
		double spring_;
		double damper_;
		std::vector<double> initState;
		bool start = true;

	};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN (SkinJointForceDistributionPlugin)

}
