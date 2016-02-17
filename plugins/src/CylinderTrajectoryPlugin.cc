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

/*
 * CylinderTrajectoryPlugin.cc
 *
 *  Created on: Feb 10, 2016
 *      Author: sumit
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

namespace gazebo
{
	class CylinderTrajectoryPlugin : public ModelPlugin
	{
	public:
		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			this->model_ 					= 	_model;

			this->joints_.resize(this->model_->GetJointCount());
			this->joints_ 					= 	this->model_->GetJoints();


			//Initialize Joint Control
//			this->jt						=	new physics::Joint(this->joints_[0]);
//			this->joints_[0]->SetEffortLimit(0,-100);
//			this->jt->AddJoint(this->joints_[0]);
//			joints 							= 	this->jt->GetJoints();
//
//			std::cout<<joints.begin()->first<<std::endl	;

			// Create a new transport node
			transport::NodePtr node(new transport::Node());

			// Initialize the node with the Model name
			node->Init(model_->GetName());

//			this->jt->SetJointPosition(this->joints.begin()->first, 20);
			this->joints_[0]->SetPosition(0,position);
			this->update_connection_ 	= 	event::Events::ConnectWorldUpdateBegin(boost::bind(&CylinderTrajectoryPlugin::UpdateJoint, this));
		}
	public:
			void OnUpdate()
			{
			}

			void UpdateJoint()
			{
//				this->joints_[0]->SetForce(0,-10);
//				this->joint_forces			= 	this->jt->GetVelocities();
//				std::cout<<this->joint_forces.size()<<std::endl;
				this->bd2 = this->model_->GetLink("cylinder_link")->GetWorldForce();
//				std::cout<<this->bd2.z << "/n ----- /n"<<std::endl;
//				if (position >= 0)
//				{
//					this->joints_[0]->SetPosition(0,position);
////					joint_forces			= 	this->jt->GetPositions();
//					this->model_->GetLink("link")->SetForce(math::Vector3(0,0,1));
//
//					this->jtwr = this->joints_[0]->GetForceTorque(2);
//					this->bd2 = this->model_->GetLink("link")->GetWorldForce();
//					std::cout<<this->bd2.z<<std::endl;
//					position = position - .01;
//				}
//				else
//				{
//					this->bd2 = this->model_->GetLink("link")->GetWorldForce();
//					std::cout<<this->bd2.z<<std::endl;
//					//this->joints_[0]->SetPosition(0, position);
////					std::cout<<position<<std::endl;
//				}
//				std::cout<<joint_forces.begin()->first<<std::endl;
			}

	private:
		physics::Joint *jt;
		physics::JointWrench jtwr;
		math::Vector3 bd2;


		std::map<std::string, physics::JointPtr> joints;
		std::map<std::string, double> joint_forces;

		double position = 5.00;
		physics::Joint_V joints_;
		physics::ModelPtr model_;
		event::ConnectionPtr update_connection_;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN (CylinderTrajectoryPlugin)
}
