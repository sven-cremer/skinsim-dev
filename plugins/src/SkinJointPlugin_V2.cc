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
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <list>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Collision.hh>

#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <boost/bind.hpp>

#include <gazebo/common/common.hh>
#include <gazebo/physics/World.hh>


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
			this->out_path 					    =	fullname;
			fullname 				    	    =	fullname + std::string("/joint_names.yaml");
			input_file_.open(fullname.c_str());

			// Open the yaml file to get joint names
			std::cout<<"Loading file: "<<fullname<<"\n";
			YAML::Node doc;
			doc 								= 	YAML::LoadAll(input_file_);
			for(std::size_t i=0;i<doc[0].size();i++)
			{
				this->joint_names_.push_back("spring_" + doc[0][i]["Joint"].as<std::string>());						// FIXME overwrites previous data
			}
			input_file_.close();


			// get pointers to joints from Gazebo
			this->model_ 						= 	_model;
			this->joints_.resize(this->model_->GetJointCount());
			this->joints_ 						= 	this->model_->GetJoints();

			//For subscription
//			gazebo::setupClient();

			// Create a new transport node
			this->receiver						=	transport::NodePtr(new transport::Node());

			// Initialize the node with the Model name
			this->receiver->Init(model_->GetName());
			this->start 						=	true;
			this->first_collison 				=	true;
			this->first_contact 				=	false;

			// Define Callback function
			this->update_connection_ 			= 	event::Events::ConnectWorldUpdateBegin(boost::bind(&SkinJointForceDistributionPlugin::UpdateJoint, this));
			this->sub 							= 	this->receiver->Subscribe("/gazebo/default/physics/contacts", &SkinJointForceDistributionPlugin::cb, this);

			const std::string topic("~/physics/feedback/contacts");
			this->m_pub 						= 	this->receiver->Advertise<gazebo::msgs::Contacts>(topic, 1);
		}

	public:
		void OnUpdate()
		{
		}

		void pub(const gazebo::msgs::Contact var)
		{
//			common::Time ttt = gazebo::common::Time::GetWallTime();
//			//
//			msgs::Contacts cts;
//			msgs::Time *yyy = cts.mutable_time();
//			yyy->set_sec(ttt.sec);
//			yyy->set_nsec(ttt.nsec);
//			msgs::Contact *ct = cts.add_contact();
//			msgs::Time *xxx = ct->mutable_time();
//			xxx->set_sec(ttt.sec);
//			xxx->set_nsec(ttt.nsec);
//			std::string* str_c1 = ct->mutable_collision1();
//			std::string* str_c2 = ct->mutable_collision2();
//			*str_c1 = var.collision1();
//			*str_c2 = var.collision2();
//			msgs::Vector3d* point = ct->add_position();
//			point->set_x(1.5);
//			point->set_y(1.5);
//			point->set_z(1.5);
//			//
//			msgs::Vector3d* normal = ct->add_normal();
//			normal->set_x(0.0);
//			normal->set_y(0.0);
//			normal->set_z(1.0);
//			//
//			ct->add_depth(0.5);
//			//
//			msgs::JointWrench* jw = ct->add_wrench();
//			jw->set_body_1_name("body_a");
//			jw->set_body_2_name("body_b");
//			math::Vector3 *a = jw->body1Force;
//			a->set_x(0.0);
//			a->set_y(0.0);
//			a->set_z(0.0);
//			msgs::Vector3d *b = jw->mutable_body_1_torque();
//			b->set_x(0.0);
//			b->set_y(0.0);
//			b->set_z(0.0);
//			msgs::Vector3d *c = jw->mutable_body_2_force();
//			c->set_x(0.0);
//			c->set_y(0.0);
//			c->set_z(0.0);
//			msgs::Vector3d *d = jw->mutable_body_2_torque();
//			d->set_x(0.0);
//			d->set_y(0.0);
//			d->set_z(0.0);
//			msgs::Set(jw->mutable_body_1_force(), var.wrench[0].body1Force);
//			this->m_pub->Publish(cts);
//			// Apply a small linear velocity to the model.
//			this->model_->SetLinearVel(math::Vector3(.05, 0, 0));
		}

		void cb(ConstContactsPtr &_msg)
		{
			const int c = _msg->contact_size();
			if ((c > 0) && (!this->first_contact))
			{
				FindCOP();
				this->first_contact = true;
			}
			if (c > 0)
			{
				const gazebo::msgs::Contact& var= _msg->contact(0);
				std::string st     = var.collision2();
				std::string last_element(st.substr(st.rfind("_") + 1));
				double x = this->model_->GetJoint("spring_joint_" + last_element)->GetAngle(0).Radian();

				physics::JointWrench wrench = this->model_->GetJoint("spring_joint_" + last_element)->GetForceTorque(0);

//				std::cout <<wrench.body1Force.z<< std::endl;
				pub(var);
			}

		}

		void FindCOP( )
		{

			double force_sum 				=	0;
			double x_sum 					=	0;
			double y_sum 					=	0;
			for (int i = 0; i < this->joints_.size(); i++)
			{
				physics::JointWrench wrench = 	this->joints_[i]->GetForceTorque(0);
				force_sum 					=	force_sum + wrench.body1Force.z;
				x_sum 						=	x_sum + (this->joints_[i]->GetWorldPose().pos.x * wrench.body1Force.z);
				y_sum 						=	y_sum + (this->joints_[i]->GetWorldPose().pos.y * wrench.body1Force.z);
			}

			std::cout << "COP : " << x_sum/force_sum << "," << y_sum/force_sum <<std::endl;


		}

		void GetCollisionBoundary()
		{
			int counter 						=	0;
			this->collision_x 					=	0;
			this->collision_y 					=	0;
			double x_max 						=	-9999;
			double x_min 						=	9999;
			double y_max 						=	-9999;
			double y_min 						=	9999;
			for (unsigned int i = 0; i < this->joints_.size(); ++i)
			{
				if (fabs(this->initState[i] - this->joints_[i]->GetAngle(0).Radian()) > .02)	//the value .02 changes when the size of skin elements change
				{
					this->collision_x 			+=	this->joints_[i]->GetWorldPose().pos.x;
					this->collision_y 			+=	this->joints_[i]->GetWorldPose().pos.y;
					counter 					+=	1;

					if (this->joints_[i]->GetWorldPose().pos.x > x_max)
						x_max 					=	this->joints_[i]->GetWorldPose().pos.x;
					if (this->joints_[i]->GetWorldPose().pos.x < x_min)
						x_min 					=	this->joints_[i]->GetWorldPose().pos.x;
					if (this->joints_[i]->GetWorldPose().pos.y > y_max)
						y_max 					=	this->joints_[i]->GetWorldPose().pos.y;
					if (this->joints_[i]->GetWorldPose().pos.y < y_min)
						y_min 					=	this->joints_[i]->GetWorldPose().pos.y;
				}
			}
			this->collision_x 					=	round(this->collision_x/counter);
			this->collision_y 					=	round(this->collision_x/counter);
			this->collision_rad 				=	((x_max - x_min) + (y_max - y_min)) / 2;

			std::cout <<  this->collision_x << "	"<< this->collision_y<< "	" <<this->collision_rad <<std::endl;
		}

		void UpdateJoint()
		{
			double rest_angle 		 		    = 	0;
			double max_vel 						=	0;
			double max_deform 					=	0;
			double current_angle 			    = 	0;
			double current_force 	 		    = 	0;
			double current_velocity  		    = 	0;
			double sens_force 				    = 	0;
			int    counter_spring 			    =	0;
			std::string data_path 			    =	this->out_path + std::string("/data.csv");

			this->spring_ 					    =	122.24 ;
			this->damper_ 				        =	1.83 ;

			double current_time 			    = 	this->model_->GetWorld()->GetSimTime().Double();
			this->out_datafile.open (data_path.c_str(), std::ios::out | std::ios::ate | std::ios::app | std::ios::binary) ;

			// Initialization step for the very first world-itiration
			if (this->start)
			{
				this->initState.resize(this->joints_.size());
				for (unsigned int i = 0; i < this->joints_.size(); ++i)
				{
					this->initState[i]   		= 	this->joints_[i]->GetAngle(0).Radian();
					this->out_datafile << this->joints_[i]->GetAngle(0).Radian() << " , ";
				}

				this->out_datafile << std::endl;

				for (unsigned int i = 0; i < this->joints_.size(); ++i)
				{
					this->out_datafile << this->joints_[i]->GetWorldPose().pos.x << " , "<< this->joints_[i]->GetWorldPose().pos.y << " , ";
				}

				this->start = false;
				this->out_datafile << std::endl;
			}

			for (unsigned int i = 0; i < this->joints_.size(); ++i)
			{
				if(fabs(max_vel) < fabs(this->joints_[i]->GetVelocity(0)))
					max_vel 					=	this->joints_[i]->GetVelocity(0);
				if(fabs(max_deform) < fabs(this->initState[i] - this->joints_[i]->GetAngle(0).Radian()))
					max_deform 					=	fabs(this->initState[i] - this->joints_[i]->GetAngle(0).Radian());
			}

			this->out_datafile << " "<< " , ";
			for (unsigned int i = 0; i < this->joints_.size(); ++i)
			{
				current_angle 	        	    = 	this->joints_[i]->GetAngle(0).Radian();
				current_velocity 	    	    = 	this->joints_[i]->GetVelocity(0);
			    math::Pose loc 		 		    =	this->joints_[i]->GetWorldPose();
			    this->force_ 					=	((rest_angle - current_angle) * spring_) - (damper_ * current_velocity);

			    if (this->joints_[i]->GetName() != "plane_joint")
				{
					if (fabs(this->initState[i] - current_angle) > .02)
					{
						if(this->first_collison)
						{
							GetCollisionBoundary();
							this->first_collison=	false;
						}

						this->joints_[i]->SetForce(0, this->force_);
//						this->out_datafile << fabs(this->initState[i] - current_angle)<< " , ";
						this->out_datafile << (this->force_)<< " , ";

//						counter_spring 			+= 	1;
//						std::cout<<1<< " ";
//						std::cout<<round(this->joints_[i]->GetForce(0) * 10)<< " ";
//						std::cout<<round(loc.pos.y)<< " ";
					}
					else if ( ! this->first_collison)
					{
						double a				=	(max_deform*spring_) - (damper_*max_vel);
						double c 				=	1;
						double b				=	pow((loc.pos.x - this->collision_x),2) + pow((loc.pos.y - this->collision_y),2);
						double exp_force 		= 	-(a*exp(-b/(pow(c,2))));
						this->joints_[i]->SetForce(0, exp_force);
//						this->out_datafile << fabs(this->initState[i] - current_angle)<< " , ";
						this->out_datafile << exp_force<< " , ";

//						std::cout<<0<< " ";
//						std::cout<<round(this->joints_[i]->GetForce(0)* 10)<< " ";
//						std::cout<<round(loc.pos.y)<< " ";
					}
					else
					{
						this->joints_[i]->SetForce(0, this->force_);
//						this->out_datafile << fabs(this->initState[i] - current_angle)<< " , ";
						this->out_datafile << joints_[i]->GetName().c_str()<< " , ";
					}
//					if (i % 15 == 0)
//						std::cout<<std::endl;


					// This sets the mass-spring-damper dynamics, currently only spring and damper
				}
			}

//			std::cout<<std::endl<<std::endl;
			this->out_datafile << std::endl;
			this->out_datafile.close();
		}

	private:

		std::vector<std::string> joint_names_;
		physics::Joint_V joints_;
		physics::ModelPtr model_;
		event::ConnectionPtr update_connection_;
		transport::SubscriberPtr sub;
		transport::PublisherPtr m_pub;
		transport::NodePtr receiver;
		YAML::Node    doc_;
		std::ifstream input_file_;
		std::ofstream out_datafile;
		std::string out_path;

		// Joint Parameters
		double spring_;
		double damper_;
		double force_;
		std::vector<double> initState;
		bool start;
		bool first_collison;
		bool first_contact;

		// Collision Parameters
		double collision_x;
		double collision_y;
		double collision_rad;

	};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN (SkinJointForceDistributionPlugin)

}
