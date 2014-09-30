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

/* Author: Isura Ranatunga
 *
 * SkinJointPlugin.cc
 *  Created on: Jul 14, 2014
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
class SkinJointPlugin : public ModelPlugin
{

public:

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {

    std::string fullname;

    getModelConfigPath( fullname, _sdf );

    fullname = fullname + std::string("/joint_names.yaml");

    sping_ = 122.24 ;
    damper_ = 1.83   ;

    input_file_.open(fullname.c_str());

    parser_.Load(input_file_);
    parser_.GetNextDocument(doc_);

    std::string scalar;
    for (unsigned i = 0; i < doc_.size(); i++)
    {
      doc_[i]["Joint"] >> scalar;
      this->joint_names_.push_back("spring_" + scalar);
    }

    input_file_.close();

    this->model_ = _model;

    // get pointers to joints from Gazebo
    this->joints_.resize(this->joint_names_.size());
    for (unsigned int i = 0; i < this->joints_.size(); ++i)
    {
      this->joints_[i] = this->model_->GetJoint(this->joint_names_[i]);
      if (!this->joints_[i])
      {
        //ROS_ERROR("SkinSim robot expected joint[%s] not present, plugin not loaded", this->jointNames[i].c_str());
        return;
      }
    }

    // Create a new transport node
    transport::NodePtr node(new transport::Node());

    // Initialize the node with the Model name
    node->Init(model_->GetName());

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&SkinJointPlugin::UpdateJoint, this));
  }

public:
  void OnUpdate()
  {
  }

  void UpdateJoint()
  {
    double rest_angle = 0;

    double current_angle = 0;
    double current_force = 0;
    double current_velocity = 0;
    double sens_force = 0;

    double current_time = this->model_->GetWorld()->GetSimTime().Double();

    for (unsigned int i = 0; i < this->joints_.size(); ++i)
    {
      current_angle = this->joints_[i]->GetAngle(0).Radian();
      current_velocity = this->joints_[i]->GetVelocity(0);

      // This sets the mass-spring-damper dynamics, currently only spring and damper
      this->joints_[i]->SetForce(0, (rest_angle - current_angle) * sping_ - damper_ * current_velocity);
    }

  }

private:

  std::vector<std::string> joint_names_;

  physics::Joint_V joints_;

  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;

  YAML::Parser  parser_;
  YAML::Node    doc_;
  std::ifstream input_file_;

  // Parameters
  double sping_;
  double damper_;

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN (SkinJointPlugin)

}
