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
 * TactileSensorPlugin.cc
 *  Created on: Jul 14, 2014
 */

#include <fstream>
#include <string>
#include <sstream>
#include <vector>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>

#include <yaml-cpp/yaml.h>

#include "SkinSim/ModelPath.hh"

#include "tactileData.pb.h"

namespace gazebo
{

class TactileSensorPlugin : public ModelPlugin
{

  transport::PublisherPtr  tactilePub ;
  transport::NodePtr       node       ;

public:

  ~TactileSensorPlugin()
  {
    // Make sure to shut everything down.
    gazebo::transport::fini();
  }

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    std::string fullname;
    getModelConfigPath( fullname, _sdf );

    fullname = fullname + std::string("/joint_names.yaml");

    input_file_.open(fullname.c_str());

    // FIXME change to gazebo errors msg
    if (input_file_)
    {
      //ROS_INFO("Load Success!");
    }
    else
    {
      //ROS_ERROR("Load Fail!");
    }

    parser_.Load(input_file_);
    parser_.GetNextDocument(doc_);

    std::string scalar;
    for (unsigned i = 0; i < doc_.size(); i++)
    {
      doc_[i]["Joint"] >> scalar;
      this->joint_names_.push_back("spring_" + scalar);
      //std::cout << "Here's the output YAML:\n---" << scalar << "---\n";
    }

    input_file_.close();

    this->model_ = _model;

    // get pointers to joints from gazebo
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
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(model_->GetName());
//    this->tactilePub = this->node->Advertise<skinsim_msgs::msgs::TactileData>("~/tacData");
    tactilePub = node->Advertise<msgs::Vector3d>("~/tacData");

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&TactileSensorPlugin::UpdateJoint, this));
  }

  // Called by the world update start event
  void OnUpdate()
  {

  }

  void UpdateJoint()
  {
    double rest_angle = 0;

    double current_angle    = 0;
    double current_force    = 0;
    double current_velocity = 0;
    double sens_force = 0;

    double current_time = this->model_->GetWorld()->GetSimTime().Double();

    skin_mass_   = 0.88625;
    skin_spring_ = 122.24 ;
    skin_damper_ = 1.83   ;

    for (unsigned int i = 0; i < this->joints_.size(); ++i)
    {
      current_angle = this->joints_[i]->GetAngle(0).Radian();
      current_velocity = this->joints_[i]->GetVelocity(0);

      // This sets the mass-spring-damper dynamics, currently only spring and damper
      sens_force += skin_spring_*(rest_angle - current_angle) - skin_damper_ * current_velocity ;
    }

    msgs::Vector3d msg;
    msg.set_x(current_time);
    msg.set_y(sens_force);
    msg.set_z(sens_force);
    tactilePub->Publish(msg);

  }


private:

  std::vector<std::string> joint_names_;

  physics::Joint_V joints_;

  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;

  YAML::Parser parser_;
  YAML::Node doc_;
  std::ifstream input_file_;

  // Parameters
  double skin_mass_;
  double skin_spring_ ;
  double skin_dir_spring_;
  double skin_damper_ ;

  double x_size_ ;
  double y_size_ ;

  double x_dt_   ;
  double y_dt_   ;

  double skin_max_ ;

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN (TactileSensorPlugin)

}



