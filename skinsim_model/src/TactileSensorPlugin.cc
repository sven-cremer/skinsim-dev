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

namespace gazebo
{
class TactileSensorPlugin : public ModelPlugin
{

public:

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    std::string fullname;
    getModelConfigPath( fullname, _sdf );

    fin.open(fullname.c_str());

    // FIXME change to gazebo errors msg
    if (fin)
    {
      //ROS_INFO("Load Success!");
    }
    else
    {
      //ROS_ERROR("Load Fail!");
    }

    parser.Load(fin);
    parser.GetNextDocument(doc);

    std::string scalar;
    for (unsigned i = 0; i < doc.size(); i++)
    {
      doc[i]["Joint"] >> scalar;
      this->jointNames.push_back("spring_" + scalar);
      //std::cout << "Here's the output YAML:\n---" << scalar << "---\n";
    }

    fin.close();

    this->model_ = _model;

    // get pointers to joints from gazebo
    this->joints.resize(this->jointNames.size());
    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      this->joints[i] = this->model_->GetJoint(this->jointNames[i]);
      if (!this->joints[i])
      {
        //ROS_ERROR("SkinSim robot expected joint[%s] not present, plugin not loaded", this->jointNames[i].c_str());
        return;
      }
    }

    // Create a new transport node
    transport::NodePtr node(new transport::Node());

    // Initialize the node with the Model name
    node->Init(model_->GetName());
  }

  // Called by the world update start event
public:
  void OnUpdate()
  {
    double rest_angle = 0;

    double current_angle    = 0;
    double current_force    = 0;
    double current_velocity = 0;
    double sens_force = 0;

    double current_time = this->model_->GetWorld()->GetSimTime().Double();
    math::Vector3 vect;

    // TODO add tactile sensor data publish

  }


private:

  std::vector<std::string> jointNames;

  physics::Joint_V joints;

  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;

  YAML::Parser parser;
  YAML::Node doc;
  std::ifstream fin;

  // Parameters
  double skin_spring ;
  double skin_dir_spring;
  double skin_damper ;

  double x_size ;
  double y_size ;

  double x_dt   ;
  double y_dt   ;

  double skin_max ;

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN (TactileSensorPlugin)

}



