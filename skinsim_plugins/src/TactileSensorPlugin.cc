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
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(model_->GetName());
//    this->tactilePub = this->node->Advertise<skinsim_msgs::msgs::TactileData>("~/tacData");
    tactilePub = node->Advertise<msgs::Vector3d>("~/tacData");

    this->m_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&TactileSensorPlugin::UpdateJoint, this));
  }

  // Called by the world update start event
public:

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

    skin_mass   = 0.88625;
    skin_spring = 122.24 ;
    skin_damper = 1.83   ;

    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      current_angle = this->joints[i]->GetAngle(0).Radian();
      current_velocity = this->joints[i]->GetVelocity(0);

      // This sets the mass-spring-damper dynamics, currently only spring and damper
      sens_force += skin_spring*(rest_angle - current_angle) - skin_damper * current_velocity ;
    }

/*
 *  // TODO add tactile sensor data publish
    skinsim_msgs::msgs::TactileData tactileMsg;
    tactileMsg.set_time( current_time );
    tactileMsg.set_force( sens_force );

    tactileMsg.set_time             ( current_time );
    tactileMsg.set_patchid          ( 0 );
    tactileMsg.set_tactelemid       ( 0 );
    tactileMsg.set_force            ( sens_force   );
    tactileMsg.set_force_noisy      ( current_time );
    tactileMsg.set_tactid           ( 0 );
    tactileMsg.set_tact_ind_force   ( current_time );
    tactileMsg.set_tact_total_force ( current_time );
    tactileMsg.set_capacitance      ( current_time );
    tactileMsg.set_capacitance_noisy( current_time );

    tactilePub->Publish(tactileMsg);
    */

    msgs::Vector3d msg;
    msg.set_x(current_time);
    msg.set_y(sens_force);
    msg.set_z(sens_force);
    tactilePub->Publish(msg);

  }


private:

  std::vector<std::string> jointNames;

  physics::Joint_V joints;

  physics::ModelPtr model_;
  event::ConnectionPtr m_updateConnection;

  YAML::Parser parser;
  YAML::Node doc;
  std::ifstream fin;

  // Parameters
  double skin_mass;
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



