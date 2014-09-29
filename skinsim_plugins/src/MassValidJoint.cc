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

#include <string>
#include <fstream>
#include "ros/ros.h"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/gazebo.hh"
#include "skinsim_msgs/inputData.h"
#include <yaml-cpp/yaml.h>

namespace gazebo
{
  class MassValidJoint : public ModelPlugin
  {
    public: 
    void Load(physics::ModelPtr _model, sdf::ElementPtr)
    {
      this->ros_node = new ros::NodeHandle("~");
      this->model = _model;
      this->joint = this->model->GetJoint("my_mass_joint");
      this->input_pub = this->ros_node->advertise<skinsim_msgs::inputData>("inputData",1);
      std::string para_ttl_file   = "/ttl_file";
      count = 1;
      std::string ttl_file; 
      if (!this->ros_node->getParam(para_ttl_file, ttl_file)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_ttl_file.c_str()); }
      fin.open(ttl_file.c_str());

      if (fin)
      {
        ROS_INFO("Load Success!");
      }
      else
      {
        ROS_ERROR("Load Fail!");
      }

      parser.Load(fin);
      parser.GetNextDocument(doc);

      double f_value;
      for (unsigned i = 0; i < doc.size(); i++)
      {
        doc[i]["ttl"] >> f_value;
        this->ttlValues.push_back(f_value);
      }
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MassValidJoint::OnUpdate, this));
      
    }

    void OnUpdate()
    {
      // Apply a small force to the model.
      skinsim_msgs::inputData inputData;
      double current_time = this->model->GetWorld()->GetSimTime().Double();
      if(count<this->ttlValues.size())
      {
        force = -ttlValues[count];
      }
      else
      {
        force = -0.05;
      }
      this->joint->SetForce(0, force);

      double current_force = this->joint->GetForce(0);

      std::cout<<count<< " ttl_force: "<<this->ttlValues[count]<<"\n";
      count++;
      inputData.input_force = current_force;
      inputData.time = current_time;


      input_pub.publish(inputData); 
    }
private:
    physics::JointPtr joint;
    physics::ModelPtr model;  
    event::ConnectionPtr updateConnection;
    ros::NodeHandle* ros_node;
    double force;
    int count;
    ros::Publisher input_pub;
    std::vector<double> ttlValues;
    
    YAML::Parser parser;
    YAML::Node doc;
    std::ifstream fin;

  };


  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MassValidJoint)
}
