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
      this->ros_node_ = new ros::NodeHandle("~");
      this->model_ = _model;
      this->joint_ = this->model_->GetJoint("my_mass_joint");
      this->input_pub = this->ros_node_->advertise<skinsim_msgs::inputData>("inputData",1);
      std::string para_ttl_file   = "/ttl_file";
      count_ = 1;
      std::string ttl_file; 
      if (!this->ros_node_->getParam(para_ttl_file, ttl_file)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_ttl_file.c_str()); }
      input_file_.open(ttl_file.c_str());

      if (input_file_)
      {
        ROS_INFO("Load Success!");
      }
      else
      {
        ROS_ERROR("Load Fail!");
      }

      parser_.Load(input_file_);
      parser_.GetNextDocument(doc_);

      double f_value;
      for (unsigned i = 0; i < doc_.size(); i++)
      {
        doc_[i]["ttl"] >> f_value;
        this->ttl_values_.push_back(f_value);
      }
      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MassValidJoint::OnUpdate, this));
      
    }

    void OnUpdate()
    {
      // Apply a small force to the model.
      skinsim_msgs::inputData inputData;
      double current_time = this->model_->GetWorld()->GetSimTime().Double();
      if(count_<this->ttl_values_.size())
      {
        force_ = -ttl_values_[count_];
      }
      else
      {
        force_ = -0.05;
      }
      this->joint_->SetForce(0, force_);

      double current_force = this->joint_->GetForce(0);

      std::cout<<count_<< " ttl_force: "<<this->ttl_values_[count_]<<"\n";
      count_++;
      inputData.input_force = current_force;
      inputData.time = current_time;


      input_publisher_.publish(inputData); 
    }
private:
    physics::JointPtr joint_;
    physics::ModelPtr model_;  
    event::ConnectionPtr update_connection_;
    ros::NodeHandle* ros_node_;
    double force_;
    int count_;
    ros::Publisher input_publisher_;
    std::vector<double> ttl_values_;
    
    YAML::Parser parser_;
    YAML::Node doc_;
    std::ifstream input_file_;

  };


  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MassValidJoint)
}
