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

/* Author: Isura Ranatunga
 *         Sven Cremer
 *
 * ControlSpecYAML.hh
 *  Created on: Jul 27, 2014
 */

#ifndef CONTROLSPECYAML_HH_
#define CONTROLSPECYAML_HH_

#include "yaml-cpp/yaml.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// Specs of the controller
struct ControllerSpec
{
  std::string name;     // Name of controller setup

  int controller_type;  // Specified in /skinsim_ros_msgs/msgs/ControllerType.msg
  int feedback_type;    // Specified in /skinsim_ros_msgs/msgs/FeedbackType.msg
  double Fd;            // Desired force
  double Kp;            // Proportional gain
  double Ki;            // Integral gain
  double Kd;            // Derivative gain
  double Kv;            // Velocity damping term
  double Ts;            // Sampling rate of controller
  double Nf;            // Filtering coefficient

  // TODO: not used
  double impCtr_Xnom;
  double impCtr_M;
  double impCtr_K;
  double impCtr_D;
};

inline void print(ControllerSpec c)
{
	std::cout<<"Name             : "<<c.name               <<"\n";
	std::cout<<" controller_type : "<<c.controller_type    <<"\n";
	std::cout<<" feedback_type	 : "<<c.feedback_type      <<"\n";
	std::cout<<" Fd: "              <<c.Fd                 <<"\n";
	std::cout<<" Kp: "              <<c.Kp                 <<"\n";
	std::cout<<" Ki: "              <<c.Ki                 <<"\n";
	std::cout<<" Kd: "              <<c.Kd                 <<"\n";
	std::cout<<" Kv: "              <<c.Kv                 <<"\n";
	std::cout<<" Ts: "              <<c.Ts                 <<"\n";
	std::cout<<" Nf: "              <<c.Nf                 <<"\n";
	std::cout<<" impCtr_Xnom : "    <<c.impCtr_Xnom        <<"\n";
	std::cout<<" impCtr_M    : "    <<c.impCtr_M           <<"\n";
	std::cout<<" impCtr_D    : "    <<c.impCtr_D           <<"\n";
	std::cout<<" impCtr_K    : "    <<c.impCtr_K           <<"\n";
}

inline void operator >> (const YAML::Node& node, ControllerSpec& ctrSpec)
{
  ctrSpec.name        = node["name"        ].as<std::string>() ;
  ctrSpec.Fd          = node["Fd"          ].as<double>() ;
  ctrSpec.Kp          = node["Kp"          ].as<double>() ;
  ctrSpec.Ki          = node["Ki"          ].as<double>() ;
  ctrSpec.Kd          = node["Kd"          ].as<double>() ;
  ctrSpec.Kv          = node["Kv"          ].as<double>() ;
  ctrSpec.Ts          = node["Ts"          ].as<double>() ;
  ctrSpec.Nf          = node["Nf"          ].as<double>() ;
  ctrSpec.impCtr_Xnom = node["impCtr_Xnom" ].as<double>() ;
  ctrSpec.impCtr_M    = node["impCtr_M"    ].as<double>() ;
  ctrSpec.impCtr_D    = node["impCtr_D"    ].as<double>() ;
  ctrSpec.impCtr_K    = node["impCtr_K"    ].as<double>() ;

  // Try to read types as integers
  try{
	  ctrSpec.controller_type =      node["controller_type"].as<int>();
  } catch (const YAML::BadConversion& e) {
	  ctrSpec.controller_type = (int)node["controller_type"].as<double>();
  }
  try{
	  ctrSpec.feedback_type =      node["feedback_type"].as<int>();
  } catch (const YAML::BadConversion& e) {
	  ctrSpec.feedback_type = (int)node["feedback_type"].as<double>();
  }
}

inline YAML::Emitter& operator << (YAML::Emitter& out, const ControllerSpec& ctrSpec)
{
    out << YAML::BeginMap;
    out << YAML::Key << "name"            << YAML::Value <<  ctrSpec.name;
    out << YAML::Key << "controller_type" << YAML::Value <<  ctrSpec.controller_type;
    out << YAML::Key << "feedback_type"   << YAML::Value <<  ctrSpec.feedback_type;
    out << YAML::Key << "Fd"              << YAML::Value <<  ctrSpec.Fd;
    out << YAML::Key << "Kp"              << YAML::Value <<  ctrSpec.Kp;
    out << YAML::Key << "Ki"              << YAML::Value <<  ctrSpec.Ki;
    out << YAML::Key << "Kd"              << YAML::Value <<  ctrSpec.Kd;
    out << YAML::Key << "Kv"              << YAML::Value <<  ctrSpec.Kv;
    out << YAML::Key << "Ts"              << YAML::Value <<  ctrSpec.Ts;
    out << YAML::Key << "Nf"              << YAML::Value <<  ctrSpec.Nf;
    out << YAML::Key << "impCtr_Xnom"     << YAML::Value <<  ctrSpec.impCtr_Xnom;
    out << YAML::Key << "impCtr_M"        << YAML::Value <<  ctrSpec.impCtr_M;
    out << YAML::Key << "impCtr_D"        << YAML::Value <<  ctrSpec.impCtr_D;
    out << YAML::Key << "impCtr_K"        << YAML::Value <<  ctrSpec.impCtr_K;
    out << YAML::EndMap;
    return out;
}

#endif /* CONTROLSPECYAML_HH_ */
