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
  std::string name    		;

  double explFctr_Kp  		;
  double explFctr_Ki  		;
  double explFctr_Kd  		;

  double impCtr_Xnom  		;
  double impCtr_M     		;
  double impCtr_K     		;
  double impCtr_D     		;

  double targetForce  		;

  int	 controller_type	;
  int 	 feedback_type		;
  double plunger_Kp			;
  double plunger_Ki			;
  double plunger_Kd			;
  double plunger_Kv			;
};

inline void print(ControllerSpec c)
{
	std::cout<<" Name            : "<<c.name               <<"\n";
	std::cout<<" controller_type : "<<c.controller_type    <<"\n";
	std::cout<<" feedback_type	 : "<<c.feedback_type	   <<"\n";
	std::cout<<" explFctr_Kp     : "<<c.explFctr_Kp        <<"\n";
	std::cout<<" explFctr_Ki     : "<<c.explFctr_Ki        <<"\n";
	std::cout<<" explFctr_Kd     : "<<c.explFctr_Kd        <<"\n";
	std::cout<<" impCtr_Xnom     : "<<c.impCtr_Xnom        <<"\n";
	std::cout<<" impCtr_K        : "<<c.impCtr_K           <<"\n";
	std::cout<<" impCtr_D        : "<<c.impCtr_D           <<"\n";
	std::cout<<" targetForce     : "<<c.targetForce        <<"\n";
	std::cout<<" plunger_Kp		 : "<<c.plunger_Kp		   <<"\n";
	std::cout<<" plunger_Ki		 : "<<c.plunger_Ki		   <<"\n";
	std::cout<<" plunger_Kd		 : "<<c.plunger_Kd		   <<"\n";
	std::cout<<" plunger_Kv		 : "<<c.plunger_Kv		   <<"\n";
}

inline void operator >> (const YAML::Node& node, ControllerSpec& ctrSpec)
{
  ctrSpec.name                = node["name"       		].as<std::string>() ;
  ctrSpec.explFctr_Kp         = node["explFctr_Kp"		].as<double>() ;
  ctrSpec.explFctr_Ki         = node["explFctr_Ki"		].as<double>() ;
  ctrSpec.explFctr_Kd         = node["explFctr_Kd"		].as<double>() ;
  ctrSpec.impCtr_Xnom         = node["impCtr_Xnom"		].as<double>() ;
  ctrSpec.impCtr_K            = node["impCtr_K"   		].as<double>() ;
  ctrSpec.impCtr_D            = node["impCtr_D"   		].as<double>() ;
  ctrSpec.targetForce         = node["targetForce"		].as<double>() ;
  ctrSpec.controller_type 	  = node["controller_type"	].as<int>() ;
  ctrSpec.feedback_type	      = node["feedback_type	"	].as<int>() ;
  ctrSpec.plunger_Kp		  = node["plunger_Kp"		].as<double>() ;
  ctrSpec.plunger_Ki		  = node["plunger_Ki"		].as<double>() ;
  ctrSpec.plunger_Kd		  = node["plunger_Kd"		].as<double>() ;
  ctrSpec.plunger_Kv		  = node["plunger_Kv"		].as<double>() ;
}

inline YAML::Emitter& operator << (YAML::Emitter& out, const ControllerSpec& ctrSpec)
{
    out << YAML::BeginMap;
    out << YAML::Key << "name"       		; out << YAML::Value <<  ctrSpec.name        		;
    out << YAML::Key << "explFctr_Kp"		; out << YAML::Value <<  ctrSpec.explFctr_Kp 		;
    out << YAML::Key << "explFctr_Ki"		; out << YAML::Value <<  ctrSpec.explFctr_Ki 		;
    out << YAML::Key << "explFctr_Kd"		; out << YAML::Value <<  ctrSpec.explFctr_Kd 		;
    out << YAML::Key << "impCtr_Xnom"		; out << YAML::Value <<  ctrSpec.impCtr_Xnom 		;
    out << YAML::Key << "impCtr_K"   		; out << YAML::Value <<  ctrSpec.impCtr_K    		;
    out << YAML::Key << "impCtr_D"   		; out << YAML::Value <<  ctrSpec.impCtr_D    		;
    out << YAML::Key << "targetForce"		; out << YAML::Value <<  ctrSpec.targetForce 		;
    out << YAML::Key << "controller_type"	; out << YAML::Value <<  ctrSpec.controller_type 	;
    out << YAML::Key << "feedback_type"		; out << YAML::Value <<  ctrSpec.feedback_type	 	;
    out << YAML::Key << "plunger_Kp"		; out << YAML::Value <<  ctrSpec.plunger_Kp		 	;
    out << YAML::Key << "plunger_Ki"		; out << YAML::Value <<  ctrSpec.plunger_Ki		 	;
    out << YAML::Key << "plunger_Kd"		; out << YAML::Value <<  ctrSpec.plunger_Kd		 	;
    out << YAML::Key << "plunger_Kv"		; out << YAML::Value <<  ctrSpec.plunger_Kv		 	;
    out << YAML::EndMap;
    return out;
}

#endif /* CONTROLSPECYAML_HH_ */
