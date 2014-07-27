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
  double explFctr_Kp  ;
  double explFctr_Ki  ;
  double explFctr_Kd  ;

  double impCtr_Xnom  ;
  double impCtr_M     ;
  double impCtr_K     ;
  double impCtr_D     ;

  double ctrType      ;
  double targetForce  ;
};

void operator >> (const YAML::Node& node, ControllerSpec& ctrSpec)
{
  node["explFctr_Kp"] >> ctrSpec.explFctr_Kp ;
  node["explFctr_Ki"] >> ctrSpec.explFctr_Ki ;
  node["explFctr_Kd"] >> ctrSpec.explFctr_Kd ;
  node["impCtr_Xnom"] >> ctrSpec.impCtr_Xnom ;
  node["impCtr_K"   ] >> ctrSpec.impCtr_K    ;
  node["impCtr_D"   ] >> ctrSpec.impCtr_D    ;
  node["ctrType"    ] >> ctrSpec.ctrType     ;
  node["targetForce"] >> ctrSpec.targetForce ;
}

YAML::Emitter& operator << (YAML::Emitter& out, const ControllerSpec& ctrSpec)
{
    out << YAML::BeginMap;
    out << YAML::Key << "explFctr_Kp"; out << YAML::Value <<  ctrSpec.explFctr_Kp ;
    out << YAML::Key << "explFctr_Ki"; out << YAML::Value <<  ctrSpec.explFctr_Ki ;
    out << YAML::Key << "explFctr_Kd"; out << YAML::Value <<  ctrSpec.explFctr_Kd ;
    out << YAML::Key << "impCtr_Xnom"; out << YAML::Value <<  ctrSpec.impCtr_Xnom ;
    out << YAML::Key << "impCtr_K"   ; out << YAML::Value <<  ctrSpec.impCtr_K    ;
    out << YAML::Key << "impCtr_D"   ; out << YAML::Value <<  ctrSpec.impCtr_D    ;
    out << YAML::Key << "ctrType"    ; out << YAML::Value <<  ctrSpec.ctrType     ;
    out << YAML::Key << "targetForce"; out << YAML::Value <<  ctrSpec.targetForce ;
    out << YAML::EndMap;
    return out;
}

#endif /* CONTROLSPECYAML_HH_ */
