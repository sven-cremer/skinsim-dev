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
 * ModelSpecYAML.hh
 *  Created on: Jul 27, 2014
 */

#ifndef MODELSPECYAML_HH_
#define MODELSPECYAML_HH_

#include "yaml-cpp/yaml.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// Specs of the model to be built
struct ModelSpec
{
  // No of elements per side
  // do nothing if 0 use size_x and size_y
  double xByX         ; // : 5.0 # 0.0

  double d_pos        ; // : 0.0025 #0.005
  double density      ; // : 1.0

  double size_x       ; // : 0.0525
  double size_y       ; // : 0.0525

  double skin_height  ; // : 0.04
  double tac_height   ; // : 0.03

  double plane_height ; // : 0.02

  double sens_rad     ; // : 1.0
  double space_wid    ; // : 4.0
};

struct BuildModelSpec
{
  std::string name;
  ModelSpec   spec;
};

void operator >> (const YAML::Node& node, ModelSpec& spec)
{
  spec.xByX   		= node["xByX"        ].as<double>() ;
  spec.d_pos        = node["d_pos"       ].as<double>() ;
  spec.density      = node["density"     ].as<double>() ;
  spec.size_x       = node["size_x"      ].as<double>() ;
  spec.size_y       = node["size_y"      ].as<double>() ;
  spec.skin_height  = node["skin_height" ].as<double>() ;
  spec.tac_height   = node["tac_height"  ].as<double>() ;
  spec.plane_height = node["plane_height"].as<double>() ;
  spec.sens_rad     = node["sens_rad"    ].as<double>() ;
  spec.space_wid    = node["space_wid"   ].as<double>() ;
}

void operator >> (const YAML::Node& node, BuildModelSpec& buildModelSpec)
{
   buildModelSpec.name = node["name"].as<std::string>() ;
   const YAML::Node& specs = node["spec"];
   ModelSpec spec;
   specs[0] >> spec;
   buildModelSpec.spec = spec;
}

YAML::Emitter& operator << (YAML::Emitter& out, const ModelSpec& spec)
{
    out << YAML::BeginMap;
    out << YAML::Key << "xByX"        ; out << YAML::Value <<  spec.xByX        ;
    out << YAML::Key << "d_pos"       ; out << YAML::Value <<  spec.d_pos       ;
    out << YAML::Key << "density"     ; out << YAML::Value <<  spec.density     ;
    out << YAML::Key << "size_x"      ; out << YAML::Value <<  spec.size_x      ;
    out << YAML::Key << "size_y"      ; out << YAML::Value <<  spec.size_y      ;
    out << YAML::Key << "skin_height" ; out << YAML::Value <<  spec.skin_height ;
    out << YAML::Key << "tac_height"  ; out << YAML::Value <<  spec.tac_height  ;
    out << YAML::Key << "plane_height"; out << YAML::Value <<  spec.plane_height;
    out << YAML::Key << "sens_rad"    ; out << YAML::Value <<  spec.sens_rad    ;
    out << YAML::Key << "space_wid"   ; out << YAML::Value <<  spec.space_wid   ;
    out << YAML::EndMap;
    return out;
}

YAML::Emitter& operator << (YAML::Emitter& out, const BuildModelSpec& buildModelSpec)
{
    out << YAML::BeginMap;
    out << YAML::Key << "name";
    out << YAML::Value << buildModelSpec.name;
    out << YAML::Key << "spec";
    out << YAML::Value;

    out << YAML::BeginSeq;
    out << buildModelSpec.spec;
    out << YAML::EndSeq;

    out << YAML::EndMap;
    return out;
}

#endif /* MODELSPECYAML_HH_ */
