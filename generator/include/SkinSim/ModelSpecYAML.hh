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
  // Skin element and patch count
  int num_elements_x                  ; // Number of skin elements per patch in x-direction
  int num_elements_y                  ; // Number of skin elements per patch in y-direction  (note: total number of elements per patch < 100)
  int num_patches_x                   ; // Number of skin patches in x-direction
  int num_patches_y                   ; // Number of skin patches in y-direction
  // Skin element properties
  double element_diameter             ; // Diameter of each skin element [meters]
  double element_height               ; // The height of the skin element from the ground [meters]
  double element_mass                 ; // Mass of skin element [kg]
  double element_spring               ; // Spring constant of skin element [N/m]
  double element_damping              ; // Damping of skin element
  // Skin plane properties
  double plane_thickness              ; // Thickness of the skin plane [meters]
  double plane_height                 ; // The height of the plane from the ground [meters]
  // Skin array placement
  double init_x                       ; // Initial x position [meters]
  double init_y                       ; // Initial y position [meters]
  double init_z                       ; // Initial z position [meters]
  std::string parent                  ; // Name of parent link
  // Data collection with ROS
  std::string  ros_namespace          ; // Namespace for ROS topic
  double update_rate                  ; // Update rate of plugin (0 means as fast as possible)
  // TODO: override count settings if length > 0
  double patch_length_x               ; // Length of skin patch in x-direction
  double patch_length_y               ; // Length of skin patch in y-direction
  double total_length_x               ; // Length of skin array in x-direction
  double total_length_y               ; // Length of skin array in y-direction
  // TODO: tactile layer properties
  double tactile_length               ; // The radius of the sensors underneath the skin layer (for which skin layer is marked in red) ?
  double tactile_separation           ; // The space between each sensor underneath the skin layer (for which there are spaces between the rows and columns of skin elements) ?
};

struct BuildModelSpec
{
  std::string name;
  ModelSpec   spec;
};

inline void print(BuildModelSpec b)
{
	std::cout<<"Name                    : "<<b.name                        <<"\n";
	std::cout<<" num_elements_x         : "<<b.spec.num_elements_x         <<"\n";
	std::cout<<" num_elements_y         : "<<b.spec.num_elements_y         <<"\n";
	std::cout<<" num_patches_x          : "<<b.spec.num_patches_x          <<"\n";
	std::cout<<" num_patches_y          : "<<b.spec.num_patches_y          <<"\n";
	std::cout<<" element_diameter       : "<<b.spec.element_diameter       <<"\n";
	std::cout<<" element_height         : "<<b.spec.element_height         <<"\n";
	std::cout<<" element_mass           : "<<b.spec.element_mass           <<"\n";
	std::cout<<" element_spring         : "<<b.spec.element_spring         <<"\n";
	std::cout<<" element_damping        : "<<b.spec.element_damping        <<"\n";
	std::cout<<" plane_thickness        : "<<b.spec.plane_thickness        <<"\n";
	std::cout<<" plane_height           : "<<b.spec.plane_height           <<"\n";
	std::cout<<" init_x                 : "<<b.spec.init_x                 <<"\n";
	std::cout<<" init_y                 : "<<b.spec.init_y                 <<"\n";
	std::cout<<" init_z                 : "<<b.spec.init_z                 <<"\n";
	std::cout<<" parent                 : "<<b.spec.parent                 <<"\n";
	std::cout<<" ros_namespace          : "<<b.spec.ros_namespace          <<"\n";
	std::cout<<" update_rate            : "<<b.spec.update_rate            <<"\n";
	std::cout<<" patch_length_x         : "<<b.spec.patch_length_x         <<"\n";
	std::cout<<" patch_length_y         : "<<b.spec.patch_length_y         <<"\n";
	std::cout<<" total_length_x         : "<<b.spec.total_length_x         <<"\n";
	std::cout<<" total_length_y         : "<<b.spec.total_length_y         <<"\n";
	std::cout<<" tactile_length         : "<<b.spec.tactile_length         <<"\n";
	std::cout<<" tactile_separation     : "<<b.spec.tactile_separation     <<"\n";
}

// Read from YAML
inline void operator >> (const YAML::Node& node, ModelSpec& spec)
{
	spec.num_elements_x      = node["num_elements_x"    ].as<int>() ;
	spec.num_elements_y      = node["num_elements_y"    ].as<int>() ;
	spec.num_patches_x       = node["num_patches_x"     ].as<int>() ;
	spec.num_patches_y       = node["num_patches_y"     ].as<int>() ;
	spec.element_diameter    = node["element_diameter"  ].as<double>() ;
	spec.element_height      = node["element_height"    ].as<double>() ;
	spec.element_mass        = node["element_mass"      ].as<double>() ;
	spec.element_spring      = node["element_spring"    ].as<double>() ;
	spec.element_damping     = node["element_damping"   ].as<double>() ;
	spec.plane_thickness     = node["plane_thickness"   ].as<double>() ;
	spec.plane_height        = node["plane_height"      ].as<double>() ;
	spec.init_x              = node["init_x"            ].as<double>() ;
	spec.init_y              = node["init_y"            ].as<double>() ;
	spec.init_z              = node["init_z"            ].as<double>() ;
	spec.parent              = node["parent"            ].as<std::string>() ;
	spec.ros_namespace       = node["ros_namespace"     ].as<std::string>() ;
	spec.update_rate         = node["update_rate"       ].as<double>() ;
	spec.patch_length_x      = node["patch_length_x"    ].as<double>() ;
	spec.patch_length_y      = node["patch_length_y"    ].as<double>() ;
	spec.total_length_x      = node["total_length_x"    ].as<double>() ;
	spec.total_length_y      = node["total_length_y"    ].as<double>() ;
	spec.tactile_length      = node["tactile_length"    ].as<double>() ;
	spec.tactile_separation  = node["tactile_separation"].as<double>() ;
}

inline void operator >> (const YAML::Node& node, BuildModelSpec& buildModelSpec)
{
	try
	{
		buildModelSpec.name = node["name"].as<std::string>();

		const YAML::Node& specs = node["spec"];
		specs[0] >> buildModelSpec.spec ;
	}
	catch (const YAML::BadConversion& e)
	{
		std::cerr<<"Bad conversion from YAML::Node to BuildModelSpec\n";
	}

	//print(buildModelSpec);
}

// Write to YAML
inline YAML::Emitter& operator << (YAML::Emitter& out, const ModelSpec& spec)
{
    out << YAML::BeginMap;
    out << YAML::Key << "num_elements_x"      ; out << YAML::Value <<  spec.num_elements_x     ;
    out << YAML::Key << "num_elements_y"      ; out << YAML::Value <<  spec.num_elements_y     ;
    out << YAML::Key << "num_patches_x"       ; out << YAML::Value <<  spec.num_patches_x      ;
    out << YAML::Key << "num_patches_y"       ; out << YAML::Value <<  spec.num_patches_y      ;
    out << YAML::Key << "element_diameter"    ; out << YAML::Value <<  spec.element_diameter   ;
    out << YAML::Key << "element_height"      ; out << YAML::Value <<  spec.element_height     ;
    out << YAML::Key << "element_mass"        ; out << YAML::Value <<  spec.element_mass       ;
    out << YAML::Key << "element_spring"      ; out << YAML::Value <<  spec.element_spring     ;
    out << YAML::Key << "element_damping"     ; out << YAML::Value <<  spec.element_damping    ;
    out << YAML::Key << "plane_thickness"     ; out << YAML::Value <<  spec.plane_thickness    ;
    out << YAML::Key << "plane_height"        ; out << YAML::Value <<  spec.plane_height       ;
    out << YAML::Key << "init_x"              ; out << YAML::Value <<  spec.init_x             ;
    out << YAML::Key << "init_y"              ; out << YAML::Value <<  spec.init_y             ;
    out << YAML::Key << "init_z"              ; out << YAML::Value <<  spec.init_z             ;
    out << YAML::Key << "parent"              ; out << YAML::Value <<  spec.parent             ;
    out << YAML::Key << "ros_namespace"       ; out << YAML::Value <<  spec.ros_namespace      ;
    out << YAML::Key << "update_rate"         ; out << YAML::Value <<  spec.update_rate        ;
    out << YAML::Key << "patch_length_x"      ; out << YAML::Value <<  spec.patch_length_x     ;
    out << YAML::Key << "patch_length_y"      ; out << YAML::Value <<  spec.patch_length_y     ;
    out << YAML::Key << "total_length_x"      ; out << YAML::Value <<  spec.total_length_x     ;
    out << YAML::Key << "total_length_y"      ; out << YAML::Value <<  spec.total_length_y     ;
    out << YAML::Key << "tactile_length"      ; out << YAML::Value <<  spec.tactile_length     ;
    out << YAML::Key << "tactile_separation"  ; out << YAML::Value <<  spec.tactile_separation ;
    out << YAML::EndMap;
    return out;
}

inline YAML::Emitter& operator << (YAML::Emitter& out, const BuildModelSpec& buildModelSpec)
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
