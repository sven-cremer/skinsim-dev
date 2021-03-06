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
  double element_height               ; // The height of the skin element above the ground [meters]
  double element_mass                 ; // Mass of skin element [kg]
  double element_spring               ; // Spring constant of skin element [N/m]
  double element_damping              ; // Damping of skin element
  // Skin plane properties
  double plane_thickness              ; // Thickness of the skin plane [meters]
  double plane_height                 ; // The height of the plane above the ground [meters]
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
  // Tactile sensor properties
  int tactile_elements_x              ; // Number of skin elements per sensor in x-direction
  int tactile_elements_y              ; // Number of skin elements per sensor in y-direction
  int tactile_separation_x            ; // Spaceing between sensors in terms of number of elements in x-direction
  int tactile_separation_y            ; // Spaceing between sensors in terms of number of elements in x-direction
  // Tactile sensor noise
  double noiseSigma                   ; // Gaussian Noise Sigma
  double noiseMu                      ; // Gaussiam Noise Mu
  double noiseAmplitude               ; // Gaussian Noise Amplitude
  // Time Delay
  double delay                        ; // Time Delay
  // Force spread model
  double spread_scaling               ; // Force spread scaling factor
  double spread_sigma                 ; // Force spread standard deviation
  // Plunger properties
  double plunger_radius               ;
  double plunger_length               ;
  double plunger_mass                 ; // Mass of plunger [kg]
  double plunger_spring               ; // Stiffness [N/m]
  double plunger_damping              ; // Damping
  bool   plunger_gravity              ; // Turn on gravity for plunger
  double plunger_offset_x             ; // Plunger Location Offset X (with respect to center of skin patch)
  double plunger_offset_y             ; // Plunger Location Offset Y
  double plunger_offset_z             ; // Plunger Location Offset Z (height above skin patch)
  // Physics engine
  int    solver_iterations            ; // Solver iterations
  double step_size                    ; // Max step size [seconds]
  double max_sim_time                 ; // When to stop simulation [seconds]
  // Data collection
  std::string topic                   ; // Which ROS topic to save
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
	std::cout<<" tactile_elements_x     : "<<b.spec.tactile_elements_x     <<"\n";
	std::cout<<" tactile_elements_y     : "<<b.spec.tactile_elements_y     <<"\n";
	std::cout<<" tactile_separation_x   : "<<b.spec.tactile_separation_x   <<"\n";
	std::cout<<" tactile_separation_y   : "<<b.spec.tactile_separation_y   <<"\n";
	std::cout<<" spread_scaling         : "<<b.spec.spread_scaling         <<"\n";
	std::cout<<" spread_sigma           : "<<b.spec.spread_sigma           <<"\n";
	std::cout<<" noise_sigma            : "<<b.spec.noiseSigma             <<"\n";
	std::cout<<" noise_mu               : "<<b.spec.noiseMu                <<"\n";
	std::cout<<" noiseAmplitude         : "<<b.spec.noiseAmplitude         <<"\n";
	std::cout<<" delay                  : "<<b.spec.delay                  <<"\n";
	std::cout<<" plunger_radius         : "<<b.spec.plunger_radius         <<"\n";
	std::cout<<" plunger_length         : "<<b.spec.plunger_length         <<"\n";
	std::cout<<" plunger_mass           : "<<b.spec.plunger_mass           <<"\n";
	std::cout<<" plunger_spring         : "<<b.spec.plunger_spring         <<"\n";
	std::cout<<" plunger_damping        : "<<b.spec.plunger_damping        <<"\n";
	std::cout<<" plunger_gravity        : "<<b.spec.plunger_gravity        <<"\n";
	std::cout<<" plunger_offset_x       : "<<b.spec.plunger_offset_x       <<"\n";
	std::cout<<" plunger_offset_y       : "<<b.spec.plunger_offset_y       <<"\n";
	std::cout<<" plunger_offset_z       : "<<b.spec.plunger_offset_z       <<"\n";
	std::cout<<" solver_iterations      : "<<b.spec.solver_iterations      <<"\n";
	std::cout<<" step_size              : "<<b.spec.step_size              <<"\n";
	std::cout<<" max_sim_time           : "<<b.spec.max_sim_time           <<"\n";
	std::cout<<" topic                  : "<<b.spec.topic                  <<"\n";
}

// Read from YAML, TODO check for tabs (i.e. \t) in the config file since this breaks the code
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
	spec.tactile_elements_x   = node["tactile_elements_x"  ].as<int>() ;
	spec.tactile_elements_y   = node["tactile_elements_y"  ].as<int>() ;
	spec.tactile_separation_x = node["tactile_separation_x"].as<int>() ;
	spec.tactile_separation_y = node["tactile_separation_y"].as<int>() ;
	spec.spread_scaling      = node["spread_scaling"    ].as<double>() ;
	spec.spread_sigma        = node["spread_sigma"      ].as<double>() ;
	spec.noiseSigma          = node["noiseSigma"        ].as<double>() ;
	spec.noiseMu             = node["noiseMu"           ].as<double>() ;
	spec.noiseAmplitude      = node["noiseAmplitude"    ].as<double>() ;
	spec.delay               = node["delay"             ].as<double>() ;
	spec.plunger_radius      = node["plunger_radius"    ].as<double>() ;
	spec.plunger_length      = node["plunger_length"    ].as<double>() ;
	spec.plunger_mass        = node["plunger_mass"      ].as<double>() ;
	spec.plunger_spring      = node["plunger_spring"    ].as<double>() ;
	spec.plunger_damping     = node["plunger_damping"   ].as<double>() ;
	spec.plunger_gravity     = node["plunger_gravity"   ].as<bool>()   ;
	spec.plunger_offset_x    = node["plunger_offset_x"  ].as<double>() ;
	spec.plunger_offset_y    = node["plunger_offset_y"  ].as<double>() ;
	spec.plunger_offset_z    = node["plunger_offset_z"  ].as<double>() ;
	spec.solver_iterations   = node["solver_iterations" ].as<int>()    ;
	spec.step_size           = node["step_size"         ].as<double>() ;
	spec.max_sim_time        = node["max_sim_time"      ].as<double>() ;
	spec.topic               = node["topic"             ].as<std::string>() ;
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
    out << YAML::Key << "num_elements_x"       << YAML::Value <<  spec.num_elements_x     ;
    out << YAML::Key << "num_elements_y"       << YAML::Value <<  spec.num_elements_y     ;
    out << YAML::Key << "num_patches_x"        << YAML::Value <<  spec.num_patches_x      ;
    out << YAML::Key << "num_patches_y"        << YAML::Value <<  spec.num_patches_y      ;
    out << YAML::Key << "element_diameter"     << YAML::Value <<  spec.element_diameter   ;
    out << YAML::Key << "element_height"       << YAML::Value <<  spec.element_height     ;
    out << YAML::Key << "element_mass"         << YAML::Value <<  spec.element_mass       ;
    out << YAML::Key << "element_spring"       << YAML::Value <<  spec.element_spring     ;
    out << YAML::Key << "element_damping"      << YAML::Value <<  spec.element_damping    ;
    out << YAML::Key << "plane_thickness"      << YAML::Value <<  spec.plane_thickness    ;
    out << YAML::Key << "plane_height"         << YAML::Value <<  spec.plane_height       ;
    out << YAML::Key << "init_x"               << YAML::Value <<  spec.init_x             ;
    out << YAML::Key << "init_y"               << YAML::Value <<  spec.init_y             ;
    out << YAML::Key << "init_z"               << YAML::Value <<  spec.init_z             ;
    out << YAML::Key << "parent"               << YAML::Value <<  spec.parent             ;
    out << YAML::Key << "ros_namespace"        << YAML::Value <<  spec.ros_namespace      ;
    out << YAML::Key << "update_rate"          << YAML::Value <<  spec.update_rate        ;
    out << YAML::Key << "patch_length_x"       << YAML::Value <<  spec.patch_length_x     ;
    out << YAML::Key << "patch_length_y"       << YAML::Value <<  spec.patch_length_y     ;
    out << YAML::Key << "total_length_x"       << YAML::Value <<  spec.total_length_x     ;
    out << YAML::Key << "total_length_y"       << YAML::Value <<  spec.total_length_y     ;
    out << YAML::Key << "tactile_elements_x"   << YAML::Value <<  spec.tactile_elements_x  ;
    out << YAML::Key << "tactile_elements_y"   << YAML::Value <<  spec.tactile_elements_y  ;
    out << YAML::Key << "tactile_separation_x" << YAML::Value <<  spec.tactile_separation_x;
    out << YAML::Key << "tactile_separation_y" << YAML::Value <<  spec.tactile_separation_y;
    out << YAML::Key << "spread_scaling"       << YAML::Value <<  spec.spread_scaling     ;
    out << YAML::Key << "spread_sigma"         << YAML::Value <<  spec.spread_sigma       ;
    out << YAML::Key << "noiseSigma"           << YAML::Value <<  spec.noiseSigma         ;
    out << YAML::Key << "noiseMu"              << YAML::Value <<  spec.noiseMu            ;
    out << YAML::Key << "noiseAmplitude"       << YAML::Value <<  spec.noiseAmplitude     ;
    out << YAML::Key << "plunger_radius"       << YAML::Value <<  spec.plunger_radius     ;
    out << YAML::Key << "plunger_length"       << YAML::Value <<  spec.plunger_length     ;
    out << YAML::Key << "plunger_mass"         << YAML::Value <<  spec.plunger_mass       ;
    out << YAML::Key << "plunger_spring"       << YAML::Value <<  spec.plunger_spring     ;
    out << YAML::Key << "plunger_damping"      << YAML::Value <<  spec.plunger_damping    ;
    out << YAML::Key << "plunger_gravity"      << YAML::Value <<  spec.plunger_gravity    ;
    out << YAML::Key << "plunger_offset_x"     << YAML::Value <<  spec.plunger_offset_x   ;
    out << YAML::Key << "plunger_offset_y"     << YAML::Value <<  spec.plunger_offset_y   ;
    out << YAML::Key << "plunger_offset_z"     << YAML::Value <<  spec.plunger_offset_z   ;
    out << YAML::Key << "solver_iterations"    << YAML::Value <<  spec.solver_iterations  ;
    out << YAML::Key << "step_size"            << YAML::Value <<  spec.step_size          ;
    out << YAML::Key << "max_sim_time"         << YAML::Value <<  spec.max_sim_time       ;
    out << YAML::Key << "delay"                << YAML::Value <<  spec.delay              ;
    out << YAML::Key << "topic"                << YAML::Value <<  spec.topic              ;
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
