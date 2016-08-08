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

/*
 * auto_generator.cpp
 *
 *  Created on: Aug 8, 2016
 *      Author: Sven Cremer
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <SkinSim/ControlSpecYAML.hh>
#include <SkinSim/ModelSpecYAML.hh>
#include <SkinSim/model_builder.h>

using namespace SkinSim;

int main(int argc, char** argv)
{

	std::string exp_name = "exp01";

	// Check the number of command-line parameters
	if (argc == 2)
	{
		// Set model file name
		exp_name = argv[1];
	}
	else
	{
		// Use default file name
		std::cout<<"\n\tUsage: "<<argv[0]<<" [EXPERIMENT NAME]\n\n";
	}

	std::string pathSkinSim = getenv ("SKINSIM_PATH");
	std::string pathExp     = pathSkinSim + "/data/" + exp_name;

	// Create experiment directory
	boost::filesystem::path dir(pathExp);
	if(!boost::filesystem::create_directory(dir))
	{
		std::cerr << "Failed to create experiment directory!" << "\n";
		return 1;
	}

	// Parameters
	std::string filename_model   = "mdlSpecs.yaml";;
	std::string filename_control = "ctrSpecs.yaml";

	std::string mdlSpecPath = pathExp + "/" + filename_model;
	std::string ctrSpecPath = pathExp + "/" + filename_control;

	// ---------------------------------------------

	// Generate model specifications

	std::vector<BuildModelSpec>  modelSpecs;
	BuildModelSpec defaultModelSpec;

	// Set default values
	defaultModelSpec.name                      = "skin_array";
	defaultModelSpec.spec.num_elements_x       = 8;
	defaultModelSpec.spec.num_elements_y       = 8;
	defaultModelSpec.spec.num_patches_x        = 1;
	defaultModelSpec.spec.num_patches_y        = 1;
	defaultModelSpec.spec.element_diameter     = 0.01;
	defaultModelSpec.spec.element_height       = 0.01;
	defaultModelSpec.spec.element_mass         = 0.0;
	defaultModelSpec.spec.element_spring       = 122.24;
	defaultModelSpec.spec.element_damping      = 1.183;
	defaultModelSpec.spec.plane_thickness      = 0.002;
	defaultModelSpec.spec.plane_height         = 0.001;
	defaultModelSpec.spec.init_x               = 0.0;
	defaultModelSpec.spec.init_y               = 0.0;
	defaultModelSpec.spec.init_z               = 0.0;
	defaultModelSpec.spec.parent               = "world";
	defaultModelSpec.spec.ros_namespace        = "skinsim";
	defaultModelSpec.spec.update_rate          = 0.0;
	defaultModelSpec.spec.patch_length_x       = 0.0;
	defaultModelSpec.spec.patch_length_y       = 0.0;
	defaultModelSpec.spec.total_length_x       = 0.0;
	defaultModelSpec.spec.total_length_y       = 0.0;
	defaultModelSpec.spec.tactile_elements_x   = 2;
	defaultModelSpec.spec.tactile_elements_y   = 2;
	defaultModelSpec.spec.tactile_separation_x = 1;
	defaultModelSpec.spec.tactile_separation_y = 1;

	for(unsigned i  = 0; i < 3 ; i++ )
	{
		BuildModelSpec tempModelSpec = defaultModelSpec;

		tempModelSpec.name = "skin_array_" + boost::lexical_cast<std::string>( i );
		tempModelSpec.spec.tactile_elements_x   = i+1;
		tempModelSpec.spec.tactile_elements_y   = i+1;
		tempModelSpec.spec.tactile_separation_x = 1;
		tempModelSpec.spec.tactile_separation_y = 1;

		modelSpecs.push_back( tempModelSpec ) ;
	}

	// Save to YAML
	YAML::Emitter mdlYAMLEmitter;
	mdlYAMLEmitter << modelSpecs;

	// Write to YAML file
	std::ofstream mdlOut(mdlSpecPath.c_str());
	std::cout<<"Saving model specs to file: "<<mdlSpecPath<<"\n";
	mdlOut << mdlYAMLEmitter.c_str();;
	mdlOut.close();

	// ---------------------------------------------

	// Generate and save SDF models
	for(unsigned i = 0; i < modelSpecs.size() ;i++)
	{
		ModelBuilder skinSimModelBuilderObject( modelSpecs[i] );	// TODO store in exp directory and update Gazebo model path
	}

	// ---------------------------------------------

	// Generate control specifications

	std::vector<ControllerSpec> ctrSpecs;
	ControllerSpec defaultControlSpec;

	// Set default values
	defaultControlSpec.name         = "efc_00_00_00" ;
	defaultControlSpec.explFctr_Kp  = 2       ;
	defaultControlSpec.explFctr_Ki  = 0.00005 ;
	defaultControlSpec.explFctr_Kd  = 0.5     ;
	defaultControlSpec.impCtr_Xnom  = 0.5     ;
	defaultControlSpec.impCtr_M     = 5       ;
	defaultControlSpec.impCtr_K     = 24      ;
	defaultControlSpec.impCtr_D     = 10      ;
	defaultControlSpec.ctrType      = 1       ;
	defaultControlSpec.targetForce  = 0.01    ;

	for(unsigned i  = 0; i < 2 ; i++ )
	{
		ControllerSpec tempControlSpec = defaultControlSpec;

		tempControlSpec.name = "control_" + boost::lexical_cast<std::string>( i );

		ctrSpecs.push_back( tempControlSpec ) ;
	}

	// Save to YAML
	YAML::Emitter ctrYAMLEmitter;
	ctrYAMLEmitter << ctrSpecs;

	// Write to YAML file
	std::ofstream ctrOut(ctrSpecPath.c_str());
	std::cout<<"Saving control specs to file: "<<ctrSpecPath<<"\n";
	ctrOut << ctrYAMLEmitter.c_str();;
	ctrOut.close();

	// ---------------------------------------------

	return 0;

}


