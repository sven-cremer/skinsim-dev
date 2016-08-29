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
		if( boost::filesystem::exists(dir) )
		{
			std::cout << "Warning: Experiment folder already exists ... will replace files!" << "\n";
		}
		else
		{
			std::cerr << "Failed to create experiment directory!" << "\n";
			return 1;
		}
	}

	// Parameters
	std::string filename_model   = "mdlSpecs.yaml";;
	std::string filename_control = "ctrSpecs.yaml";

	std::string mdlSpecPath = pathExp + "/" + filename_model;
	std::string ctrSpecPath = pathExp + "/" + filename_control;

	std::string defaultModelSpecPath = pathSkinSim + "/generator/config/model_params.yaml";

	// ---------------------------------------------

	// Generate model specifications

	std::vector<BuildModelSpec>  modelSpecs;
	BuildModelSpec defaultModelSpec;

	// Read default YAML model file
	std::ifstream fin(defaultModelSpecPath.c_str());
	YAML::Node doc_model;
	std::cout<<"Loading file: "<<defaultModelSpecPath<<"\n";
	doc_model = YAML::LoadAll(fin);
	doc_model[0][0] >> defaultModelSpec;
	std::cout<<"DEFAULT VALUES:\n";
	print(defaultModelSpec);

	// Set default values

	/* Shook, "Experimental testbed for robotic skin characterization and interaction control", 2014.
	 * Table 4-6: Parameters for 4mm Frubber skin
	 *   k = k1+k2 = 1523+481  = 2004 [N/m]
	 *   b = b1+b2 = 242.6+243 = 485.6 [Ns/m]
	 *
	 * Assumption 1:  # elements inside circle    pi*r^2   pi
	 *                ------------------------- = ------= ---
	 *                # elements inside square    (2r)^2   4
	 *
	 * Assumption 2: element diameter is fixed to 1.00cm
	 *               skin patch is 30 elements wide
	 *               plunger is 16 spheres wide, i.e. d=0.16
	 *
	 *                               pi
	 * (# elements inside circle) = --- * (16^2) = 201
	 *                               4
	 * k_element =  2004  * (0.16/0.99) * (1/201) = 1.611 [N/m]
	 * b_element =  485.6 * (0.16/0.99) * (1/201) = 0.390 [N/m]
	 *
	 */
	//defaultModelSpec.spec.element_spring       = 10.0;  // TODO compute this automatically from plunger diameter
	//defaultModelSpec.spec.element_damping      = 0.1;   // TODO compute this automatically from plunger diameter

	//modelSpecs.push_back( defaultModelSpec ) ;


	// Tactile layout
	for(unsigned i  = 3; i < 4 ; i++ )		// Tactile size
	{
		for(unsigned j  = 3; j < 4 ; j++ )	// Tactile separation
		{
			BuildModelSpec tempModelSpec = defaultModelSpec;

			tempModelSpec.name = "skin_array_s_" + boost::lexical_cast<std::string>( i ) + "_sep_" + boost::lexical_cast<std::string>( j );
			tempModelSpec.spec.tactile_elements_x   = i;
			tempModelSpec.spec.tactile_elements_y   = i;
			tempModelSpec.spec.tactile_separation_x = j;
			tempModelSpec.spec.tactile_separation_y = j;

			modelSpecs.push_back( tempModelSpec ) ;
		}
	}
/*
	// Model parameters
	for(unsigned i  = 0; i < 3 ; i++ )
	{
		BuildModelSpec tempModelSpec = defaultModelSpec;

		tempModelSpec.name = "skin_array_" + boost::lexical_cast<std::string>( i );
		tempModelSpec.spec.element_spring       = 10.0;
		tempModelSpec.spec.element_damping      = 0.10;
		defaultModelSpec.spec.element_mass      = 0.0001*pow(10,i+1);
		modelSpecs.push_back( tempModelSpec ) ;
	}
*/

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
	defaultControlSpec.name         	= "efc_00_00_00" ;
	defaultControlSpec.explFctr_Kp  	= 0.2     ;
	defaultControlSpec.explFctr_Ki  	= 0.0     ;
	defaultControlSpec.explFctr_Kd  	= 0.0     ;
	defaultControlSpec.impCtr_Xnom  	= 0       ;
	defaultControlSpec.impCtr_M     	= 0       ;
	defaultControlSpec.impCtr_K     	= 0       ;
	defaultControlSpec.impCtr_D     	= 0       ;
	defaultControlSpec.targetForce  	= -2      ;
	defaultControlSpec.controller_type	= 1		  ;		//DIRECT=0, FORCE_BASED_FORCE_CONTROL=1, POSITION_BASED_FORCE_CONTROL=2, IMPEDANCE_CONTROL=3
	defaultControlSpec.feedback_type	= 1 	  ; 	//PLUNGER_LOAD_CELL=0, TACTILE_APPLIED=1, TACTILE_SENSED=2
	defaultControlSpec.plunger_Kp 		= 1		  ;
	defaultControlSpec.plunger_Ki 		= 0 	  ;
	defaultControlSpec.plunger_Kd 		= 0 	  ;
	defaultControlSpec.plunger_Kv		= 0 	  ;

	int num = 0;
//	for(unsigned j  = 1; j < 3; j++ ) // Feedback type
//	{
//		for(unsigned i  = 0; i < 1; i++ ) // Kp
//		{
//			ControllerSpec tempControlSpec = defaultControlSpec;
//
//			tempControlSpec.name = "control_" + boost::lexical_cast<std::string>( num );
//			tempControlSpec.explFctr_Kp = 0.5*i;
//			tempControlSpec.explFctr_Kd = 0.0;
//			tempControlSpec.controller_type = j;
//			ctrSpecs.push_back( tempControlSpec ) ;
//			num++;
//		}
//	}
	ctrSpecs.push_back( defaultControlSpec ) ;
//	for(unsigned i  = 0; i < 10 ; i++ )
//	{
//		ControllerSpec tempControlSpec = defaultControlSpec;
//
//		tempControlSpec.name = "control_" + boost::lexical_cast<std::string>( i );
//		tempControlSpec.explFctr_Kp = 1.6;
//		tempControlSpec.explFctr_Kd = 0.1*i;
//		ctrSpecs.push_back( tempControlSpec ) ;
//	}

	// Save to YAML
	YAML::Emitter ctrYAMLEmitter;
	ctrYAMLEmitter << ctrSpecs;

	// Write to YAML file
	std::ofstream ctrOut(ctrSpecPath.c_str());
	std::cout<<"Saving control specs to file: "<<ctrSpecPath<<"\n";
	ctrOut << ctrYAMLEmitter.c_str();;
	ctrOut.close();

	// ---------------------------------------------

	std::cout<<"\nNumber of models generated:      "<<modelSpecs.size();
	std::cout<<"\nNumber of controllers generated: "<<ctrSpecs.size();
	std::cout<<"\nTotal configurations: "<<modelSpecs.size()*ctrSpecs.size()<<"\n";

	return 0;

}


