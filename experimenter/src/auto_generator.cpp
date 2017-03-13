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
	// Set default Model values

	std::vector<BuildModelSpec>  modelSpecs;
	BuildModelSpec defaultModelSpec;

	// Read default YAML model file
	std::ifstream fin(defaultModelSpecPath.c_str());
	YAML::Node doc_model;
	std::cout<<"Loading file: "<<defaultModelSpecPath<<"\n";
	doc_model = YAML::LoadAll(fin);
	doc_model[0][0] >> defaultModelSpec;
	std::cout<<"DEFAULT MODEL VALUES:\n";
	print(defaultModelSpec);

	// ---------------------------------------------
	// Set default Controller values

	std::vector<ControllerSpec> ctrSpecs;
	ControllerSpec defaultControlSpec;

	// Set default values
	defaultControlSpec.name         	 = "efc_00_00_00" ;
	defaultControlSpec.impCtr_Xnom  	 = 0;
	defaultControlSpec.impCtr_M     	 = 0;
	defaultControlSpec.impCtr_D     	 = 0;
	defaultControlSpec.impCtr_K     	 = 0;

	defaultControlSpec.controller_type   = 4;    //DIRECT=0, FORCE_BASED_FORCE_CONTROL=1, POSITION_BASED_FORCE_CONTROL=2, IMPEDANCE_CONTROL=3, DIGITAL_PID=4
	defaultControlSpec.feedback_type     = 2;    //PLUNGER_LOAD_CELL=0, TACTILE_APPLIED=1, TACTILE_SENSED=2
	defaultControlSpec.Fd                = 1;

	defaultControlSpec.Kp                = 2.0; //420; //0.001;
	defaultControlSpec.Ki                = 20.0; //5200; //0.000008333;
	defaultControlSpec.Kd                = 0.1; //-3.05;//0.0003;
	defaultControlSpec.Kv                = 0.0;

	defaultControlSpec.Ts                = 0.005;
	defaultControlSpec.Nf                = 10; //0.045;//10;

	std::cout<<"DEFAULT CONTROL VALUES:\n";
	print(defaultControlSpec);

	// ---------------------------------------------
	// Generate model specifications

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

/*
	// Tactile layout
	for(unsigned i  = 1; i < 5 ; i++ )		// Tactile size
	{
		for(unsigned j  = 1; j < 5 ; j++ )	// Tactile separation
		{
			BuildModelSpec tempModelSpec = defaultModelSpec;

			tempModelSpec.name = "skin_array_s_" + boost::lexical_cast<std::string>( i ) + "_sep_" + boost::lexical_cast<std::string>( j );
			tempModelSpec.spec.tactile_elements_x   = i;
			tempModelSpec.spec.tactile_elements_y   = i;
			tempModelSpec.spec.tactile_separation_x = j;
			tempModelSpec.spec.tactile_separation_y = j;

			//tempModelSpec.spec.plunger_offset_x = j*tempModelSpec.spec.element_diameter*0.5;	// Assume this gives max COP error
			//tempModelSpec.spec.plunger_offset_y = j*tempModelSpec.spec.element_diameter*0.5;

			modelSpecs.push_back( tempModelSpec ) ;
		}
	}
*/
/*
	// Plunger Offset value
	for (int i  = 0; i < 13 ; i++ )
	{
		BuildModelSpec tempModelSpec = defaultModelSpec;

		tempModelSpec.name = "skin_array_s_" + boost::lexical_cast<std::string>( 3 ) + "_sep_" + boost::lexical_cast<std::string>( 3 ) + "_offset_" + boost::lexical_cast<std::string>( i );
		tempModelSpec.spec.tactile_elements_x   = 3;
		tempModelSpec.spec.tactile_elements_y   = 3;
		tempModelSpec.spec.tactile_separation_x = 3;
		tempModelSpec.spec.tactile_separation_y = 3;

		tempModelSpec.spec.plunger_offset_x = i*0.0025;
		tempModelSpec.spec.plunger_offset_y = i*0.0025;

		modelSpecs.push_back( tempModelSpec ) ;
	}
*/
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
/*
	// Compute Ts from N
	int idx = 0;
	const int A = 1;
	int ix [A] = { 3};
	int jx [A] = { 2};

//	for(unsigned i  = 1; i < 5 ; i++ )		// Tactile size
//	{
//		for(unsigned j  = 1; j < 5 ; j++ )	// Tactile separation
//		{
	for(unsigned k  = 0; k < A ; k++ )	// Tactile separation
	{
		int i = ix[k];
		int j = jx[k];

			BuildModelSpec tempModelSpec = defaultModelSpec;

			tempModelSpec.name = "skin_array_s_" + boost::lexical_cast<std::string>( i ) + "_sep_" + boost::lexical_cast<std::string>( j );
			tempModelSpec.spec.tactile_separation_x = j;
			tempModelSpec.spec.tactile_separation_y = j;
			tempModelSpec.spec.tactile_elements_x   = i;
			tempModelSpec.spec.tactile_elements_y   = i;
			tempModelSpec.spec.solver_iterations    = 750;


			modelSpecs.push_back( tempModelSpec ) ;


			// Compute number of sensors N
			int total_elements_x = tempModelSpec.spec.num_elements_x*tempModelSpec.spec.num_patches_x;
			int total_elements_y = tempModelSpec.spec.num_elements_y*tempModelSpec.spec.num_patches_y;
			int unit_size_x      = tempModelSpec.spec.tactile_elements_x+tempModelSpec.spec.tactile_separation_x;
			int unit_size_y      = tempModelSpec.spec.tactile_elements_y+tempModelSpec.spec.tactile_separation_y;
			int total_sensors_x  = total_elements_x/unit_size_x;	// Note: integer devision rounds down
			int total_sensors_y  = total_elements_y/unit_size_y;

			// Check if there is room for one more sensor
			if(total_elements_x - total_sensors_x*unit_size_x >=  tempModelSpec.spec.tactile_elements_x )
				total_sensors_x++;
			if(total_elements_y - total_sensors_y*unit_size_y >=  tempModelSpec.spec.tactile_elements_y )
				total_sensors_y++;
			int N = total_sensors_y*total_sensors_y;

			// Control Specs
			ControllerSpec tempControlSpec = defaultControlSpec;
			tempControlSpec.name = "control_" + boost::lexical_cast<std::string>( idx );
			tempControlSpec.Ts = 0.0001*(double)N;		// Assume a linear mapping
			ctrSpecs.push_back( tempControlSpec ) ;
			idx++;

			std::cout<<"Size & Sep: "<<i<<", "<<j<<"\tN: "<<N<<"\tTs: "<<tempControlSpec.Ts<<"\n";

			//}
		}
	//}
*/
	double contacts = 120;

	//Build one model
	BuildModelSpec tempModelSpec = defaultModelSpec;
	tempModelSpec.name = "skin_array_s_" + boost::lexical_cast<std::string>( 1 ) + "_sep_" + boost::lexical_cast<std::string>( 2 );
	tempModelSpec.spec.tactile_separation_x = 2;
	tempModelSpec.spec.tactile_separation_y = 2;
	tempModelSpec.spec.tactile_elements_x   = 1;
	tempModelSpec.spec.tactile_elements_y   = 1;
	tempModelSpec.spec.element_mass         = 0.01;
	tempModelSpec.spec.element_damping      = 242.6/(contacts*6); //2.02;		Si: 248.6/contacts    Frubber: 242.6
	tempModelSpec.spec.element_spring       = 1523 /(contacts*6); //12.69;		Si: 4338/contacts     Frubber: 1523
	tempModelSpec.spec.element_diameter     = 0.0016644;
	tempModelSpec.spec.element_height       = 0.0096644;
	tempModelSpec.spec.spread_scaling       = 1.0; //0.085;
	tempModelSpec.spec.spread_sigma         = 0.00102;
	tempModelSpec.spec.plunger_radius       = 0.010;
	tempModelSpec.spec.plunger_mass         = 1.0;
	tempModelSpec.spec.plane_thickness      = 0.0005;
	tempModelSpec.spec.plunger_length       = 0.03;

	tempModelSpec.spec.noiseAmplitude       = 0.0;
	tempModelSpec.spec.noiseSigma           = 0.0;

	tempModelSpec.spec.max_sim_time         = 1.0;
	tempModelSpec.spec.solver_iterations    = 750;
	tempModelSpec.spec.step_size            = 0.001;

	//modelSpecs.push_back( tempModelSpec ) ;
	/*
	int sepArray[] = {2,5,8};
	for(int j  = 0; j < 3 ; j++ )	// Tactile separation
	{
		//BuildModelSpec tempModelSpec = defaultModelSpec;

		tempModelSpec.name = "skin_array_s_1_sep_" + boost::lexical_cast<std::string>( sepArray[j] );
		tempModelSpec.spec.tactile_separation_x = sepArray[j];
		tempModelSpec.spec.tactile_separation_y = sepArray[j];

		// Test different offsets
		//tempModelSpec.spec.plunger_offset_x = j*tempModelSpec.spec.element_diameter*0.25;
		//tempModelSpec.spec.plunger_offset_y = j*tempModelSpec.spec.element_diameter*0.25;

		modelSpecs.push_back( tempModelSpec ) ;
	}
	*/

	// PID tuning
	double testValues[] = {0.05,0.1,0.5,1.0,1.5,2.0,2.5,3.0,3.5,4.0};
	for(int j  = 0; j < 10 ; j++ )
	{
		defaultControlSpec.name = "control_" + boost::lexical_cast<std::string>( j );
		defaultControlSpec.Kp = testValues[j];
		defaultControlSpec.Ki = 0.0;
		defaultControlSpec.Kd = 0.0;
		defaultControlSpec.Nf = 1.0;

		modelSpecs.push_back( tempModelSpec ) ;
		tempModelSpec.name = "skin_array_s_1_sep_2_" + boost::lexical_cast<std::string>( j );
		ctrSpecs.push_back( defaultControlSpec ) ;
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


	//ctrSpecs.push_back( defaultControlSpec ) ;

/*
	// Test different Ts values
	double Kdata = 0.0002;
	int numSensors[] = {8,4,3};
	double freq[] = {4,16,36};
	for(unsigned i  = 0; i < 3 ; i++ )
	{
		ControllerSpec tempControlSpec = defaultControlSpec;

		tempControlSpec.name = "control_" + boost::lexical_cast<std::string>( i );
//		tempControlSpec.Ts = Kdata*numSensors[i];
//		tempControlSpec.Ts = 1.0/freq[i];
		ctrSpecs.push_back( tempControlSpec ) ;
	}
*/
/*
	// PID tuning
	int num = 1;
	for(int i  = 0; i < 4; i++ )
	{
		ControllerSpec tempControlSpec = defaultControlSpec;

		tempControlSpec.name = "control_" + boost::lexical_cast<std::string>( num );
		//tempControlSpec.Ts = 0.0001 * pow(10, i);
		//tempControlSpec.controller_type = j;

		tempControlSpec.Kp = 0.01 * pow(10, i);
		ctrSpecs.push_back( tempControlSpec ) ;
		num++;
	}
*/

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


