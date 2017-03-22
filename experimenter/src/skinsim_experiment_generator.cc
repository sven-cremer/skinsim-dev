/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, UT Arlington
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
 * skinsim_experiment_generator.cc
 *
 *  Created on: Mar 21, 2017
 *      Author: Sven Cremer
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// Boost
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

// SkinSim
#include <SkinSim/ControlSpecYAML.hh>
#include <SkinSim/ModelSpecYAML.hh>
#include <SkinSim/model_builder.h>


using namespace SkinSim;
//using namespace std;

class SkinSimExperimentGenerator
{
private:
	// Path and file names
	std::string exp_name;
	std::string pathSkinSim;
	std::string pathExp;
	std::string filename_model;
	std::string filename_control;
	std::string mdlSpecPath;
	std::string ctrSpecPath;
	std::string defaultModelSpecPath;

	BuildModelSpec defaultModelSpec;
	ControllerSpec defaultControlSpec;

	std::vector<BuildModelSpec> list_mdlSpecs;
	std::vector<ControllerSpec> list_ctrSpecs;



public:

	enum Gain { P, I, D };
	enum Model { Mass, Spring, Damper };

	SkinSimExperimentGenerator(std::string exp_name_)
	{
		setPaths(exp_name_);
		setDefaultModelValues();
		setDefaultControlValues();
	}

	~SkinSimExperimentGenerator()
	{

	}

	void setPaths(std::string exp_name_)
	{
		exp_name = exp_name_;

		pathSkinSim = getenv ("SKINSIM_PATH");
		pathExp     = pathSkinSim + "/data/" + exp_name;

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
				//return 1;
			}
		}

		filename_model   = "mdlSpecs.yaml";;
		filename_control = "ctrSpecs.yaml";

		mdlSpecPath = pathExp + "/" + filename_model;
		ctrSpecPath = pathExp + "/" + filename_control;

		defaultModelSpecPath = pathSkinSim + "/generator/config/model_params.yaml";
	}

	void resetSpecs()
	{
		list_mdlSpecs.clear();
		list_mdlSpecs.clear();
		setDefaultModelValues();
		setDefaultControlValues();
	}

	void setDefaultModelValues()
	{
		// Read default YAML model file
		std::ifstream fin(defaultModelSpecPath.c_str());
		YAML::Node doc_model;
		std::cout<<"Loading file: "<<defaultModelSpecPath<<"\n";
		doc_model = YAML::LoadAll(fin);
		doc_model[0][0] >> defaultModelSpec;

		// Set other values
		double contacts = 120;
		defaultModelSpec.name = "skin_array_s_" + boost::lexical_cast<std::string>( 1 ) + "_sep_" + boost::lexical_cast<std::string>( 2 );
		defaultModelSpec.spec.tactile_separation_x = 5;
		defaultModelSpec.spec.tactile_separation_y = 5;
		defaultModelSpec.spec.tactile_elements_x   = 1;
		defaultModelSpec.spec.tactile_elements_y   = 1;
		defaultModelSpec.spec.element_mass         = 0.01;
		defaultModelSpec.spec.element_damping      = 242.6/(contacts*6); //2.02;		Si: 248.6/contacts    Frubber: 242.6
		defaultModelSpec.spec.element_spring       = 1523 /(contacts*6); //12.69;		Si: 4338/contacts     Frubber: 1523
		defaultModelSpec.spec.element_diameter     = 0.0016644;
		defaultModelSpec.spec.element_height       = 0.0096644;
		defaultModelSpec.spec.spread_scaling       = 1.0; //0.085;
		defaultModelSpec.spec.spread_sigma         = 0.00102;
		defaultModelSpec.spec.plunger_radius       = 0.010;
		defaultModelSpec.spec.plunger_mass         = 1.0;
		defaultModelSpec.spec.plane_thickness      = 0.0005;
		defaultModelSpec.spec.plunger_length       = 0.03;

		defaultModelSpec.spec.noiseAmplitude       = 0.0;
		defaultModelSpec.spec.noiseSigma           = 0.0;

		defaultModelSpec.spec.max_sim_time         = 1.0;
		defaultModelSpec.spec.solver_iterations    = 750;
		defaultModelSpec.spec.step_size            = 0.001;

		std::cout<<"\n##### DEFAULT MODEL SPECS #####\n";
		print(defaultModelSpec);
	}

	void setDefaultControlValues()
	{
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

		defaultControlSpec.Ts                = 0.01;
		defaultControlSpec.Nf                = 10; //0.045;//10;

		std::cout<<"\n##### DEFAULT CONTROL SPECS #####\n";
		print(defaultControlSpec);
	}

	// Change element model parameter
	void setModelParameter(std::vector<double> values, SkinSimExperimentGenerator::Model m)
	{
		for(int i  = 0; i < values.size(); i++ )
		{
			BuildModelSpec tempModelSpec = defaultModelSpec;

			tempModelSpec.name = "skin_array_s_" + boost::lexical_cast<std::string>( tempModelSpec.spec.tactile_elements_x ) + "_sep_" + boost::lexical_cast<std::string>( tempModelSpec.spec.tactile_separation_x ) + "_" + boost::lexical_cast<std::string>( i );

			switch(m)
			{
			case SkinSimExperimentGenerator::Mass:
				tempModelSpec.spec.element_mass = values[i];
				break;
			case SkinSimExperimentGenerator::Spring:
				tempModelSpec.spec.element_spring = values[i];
				break;
			case SkinSimExperimentGenerator::Damper:
				tempModelSpec.spec.element_damping = values[i];
				break;
			}
			list_mdlSpecs.push_back( tempModelSpec ) ;
		}
	}

	// Tactile layout
	void setTactileLayout(std::vector<int> tactileSizes, std::vector<int> tactileSeparations)
	{
		int N = tactileSizes.size();
		int M = tactileSeparations.size();

		for(unsigned i  = 0; i < N ; i++ )		// Tactile size
		{
			for(unsigned j  = 0; j < M ; j++ )	// Tactile separation
			{
				BuildModelSpec tempModelSpec = defaultModelSpec;
				int size = tactileSizes[i];
				int sep  = tactileSeparations[j];

				tempModelSpec.name = "skin_array_s_" + boost::lexical_cast<std::string>( size ) + "_sep_" + boost::lexical_cast<std::string>( sep );
				tempModelSpec.spec.tactile_elements_x   = size;
				tempModelSpec.spec.tactile_elements_y   = size;
				tempModelSpec.spec.tactile_separation_x = sep;
				tempModelSpec.spec.tactile_separation_y = sep;

				list_mdlSpecs.push_back( tempModelSpec ) ;
			}
		}
	}

	// Plunger Offset value
	void setPlungerOffset(std::vector<double> dx, std::vector<double> dy)
	{
		if( dx.size() != dy.size() )
		{
			std::cerr<<"setPlungerOffset: dx and dy dimensions do not agree!\n";
			return;
		}

		for (int i  = 0; i < dx.size() ; i++ )
		{
			BuildModelSpec tempModelSpec = defaultModelSpec;

			tempModelSpec.name = "skin_array_s_" + boost::lexical_cast<std::string>( tempModelSpec.spec.tactile_elements_x ) + "_sep_" + boost::lexical_cast<std::string>( tempModelSpec.spec.tactile_separation_x ) + "_offset_" + boost::lexical_cast<std::string>( i );

			tempModelSpec.spec.plunger_offset_x = dx[i];
			tempModelSpec.spec.plunger_offset_y = dy[i];

			list_mdlSpecs.push_back( tempModelSpec ) ;
		}
	}

	// Noise
	void addNoise(std::vector<double> amplitude, std::vector<double> sigma)
	{

	}

	// Test different Ts values
	void setTimeStep(std::vector<double> Ts)
	{
		for(unsigned i  = 0; i < Ts.size() ; i++ )
		{
			ControllerSpec tempControlSpec = defaultControlSpec;

			tempControlSpec.name = "control_" + boost::lexical_cast<std::string>( i );
			tempControlSpec.Ts = Ts[i];
			list_ctrSpecs.push_back( tempControlSpec ) ;
		}
	}

	// PID tuning
	void setPIDgains(std::vector<double> values, SkinSimExperimentGenerator::Gain g)
	{
		for(int i  = 0; i < values.size(); i++ )
		{
			ControllerSpec tempControlSpec = defaultControlSpec;

			tempControlSpec.name = "control_" + boost::lexical_cast<std::string>( i );

			switch(g)
			{
			case SkinSimExperimentGenerator::P:
				tempControlSpec.Kp = values[i];
				break;
			case SkinSimExperimentGenerator::I:
				tempControlSpec.Ki = values[i];
				break;
			case SkinSimExperimentGenerator::D:
				tempControlSpec.Kd = values[i];
				break;
			}
			list_ctrSpecs.push_back( tempControlSpec ) ;
		}
	}

	void duplicateModelSpecs()
	{
		duplicateModelSpecs(list_ctrSpecs.size());
	}

	void duplicateControlSpecs()
	{
		duplicateControlSpecs(list_mdlSpecs.size());
	}

	void duplicateModelSpecs(int N)
	{
		for(int i  = 0; i < N; i++ )
		{
			list_mdlSpecs.push_back( defaultModelSpec ) ;
		}
	}

	void duplicateControlSpecs(int N)
	{
		for(int i  = 0; i < N; i++ )
		{
			list_ctrSpecs.push_back( defaultControlSpec ) ;
		}
	}

	void saveFiles()
	{
		if( list_mdlSpecs.size() != list_ctrSpecs.size() )
		{
			std::cerr<<"saveFiles: number of model and control specs do not agree!\n";
			return;
		}

		// ---------------------------------------------
		// Generate and save SDF models
		for(unsigned i = 0; i < list_mdlSpecs.size() ;i++)
		{
			ModelBuilder skinSimModelBuilderObject( list_mdlSpecs[i] );	// TODO store in exp directory and update Gazebo model path
		}

		// ---------------------------------------------
		// Generate model specifications

		// Save to YAML
		YAML::Emitter mdlYAMLEmitter;
		mdlYAMLEmitter << list_mdlSpecs;

		// Write to YAML file
		std::ofstream mdlOut(mdlSpecPath.c_str());
		std::cout<<"Saving model specs to file: "<<mdlSpecPath<<"\n";
		mdlOut << mdlYAMLEmitter.c_str();;
		mdlOut.close();

		// ---------------------------------------------
		// Generate control specifications

		// Save to YAML
		YAML::Emitter ctrYAMLEmitter;
		ctrYAMLEmitter << list_ctrSpecs;

		// Write to YAML file
		std::ofstream ctrOut(ctrSpecPath.c_str());
		std::cout<<"Saving control specs to file: "<<ctrSpecPath<<"\n";
		ctrOut << ctrYAMLEmitter.c_str();;
		ctrOut.close();

		// ---------------------------------------------
		// Print result
		std::cout<<"\nNumber of models generated:      "<<list_mdlSpecs.size();
		std::cout<<"\nNumber of controllers generated: "<<list_ctrSpecs.size()<<"\n";
	}

};
