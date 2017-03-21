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

#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

//#include <boost/filesystem.hpp>
//#include <boost/lexical_cast.hpp>
//
//#include <SkinSim/ControlSpecYAML.hh>
//#include <SkinSim/ModelSpecYAML.hh>
//#include <SkinSim/model_builder.h>

#include <experimenter/src/skinsim_experiment_generator.cc>

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

	SkinSimExperimentGenerator generatorObj(exp_name);

	// ---------------------------------------------

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

	// ---------------------------------------------
	// Experiments with different models

	// Tactile layout
	std::vector<int> siz;
	std::vector<int> sep;
	siz += 1;
	sep += 2, 5, 8;
	generatorObj.setTactileLayout(siz, sep);
	generatorObj.duplicateControlSpecs();

	// Tactile layout + max plunger offset
	// TODO

	// Plunger Offset value
	std::vector<double> dx;
	std::vector<double> dy;
	for (int i  = 0; i < 13 ; i++ )
	{
		dx.push_back(i*0.0025);
		dy.push_back(i*0.0025);
	}
	generatorObj.setPlungerOffset(dx, dy);
	generatorObj.duplicateControlSpecs();

	// Model parameters
	std::vector<double> v;
	for(unsigned i  = 0; i < 3 ; i++ )
	{
		v.push_back( 0.0001*pow(10,i+1) );
	}
	generatorObj.setModelParameter(v, SkinSimExperimentGenerator::Mass);
	generatorObj.duplicateControlSpecs();

	// ---------------------------------------------
	// Experiments with different controllers

	// PID tuning
	std::vector<double> g;
	g += 0.05,0.1,0.5,1.0,1.5,2.0,2.5,3.0,3.5,4.0;
	//for(int i  = 0; i < 4; i++ )
	//	g.push_back( 0.01 * pow(10, i) );
	generatorObj.setPIDgains(g, SkinSimExperimentGenerator::P);
	generatorObj.duplicateModelSpecs();


	// Test different Ts values
	double Kdata = 0.0002;
	double numSensors[] = {8,4,3};
	double freq[]       = {4,16,36};

	std::vector<double> Ts;
	for(unsigned i  = 0; i < 3 ; i++ )
	{
		//Ts.push_back( 0.0001 * pow(10, i) ) ;
		//Ts.push_back( Kdata*numSensors[i] ) ;
		Ts.push_back( 1.0/freq[i] ) ;
	}
	generatorObj.setTimeStep(Ts);
	generatorObj.duplicateModelSpecs();

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

	// ---------------------------------------------
	// Generate and save files
	generatorObj.saveFiles();


	return 0;

}


