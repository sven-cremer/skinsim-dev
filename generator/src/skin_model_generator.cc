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
           Ahsan Habib
 *
 * skinmodel_sdf_generator.cpp
 *  Created on: Jan 16, 2014
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <SkinSim/model_builder.h>
//#include <SkinSim/ModelBuilder.hh>
#include <SkinSim/ModelSpecYAML.hh>

using namespace SkinSim;
using namespace std;

int main(int argc, char** argv)
{
  BuildModelSpec modelSpecs;

  double xByX         			= 0.0 ;

  double thick_board 	        = 0.2 ;
  double density      	        = 0.0 ;
  double size_x                 = 1.5 ;
  double size_y                 = 1.5 ;

  double skin_height            = 1.3 ;
  double plane_height           = 0.4 ;
  double skin_element_diameter                  = 0.5 ;

  double tactile_length         = 1.0 ;
  double tactile_separation    	= 3.0 ;

  double tactile_height         = 0.05;		//Added Tac_Height

  // Read YAML files
  std::string pathString( getenv ("SKINSIM_PATH") );
  std::string configFilePath = pathString + "/generator/config/model_params.yaml";
  std::ifstream fin(configFilePath.c_str());

  std::cout<<"Loading: "<<configFilePath<<"\n";
  YAML::Node doc;
  doc = YAML::LoadAll(fin);
  //std::cout<<"File Loaded"<<"\n";
  for(std::size_t i=0;i<doc[0].size();i++)
  {
	  //std::cout<<"Spec Loaded: \n"<< doc[0][i] <<"\n";
	  doc[0][i] >> modelSpecs;						// FIXME overwrites previous data

	  std::cout<<"Spec Stored: \n";
	  print(modelSpecs);
  }

  // Create model files
  ModelBuilder skinSimModelBuilderObject( modelSpecs );

  fin.close();

  return 0;

}

