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
 *         Ahsan Habib
 *         Sven Cremer
 *
 *  skin_model_generator.cc
 *  Created on: Jan 16, 2014
 */

#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <SkinSim/model_builder.h>
#include <SkinSim/ModelSpecYAML.hh>

using namespace SkinSim;
using namespace std;

int main(int argc, char** argv)
{

  std::string filename_model = "model_params.yaml";

  // Check the number of command-line parameters
  if (argc == 2)
  {
	  // Set model file name
	  filename_model = argv[1];
  }
  else
  {
	  // Use default file name
	  std::cout<<"Usage: "<<argv[0]<<" [MODEL FILENAME]\n";
  }

  // Read YAML files
  std::string pathString( getenv ("SKINSIM_PATH") );
  std::string configFilePath = pathString + "/generator/config/" + filename_model;
  std::ifstream fin(configFilePath.c_str());

  std::cout<<"Loading: "<<configFilePath<<"\n";
  YAML::Node doc;
  doc = YAML::LoadAll(fin);
  //std::cout<<"File Loaded"<<"\n";
  for(std::size_t i=0;i<doc[0].size();i++)
  {
	  // Load model specifications
	  BuildModelSpec modelSpecs;

	  //std::cout<<"YAML data: \n"<< doc[0][i] <<"\n---\n";
	  doc[0][i] >> modelSpecs;

	  // Print result
	  std::cout<<"### Specs Loaded ###\n";
	  print(modelSpecs);

	  // Create model file
	  ModelBuilder skinSimModelBuilderObject( modelSpecs );
  }

  fin.close();

  return 0;

}

