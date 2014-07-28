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
 * genExpSpecs.cc
 *  Created on: Jul 28, 2014
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>

#include <SkinSim/ControlSpecYAML.hh>
#include <SkinSim/ModelSpecYAML.hh>

int main(int argc, char** argv)
{
  std::vector<ControllerSpec> ctrSpecs;
  std::vector<ModelSpec>      mdlSpecs;

  // Write YAML files
  std::string pathString( getenv ("SKINSIM_PATH") );
  std::string ctrSpecPath = pathString + "/skinsim_test/config/ctrSpecs.yaml";
  std::string mdlSpecPath = pathString + "/skinsim_test/config/mdlSpecs.yaml";

  std::ofstream ctrOut(ctrSpecPath.c_str());
  std::ofstream mdlOut(mdlSpecPath.c_str());

  YAML::Emitter ctrYAMLEmitter;
  YAML::Emitter mdlYAMLEmitter;

  int expNum = 10;

  ControllerSpec tempSpec;

  tempSpec.name         = "testSpec" ;
  tempSpec.explFctr_Kp  = 0 ;
  tempSpec.explFctr_Ki  = 0 ;
  tempSpec.explFctr_Kd  = 0 ;
  tempSpec.impCtr_Xnom  = 0 ;
  tempSpec.impCtr_M     = 0 ;
  tempSpec.impCtr_K     = 0 ;
  tempSpec.impCtr_D     = 0 ;
  tempSpec.ctrType      = 0 ;
  tempSpec.targetForce  = 0 ;

  ctrSpecs.push_back( tempSpec );

  // Save controller specs
  ctrYAMLEmitter << YAML::BeginSeq;
  for(unsigned i  =0; i < ctrSpecs.size() ;i++)
  {
     ctrYAMLEmitter << ctrSpecs;
  }
  ctrYAMLEmitter << YAML::EndSeq;

  // Save model specs
  mdlYAMLEmitter << YAML::BeginSeq;
  for(unsigned i  =0; i < ctrSpecs.size() ;i++)
  {
    mdlYAMLEmitter << mdlSpecs;
  }
  mdlYAMLEmitter << YAML::EndSeq;

  ctrOut << ctrYAMLEmitter.c_str();;
  ctrOut.close();

  mdlOut << mdlYAMLEmitter.c_str();;
  mdlOut.close();


  return 0;

}



