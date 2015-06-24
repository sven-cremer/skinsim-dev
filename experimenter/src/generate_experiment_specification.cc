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
#include <boost/lexical_cast.hpp>

#include <SkinSim/ControlSpecYAML.hh>
#include <SkinSim/ModelSpecYAML.hh>

int main(int argc, char** argv)
{
  std::vector<BuildModelSpec>  mdlSpecs;

  // Write YAML files
  std::string pathString( getenv ("SKINSIM_PATH") );
  std::string mdlSpecPath = pathString + "/experimenter/config/mdlSpecs.yaml";

  std::ofstream mdlOut(mdlSpecPath.c_str());

  YAML::Emitter mdlYAMLEmitter;

  BuildModelSpec tempMdlSpecs;

  for(unsigned i  = 5; i < 100 ; i += 5 )
  {
    tempMdlSpecs.name              = "spring_array" + boost::lexical_cast<std::string>( i );
    tempMdlSpecs.spec.xByX         = i   ;
    tempMdlSpecs.spec.density      = 1.0 ;
    tempMdlSpecs.spec.size_x       = 0.0525 ;
    tempMdlSpecs.spec.size_y       = 0.0525 ;
    tempMdlSpecs.spec.skin_height  = 0.04 ;
    tempMdlSpecs.spec.tac_height   = 0.03;
    tempMdlSpecs.spec.plane_height = 0.4 ;
    tempMdlSpecs.spec.d_pos        = 0.0025 ;
    tempMdlSpecs.spec.sens_rad     = 1.0 ;
    tempMdlSpecs.spec.space_wid    = 4.0 ;
    mdlSpecs.push_back( tempMdlSpecs ) ;
  }

  // Save model specs
  mdlYAMLEmitter << YAML::BeginSeq;
  for(unsigned i  =0; i < mdlSpecs.size() ;i++)
  {
    mdlYAMLEmitter << mdlSpecs[i];
  }
  mdlYAMLEmitter << YAML::EndSeq;

  std::cout<<"Saving model specs to file: "<<mdlSpecPath<<"\n";
  mdlOut << mdlYAMLEmitter.c_str();;
  mdlOut.close();

  // ---------------------------------------------

  std::vector<ControllerSpec> ctrSpecs;

  // Write YAML files
  std::string ctrSpecPath = pathString + "/experimenter/config/ctrSpecs.yaml";

  std::ofstream ctrOut(ctrSpecPath.c_str());

  YAML::Emitter ctrYAMLEmitter;

  ControllerSpec tempSpec;

  tempSpec.name         = "efc_00_00_00" ;
  tempSpec.explFctr_Kp  = 2       ;
  tempSpec.explFctr_Ki  = 0.00005 ;
  tempSpec.explFctr_Kd  = 0.5     ;
  tempSpec.impCtr_Xnom  = 0.5     ;
  tempSpec.impCtr_M     = 5       ;
  tempSpec.impCtr_K     = 24      ;
  tempSpec.impCtr_D     = 10      ;
  tempSpec.ctrType      = 1       ;
  tempSpec.targetForce  = 0.01    ;

  ctrSpecs.push_back( tempSpec );

  // Save controller specs
  ctrYAMLEmitter << YAML::BeginSeq;
  for(unsigned i  =0; i < ctrSpecs.size() ;i++)
  {
     ctrYAMLEmitter << ctrSpecs;
  }
  std::cout<<"Saving ctr specs to file: "<<ctrSpecPath<<"\n";
  ctrYAMLEmitter << YAML::EndSeq;

  ctrOut << ctrYAMLEmitter.c_str();;
  ctrOut.close();



  return 0;

}



