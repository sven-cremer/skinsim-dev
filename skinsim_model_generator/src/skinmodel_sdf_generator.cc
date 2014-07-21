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

#include <fstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <SkinSim/ModelBuilder.hh>

using namespace SkinSim;
using namespace std;

int main(int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "skinmodel_sensor_resol");
  ros::NodeHandle nh;

  std::string para_model_name   = "/model_name";
  std::string para_sdf_dir_name = "/sdf_dir_name";
  std::string para_sdf_filename = "/sdf_filename";
  std::string para_jcf_filename = "/joint_config_filename";
  std::string para_tid_filename = "/tactile_id_filename";

  std::string para_xByX         = "xByX";
  std::string para_density      = "density";
  std::string para_size_x       = "size_x" ;
  std::string para_size_y       = "size_y" ;

  std::string para_skin_height  = "skin_height";
  std::string para_plane_height = "plane_height";
  std::string para_d_pos        = "d_pos"  ;

  std::string para_sens_rad     = "sens_rad";
  std::string para_space_wid    = "space_wid" ;

  std::string model_name        = "spring_array";
  std::string sdf_dir_name      = "~/";
  std::string sdf_filename      = "model.sdf";

  double xByX   = 0.0;

  double density= 0.0;
  double size_x = 1.5;
  double size_y = 1.5;

  double skin_height = 1.3;
  double plane_height= 0.4;
  double d_pos  = 0.5;

  double sens_rad = 1.0;
  double space_wid = 3.0;

  if (!nh.getParam( para_model_name   , model_name   )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_model_name.c_str())  ; }
  if (!nh.getParam( para_sdf_dir_name , sdf_dir_name )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_sdf_dir_name.c_str()); }
  if (!nh.getParam( para_sdf_filename , sdf_filename )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_sdf_filename.c_str()); }

  if (!nh.getParam( para_xByX   , xByX    )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_xByX.c_str())   ; }
  if (!nh.getParam( para_d_pos  , d_pos   )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_d_pos  .c_str()); }
  if (!nh.getParam( para_density, density )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_density.c_str()); }

  // FIXME this assumes square patches
  if( xByX != 0.0 )
  {
    size_x = d_pos*xByX ;
    size_y = d_pos*xByX ;
  }else
  {
    if (!nh.getParam(para_size_x , size_x  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_size_x .c_str()); }
    if (!nh.getParam(para_size_y , size_y  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_size_y .c_str()); }
  }

  if (!nh.getParam(para_skin_height, skin_height )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_skin_height.c_str()); }
  if (!nh.getParam(para_plane_height, plane_height )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_plane_height .c_str()); }

  if (!nh.getParam(para_sens_rad , sens_rad )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_sens_rad.c_str()); }
  if (!nh.getParam(para_space_wid , space_wid  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_space_wid.c_str()); }

  // Create model files
  SkinSimModelBuilder skinSimModelBuilderObject( model_name   ,
                                                 sdf_filename ,
                                                 xByX         ,
                                                 density      ,
                                                 size_x       ,
                                                 size_y       ,
                                                 skin_height  ,
                                                 plane_height ,
                                                 d_pos        ,
                                                 sens_rad     ,
                                                 space_wid     );

  return 0;

}

