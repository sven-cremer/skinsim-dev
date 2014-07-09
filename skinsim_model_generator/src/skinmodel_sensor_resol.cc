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

int main(int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "skinmodel_sensor_resol");
  ros::NodeHandle nh;

  SkinSimModelBuilder test;

  std::string para_sdf_filename = "/sdf_filename";
  std::string para_jcf_filename = "/joint_config_filename";
  std::string para_tid_filename = "/tactile_id_filename";

  std::string sdf_filename          ; // = "model.sdf";
  std::string joint_config_filename ; // = "joint_names.yaml";
  std::string tactile_id_filename;    // = "tactile_id.yaml";

  Eigen::Vector4d skin_ambient ;
  Eigen::Vector4d skin_diffuse ;
  Eigen::Vector4d skin_specular;
  Eigen::Vector4d skin_emissive;

  Eigen::Vector4d base_ambient ;
  Eigen::Vector4d base_diffuse ;
  Eigen::Vector4d base_specular;
  Eigen::Vector4d base_emissive;

  ////                 R    G    B
  //skin_ambient  << 1.0, 1.0, 1.0, 1.0 ;
  //skin_diffuse  << 1.0, 1.0, 1.0, 1.0 ;
  skin_specular << 0.1, 0.1, 0.1, 1.0 ;
  skin_emissive = Eigen::Vector4d::Zero();

  base_ambient  << 1.0, 1.0, 1.0, 1.0 ;
  base_diffuse  << 1.0, 1.0, 1.0, 1.0 ;
  base_specular << 0.1, 0.1, 0.1, 1.0 ;
  base_emissive = Eigen::Vector4d::Zero();

  if (!nh.getParam(para_sdf_filename , sdf_filename           )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_sdf_filename.c_str()); }
  if (!nh.getParam(para_jcf_filename , joint_config_filename  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_jcf_filename.c_str()); }
  if (!nh.getParam(para_tid_filename , tactile_id_filename  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_tid_filename.c_str()); }

  std::string para_density      = "density";
  std::string para_size_x       = "size_x" ;
  std::string para_size_y       = "size_y" ;

  std::string para_skin_height  = "skin_height";
  std::string para_plane_height = "plane_height";

  std::string para_d_pos        = "d_pos"  ;

  std::string para_sens_rad       = "sens_rad";
  std::string para_space_wid      = "space_wid" ;

  double density     ;
  double size_x = 1.5;
  double size_y = 1.5;

  double skin_height = 1.3;

  double plane_height= 0.4;

  double d_pos  = 0.5;

  double sens_rad = 1.0;
  double space_wid = 3.0;


  if (!nh.getParam(para_density, density )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_density.c_str()); }
  if (!nh.getParam(para_size_x , size_x  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_size_x .c_str()); }
  if (!nh.getParam(para_size_y , size_y  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_size_y .c_str()); }

  if (!nh.getParam(para_skin_height, skin_height )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_skin_height.c_str()); }

  if (!nh.getParam(para_plane_height, plane_height )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_plane_height .c_str()); }

  if (!nh.getParam(para_d_pos  , d_pos   )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_d_pos  .c_str()); }

  if (!nh.getParam(para_density, density )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_density.c_str()); }
  if (!nh.getParam(para_size_x , size_x  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_size_x .c_str()); }
  if (!nh.getParam(para_size_y , size_y  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_size_y .c_str()); }
  if (!nh.getParam(para_size_y , size_y  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_size_y .c_str()); }

  if (!nh.getParam(para_sens_rad , sens_rad )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_sens_rad.c_str()); }
  if (!nh.getParam(para_space_wid , space_wid  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_space_wid.c_str()); }

  YAML::Emitter out;
  std::ofstream fout(joint_config_filename.c_str());

  YAML::Emitter out1;
  std::ofstream fout2(tactile_id_filename.c_str());

//  sdf::SDFPtr robot(new sdf::SDF());
//  sdf::init(robot);

  Eigen::VectorXd pose;
  Eigen::Vector3d box_size;
  Eigen::Vector3d axis;
  double radius;

  pose.resize(6,1);
  pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  test.generateModelStart("spring_board", pose );

  pose << 0, 0, plane_height, 0, 0, 0;
  box_size << 1.5*size_x, 1.5*size_y, d_pos/10;

  test.addLink( "plane",
                20,
                "collision",
                "visual",
                box_size,
                pose,
                base_ambient ,
                base_diffuse ,
                base_specular,
                base_emissive );

  axis << 0, 0, 1;

  test.addPlaneJoint( "plane_joint",
                 "prismatic",
                 "world",
                 "plane",
                 axis );



  double skin_no = (double)(size_x/d_pos)*(size_y/d_pos);
  int x_skin_len = (int)(size_x/d_pos);
  int y_skin_len = (int)(size_y/d_pos);
  int skin_ix [y_skin_len][x_skin_len];
  int ix = 1;
  for ( int i = 0; i<y_skin_len; i++)
  {
    for( int j = 0; j<x_skin_len; j++)
    {
      skin_ix[i][j] = ix;
      ix++;
    }
  }
  int cent_x = (int)x_skin_len/2;
  int cent_y = (int)y_skin_len/2;
  std::vector <int> search_pts_x;  
  std::vector <int> search_pts_y;
  std::vector <int> sens_cent_ix;

  std::string path_cent = ros::package::getPath("skinsim_model") + "/config/tactile_cent_id.txt";
  std::ofstream tact_cent_rec;
  tact_cent_rec.open(path_cent.c_str());

  // ---------down ------------
  for(int iy=cent_y; iy<=(y_skin_len-1); iy=iy+space_wid)
  {
    if((iy+sens_rad)<=(y_skin_len-1))
    {
      // --- left ----
      for(int ix=cent_x;ix>=0;ix=ix-space_wid)
      {
        if((ix-sens_rad)>=0)
        {
          search_pts_x.push_back(ix);
          search_pts_y.push_back(iy);
          sens_cent_ix.push_back(skin_ix[iy][ix]);
          tact_cent_rec << skin_ix[iy][ix]<<' ';
        }
      }
      // --- right ----
      for(int ix=(cent_x+space_wid);ix<=(x_skin_len-1);ix=ix+space_wid)
      {
        if((ix+sens_rad)<=(x_skin_len-1))
        {
          search_pts_x.push_back(ix);
          search_pts_y.push_back(iy);
          sens_cent_ix.push_back(skin_ix[iy][ix]);
          tact_cent_rec << skin_ix[iy][ix]<<' ';
        }
      }
    }
  }



  // ---------upper ------------
  for(int iy=(cent_y-space_wid); iy>=0; iy=iy-space_wid)
  {
    if((iy-sens_rad)>=0)
    {
      // --- left ----
      for(int ix=cent_x;ix>=0;ix=ix-space_wid)
      {
        if((ix-sens_rad)>=0)
        {
          search_pts_x.push_back(ix);
          search_pts_y.push_back(iy);
          sens_cent_ix.push_back(skin_ix[iy][ix]);
          tact_cent_rec << skin_ix[iy][ix]<<' ';
        }
      }
      // --- right ----
      for(int ix=(cent_x+space_wid);ix<=(x_skin_len-1);ix=ix+space_wid)
      {
        if((ix+sens_rad)<=(x_skin_len-1))
        {
          search_pts_x.push_back(ix);
          search_pts_y.push_back(iy);
          sens_cent_ix.push_back(skin_ix[iy][ix]);
          tact_cent_rec << skin_ix[iy][ix]<<' ';
        }
      }
    }
  }

  tact_cent_rec.close();
  std::vector <int> sens_cent_disp;
  sens_cent_disp = sens_cent_ix;
  std::sort(sens_cent_disp.begin(), sens_cent_disp.end(), std::greater<int>());

  std::vector <int> tact_sens_ix;
  std::string path = ros::package::getPath("skinsim_model") + "/config/tactile_id.txt";
  std::ofstream tact_rec;
  tact_rec.open(path.c_str());

  for(int pt_ix=0; pt_ix<=(sens_cent_ix.size()-1); pt_ix++)
  {
    int tact_cent_x = search_pts_x[pt_ix];
    int tact_cent_y = search_pts_y[pt_ix];
    
    for(int i = 0; i<=(y_skin_len-1);i++)
    {
      for(int j = 0; j<=(x_skin_len-1);j++)
      {
        if((abs(i - tact_cent_y)<=sens_rad)&&(abs(j - tact_cent_x)<=sens_rad))
        {
          tact_sens_ix.push_back(skin_ix[i][j]);
          tact_rec << skin_ix[i][j]<<' ';
        } 
      }
    }
    if(pt_ix!=sens_cent_ix.size())
    {
      tact_rec<<'\n';
    }
  }
  tact_rec.close();
  std::sort(tact_sens_ix.begin(), tact_sens_ix.end(), std::greater<int>());


  double pos_x = -(size_x-d_pos)/2;
  double pos_y = (size_y-d_pos)/2;

  out << YAML::BeginSeq;
  //out1<< YAML::BeginSeq;
  int x_ix = 1, y_ix = 1;
  

  for( int i = 1; i <= skin_no; i++ )
  {

    std::ostringstream convert;
    convert << i;
    
    if( x_ix > x_skin_len )
    {
      x_ix = 1;
      y_ix++;
    }

    if(skin_ix[y_ix-1][x_ix-1]==tact_sens_ix.back())
    //if(skin_ix[y_ix-1][x_ix-1]==sens_cent_disp.back())
    {
       skin_ambient  << 1.0, 0.0, 0.0, 1.0 ; 
       skin_diffuse  << 1.0, 0.0, 0.0, 1.0 ;
       tact_sens_ix.pop_back();
       //sens_cent_disp.pop_back();     
    }
    else
    {
       skin_ambient  << 1.0, 1.0, 1.0, 1.0 ;
       skin_diffuse  << 1.0, 1.0, 1.0, 1.0 ;

    } 

    pose << pos_x, pos_y, skin_height, 0, 0, 0;
    radius = d_pos/2;

    test.addLink( "spring_" + convert.str(),
	          0.00235,
	          "sphere_collision",
	          "visual",
	          radius,
	          pose,
	          skin_ambient ,
	          skin_diffuse ,
	          skin_specular,
	          skin_emissive );


    axis << 0, 0, 1;

    test.addJoint( "spring_joint_" + convert.str(),
                   "prismatic",
                   "plane",
                   "spring_" + convert.str(),
                   axis );

//    std::cout << pos_x << " " << pos_y << "\n";

    pos_x = pos_x + d_pos;
    
    if( pos_x > size_x/2)
    {
      pos_x = -(size_x-d_pos)/2;
      pos_y = pos_y - d_pos;
      //std::cout << " y_id \n";
    }
    x_ix++; 
   out << YAML::BeginMap;
   out << YAML::Key << "Joint" << YAML::Value << "joint_" + convert.str();
   out << YAML::EndMap;
  

  }

  out << YAML::EndSeq;

  // Write YAML file and close
  fout << out.c_str();
  fout.close();
  fout2.close();

  ////////////////////

  test.addPlugin( "skin_tactile_joint" , "libskin_tactile_joint.so" );
  //test.addPlugin( "plane_joint"  , "libplane_joint.so" );

  test.saveSDFFile( sdf_filename );

  return 0;

}

