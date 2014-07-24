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
 * ModelBuilder.hh
 *  Created on: Jul 9, 2014
 */

#ifndef MODELBUILDER_HH_
#define MODELBUILDER_HH_

#include "sdf/sdf.hh"
#include <iostream>
#include <fstream>
#include <stdlib.h>

#include <boost/filesystem.hpp>

namespace SkinSim
{

class SkinSimModelBuilder
{
private:
  std::ostringstream m_sdfStream;
  sdf::SDF m_sdfParsed;

  void generateSDFHeader()
  {
    m_sdfStream << "<?xml version='1.0' ?>\n"
                << "<sdf version='1.4'>\n";;
  }

  void generateModelEnd()
  {
    m_sdfStream << "\n  </model>\n"
                << "</sdf>";
  }

  typedef enum
  {
    BOX = 0,
    SPHERE
  } GEOMETRY_TYPE;

public:
  SkinSimModelBuilder( )
  {
    generateSDFHeader();
  }

  SkinSimModelBuilder(  std::string model_name            ,
                          std::string sdf_filename          ,
                          double xByX                       ,
                          double density                    ,
                          double size_x                     ,
                          double size_y                     ,
                          double skin_height                ,
                          double plane_height               ,
                          double d_pos                      ,
                          double sens_rad                   ,
                          double space_wid                   )
  {
    generateSDFHeader();
    createModelFiles( model_name            ,
                      sdf_filename          ,
                      xByX                  ,
                      density               ,
                      size_x                ,
                      size_y                ,
                      skin_height           ,
                      plane_height          ,
                      d_pos                 ,
                      sens_rad              ,
                      space_wid              );
  }

  ~SkinSimModelBuilder()
  {

  }

  void generateModelStart( std::string name, Eigen::VectorXd & pose )
  {
    m_sdfStream << "  <model name='" + name + "'>\n"
                << "    <pose>"<< pose.transpose() << "</pose>\n";
  }

  void addGeometry( double radius )
  {
    m_sdfStream << "      <geometry>\n"
                << "        <sphere>\n"
                << "          <radius>" << radius << "</radius>\n"
                << "        </sphere>\n"
                << "      </geometry>\n";
  }

  void addGeometry( Eigen::Vector3d & box_size )
  {
    m_sdfStream << "      <geometry>\n"
                << "        <box>\n"
                << "          <size>" << box_size.transpose() << "</size>\n"
                << "        </box>\n"
                << "      </geometry>\n";
  }

  void addSurface()
  {
    m_sdfStream << "        <surface>\n"
                << "          <friction>\n"
                << "            <ode>\n"
                << "              <mu>0.0</mu>\n"
                << "              <mu2>0.0</mu2>\n"
                << "            </ode>\n"
                << "          </friction>\n"
                << "        </surface>\n";
  }

  void addMaterial( Eigen::Vector4d & ambient ,
                     Eigen::Vector4d & diffuse ,
                     Eigen::Vector4d & specular,
                     Eigen::Vector4d & emissive  )
  {
    m_sdfStream << "      <material>\n"
                << "        <ambient>"  << ambient .transpose() << "</ambient>\n"
                << "        <diffuse>"  << diffuse .transpose() << "</diffuse>\n"
                << "        <specular>" << specular.transpose() << "</specular>\n"
                << "        <emissive>" << emissive.transpose() << "</emissive>\n"
                << "      </material>\n";
  }

  void addCollision( std::string collision_name,  double radius )
  {
    m_sdfStream << "    <collision name='" + collision_name + "'>\n";
                addGeometry( radius );
                addSurface();
    m_sdfStream << "    </collision>\n";
  }

  void addCollision( std::string collision_name, Eigen::Vector3d & box_size )
  {
    m_sdfStream << "    <collision name='" + collision_name + "'>\n";
                addGeometry( box_size );
                addSurface();
    m_sdfStream << "    </collision>\n";
  }

  void addVisual( std::string visual_name,
                   double radius,
                   Eigen::Vector4d & ambient ,
                   Eigen::Vector4d & diffuse ,
                   Eigen::Vector4d & specular,
                   Eigen::Vector4d & emissive  )
  {
    m_sdfStream << "    <visual name='" + visual_name + "'>\n";
                addGeometry( radius );
                addMaterial( ambient ,
                             diffuse ,
                             specular,
                             emissive );
    m_sdfStream << "    </visual>\n";
  }

  void addVisual( std::string visual_name,
                   Eigen::Vector3d & box_size,
                   Eigen::Vector4d & ambient ,
                   Eigen::Vector4d & diffuse ,
                   Eigen::Vector4d & specular,
                   Eigen::Vector4d & emissive  )
  {
    m_sdfStream << "    <visual name='" + visual_name + "'>\n";
                addGeometry( box_size );
                addMaterial( ambient ,
                             diffuse ,
                             specular,
                             emissive );
    m_sdfStream << "    </visual>\n";
  }

  void addInertia( double mass )
  {
    m_sdfStream << "    <inertial>\n"
                << "      <mass>"<< mass <<"</mass>\n"
                << "    </inertial>\n";
  }

  void addLink( std::string link_name,
                 double mass,
                 std::string collision_name,
                 std::string visual_name,
                 double radius,
                 Eigen::VectorXd & pose,
                 Eigen::Vector4d & ambient ,
                 Eigen::Vector4d & diffuse ,
                 Eigen::Vector4d & specular,
                 Eigen::Vector4d & emissive  )
  {
    m_sdfStream << "  <link name='" + link_name + "'>\n"
                << "    <pose>"<< pose.transpose() << "</pose>\n";

                addInertia( mass );
                addCollision( collision_name, radius );
                addVisual( visual_name,
                           radius,
                           ambient ,
                           diffuse ,
                           specular,
                           emissive );

    m_sdfStream << "   </link>\n";
  }

  void addLink( std::string link_name,
                 double mass,
                 std::string collision_name,
                 std::string visual_name,
                 Eigen::Vector3d & box_size,
                 Eigen::VectorXd & pose,
                 Eigen::Vector4d & ambient ,
                 Eigen::Vector4d & diffuse ,
                 Eigen::Vector4d & specular,
                 Eigen::Vector4d & emissive  )
  {
    m_sdfStream << "  <link name='" + link_name + "'>\n"
                << "    <pose>"<< pose.transpose() << "</pose>\n";

                addInertia( mass );
                addCollision( collision_name, box_size );
                addVisual( visual_name,
                           box_size,
                           ambient ,
                           diffuse ,
                           specular,
                           emissive );

    m_sdfStream << "   </link>\n";
  }

  void addJoint( std::string joint_name,
                  std::string joint_type,
                  std::string parent,
                  std::string child,
                  Eigen::Vector3d & axis )
  {
    m_sdfStream << "  <joint name = '" + joint_name + "' type='" + joint_type + "'>\n"
                << "    <parent>" + parent + "</parent>\n"
                << "      <child>" + child + "</child>\n"
                << "      <axis>\n"
                << "        <xyz>" << axis.transpose() << "</xyz>\n"
                << "        <limit>\n"
                << "          <lower>" << -1e+16 << "</lower>\n"
                << "          <upper>" <<  1e+16 << "</upper>\n"
                << "        </limit>\n"
                << "      </axis>\n"
                << "  </joint>\n";
  }

  void addPlaneJoint( std::string joint_name,
                        std::string joint_type,
                        std::string parent,
                        std::string child,
                        Eigen::Vector3d & axis,
                        double upper_limit,
                        double lower_limit )
  {
    m_sdfStream << "  <joint name = '" + joint_name + "' type='" + joint_type + "'>\n"
                << "    <parent>" + parent + "</parent>\n"
                << "      <child>" + child + "</child>\n"
                << "      <axis>\n"
                << "        <xyz>" << axis.transpose() << "</xyz>\n"
                << "        <limit>\n"
                << "          <lower>" << lower_limit << "</lower>\n"
                << "          <upper>" << upper_limit << "</upper>\n"
                << "        </limit>\n"
                << "      </axis>\n"
                << "  </joint>\n";
  }

  void addPlugin( std::string plugin_name, std::string plugin_filename, std::string & sdf_filename, std::string & model_name )
  {
    m_sdfStream << "\n  <plugin name='" + plugin_name + "' filename='" + plugin_filename + "' />\n"
                << "    <file_name>" << genModelDirectory( sdf_filename, model_name ) << "</file_name>";
  }

  std::string genModelDirectory( std::string & sdf_filename, std::string & model_name )
  {
    boost::filesystem::path dir_path ( sdf_filename );

    try
    {
      if (boost::filesystem::exists(dir_path))    // does p actually exist?
      {
        if (boost::filesystem::is_regular_file(dir_path))        // is p a regular file?
        {
          dir_path = dir_path.branch_path();
        }

        if (boost::filesystem::is_directory(dir_path))      // is p a directory?
        {
          boost::filesystem::path dir(dir_path / model_name);
          if(boost::filesystem::create_directory(dir)) {
            //ROS_WARN_STREAM( "Success" );
          }
        }
      }
    }
    catch (const boost::filesystem::filesystem_error& ex)
    {
      //ROS_ERROR_STREAM( ex.what() );
    }

    std::string filepath = dir_path.string() + "/" + model_name + "/";

    return filepath;
  }

  void saveSDFFile( std::string & sdf_filename, std::string & model_name )
  {
    generateModelEnd();

    std::string filename = genModelDirectory( sdf_filename, model_name ) + model_name + ".sdf";

    m_sdfParsed.SetFromString( m_sdfStream.str() );
    m_sdfParsed.Write( filename );
  }

  void saveConfigFile( std::string & sdf_filename, std::string & model_name )
  {
    std::ostringstream modelConfig;

    modelConfig << "<?xml version='1.0'?>                     \n"
                << "                                          \n"
                << "<model>                                   \n"
                << "  <name>" << model_name << "</name>       \n"
                << "  <version>1.0</version>                  \n"
                << "  <sdf >" << model_name << ".sdf</sdf>    \n"
                << "                                          \n"
                << "  <author>                                \n"
                << "    <name>Isura Ranatunga</name>          \n"
                << "    <email>isura@ieee.org</email>         \n"
                << "  </author>                               \n"
                << "                                          \n"
                << "  <description>                           \n"
                << "    A Simple "<< model_name << "          \n"
                << "  </description>                          \n"
                << "</model>                                  \n";

    std::string filename = genModelDirectory( sdf_filename, model_name ) + "model.config";
    saveFile( filename, modelConfig );
  }

  void saveFile( std::string & filename, std::ostringstream & model )
  {
    std::ofstream saveToFile;
    saveToFile.open ( filename.c_str() );
    saveToFile << model.str();
    saveToFile.close();
  }

  void createModelFiles( std::string model_name            ,
                           std::string sdf_filename          ,
                           double xByX                       ,
                           double density                    ,
                           double size_x                     ,
                           double size_y                     ,
                           double skin_height                ,
                           double plane_height               ,
                           double d_pos                      ,
                           double sens_rad                   ,
                           double space_wid                   )
  {
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

    std::string modelDirectory = genModelDirectory( sdf_filename, model_name );

    std::string joint_config_filename = modelDirectory + "joint_names.yaml";
    std::string tactile_id_filename   = modelDirectory + "tactile_id.yaml";

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

    generateModelStart( model_name, pose );

    pose << 0, 0, plane_height, 0, 0, 0;
    box_size << 1.5*size_x, 1.5*size_y, d_pos/10;

    addLink( "plane",
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

    //robot is moving
    addPlaneJoint( "plane_joint",
                   "prismatic",
                   "world",
                   "plane",
                   axis,
                   -1e+16,
                   1e+16 );

    //environment is moving
    /*test.addPlaneJoint( "plane_joint",
                   "prismatic",
                   "world",
                   "plane",
                   axis,
                   0,
                   0 );*/

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

    std::string pathString( getenv ("SKINSIM_PATH") );

    std::string path_cent = pathString + "skinsim_model/config/tactile_cent_id.txt";
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

    std::string path = pathString + "skinsim_model/config/tactile_id.txt";
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

      addLink( "spring_" + convert.str(),
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

      addJoint( "spring_joint_" + convert.str(),
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

    addPlugin( "skin_tactile_joint" , "libTactileSensorPlugin.so", sdf_filename, model_name );
    addPlugin( "skin_tactile_joint" , "libSkinJointPlugin.so", sdf_filename, model_name );

    addPlugin( "plane_joint"  , "libplane_joint.so" );

    saveSDFFile( sdf_filename, model_name );
    saveConfigFile( sdf_filename, model_name );

  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}

#endif /* MODELBUILDER_HH_ */
