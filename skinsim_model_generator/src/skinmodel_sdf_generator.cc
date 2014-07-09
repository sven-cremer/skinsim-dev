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
 * skinmodel_sdf_generator.cpp
 *  Created on: Jan 16, 2014
 */

#include <fstream>
#include <string>

#include <ros/ros.h>
#include "sdf/sdf.hh"

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

class SkinSimModelBuilder
{
private:
  std::ostringstream sdf_stream_;
  sdf::SDF sdfParsed_;

  void generateSDFHeader()
  {
    sdf_stream_ << "<?xml version='1.0' ?>"
                << "<sdf version='1.4'>"
                << " ";
  }

  void generateModelEnd()
  {
    sdf_stream_ << "    </model>"
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
  ~SkinSimModelBuilder()
  {

  }

  void generateModelStart( std::string name, Eigen::VectorXd & pose )
  {
    sdf_stream_ << "<model name='" + name + "'>"
                << "<pose>"<< pose << "</pose>"
                << " ";
  }

  void addGeometry( double radius )
  {
    sdf_stream_ << "      <geometry>"
                << "        <sphere>"
                << "          <radius>" << radius << "</radius>"
                << "        </sphere>"
                << "      </geometry>";
  }

  void addGeometry( Eigen::Vector3d & box_size )
  {
    sdf_stream_ << "    <geometry>"
                << "       <box>"
                << "         <size>" << box_size << "</size>"
                << "       </box>"
                << "     </geometry>";
  }

  void addSurface()
  {
    sdf_stream_ << "        <surface>"
                << "          <friction>"
                << "            <ode>"
                << "              <mu>0.0</mu>"
                << "              <mu2>0.0</mu2>"
                << "            </ode>"
                << "          </friction>"
                << "        </surface>";
  }

  void addMaterial( Eigen::Vector4d & ambient ,
                    Eigen::Vector4d & diffuse ,
                    Eigen::Vector4d & specular,
                    Eigen::Vector4d & emissive  )
  {
    sdf_stream_ << "  <material>"
                << "    <ambient>"  << ambient  << "</ambient>"
                << "    <diffuse>"  << diffuse  << "</diffuse>"
                << "    <specular>" << specular << "</specular>"
                << "    <emissive>" << emissive << "</emissive>"
                << "  </material>";
  }

  void addCollision( std::string collision_name,  double radius )
  {
    sdf_stream_ << "    <collision name='" + collision_name + "'>";
                addGeometry( radius );
                addSurface();
    sdf_stream_ << "    </collision>";
  }

  void addCollision( std::string collision_name, Eigen::Vector3d & box_size )
  {
    sdf_stream_ << "    <collision name='" + collision_name + "'>";
                addGeometry( box_size );
                addSurface();
    sdf_stream_ << "    </collision>";
  }

  void addVisual( std::string visual_name,
                  double radius,
                  Eigen::Vector4d & ambient ,
                  Eigen::Vector4d & diffuse ,
                  Eigen::Vector4d & specular,
                  Eigen::Vector4d & emissive  )
  {
    sdf_stream_ << "    <visual name='" + visual_name + "'>";
                addGeometry( radius );
                addMaterial( ambient ,
                             diffuse ,
                             specular,
                             emissive );
    sdf_stream_ << "    </visual>";
  }

  void addVisual( std::string visual_name,
                  Eigen::Vector3d & box_size,
                  Eigen::Vector4d & ambient ,
                  Eigen::Vector4d & diffuse ,
                  Eigen::Vector4d & specular,
                  Eigen::Vector4d & emissive  )
  {
    sdf_stream_ << "    <visual name='" + visual_name + "'>";
                addGeometry( box_size );
                addMaterial( ambient ,
                             diffuse ,
                             specular,
                             emissive );
    sdf_stream_ << "    </visual>";
  }

  void addInertia( double mass )
  {
    sdf_stream_ << "    <inertial>"
                << "      <mass>"<< mass <<"</mass>"
                << "    </inertial>";
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
    sdf_stream_ << "  <link name='" + link_name + "'>"
                << "    <pose>"<< pose << "</pose>";

                addInertia( mass );
                addCollision( collision_name, radius );
                addVisual( visual_name,
                           radius,
                           ambient ,
                           diffuse ,
                           specular,
                           emissive );

    sdf_stream_ << "   </link>";
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
    sdf_stream_ << "  <link name='" + link_name + "'>"
                << "    <pose>"<< pose << "</pose>";

                addInertia( mass );
                addCollision( collision_name, box_size );
                addVisual( visual_name,
                           box_size,
                           ambient ,
                           diffuse ,
                           specular,
                           emissive );

    sdf_stream_ << "   </link>";
  }

  void addJoint( std::string joint_name,
                 std::string joint_type,
                 std::string parent,
                 std::string child,
                 Eigen::Vector3d & axis )
  {
    sdf_stream_ << "  <joint name = '" + joint_name + "' type='" + joint_type + "'>"
                << "    <parent>" + parent + "</parent>"
                << "      <child>" + child + "</child>"
                << "      <axis>"
                << "        <xyz>" << axis << "</xyz>"
                << "        <limit>"
                << "          <lower>" << -1e+16 << "</lower>"
                << "          <upper>" <<  1e+16 << "</upper>"
                << "        </limit>"
                << "      </axis>"
                << "  </joint>";
  }


  void addPlugin( std::string plugin_name, std::string plugin_filename )
  {
    sdf_stream_ << "<plugin name='" + plugin_name + "' filename='" + plugin_filename + "' />";
  }

  void saveSDFFile( std::string sdf_filename )
  {
    generateModelEnd();
    sdfParsed_.SetFromString(sdf_stream_.str());
    sdfParsed_.Write( sdf_filename );
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


int main(int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "skinmodel_sdf_generator");
  ros::NodeHandle nh;

  SkinSimModelBuilder test;

  std::string para_sdf_filename = "/sdf_filename";
  std::string para_jcf_filename = "/joint_config_filename";

  std::string sdf_filename          ; // = "model.sdf";
  std::string joint_config_filename ; // = "joint_names.yaml";

  Eigen::Vector4d skin_ambient ;
  Eigen::Vector4d skin_diffuse ;
  Eigen::Vector4d skin_specular;
  Eigen::Vector4d skin_emissive;

  Eigen::Vector4d tactile_ambient ;
  Eigen::Vector4d tactile_diffuse ;
  Eigen::Vector4d tactile_specular;
  Eigen::Vector4d tactile_emissive;

  Eigen::Vector4d base_ambient ;
  Eigen::Vector4d base_diffuse ;
  Eigen::Vector4d base_specular;
  Eigen::Vector4d base_emissive;

  //                 R    G    B
  skin_ambient  << 1.0, 1.0, 1.0, 1.0 ;
  skin_diffuse  << 1.0, 1.0, 1.0, 1.0 ;
  skin_specular << 0.1, 0.1, 0.1, 1.0 ;
  skin_emissive = Eigen::Vector4d::Zero();

  tactile_ambient  << 1.0, 0.0, 0.0, 1.0 ;
  tactile_diffuse  << 1.0, 0.0, 0.0, 1.0 ;
  tactile_specular << 0.1, 0.1, 0.1, 1.0 ;
  tactile_emissive = Eigen::Vector4d::Zero();

  base_ambient  << 1.0, 1.0, 1.0, 1.0 ;
  base_diffuse  << 1.0, 1.0, 1.0, 1.0 ;
  base_specular << 0.1, 0.1, 0.1, 1.0 ;
  base_emissive = Eigen::Vector4d::Zero();

  if (!nh.getParam(para_sdf_filename , sdf_filename           )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_sdf_filename.c_str()); }
  if (!nh.getParam(para_jcf_filename , joint_config_filename  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_jcf_filename.c_str()); }

  std::string para_density      = "density";
  std::string para_size_x       = "size_x" ;
  std::string para_size_y       = "size_y" ;

  std::string para_skin_height  = "skin_height";
  std::string para_tac_height   = "tac_height";

  std::string para_plane_height = "plane_height";

  std::string para_d_pos        = "d_pos"  ;

  double density     ;
  double size_x = 1.5;
  double size_y = 1.5;

  double skin_height = 1.3;
  double tac_height  = 0.4;

  double plane_height= 0.4;

  double d_pos  = 0.5;


  if (!nh.getParam(para_density, density )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_density.c_str()); }
  if (!nh.getParam(para_size_x , size_x  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_size_x .c_str()); }
  if (!nh.getParam(para_size_y , size_y  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_size_y .c_str()); }

  if (!nh.getParam(para_skin_height, skin_height )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_skin_height.c_str()); }
  if (!nh.getParam(para_tac_height , tac_height  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_tac_height .c_str()); }

  if (!nh.getParam(para_plane_height, plane_height )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_plane_height .c_str()); }

  if (!nh.getParam(para_d_pos  , d_pos   )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_d_pos  .c_str()); }

  YAML::Emitter out;
  std::ofstream fout(joint_config_filename.c_str());

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
  box_size << 2.5*size_x, 2.5*size_y, d_pos/10;

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

  test.addJoint( "plane_joint",
                 "prismatic",
                 "world",
                 "plane",
                 axis );

  ////////////////////


  double tactile_size_x = d_pos   ;
  double tactile_size_y = d_pos   ;
  double tactile_size_z = d_pos/10;

  double pos_x = -size_x;
  double pos_y =  size_y;

  double sensor_no = (double)(pos_y - pos_x)/d_pos + 1;
  sensor_no = sensor_no*sensor_no;

  out << YAML::BeginSeq;

  for( int i = 1; i <= sensor_no; i++ )
  {

    std::ostringstream convert;
    convert << i;


    pose << pos_x, pos_y, skin_height, 0, 0, 0;
    radius = d_pos/2;

    test.addLink( "spring_" + convert.str(),
                  0.001,
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
                   "tactile_" + convert.str(),
                   "spring_" + convert.str(),
                   axis );

    ////////////////////
    pose << pos_x, pos_y, tac_height, 0, 0, 0;
    box_size << tactile_size_x, tactile_size_y, tactile_size_z;

    test.addLink( "tactile_" + convert.str(),
                  0.001,
                  "square_collision",
                  "visual",
                  box_size,
                  pose,
                  tactile_ambient ,
                  tactile_diffuse ,
                  tactile_specular,
                  tactile_emissive );

    axis << 0, 0, 1;

    test.addJoint( "tactile_joint_" + convert.str(),
                   "prismatic",
                   "plane",
                   "tactile_" + convert.str(),
                   axis );

//    std::cout << pos_x << " " << pos_y << "\n";

    pos_x = pos_x + d_pos;

    if( pos_x > size_x )
    {
      pos_x = -size_x;
      pos_y = pos_y - d_pos;
//      std::cout << " ------ \n";
    }

   out << YAML::BeginMap;
   out << YAML::Key << "Joint" << YAML::Value << "joint_" + convert.str();
   out << YAML::EndMap;

  }

  out << YAML::EndSeq;

  // Write YAML file and close
  fout << out.c_str();
  fout.close();

  ////////////////////

  test.addPlugin( "spring_joint" , "libspring_joint.so" );
  test.addPlugin( "tactile_joint", "libtactile_joint.so" );
  test.addPlugin( "plane_joint"  , "libplane_joint.so" );

  test.saveSDFFile( sdf_filename );

  return 0;

}

