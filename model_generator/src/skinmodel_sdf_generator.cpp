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

#include "sdf/sdf.hh"
//#include "sdf/parser_urdf.hh"
//#include "urdf/model.h"

#include <fstream>
#include <string>

#include <Eigen/Core>

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

  void addMaterial( double radius )
  {

  }

  void addMaterial(  )
  {
    sdf_stream_ << "  <material>"
                << "    <ambient>1 0 0 1</ambient>"
                << "    <diffuse>1 0 0 1</diffuse>"
                << "    <specular>0.1 0.1 0.1 1</specular>"
                << "    <emissive>0 0 0 0</emissive>"
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

  void addVisual( std::string visual_name, double radius )
  {
    sdf_stream_ << "    <visual name='" + visual_name + "'>";
                addGeometry( radius );
                addMaterial( radius );
    sdf_stream_ << "    </visual>";
  }

  void addVisual( std::string visual_name, Eigen::Vector3d & box_size )
  {
    sdf_stream_ << "    <visual name='" + visual_name + "'>";
                addGeometry( box_size );
                addMaterial( );
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
                Eigen::VectorXd & pose )
  {
    sdf_stream_ << "  <link name='" + link_name + "'>"
                << "    <pose>"<< pose << "</pose>";

                addInertia( mass );
                addCollision( collision_name, radius );
                addVisual( visual_name, radius );

    sdf_stream_ << "   </link>";
  }

  void addLink( std::string link_name,
                double mass,
                std::string collision_name,
                std::string visual_name,
                Eigen::Vector3d & box_size,
                Eigen::VectorXd & pose )
  {
    sdf_stream_ << "  <link name='" + link_name + "'>"
                << "    <pose>"<< pose << "</pose>";

                addInertia( mass );
                addCollision( collision_name, box_size );
                addVisual( visual_name, box_size );

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

  SkinSimModelBuilder test;

  std::string sdf_filename = "model.sdf";

//  sdf::SDFPtr robot(new sdf::SDF());
//  sdf::init(robot);

  Eigen::VectorXd pose;
  Eigen::Vector3d box_size;
  Eigen::Vector3d axis;
  double radius;

  pose.resize(6,1);
  pose << 0.0, 0.0, 0.5, 0.0, 0.0, 0.0;

  test.generateModelStart("spring_board", pose );

  pose << 0, 0, 0.1, 0, 0, 0;
  box_size << 4, 4, 0.2;

  test.addLink( "plane",
                20,
                "collision",
                "visual",
                box_size,
                pose );

  axis << 0, 0, 1;

  test.addJoint( "plane_joint",
                 "prismatic",
                 "world",
                 "plane",
                 axis );

  ////////////////////

  double pos_x = -1.5;
  double pos_y = 1.5;

  for( int i = 1; i<50; i++ )
  {

    std::ostringstream convert;
    convert << i;

    pose << pos_x, pos_y, 1.3, 0, 0, 0;
    radius = 0.25;

    test.addLink( "spring_" + convert.str(),
                  0.001,
                  "sphere_collision",
                  "visual",
                  radius,
                  pose );

    axis << 0, 0, 1;

    test.addJoint( "joint_" + convert.str(),
                   "prismatic",
                   "tactile_" + convert.str(),
                   "spring_" + convert.str(),
                   axis );

    ////////////////////
    pose << pos_x, pos_y, 0.4, 0, 0, 0;
    box_size << 0.5, 0.5, 0.01;

    test.addLink( "tactile_" + convert.str(),
                  0.001,
                  "square_collision",
                  "visual",
                  box_size,
                  pose );

    axis << 0, 0, 1;

    test.addJoint( "tactile_joint_" + convert.str(),
                   "prismatic",
                   "plane",
                   "tactile_" + convert.str(),
                   axis );

//    std::cout << pos_x << " " << pos_y << "\n";

    pos_x = pos_x + 0.5;

    if( pos_x > 1.5 )
    {
      pos_x = -1.5;
      pos_y = pos_y - 0.5;
//      std::cout << " ------ \n";
    }


  }

  ////////////////////

  test.addPlugin( "spring_joint" , "libspring_joint.so" );
  test.addPlugin( "tactile_joint", "libtactile_joint.so" );
  test.addPlugin( "plane_joint"  , "libplane_joint.so" );

  test.saveSDFFile( sdf_filename );

//  /*
//   * Add define robot model
//   */
//
//  sdf::ElementPtr element_desc;
//  element_desc.reset(new sdf::Element);
//
//  robot->root->AddElement("model");
//
//  element_desc->SetName("spring_joint");
////  robot->root->GetElement("model")->SetDescription("test description");
//
////  sdf::Pose pose_test;
//
//  robot->root->GetElement("model")->AddAttribute("name", "string", "spring_board", 0, "000009");
////  robot->root->GetElement("model")->AddAttribute("name", "string", "spring_board", 0, "000009");
//
//  /*
//   * Write to sdf file
//   * TODO make this a class member function
//   */
//  robot->Write(sdf_filename);

  return 0;

}

