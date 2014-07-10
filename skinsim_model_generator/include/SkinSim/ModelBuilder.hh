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

namespace SkinSim
{

class SkinSimModelBuilder
{
private:
  std::ostringstream m_sdfStream;
  sdf::SDF m_sdfParsed;

  void generateSDFHeader()
  {
    m_sdfStream << "<?xml version='1.0' ?>"
                << "<sdf version='1.4'>"
                << " ";
  }

  void generateModelEnd()
  {
    m_sdfStream << "    </model>"
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
    m_sdfStream << "<model name='" + name + "'>"
                << "<pose>"<< pose << "</pose>"
                << " ";
  }

  void addGeometry( double radius )
  {
    m_sdfStream << "      <geometry>"
                << "        <sphere>"
                << "          <radius>" << radius << "</radius>"
                << "        </sphere>"
                << "      </geometry>";
  }

  void addGeometry( Eigen::Vector3d & box_size )
  {
    m_sdfStream << "    <geometry>"
                << "       <box>"
                << "         <size>" << box_size << "</size>"
                << "       </box>"
                << "     </geometry>";
  }

  void addSurface()
  {
    m_sdfStream << "        <surface>"
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
    m_sdfStream << "  <material>"
                << "    <ambient>"  << ambient  << "</ambient>"
                << "    <diffuse>"  << diffuse  << "</diffuse>"
                << "    <specular>" << specular << "</specular>"
                << "    <emissive>" << emissive << "</emissive>"
                << "  </material>";
  }

  void addCollision( std::string collision_name,  double radius )
  {
    m_sdfStream << "    <collision name='" + collision_name + "'>";
                addGeometry( radius );
                addSurface();
    m_sdfStream << "    </collision>";
  }

  void addCollision( std::string collision_name, Eigen::Vector3d & box_size )
  {
    m_sdfStream << "    <collision name='" + collision_name + "'>";
                addGeometry( box_size );
                addSurface();
    m_sdfStream << "    </collision>";
  }

  void addVisual( std::string visual_name,
                   double radius,
                   Eigen::Vector4d & ambient ,
                   Eigen::Vector4d & diffuse ,
                   Eigen::Vector4d & specular,
                   Eigen::Vector4d & emissive  )
  {
    m_sdfStream << "    <visual name='" + visual_name + "'>";
                addGeometry( radius );
                addMaterial( ambient ,
                             diffuse ,
                             specular,
                             emissive );
    m_sdfStream << "    </visual>";
  }

  void addVisual( std::string visual_name,
                   Eigen::Vector3d & box_size,
                   Eigen::Vector4d & ambient ,
                   Eigen::Vector4d & diffuse ,
                   Eigen::Vector4d & specular,
                   Eigen::Vector4d & emissive  )
  {
    m_sdfStream << "    <visual name='" + visual_name + "'>";
                addGeometry( box_size );
                addMaterial( ambient ,
                             diffuse ,
                             specular,
                             emissive );
    m_sdfStream << "    </visual>";
  }

  void addInertia( double mass )
  {
    m_sdfStream << "    <inertial>"
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
    m_sdfStream << "  <link name='" + link_name + "'>"
                << "    <pose>"<< pose << "</pose>";

                addInertia( mass );
                addCollision( collision_name, radius );
                addVisual( visual_name,
                           radius,
                           ambient ,
                           diffuse ,
                           specular,
                           emissive );

    m_sdfStream << "   </link>";
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
    m_sdfStream << "  <link name='" + link_name + "'>"
                << "    <pose>"<< pose << "</pose>";

                addInertia( mass );
                addCollision( collision_name, box_size );
                addVisual( visual_name,
                           box_size,
                           ambient ,
                           diffuse ,
                           specular,
                           emissive );

    m_sdfStream << "   </link>";
  }

  void addJoint( std::string joint_name,
                  std::string joint_type,
                  std::string parent,
                  std::string child,
                  Eigen::Vector3d & axis )
  {
    m_sdfStream << "  <joint name = '" + joint_name + "' type='" + joint_type + "'>"
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

  void addPlaneJoint( std::string joint_name,
                        std::string joint_type,
                        std::string parent,
                        std::string child,
                        Eigen::Vector3d & axis,
                        double upper_limit,
                        double lower_limit )
  {
    m_sdfStream << "  <joint name = '" + joint_name + "' type='" + joint_type + "'>"
                << "    <parent>" + parent + "</parent>"
                << "      <child>" + child + "</child>"
                << "      <axis>"
                << "        <xyz>" << axis << "</xyz>"
                << "        <limit>"
                << "          <lower>" << lower_limit << "</lower>"
                << "          <upper>" << upper_limit << "</upper>"
                << "        </limit>"
                << "      </axis>"
                << "  </joint>";
  }


  void addPlugin( std::string plugin_name, std::string plugin_filename )
  {
    m_sdfStream << "<plugin name='" + plugin_name + "' filename='" + plugin_filename + "' />";
  }

  void saveSDFFile( std::string sdf_filename )
  {
    generateModelEnd();
    m_sdfParsed.SetFromString(m_sdfStream.str());
    m_sdfParsed.Write( sdf_filename );
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}


#endif /* MODELBUILDER_HH_ */
