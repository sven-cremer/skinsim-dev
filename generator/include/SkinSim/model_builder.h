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

/*
 * model_builder.h
 *
 *  Created on: Jul 25, 2016
 *      Author: Sven Cremer
 */

#ifndef MODEL_BUILDER_H_
#define MODEL_BUILDER_H_


#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <sdf/sdf.hh>
#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <SkinSim/ModelSpecYAML.hh>

namespace SkinSim
{

class ModelBuilder
{
private:
	std::ostringstream m_sdfStream;
	std::string pathString;
	BuildModelSpec m_;

	int total_elements_x;		// elements/patch * patches
	int total_elements_y;
	int unit_size_x;			// tactile elements + tactile separation
	int unit_size_y;
	int total_sensors_x;		// floor(total_elements/unit_size) + 1 if there is room
	int total_sensors_y;

	void generateSDFHeader();

	void generateModelEnd();

	typedef enum
	{
		BOX = 0,
		SPHERE
	} GEOMETRY_TYPE;

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	ModelBuilder( );

	ModelBuilder( BuildModelSpec modelSpecs );

	~ModelBuilder();

	void initSkinSimModelBuilder();

	void generateModelStart( std::string name, Eigen::VectorXd & pose );

	void addGeometry( double radius );

	void addGeometry( Eigen::Vector3d & box_size );

	void addGeometry( double radius, double length );

	void addSurface();

	void addMaterial( Eigen::Vector4d & ambient ,
			Eigen::Vector4d & diffuse ,
			Eigen::Vector4d & specular,
			Eigen::Vector4d & emissive  );

	void addCollision( std::string collision_name,  double radius );

	void addCollision( std::string collision_name, Eigen::Vector3d & box_size );

	void addCollision( std::string collision_name,  double radius, double length );

	void addVisual( std::string visual_name,
			double radius,
			Eigen::Vector4d & ambient ,
			Eigen::Vector4d & diffuse ,
			Eigen::Vector4d & specular,
			Eigen::Vector4d & emissive  );

	void addVisual( std::string visual_name,
			Eigen::Vector3d & box_size,
			Eigen::Vector4d & ambient ,
			Eigen::Vector4d & diffuse ,
			Eigen::Vector4d & specular,
			Eigen::Vector4d & emissive  );

	void addVisual( std::string visual_name,
				double radius,
				double length);

	void addInertia( double mass );

	void addLink( std::string link_name,
			double mass,
			std::string collision_name,
			std::string visual_name,
			double radius,
			Eigen::VectorXd & pose,
			Eigen::Vector4d & ambient ,
			Eigen::Vector4d & diffuse ,
			Eigen::Vector4d & specular,
			Eigen::Vector4d & emissive  );

	void addLink( std::string link_name,
			double mass,
			std::string collision_name,
			std::string visual_name,
			Eigen::Vector3d & box_size,
			Eigen::VectorXd & pose,
			Eigen::Vector4d & ambient ,
			Eigen::Vector4d & diffuse ,
			Eigen::Vector4d & specular,
			Eigen::Vector4d & emissive  );

	void addPlungerLink( std::string link_name,
			double mass,
			std::string collision_name,
			std::string visual_name,
			double radius,
			double length,
			Eigen::VectorXd & pose);

	void addJoint( std::string joint_name,
			std::string joint_type,
			std::string parent,
			std::string child,
			Eigen::Vector3d & axis );

	void addPlungerJoint( std::string joint_name,
			std::string joint_type,
			std::string parent,
			std::string child,
			Eigen::Vector3d & axis );

	void addPlaneJoint( std::string joint_name,
			std::string joint_type,
			std::string parent,
			std::string child,
			Eigen::Vector3d & axis,
			double upper_limit,
			double lower_limit );

	void addSensor(std::string sensor_name,
			std::string sensor_type,
			std::string collision_name);

	void addPlungerPlugin( std::string plugin_name,
			std::string plugin_filename,
			std::string plugin_file_name,
			double kp,
			double kd,
			double kv,
			double jointPGain,
			double jointIGain,
			double jointDGain);

	void addPlugin( std::string plugin_name, std::string plugin_filename);

	std::string getDirPath( std::string & model_name );

	std::string genModelDirectory( std::string & model_name );

	std::string genWorldDirectory( std::string & model_name );

	void saveSDFFile( std::string & model_name );
	void saveSDFFile( std::string & model_name,  std::string & version);

	void saveConfigFile( std::string & model_name );

	void saveWorldFile( std::string & model_name );

	void saveFile( std::string & filename, std::ostringstream & model );

	void createModelFiles( BuildModelSpec modelSpecs_  );
	void createPlungerModelFiles(std::string model_name);

	void createSkinPatchElements(
			std::string patch_name,
			int patch_ix,
			int patch_iy,
			YAML::Emitter& out_joint_names,
			YAML::Emitter& out_tactile_id,
			double element_diameter,
			double element_mass,
			double num_elements_x,
			double num_elements_y,
			double pos_x,
			double pos_y,
			double pos_z);

	void createPlane(
			std::string link_name,
			std::string parent_name,
			double link_mass,
			double length_x,
			double length_y,
			double length_z,
			double pos_x,
			double pos_y,
			double pos_z,
			Eigen::Vector4d color);

};

}

#endif /* MODEL_BUILDER_H_ */
