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
 * model_builder.cpp
 *
 *  Created on: Jul 25, 2016
 *      Author: Sven Cremer
 */

#include <SkinSim/model_builder.h>

namespace SkinSim
{

//////////////////////////////////////////////////////////////////////////
// Constructor
ModelBuilder::ModelBuilder( )
{
	initSkinSimModelBuilder();
}

ModelBuilder::ModelBuilder(  std::string model_name                        ,
		double xByX                                 ,
		double thick_board				          ,
		double density                              ,
		double size_x                               ,
		double size_y                               ,
		double skin_height                          ,
		double plane_height                         ,
		double tactile_height					      ,
		double skin_element_diameter                ,
		double tactile_length                       ,
		double tactile_separation                   )
{
	initSkinSimModelBuilder();
	createModelFiles( model_name                      ,
			xByX                            ,
			thick_board			          ,
			density                         ,
			size_x                          ,
			size_y                          ,
			skin_height                     ,
			plane_height                    ,
			tactile_height			      ,
			skin_element_diameter           ,
			tactile_length                  ,
			tactile_separation              );
}

//////////////////////////////////////////////////////////////////////////
// Destructor
ModelBuilder::~ModelBuilder()
{

}

void ModelBuilder::initSkinSimModelBuilder()
{
	// Set SkinSim path
	pathString = getenv ("SKINSIM_PATH");
	generateSDFHeader();
}

void ModelBuilder::generateSDFHeader()
{
	m_sdfStream << "<?xml version='1.0' ?>\n"
			<< "<sdf version='1.5'>\n";
}

void ModelBuilder::generateModelEnd()
{
	m_sdfStream << "\n  </model>\n"
			<< "</sdf>";
}

void ModelBuilder::generateModelStart( std::string name, Eigen::VectorXd & pose )
{
	m_sdfStream << "  <model name='" + name + "'>\n"
			<< "    <pose>"<< pose.transpose() << "</pose>\n";
}

void ModelBuilder::addGeometry( double radius )
{
	m_sdfStream << "      <geometry>\n"
			<< "        <sphere>\n"
			<< "          <radius>" << radius*0.95 << "</radius>\n"
			<< "        </sphere>\n"
			<< "      </geometry>\n";
}

void ModelBuilder::addGeometry( Eigen::Vector3d & box_size )
{
	m_sdfStream << "      <geometry>\n"
			<< "        <box>\n"
			<< "          <size>" << box_size.transpose() << "</size>\n"
			<< "        </box>\n"
			<< "      </geometry>\n";
}

void ModelBuilder::addSurface()
{
	m_sdfStream << "        <surface>\n"
			<< "          <friction>\n"
			<< "            <ode>\n"
			<< "              <mu>0.0</mu>\n"
			<< "              <mu2>0.0</mu2>\n"
			<< "            </ode>\n"
			<< "          </friction>\n"
			<< "		  <bounce>\n"
			<< "        	<restitution_coefficient>0</restitution_coefficient>\n"
			<< "       		<threshold>0</threshold>\n"
			<< "	     </bounce>\n"
			<< "   	     <contact>\n"
			<< "       		<ode>\n"
			<< "		   		<soft_cfm>0</soft_cfm>\n"
			<< "       	   		<soft_erp>0.200000</soft_erp>\n"
			<< "       	   		<kp>10000000000000.000000</kp>\n"
			<< "       	   		<kd>100000000000.000000</kd>\n"
			<< "      	   		<max_vel>-1</max_vel>\n"
			<< "       	   		<min_depth>0</min_depth>\n"
			<< "       	 	</ode>\n"
			<< "   	     </contact>\n"
			<< "       </surface>\n";
}

void ModelBuilder::addMaterial( Eigen::Vector4d & ambient ,
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

void ModelBuilder::addCollision( std::string collision_name,  double radius )
{
	m_sdfStream << "    <collision name='" + collision_name + "'>\n";
	m_sdfStream << "		<pose>0.000000 0.000000 0.0 0.000000 -0.000000 0.000000</pose>\n";
	addGeometry( radius );
	addSurface();
	m_sdfStream << "    </collision>\n";
}

void ModelBuilder::addCollision( std::string collision_name, Eigen::Vector3d & box_size )
{
	m_sdfStream << "    <collision name='" + collision_name + "'>\n";
	m_sdfStream << "		<pose>0.000000 0.000000 0.0 0.000000 -0.000000 0.000000</pose>\n";
	addGeometry( box_size );
	addSurface();
	m_sdfStream << "    </collision>\n";
}

void ModelBuilder::addVisual( std::string visual_name,
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

void ModelBuilder::addVisual( std::string visual_name,
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

void ModelBuilder::addInertia( double mass )
{
	m_sdfStream << "    <inertial>\n"
			<< "      <mass>"<< mass <<"</mass>\n"
			<< "    </inertial>\n";
}

void ModelBuilder::addLink( std::string link_name,
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
			<< "	<self_collide>0</self_collide>\n"
			<< "    <gravity>0</gravity>\n"
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

void ModelBuilder::addLink( std::string link_name,
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
			<< "	<self_collide>0</self_collide>\n"
			<< "    <gravity>0</gravity>\n"
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

void ModelBuilder::addJoint( std::string joint_name,
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
			<< "          <lower>" << -0.5 << "</lower>\n"
			<< "          <upper>" <<  2.0 << "</upper>\n"
			<< "        </limit>\n"
			<< "      </axis>\n"
			//				<< "    <sensor name='contact_" + joint_name + "' type='force_torque'>"
			//				<< "		<topic> test_ "+ joint_name + " </topic>"
			//				<< "		<update_rate> 5 </update_rate>"
			//				<< "  		<always_on>true</always_on>"
			//				<< "		<visualize>true</visualize>"
			//				<< " 		<noise>"
			//				<< "        	<type>gaussian</type>"
			//				<< "            <mean>0.0</mean>"
			//				<< "            <stddev>0.01</stddev>"
			//				<< " 		</noise>"
			//				<< "   	</sensor>"
			<< "  </joint>\n";
}

void ModelBuilder::addPlaneJoint( std::string joint_name,
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

void ModelBuilder::addPlugin( std::string plugin_name, std::string plugin_filename, std::string & model_name )
{
	std::string ros_namespace = "skinsim";
	double update_rate = 0.0;
	double mass = 0.0;
	double spring = 122.24;
	double damping = 1.83;

	m_sdfStream << "\n  <plugin name='" + plugin_name + "' filename='" + plugin_filename + "' >\n"
			<< "    <fileName>"     << model_name << "</fileName>\n"
			<< "    <rosNamespace>" << ros_namespace << "</rosNamespace>\n"
			<< "    <updateRate>"   << update_rate << "</updateRate>\n"
			<< "    <mass>"         << mass << "</mass>\n"
			<< "    <spring>"       << spring << "</spring>\n"
			<< "    <damping>"      << damping << "</damping>\n"
			<< "  </plugin>";
}

std::string ModelBuilder::getDirPath( std::string & model_name )
{
	//    boost::filesystem::path dir_path ( sdf_filename );

	//    try
	//    {
	//      if (boost::filesystem::exists(dir_path))    // does p actually exist?
	//      {
	//        if (boost::filesystem::is_regular_file(dir_path))        // is p a regular file?
	//        {
	//          dir_path = dir_path.branch_path();
	//          dir_path = dir_path.branch_path();
	//        }
	//
	//        if (boost::filesystem::is_directory(dir_path))      // is p a directory?
	//        {
	//          boost::filesystem::path dir(dir_path / model_name);
	//          dir_path = dir_path.branch_path();
	//          if(boost::filesystem::create_directory(dir))
	//          {
	//            //ROS_WARN_STREAM( "Success" );
	//          }
	//        }
	//      }
	//    }
	//    catch (const boost::filesystem::filesystem_error& ex)
	//    {
	//      //ROS_ERROR_STREAM( ex.what() );
	//    }

	//    return dir_path.string();

	return pathString + "/model";

}

std::string ModelBuilder::genModelDirectory( std::string & model_name )
{
	std::string filepath = getDirPath( model_name ) + "/models/" + model_name + "/";
	boost::filesystem::path dir(filepath);
	if(boost::filesystem::create_directory(dir))
	{
		//ROS_WARN_STREAM( "Success" );
	}
	return filepath;
}

std::string ModelBuilder::genWorldDirectory( std::string & model_name )
{
	std::string filepath = getDirPath( model_name ) + "/worlds/";
	return filepath;
}

void ModelBuilder::saveSDFFile( std::string & model_name )
{
	generateModelEnd();

	std::string filename = genModelDirectory( model_name ) + model_name + ".sdf";
	std::cout<<"Saving: "<<filename.c_str()<<"\n";

	m_sdfParsed.SetFromString( m_sdfStream.str() );
	m_sdfParsed.Write( filename );
}

void ModelBuilder::saveConfigFile( std::string & model_name )
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

	std::string filename = genModelDirectory( model_name ) + "model.config";
	saveFile( filename, modelConfig );
}

void ModelBuilder::saveWorldFile( std::string & model_name )
{
	std::ostringstream modelConfig;

	modelConfig << "<?xml version='1.0'?>                                           \n"
			<< "<gazebo version='1.3'>                                          \n"
			<< "<world name='default'>                                          \n"
			<< "                                                                \n"
			<< "<include>                                                       \n"
			<< "  <uri>model://ground_plane</uri>                               \n"
			<< "</include>                                                      \n"
			<< "                                                                \n"
			<< "<include>                                                       \n"
			<< "  <uri>model://sun</uri>                                        \n"
			<< "</include>                                                      \n"
			<< "                                                                \n"
			<< "<include>                                                       \n"
			<< "  <uri>model://" << model_name << "</uri>                       \n"
			<< "</include>                                                      \n"
			<< "                                                                \n"
			<< "<physics type='ode'>                                            \n"
			<< "  <gravity>0.0 0.0 -9.8</gravity>                               \n"
			<< "  <ode>                                                         \n"
			<< "    <solver>                                                    \n"
			<< "      <iters>150</iters>                                        \n"
			<< "    </solver>                                                   \n"
			<< "    <constraints>                                               \n"
			<< "      <cfm>0.2</cfm>                                            \n"
			<< "    </constraints>                                              \n"
			<< "  </ode>                                                        \n"
			<< "</physics>                                                      \n"
			<< "                                                                \n"
			//                << "                                                                \n"
			//                << "<include>                                                       \n"
			//                << "  <uri>model://box</uri>                                        \n"
			//                << "  <pose>0 0 0.055 0 0 0</pose>                                  \n"
			//                << "</include>                                                      \n"
			<< "                                                                \n"
			<< "<gui fullscreen='0'>                                            \n"
			<< "  <camera name='user_camera'>                                   \n"
			<< "    <pose>0.130675 -0.121126 0.095229 0 0.347643 2.35619</pose> \n"
			<< "    <view_controller>orbit</view_controller>                    \n"
			<< "  </camera>                                                     \n"
			<< "</gui>                                                          \n"
			<< "                                                                \n"
			<< "</world>                                                        \n"
			<< "</gazebo>                                                       \n";

	std::string filename = genWorldDirectory( model_name ) + model_name + ".world";
	saveFile( filename, modelConfig );
}

void ModelBuilder::saveFile( std::string & filename, std::ostringstream & model )
{
	std::cout<<"Saving: "<<filename.c_str()<<"\n";
	std::ofstream saveToFile;
	saveToFile.open ( filename.c_str() );
	saveToFile << model.str();
	saveToFile.close();
}

void ModelBuilder::createModelFiles( std::string model_name                        ,
		double xByX                                 ,
		double thick_board				           ,
		double density                              ,
		double size_x                               ,
		double size_y                               ,
		double skin_height                          ,
		double plane_height                         ,
		double tactile_height				       ,
		double skin_element_diameter                ,
		double tactile_length                       ,
		double tactile_separation                   )
{
	Eigen::Vector4d color;
	color  << 1.0, 1.0, 1.0, 1.0 ;

	std::string modelDirectory = genModelDirectory( model_name );

	std::string joint_config_filename = modelDirectory + "joint_names.yaml";
	std::string tactile_id_filename   = modelDirectory + "tactile_id.yaml";

	YAML::Emitter out;
	std::ofstream fout(joint_config_filename.c_str());

	YAML::Emitter out1;
	std::ofstream fout2(tactile_id_filename.c_str());

	//  sdf::SDFPtr robot(new sdf::SDF());
	//  sdf::init(robot);

	Eigen::VectorXd pose;
	pose.resize(6,1);
	pose << 0.0, 0.0, 0.05, 0.0, 0.0, 0.0;		// Pose of spring board model

	generateModelStart( model_name, pose );

	// Parameters

	int num_patches_x = 2;
	int num_patches_y = 3;

	double plane_mass = 20.0;
	double element_mass = 0.002;
	double num_elements_x = 8;
	double num_elements_y = 5;

	double patch_length_x  = num_elements_x*skin_element_diameter;
	double patch_length_y  = num_elements_y*skin_element_diameter;
	double patch_length_z  = thick_board*1.5;

	double total_length_x = num_patches_x*patch_length_x;
	double total_length_y = num_patches_y*patch_length_y;
    double total_length_z = thick_board;

	//////////////////////////////////////////////////////
	// WORLD -> PLANE

	//color  << 0.6, 0.6, 0.6, 1.0 ;
	color  << 1.0, 0.0, 0.0, 1.0 ;

	std::string parent = "world";	// TODO try mounting on a PR2 link, e.g. "r_forearm_roll_link"
	std::string plane  = "plane";

	createPlane(
			plane,
			parent,
			20.0,
			total_length_x,
			total_length_y,
			total_length_z,
			0.0,
			0.0,
			plane_height,
			color);

	///////////////////////////////////////////////////////
	// PLANE -> PATCH_X

	out << YAML::BeginSeq;

	color  << 1.0, 1.0, 1.0, 1.0 ;

	// Shift to center
	double pos_x = 0.0 + patch_length_x/2 - patch_length_x*(double)num_patches_x/2;
	double pos_y = 0.0 + patch_length_y/2 - patch_length_y*(double)num_patches_y/2;
	double pos_z = skin_height + plane_height + tactile_height;

	int index = 0;
	for( int ix = 0; ix < num_patches_x; ix++ )
	{
		for( int iy = 0; iy < num_patches_y; iy++ )
		{

			double x = pos_x + ix*patch_length_x;
			double y = pos_y + iy*patch_length_y;

			std::string patch = "patch_" + boost::lexical_cast<std::string>(index);

			// Create skin patch plane
			createPlane(
					patch,
					"plane",
					plane_mass,
					0.95*patch_length_x,
					0.95*patch_length_y,
					patch_length_z,
					x,
					y,
					plane_height,
					color);


			createSkinPatchElements(
					patch,
					out,
					skin_element_diameter,
					element_mass,
					num_elements_x,
					num_elements_y,
					x,
					y,
					pos_z);

			index++;
		} // End for patch
	}

	out << YAML::EndSeq;

	// Write YAML file and close
	std::cout<<"Saving: "<<joint_config_filename.c_str()<<"\n";
	fout << out.c_str();
	fout.close();

	fout2.close();		// TODO nothing is saved to file

	//    addPlugin( "skinsimTactileSensor", "libTactileSensorPlugin.so", model_name );
	//    addPlugin( "skinsimSkinJoint", "libSkinJointPlugin.so", model_name );			// <- Simple plugin that works
	//    addPlugin( "skinsimPlaneJoint", "libPlaneJoint.so", model_name );
	//    addPlugin( "skinsimSkinJoint", "libSkinJointForceDistributionPlugin.so", model_name );
	//    addPlugin( "skinsimSkinJoint", "libSkinJointPlugin_V2.so", model_name );
	addPlugin( "SkinJointGazeboRos", "libSkinJointGazeboRos.so", model_name );

	// Save files
	saveSDFFile(    model_name );
	saveConfigFile( model_name );
	saveWorldFile(  model_name );

}

void ModelBuilder::createPlane(
		std::string link_name,
		std::string parent_name,
		double link_mass,
		double length_x,
		double length_y,
		double length_z,
		double pos_x,
		double pos_y,
		double pos_z,
		Eigen::Vector4d color)
{

	Eigen::Vector4d base_ambient ;
	Eigen::Vector4d base_diffuse ;
	Eigen::Vector4d base_specular;
	Eigen::Vector4d base_emissive;
	//                R    G    B    a
	base_ambient  << 1.0, 1.0, 1.0, 1.0 ;
	base_diffuse  << 1.0, 1.0, 1.0, 1.0 ;
	base_specular << 0.1, 0.1, 0.1, 1.0 ;
	base_emissive = Eigen::Vector4d::Zero();

	base_diffuse = color;

	Eigen::VectorXd pose;
	pose.resize(6,1);
	pose << pos_x, pos_y, pos_z, 0, 0, 0;

	Eigen::Vector3d axis;
	axis << 0, 0, 1;

	Eigen::Vector3d box_size;
	box_size << length_x, length_y, length_z;

	std::string link_joint = link_name + "_joint";

	addLink(link_name,
			link_mass,
			"collision",
			"visual",
			box_size,
			pose,
			base_ambient ,
			base_diffuse ,
			base_specular,
			base_emissive );

	// Create a "fixed" joint by giving it +/- zero limits
	addPlaneJoint( link_joint,
			"prismatic",
			parent_name,
			link_name,
			axis,
			-0,
			 0);

}

void ModelBuilder::createSkinPatchElements(
		std::string patch_name,
		YAML::Emitter& out,
		double element_diameter,
		double element_mass,
		double num_elements_x,
		double num_elements_y,
		double pos_x,
		double pos_y,
		double pos_z)
{

	int num_elements = num_elements_x*num_elements_y;
	double radius = element_diameter/2;
	double x, y, z;

	Eigen::Vector4d skin_ambient ;
	Eigen::Vector4d skin_diffuse ;
	Eigen::Vector4d skin_specular;
	Eigen::Vector4d skin_emissive;
	//                R    G    B    a
	skin_ambient  << 1.0, 1.0, 1.0, 1.0 ;
	skin_diffuse  << 1.0, 1.0, 1.0, 1.0 ;
//	skin_diffuse  << 1.0, 1.0, 1.0, 0.5 ;
//	skin_specular << 0.1, 0.1, 0.1, 1.0 ;
	skin_specular << 0.5, 0.5, 0.5, 1.0 ;
	skin_emissive = Eigen::Vector4d::Zero();

	Eigen::VectorXd pose;
	pose.resize(6,1);

	Eigen::Vector3d axis;
	axis << 0, 0, 1;

	// Shift to center
	pos_x = pos_x + radius - element_diameter*(double)num_elements_x/2;
	pos_y = pos_y + radius - element_diameter*(double)num_elements_y/2;
	z = pos_z + radius;

	// Add skin elements
	int i = 0;
	for( int ix = 0; ix < num_elements_x; ix++ )
	{
		for( int iy = 0; iy < num_elements_y; iy++ )
		{

			x = pos_x + ix*element_diameter;
			y = pos_y + iy*element_diameter;

			pose << x, y, z, 0, 0, 0;

			std::string spring = patch_name + "_spring_" + boost::lexical_cast<std::string>(i);
			std::string spring_joint = spring + "_joint" ;
			std::string collision = patch_name + "_sphere_collision_" + boost::lexical_cast<std::string>(i);


			addLink(spring,
					element_mass,
					collision,
					"visual",
					radius,
					pose,
					skin_ambient ,
					skin_diffuse ,
					skin_specular,
					skin_emissive );

			addJoint(spring_joint,
					"prismatic",
					patch_name,
					spring,
					axis );

			//std::cout << pos_x << " " << pos_y << "\n";

			// Store name
			out << YAML::BeginMap;
			out << YAML::Key << "Joint" << YAML::Value << spring_joint;
			out << YAML::EndMap;

			i++;
		}

	}
}

//////////////////////////////////////////////////////////////
}
