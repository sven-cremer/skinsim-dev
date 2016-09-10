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

ModelBuilder::ModelBuilder( BuildModelSpec modelSpecs )
{
	plunger_name = "generated_plunger";

	// Create skin array models
	initSkinSimModelBuilder();
	createModelFiles( modelSpecs );

	//Create Plunger Model
	if(!checkModelDirectory(plunger_name))	// Only generate plunger once	FIXME but create at least one plunger
	{
		std::cout<<"Generating plunger ...\n";
		createPlungerModelFiles(plunger_name);
	}
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
	double scaled_radius = radius*1.0;
	m_sdfStream << "      <geometry>\n"
			<< "        <sphere>\n"
			<< "          <radius>" << scaled_radius << "</radius>\n"
			<< "        </sphere>\n"
			<< "      </geometry>\n";
	//std::cout<<"[ModelBuilder::addGeometry] Actual radius used: "<<scaled_radius<<"\n"; 	// TODO remove scaling
}

void ModelBuilder::addGeometry( Eigen::Vector3d & box_size )
{
	m_sdfStream << "      <geometry>\n"
			<< "        <box>\n"
			<< "          <size>" << box_size.transpose() << "</size>\n"
			<< "        </box>\n"
			<< "      </geometry>\n";
}

void ModelBuilder::addGeometry( double radius, double length )
{
	double scaled_radius = radius*1.0;
	m_sdfStream << "              <geometry>\n"
                << "                  <cylinder>\n"
                << "                       <radius>" << scaled_radius << "</radius>\n"
                << "                       <length>" << length << "</length>\n"
                << "                  </cylinder>\n"
                << "              </geometry>\n";
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
	m_sdfStream << "		<pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>\n";
	addGeometry( radius );
	//addSurface();
	m_sdfStream << "    </collision>\n";
}

void ModelBuilder::addCollision( std::string collision_name, Eigen::Vector3d & box_size )
{
	m_sdfStream << "    <collision name='" + collision_name + "'>\n";
	m_sdfStream << "		<pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>\n";
	addGeometry( box_size );
	addSurface();
	m_sdfStream << "    </collision>\n";
}

void ModelBuilder::addCollision( std::string collision_name, double radius, double length )
{
	m_sdfStream << "          <collision name='" + collision_name + "'>\n";
//	m_sdfStream << "		<pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>\n";
	addGeometry( radius, length );
	//addSurface();
	m_sdfStream << "              <max_contacts>500</max_contacts>\n";
	m_sdfStream << "          </collision>\n\n";
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


void ModelBuilder::addVisual( std::string visual_name,
                              double radius,
                              double length)
{
	m_sdfStream << "          <visual name='" + visual_name + "'>\n";
	addGeometry( radius, length );
	m_sdfStream << "              <material>\n"
			    << "                <script>\n"
			    << "                  <uri>file://media/materials/scripts/gazebo.material</uri>\n"
			    << "                  <name>Gazebo/Blue</name>\n"
			    << "                </script>\n"
			    << "              </material>\n";
	m_sdfStream << "          </visual>\n\n";
}

void ModelBuilder::addInertia( double mass )
{
	double Ixx = 0.4*mass*pow((0.5*m_.spec.element_diameter),2);
	std::string I = boost::lexical_cast<std::string>(Ixx);
    m_sdfStream << "    <inertial>\n"
                << "        <mass>"<< mass <<"</mass>\n"
//                << "        <inertia>                      \n"
//                << "            <ixx>"<<I.c_str()<<"</ixx> \n"
//                << "            <ixy>0</ixy>               \n"
//                << "            <ixz>0</ixz>               \n"
//                << "            <iyy>"<<I.c_str()<<"</iyy> \n"
//                << "            <iyz>0</iyz>               \n"
//                << "            <izz>"<<I.c_str()<<"</izz> \n"
//                << "        </inertia>                     \n"
                << "    </inertial>\n";
}


void ModelBuilder::addSensor(std::string sensor_name,
                             std::string sensor_type,
                             std::string collision_name)
{
	m_sdfStream << "          <sensor name='" + sensor_name + "' type='"+ sensor_type +"'>\n"
                << "              <" + sensor_type + ">\n"
                << "                  <collision>"<<collision_name<<"</collision> \n"
                << "              </" + sensor_type + ">\n"
				<< "              <!--update_rate> 200 </update_rate-->\n"
                << "          </sensor>\n\n";
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
	//addCollision( collision_name, box_size );			// FIXME uncomment
	addVisual( visual_name,
			box_size,
			ambient ,
			diffuse ,
			specular,
			emissive );
	m_sdfStream << "   </link>\n";
}

void ModelBuilder::addPlungerLink( std::string link_name,
		double mass,
		std::string collision_name,
		std::string visual_name,
		double radius,
		double length,
		Eigen::VectorXd & pose)
{
	m_sdfStream << "\n"
                << "        <link name='" + link_name + "'>\n\n";

	addInertia( mass );

	addCollision( collision_name, radius, length );

	addVisual( visual_name, radius, length);

	addSensor("plunger_sensor", "contact",collision_name);

	m_sdfStream << "\n"
                << "          <self_collide>0</self_collide>\n"
                << "          <kinematic>0</kinematic>\n"
                << "          <gravity>0</gravity>\n\n";

	m_sdfStream << "        </link>\n\n";
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
			<< "          <upper>" <<  0.5 << "</upper>\n"
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
			<< "      <physics>\n"
			// Toggle for ODEJoint::GetForceTorque
			<< "            <provide_feedback>false</provide_feedback>\n"
			// Switch between implicit and explicit in ODEJoint::ApplyStiffnessDamping()
			<< "            <ode>\n"
			<< "                  <implicit_spring_damper>false</implicit_spring_damper>\n"
			<< "            </ode>\n"
			<< "      </physics>\n"
			<< "  </joint>\n";
}

void ModelBuilder::addPlungerJoint( std::string joint_name,
		std::string joint_type,
		std::string parent,
		std::string child,
		Eigen::Vector3d & axis )
{
	m_sdfStream << "        <joint name = '" + joint_name + "' type='" + joint_type + "'>\n"
                << "            <parent>" + parent + "</parent>\n"
                << "            <child>" + child + "</child>\n"
                << "            <axis>\n"
                << "               <xyz>" << axis.transpose() << "</xyz>\n"
                << "               <limit>\n"
                << "                  <lower>" << -0.5 << "</lower>\n"
                << "                  <upper>" <<  1.5 << "</upper>\n"
                << "               </limit>\n"
//                << "               <use_parent_model_frame>0</use_parent_model_frame>\n"
                << "            </axis>\n"
                << "            <physics>\n"
                << "                <provide_feedback>1</provide_feedback>\n"
                << "                <ode>\n"
                << "                   <implicit_spring_damper>0</implicit_spring_damper>\n"
                << "                </ode>\n"
                << "            </physics>\n"
                << "        </joint>\n\n";
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

void ModelBuilder::addPlugin( std::string plugin_name, std::string plugin_filename)
{

	m_sdfStream << "\n  <plugin name='" + plugin_name + "' filename='" + plugin_filename + "' >\n"
			<< "    <fileName>"       << m_.name << "</fileName>\n"
			<< "    <rosNamespace>"   << m_.spec.ros_namespace   << "</rosNamespace>\n"
			<< "    <updateRate>"     << m_.spec.update_rate     << "</updateRate>\n"
			<< "    <mass>"           << m_.spec.element_mass    << "</mass>\n"
			<< "    <spring>"         << m_.spec.element_spring  << "</spring>\n"
			<< "    <damping>"        << m_.spec.element_damping << "</damping>\n"
			<< "    <spreadScaling>"  << m_.spec.spread_scaling  << "</spreadScaling>\n"
			<< "    <spreadSigma>"    << m_.spec.spread_sigma    << "</spreadSigma>\n"
			<< "    <noiseSigma>"     << m_.spec.noiseSigma      << "</noiseSigma>\n"
			<< "    <noiseMu>"        << m_.spec.noiseMu         << "</noiseMu>\n"
			<< "    <noiseAmplitude>" << m_.spec.noiseAmplitude  << "</noiseAmplitude>\n"
			<< "    <delay>"          << m_.spec.delay           << "</delay>\n"
			<< "  </plugin>";
}


void ModelBuilder::addPlungerPlugin( std::string plugin_name,
			std::string plugin_filename,
			std::string plugin_file_name,
			double kp,
			double kd,
			double kv,
			double jointPGain,
			double jointIGain,
			double jointDGain,
			double mass)
{
	m_sdfStream << "  <plugin name='" + plugin_name + "' filename='" + plugin_filename + "' >\n"
				<< "    <fileName>"       << plugin_file_name        << "</fileName>\n"
				<< "    <rosNamespace>"   << m_.spec.ros_namespace   << "</rosNamespace>\n"
				<< "    <updateRate>"     << m_.spec.update_rate     << "</updateRate>\n"
				<< "    <Kp>"             << kp                      << "</Kp>\n"
				<< "    <Kd>"             << kd                      << "</Kd>\n"
				<< "    <Kv>"             << kv                      << "</Kv>\n"
				<< "    <JointPgain>"     << jointPGain              << "</JointPgain>\n"
				<< "    <JointIgain>"     << jointIGain              << "</JointIgain>\n"
				<< "    <JointDgain>"     << jointDGain              << "</JointDgain>\n"
				<< "    <mass>"           << mass                    << "</mass>\n"
				<< "    <springEnv>"      << m_.spec.element_spring  << "</springEnv>\n"
				<< "  </plugin>\n";
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

bool ModelBuilder::checkModelDirectory( std::string & model_name )
{
	std::string filepath = getDirPath( model_name ) + "/models/" + model_name + "/";
	boost::filesystem::path dir(filepath);
	return boost::filesystem::exists(dir);
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

	sdf::SDF m_sdfParsed;
	m_sdfParsed.version = "1.5";
	m_sdfParsed.SetFromString( m_sdfStream.str() );
	m_sdfParsed.Write( filename );
}

void ModelBuilder::saveSDFFile( std::string & model_name, std::string & version )
{
	generateModelEnd();

	std::string filename = genModelDirectory( model_name ) + model_name + ".sdf";
	std::cout<<"Saving: "<<filename.c_str()<<"\n";

	sdf::SDF m_sdfParsed;
	m_sdfParsed.version = version;
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

	// Place plunger 0.05 cm above skin
	double plunger_length = 0.10;
	double plunger_height = 0.10;
	plunger_z  =  m_.spec.element_height + m_.spec.element_diameter
			      +(plunger_height-0.5*plunger_length) + 0.0005;

	// Move plunger to center of tactile patches
	plunger_x = (total_sensors_x*unit_size_x - total_elements_x - m_.spec.tactile_separation_x)*m_.spec.element_diameter*0.5;
	plunger_y = (total_sensors_y*unit_size_y - total_elements_y - m_.spec.tactile_separation_y)*m_.spec.element_diameter*0.5;

	// Add plunger offset
	plunger_x += m_.spec.plunger_offset_x;
	plunger_y += m_.spec.plunger_offset_y;

	// Convert to strings
	std::string px = boost::lexical_cast<std::string>(plunger_x);
	std::string py = boost::lexical_cast<std::string>(plunger_y);
	std::string pz = boost::lexical_cast<std::string>(plunger_z);

	// Physics engine
	std::string step_size = boost::lexical_cast<std::string>(m_.spec.step_size);
	std::string iters     = boost::lexical_cast<std::string>(m_.spec.solver_iterations);

	modelConfig << "<?xml version='1.0'?>"                                                   <<"\n"
			<< "<sdf version='1.5'>"                                                         <<"\n"
			<< "  <world name='default'>"                                                    <<"\n"
			<< "    "                                                                        <<"\n"
			<< "    <include>"                                                               <<"\n"
			<< "      <uri>model://ground_plane</uri>"                                       <<"\n"
			<< "    </include>"                                                              <<"\n"
			<< "    "                                                                        <<"\n"
			<< "    <include>"                                                               <<"\n"
			<< "      <uri>model://sun</uri>"                                                <<"\n"
			<< "    </include>"                                                              <<"\n"
			<< "    "                                                                        <<"\n"
			<< "    <include>"                                                               <<"\n"
			<< "      <uri>model://" << model_name << "</uri>"                               <<"\n"
			<< "      <pose>0 0 0.0 0 0 0</pose>"                                            <<"\n"
			<< "    </include>"                                                              <<"\n"
			<< "    "                                                                        <<"\n"
			<< "    <include>"                                                               <<"\n"
			<< "      <uri>model://"<<plunger_name.c_str() <<"</uri>"                        <<"\n"
			<< "      <pose>"<<px.c_str()<<" "<<py.c_str()<<" "<<pz.c_str()<<" 0 0 0</pose>" <<"\n"
			<< "    </include>"                                                              <<"\n"
			<< "    "                                                                        <<"\n"
			<< "    <physics type='ode'>"                                                    <<"\n"
			<< "      <gravity>0.0 0.0 -9.8</gravity>"                                       <<"\n"
			<< "      <max_step_size>"<<step_size.c_str()<<"</max_step_size>"                <<"\n"
//			<< "      <real_time_factor>1</real_time_factor>"                                <<"\n"
			<< "      <real_time_update_rate>0</real_time_update_rate>"                      <<"\n" 	// Run the simulation as fast as possible
			<< "      <ode>"                                                                 <<"\n"
			<< "        <solver>"                                                            <<"\n"
//			<< "          <type>quick</type>"                                                <<"\n"		// TODO define solver type
			<< "          <iters>"<<iters.c_str()<<"</iters>"                                <<"\n"
			<< "          <sor>1.3</sor>"                                                    <<"\n"
			<< "        </solver>"                                                           <<"\n"
			<< "        <constraints>"                                                       <<"\n"
			<< "          <cfm>0.0</cfm>"                                                    <<"\n"
			<< "          <erp>0.2</erp>"                                                    <<"\n"
			<< "          <contact_max_correcting_vel>100</contact_max_correcting_vel>"      <<"\n"
			<< "          <contact_surface_layer>0.001</contact_surface_layer>"              <<"\n"
			<< "        </constraints>"                                                      <<"\n"
			<< "      </ode>"                                                                <<"\n"
			<< "    </physics>"                                                              <<"\n"
			<< "    "                                                                        <<"\n"
			<< "    <gui fullscreen='0'>"                                                    <<"\n"
			<< "      <camera name='user_camera'>"                                           <<"\n"
			<< "        <pose>0.0 -0.46 0.27 0.0 0.48 1.56</pose>"                           <<"\n"
			<< "        <view_controller>orbit</view_controller>"                            <<"\n"
			<< "      </camera>"                                                             <<"\n"
			<< "    </gui>"                                                                  <<"\n"
			<< "    "                                                                        <<"\n"
			<< "  </world>"                                                                  <<"\n"
			<< "</sdf>"                                                                      <<"\n";

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


void ModelBuilder::createPlungerModelFiles(std::string model_name )
{
	m_sdfStream.str("");
	m_sdfStream.clear();
	m_sdfStream << "<?xml version='1.0' ?>\n";
	m_sdfStream << "<sdf version='1.4'>\n";

	std::string modelDirectory = genModelDirectory( model_name );

	Eigen::VectorXd pose;
	pose.resize(6,1);

	// Initial position of skin array model
	pose << m_.spec.init_x, m_.spec.init_y, 0.105, 0.0, 0.0, 0.0;		//TODO: Add Z Coordinate to plunger
	generateModelStart( model_name, pose );

	m_sdfStream << "        <static>false</static>\n";

	//////////////////////////////////////////////////////

	Eigen::VectorXd pose2;
	pose2.resize(6,1);
	pose2 << 0, 0, 0, 0, 0, 0;

	addPlungerLink(	"plunger_link",
			        m_.spec.plunger_mass,
				    "plunger_collision",
				    "plunger_visual",
					m_.spec.plunger_radius,
					m_.spec.plunger_length,
				    pose2);


	Eigen::Vector3d axis;
	axis << 0, 0, 1;

	addPlungerJoint( "plunger_joint",
                     "prismatic",
                     m_.spec.parent,
                     "plunger_link",
                     axis );

	addPlungerPlugin( "Plunger",
				      "libPlunger.so",
				      "plunger",
				      m_.spec.plunger_mass,
				      m_.spec.plunger_spring,
				      m_.spec.plunger_damping,
				      0.5,
				      0.1,
				      0.0,
					  m_.spec.plunger_mass);

	///////////////////////////////////////////////////////

	std::string version = "1.4";

	// Save files
	saveSDFFile(   model_name, version);
	saveConfigFile(model_name);

}


void ModelBuilder::createModelFiles( BuildModelSpec modelSpecs_ )
{
	// Store model specs
	m_ = modelSpecs_;

	// Compute additional parameters
	total_elements_x = m_.spec.num_elements_x*m_.spec.num_patches_x;
	total_elements_y = m_.spec.num_elements_y*m_.spec.num_patches_y;
	unit_size_x      = m_.spec.tactile_elements_x+m_.spec.tactile_separation_x;
	unit_size_y      = m_.spec.tactile_elements_y+m_.spec.tactile_separation_y;
	total_sensors_x  = total_elements_x/unit_size_x;	// Note: integer devision rounds down
	total_sensors_y  = total_elements_y/unit_size_y;

	// Check if there is room for one more sensor
	if(total_elements_x - total_sensors_x*unit_size_x >=  m_.spec.tactile_elements_x )
		total_sensors_x++;
	if(total_elements_y - total_sensors_y*unit_size_y >=  m_.spec.tactile_elements_y )
		total_sensors_y++;

	Eigen::Vector4d color;
	color  << 1.0, 1.0, 1.0, 1.0 ;

	std::string modelDirectory = genModelDirectory( m_.name );

	std::string joint_config_filename = modelDirectory + "joint_names.yaml";
	std::string tactile_id_filename   = modelDirectory + "tactile_id.yaml";

	YAML::Emitter out_joint_names;
	std::ofstream fout(joint_config_filename.c_str());

	YAML::Emitter out_tactile_id;
	std::ofstream fout2(tactile_id_filename.c_str());

	//  sdf::SDFPtr robot(new sdf::SDF());
	//  sdf::init(robot);

	Eigen::VectorXd pose;
	pose.resize(6,1);

	// Initial position of skin array model
	pose << m_.spec.init_x, m_.spec.init_y, m_.spec.init_z, 0.0, 0.0, 0.0;
	generateModelStart( m_.name, pose );

	// Parameters
	double plane_mass   = 0.01;		    // Not too small because of stability issue (but has otherwise no effect)

	// TODO check if length > 0
	m_.spec.patch_length_x  = m_.spec.num_elements_x*m_.spec.element_diameter;
	m_.spec.patch_length_y  = m_.spec.num_elements_y*m_.spec.element_diameter;

	m_.spec.total_length_x = m_.spec.num_patches_x*m_.spec.patch_length_x;
	m_.spec.total_length_y = m_.spec.num_patches_y*m_.spec.patch_length_y;

	//////////////////////////////////////////////////////
	// WORLD -> PLANE

	color  << 1.0, 0.0, 0.0, 1.0 ;

	//std::string parent = "world";	// TODO try mounting on a PR2 link, e.g. "r_forearm_roll_link"
	std::string plane  = "plane";

	createPlane(
			plane,
			m_.spec.parent,
			plane_mass,
			m_.spec.total_length_x,
			m_.spec.total_length_y,
			m_.spec.plane_thickness,
			0.0,
			0.0,
			m_.spec.plane_height,
			color);

	///////////////////////////////////////////////////////

//	TODO create border or specify coordinate of first tactile sensor, using (0,0) right now
//			int border_x = 1;
//	int border_y = 1;
//	if( (ix>border_x-1) && (ix<total_elements_x-border_x) && (iy>border_y-1) && (iy<total_elements_y-border_y) )

	///////////////////////////////////////////////////////
	// PLANE -> PATCH_X

	out_joint_names << YAML::BeginSeq;
	out_tactile_id << YAML::BeginMap;

	color  << 1.0, 1.0, 1.0, 1.0 ;

	// Shift to center
	double pos_x = 0.0 + m_.spec.patch_length_x/2 - m_.spec.patch_length_x*(double)m_.spec.num_patches_x/2;
	double pos_y = 0.0 + m_.spec.patch_length_y/2 - m_.spec.patch_length_y*(double)m_.spec.num_patches_y/2;

	// Create patches
	/*
	 * Layout:   y
	 *           ^ 2, 3
	 *           | 0, 1
	 *           -------> x
	 */
	int patch_idx = 0;
	for( int iy = 0; iy < m_.spec.num_patches_y; iy++ )
	{
		for( int ix = 0; ix < m_.spec.num_patches_x; ix++ )
		{

			double x = pos_x + ix*m_.spec.patch_length_x;
			double y = pos_y + iy*m_.spec.patch_length_y;

			std::string patch = "patch_" + boost::lexical_cast<std::string>(patch_idx);

			// Create skin patch plane
			createPlane(
					patch,
					"plane",
					plane_mass,
					0.95*m_.spec.patch_length_x,
					0.95*m_.spec.patch_length_y,
					1.50*m_.spec.plane_thickness,
					x,
					y,
					m_.spec.plane_height,
					color);


			createSkinPatchElements(
					patch,
					ix,
					iy,
					out_joint_names,
					out_tactile_id,
					m_.spec.element_diameter,
					m_.spec.element_mass,
					m_.spec.num_elements_x,
					m_.spec.num_elements_y,
					x,
					y,
					m_.spec.element_height);

			patch_idx++;
		}
	}

	out_joint_names << YAML::EndSeq;
	out_tactile_id << YAML::EndMap;

	// Write to YAML file and close
	std::cout<<"Saving: "<<joint_config_filename.c_str()<<"\n";
	fout << out_joint_names.c_str();
	fout.close();

	std::cout<<"Saving: "<<tactile_id_filename.c_str()<<"\n";
	fout2 << out_tactile_id.c_str();
	fout2.close();

	//    addPlugin( "skinsimTactileSensor", "libTactileSensorPlugin.so", model_name );
	//    addPlugin( "skinsimSkinJoint", "libSkinJointPlugin.so", model_name );			// <- Simple plugin that works
	//    addPlugin( "skinsimPlaneJoint", "libPlaneJoint.so", model_name );
	//    addPlugin( "skinsimSkinJoint", "libSkinJointForceDistributionPlugin.so", model_name );
	//    addPlugin( "skinsimSkinJoint", "libSkinJointPlugin_V2.so", model_name );
	addPlugin( "SkinJointGazeboRos", "libSkinJointGazeboRos.so");

	// Save files
	saveSDFFile(   m_.name );
	saveConfigFile(m_.name );
	saveWorldFile( m_.name );

	// Save plunger position ?
//	std::string plunger_pose_filename = modelDirectory + "plunger_pose.yaml";
//	std::ofstream fout3(plunger_pose_filename.c_str());
//	YAML::Emitter out3;
//	out << YAML::BeginMap;
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
	std::string link_collision = link_name + "_collision";

	// TODO Make a plane link
	addLink(link_name,
			link_mass,
			link_collision,
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
		double pos_z)
{

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

	// Create skin elements inside patch
	/*
	 * Layout:   y
	 *           ^ 6, 7, 8
	 *           | 3, 4, 5
	 *           | 0, 1, 2
	 *           ----------> x
	 */
	int spring_idx = 0;
	for( int iy = 0; iy < num_elements_y; iy++ )
	{
		for( int ix = 0; ix < num_elements_x; ix++ )
		{
			x = pos_x + ix*element_diameter;
			y = pos_y + iy*element_diameter;

			pose << x, y, z, 0, 0, 0;

			std::string spring = patch_name + "_spring_" + boost::lexical_cast<std::string>(spring_idx);
			std::string spring_joint = spring + "_joint" ;
			std::string collision = spring + "_collision";

			// Check if element is part of a tactile sensor
			bool sensor_x = false;
			int global_ix = (patch_ix*m_.spec.num_elements_x + ix);			// Allow sensors to span across patches
			int unit_ix = global_ix % unit_size_x;							// Index inside a unit

			if(global_ix > unit_size_x*total_sensors_x - m_.spec.tactile_separation_x-1)	// Check if enough elements are left to create a sensor
				sensor_x = false;
			else
				if(unit_ix<m_.spec.tactile_elements_x)						// Check if it could be a tactile element
					sensor_x = true;

			bool sensor_y = false;
			int global_iy = (patch_iy*m_.spec.num_elements_y + iy);			// Allow sensors to span across patches
			int unit_iy = global_iy % unit_size_y;							// Index inside a unit
			if(global_iy > unit_size_y*total_sensors_y - m_.spec.tactile_separation_y - 1)	// Check if enough elements are left to create a sensor
				sensor_y = false;
			else
				if(unit_iy<m_.spec.tactile_elements_y)						// Check if it could be a tactile element
					sensor_y = true;

			if(sensor_x && sensor_y)
			{
				skin_diffuse  << 1.0, 0.0, 0.0, 1.0;						// Make tactile element red

				// Compute sensor index
				int ind_x = global_ix/unit_size_x;
				int ind_y = global_iy/unit_size_y;
				int sensor_ind = ind_y*total_sensors_x+ind_x;				// Count along rows, i.e. 0, 1, 2, ...

				// Store tactile sensor ID
				out_tactile_id << YAML::Key << spring_joint << YAML::Value << sensor_ind;
			}
			else
				skin_diffuse  << 1.0, 1.0, 1.0, 1.0;						// Make non-sensor element White

			// Add link and joint
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

			// Store name
			out_joint_names << YAML::Value << spring_joint;

			spring_idx++;
		}
	}

}
//////////////////////////////////////////////////////////////
}
