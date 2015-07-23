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
 * automatedTest.cc
 *  Created on: Jul 21, 2014
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>

#include <sdf/sdf.hh>

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "gazebo/transport/transport.hh"

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/gazebo_config.h"
#include "gazebo/Server.hh"

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <SkinSim/ModelBuilder.hh>
#include <SkinSim/ModelSpecYAML.hh>
#include <SkinSim/ControlSpecYAML.hh>

using namespace gazebo;

using namespace SkinSim;
using namespace std;

class SkinSimTestingFramework
{

protected: Server *server;
protected: boost::thread *serverThread;

protected: common::StrStr_M m_gazeboParams;

protected: common::Time simTime, realTime, pauseTime;
private: double percentRealTime;
private: bool paused;
private: bool serverRunning;
protected: transport::NodePtr node;
protected: transport::SubscriberPtr statsSub;

public:
SkinSimTestingFramework()
{
	std::cout << "SkinSim Testing Framework Constructor" << std::endl;
	this->server = NULL;

//	this->node = transport::NodePtr(new transport::Node());
//	this->node->Init();
//	this->statsSub = this->node->Subscribe("~/world_stats", &SkinSimTestingFramework::OnStats, this);
//	std::cout << "Initialized Transport Node" << std::endl;
}

void OnStats(ConstWorldStatisticsPtr &_msg)
{
	this->simTime = msgs::Convert(_msg->sim_time());
	this->realTime = msgs::Convert(_msg->real_time());
	this->pauseTime = msgs::Convert(_msg->pause_time());
	this->paused = _msg->paused();


	this->serverRunning = true;
}

void SetPause(bool _pause)
{
	physics::pause_worlds(_pause);
}

void Unload()
{
	gzdbg << "ServerFixture::Unload" << std::endl;
	this->serverRunning = false;
	if (this->node)
		this->node->Fini();

	if (this->server)
	{
		this->server->Stop();

		if (this->serverThread)
		{
			this->serverThread->join();
		}
	}

	delete this->serverThread;
	this->serverThread = NULL;
}

void RunServer(const std::string &_worldFilename, bool _paused, const std::string &_physics)
{

	std::cout << "SkinSimTestingFramework::RunServer()" << std::endl;
	this->server = new Server();
	this->server->PreLoad();
	if (_physics.length())
		this->server->LoadFile(_worldFilename,  _physics);
	else
		this->server->LoadFile(_worldFilename);

	if (!rendering::get_scene(
			gazebo::physics::get_world()->GetName()))
	{
		rendering::create_scene(
				gazebo::physics::get_world()->GetName(), false, true);
	}

	this->server->SetParams( m_gazeboParams );

	this->SetPause(_paused);

	this->server->Run();

	this->server->Fini();

	delete this->server;
	this->server = NULL;
}

void saveControlSpec( std::string expName )
{
	// Write YAML files
	std::string pathString( getenv ("SKINSIM_PATH") );
	std::string ctrSpecPath = pathString + "/model/config/ctr_config.yaml";
	std::ofstream ctrOut(ctrSpecPath.c_str());

	YAML::Emitter ctrYAMLEmitter;

	ControllerSpec ctrSpec;

	ctrSpec.name         = expName ;
	ctrSpec.explFctr_Kp  = 2       ;
	ctrSpec.explFctr_Ki  = 0.00005 ;
	ctrSpec.explFctr_Kd  = 0.5     ;
	ctrSpec.impCtr_Xnom  = 0.5     ;
	ctrSpec.impCtr_M     = 5       ;
	ctrSpec.impCtr_K     = 24      ;
	ctrSpec.impCtr_D     = 10      ;
	ctrSpec.ctrType      = 1       ;
	ctrSpec.targetForce  = 0.01    ;

	// Save controller specs
	ctrYAMLEmitter << YAML::BeginSeq;
	ctrYAMLEmitter << ctrSpec;
	ctrYAMLEmitter << YAML::EndSeq;

	ctrOut << ctrYAMLEmitter.c_str();;
	ctrOut.close();
}

void runTests()
{
	std::cout << "TestingFramework::runTests()" << std::endl;

	BuildModelSpec modelSpecs;

	double xByX         = 0.0 ;

	double density      = 0.0 ;
	double size_x       = 1.5 ;
	double size_y       = 1.5 ;

	double skin_height  = 1.3 ;
	double plane_height = 0.4 ;
	double d_pos        = 0.5 ;

	double sens_rad     = 1.0 ;
	double space_wid    = 3.0 ;

	// Read YAML files
	std::string pathString( getenv ("SKINSIM_PATH") );
	std::string configFilePath = pathString + "/experimenter/config/mdlSpecs.yaml";
	std::ifstream fin(configFilePath.c_str());
	YAML::Node doc;
	std::cout<<"Loading file: "<<configFilePath<<"\n";
	doc = YAML::LoadAll(fin);

	std::string _worldFilename("~");
	bool _paused = false;
	std::string _physics = "ode";

	for(unsigned i=0;i<doc[0].size();i++)
	{
		doc[0][i] >> modelSpecs;

		// FIXME this assumes square patches
		if( modelSpecs.spec.xByX != 0.0 )
		{
			modelSpecs.spec.size_x = modelSpecs.spec.skin_element_diameter*modelSpecs.spec.xByX ;
			modelSpecs.spec.size_y = modelSpecs.spec.skin_element_diameter*modelSpecs.spec.xByX ;
		}

		std::cout << std::endl << "Experiment number: " << i + 1 << " out of " << doc[0].size() << std::endl;

		// Create model files
		SkinSimModelBuilder skinSimModelBuilderObject( modelSpecs.name              ,
				modelSpecs.spec.xByX                   ,
				modelSpecs.spec.thick_board            ,
				modelSpecs.spec.density                ,
				modelSpecs.spec.size_x                 ,
				modelSpecs.spec.size_y                 ,
				modelSpecs.spec.skin_height            ,
				modelSpecs.spec.plane_height           ,
				modelSpecs.spec.tactile_height	       ,
				modelSpecs.spec.skin_element_diameter  ,
				modelSpecs.spec.tactile_length         ,
				modelSpecs.spec.tactile_separation     );

		delete this->server;
		this->server = NULL;

		double Ne = modelSpecs.spec.xByX * modelSpecs.spec.xByX;

		std::string expName = "efc_"                                                        +
				boost::lexical_cast<std::string>( Ne )                        +
				"_"                                                           +
				boost::lexical_cast<std::string>( modelSpecs.spec.tactile_length  ) +
				"_"                                                           +
				boost::lexical_cast<std::string>( modelSpecs.spec.tactile_separation )  ;

		saveControlSpec( expName );

		// Point to newly created world file location
		_worldFilename = pathString + "/model/worlds/" + modelSpecs.name + ".world";

		m_gazeboParams["iterations"] = "5000";

		std::cout << "Initializing World" << std::endl;
		// Create, load, and run the server in its own thread
		this->serverThread = new boost::thread(
				boost::bind(&SkinSimTestingFramework::RunServer, this, _worldFilename,
						_paused, _physics));

		/********/
		this->node = transport::NodePtr(new transport::Node());
		this->node->Init();
		this->statsSub = this->node->Subscribe("~/world_stats", &SkinSimTestingFramework::OnStats, this);
		std::cout << "Initialized Transport Node" << std::endl;
		/*******/

		std::cout << "Waiting for world completion" << std::endl;
		// Wait for the server to come up
		// Use a 60 second timeout.
		int waitCount = 0, maxWaitCount = 6000;
		while ((!this->server || !this->server->GetInitialized()) && (++waitCount < maxWaitCount) )
				common::Time::MSleep(100);

				std::cout << "ServerFixture load in "
				<< static_cast<double>(waitCount)/10.0
				<< " seconds, timeout after "
				<< static_cast<double>(maxWaitCount)/10.0
				<< " seconds\n" ;

		while( physics::worlds_running() )
		{
			common::Time::MSleep(100);

		}

		std::cout << std::endl << "Time: " << simTime.sec << " sec " << simTime.nsec << " nsec " << std::endl;


		Unload();

	}

	delete this->server;

}

};



int main(int argc, char** argv)
{

	SkinSimTestingFramework skinSimTestingFrameworkObject;
	skinSimTestingFrameworkObject.runTests();

	return 0;

}



