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
#include <iomanip>
#include <sstream>
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

#include <SkinSim/ModelSpecYAML.hh>
#include <SkinSim/ControlSpecYAML.hh>

using namespace gazebo;
//using namespace SkinSim;
//using namespace std;

class SkinSimTestingFramework
{

protected: Server* server;
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
	std::cout<< "ServerFixture::Unload" << std::endl;
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

	std::cout << "SkinSimTestingFramework::RunServer() - START" << std::endl;

	this->server = new Server();

	std::string ros_api_plugin  = "/opt/ros/jade/lib/libgazebo_ros_api_plugin.so";
	std::string ros_path_plugin = "/opt/ros/jade/lib/libgazebo_ros_paths_plugin.so";
	std::string world_file = _worldFilename; //"/home/sven/skin_ws/src/skinsim-dev/model/worlds/skin_array_0.world";

	// Create fake command-line parameters for Gazebo server
	// This seems to be the best way to set parameters such as sensor plugins
	int numArgs = 6;
	char **v = new char * [numArgs];

	v[0] = new char [strlen( "--verbose"             ) + 1];
	v[1] = new char [strlen( "-s"                    ) + 1];
	v[2] = new char [strlen( ros_path_plugin.c_str() ) + 1];
	v[3] = new char [strlen( "-s"                    ) + 1];
	v[4] = new char [strlen( ros_api_plugin.c_str()  ) + 1];
	v[5] = new char [strlen( world_file.c_str()      ) + 1];

	strcpy( v[0] , "--verbose"             );
	strcpy( v[1] , "-s"                    );
	strcpy( v[2] , ros_path_plugin.c_str() );
	strcpy( v[3] , "-s"                    );
	strcpy( v[4] , ros_api_plugin.c_str()  );
	strcpy( v[5] , world_file.c_str()      );

	// TODO --physics ode, --seed 0.0

	if (!server->ParseArgs(numArgs, v))
	{
		std::cerr<<"Failed parsing server arguments!\n";
	}

	//	if(! this->server->PreLoad() )
	//		std::cerr<<"Failed to preload the Gazebo server";

	//	if (_physics.length())
	//		this->server->LoadFile(_worldFilename, _physics);
	//	else
	//		this->server->LoadFile(_worldFilename);
	//
	//	if (!rendering::get_scene(
	//			gazebo::physics::get_world()->GetName()))
	//	{
	//		rendering::create_scene(
	//				gazebo::physics::get_world()->GetName(), false, true);
	//	}

	this->server->SetParams(m_gazeboParams);
	this->SetPause(_paused);

	this->server->Run();

	this->server->Fini();

	std::cout << "SkinSimTestingFramework::RunServer() - DONE" << std::endl;

	// Deallocate variables
	delete this->server;
	this->server = NULL;

	for(int i = 0; i < numArgs; i++)
		delete v[i];
	delete v;
}

void saveData( std::string exp_name )
{
	// TODO
}

void runTests(std::string exp_name)
{
	std::cout << "TestingFramework::runTests()" << std::endl;

	// Parameters
	std::string filename_model   = "mdlSpecs.yaml";;
	std::string filename_control = "ctrSpecs.yaml";

	// Create paths
	std::string pathSkinSim = getenv ("SKINSIM_PATH");
	std::string pathExp     = pathSkinSim + "/data/" + exp_name;
	std::string mdlSpecPath = pathExp + "/" + filename_model;
	std::string ctrSpecPath = pathExp + "/" + filename_control;

	// Read YAML model file
	std::ifstream fin(mdlSpecPath.c_str());
	YAML::Node doc_model;
	std::cout<<"Loading file: "<<mdlSpecPath<<"\n";
	doc_model = YAML::LoadAll(fin);

	// Read YAML control file
	std::ifstream fin2(ctrSpecPath.c_str());
	YAML::Node doc_control;
	std::cout<<"Loading file: "<<ctrSpecPath<<"\n";
	doc_control = YAML::LoadAll(fin2);

	// Gazebo parameters
	m_gazeboParams["iterations"] = "2000";	// Number of iterations

	//	for (std::map<std::string,std::string>::iterator it=m_gazeboParams.begin(); it!=m_gazeboParams.end(); ++it)
	//	    std::cout << it->first << " => " << it->second << '\n';
	//	std::cout<<"\n\n";


	std::string _worldFilename("~");
	bool _paused = false;
	std::string _physics = "ode";

	BuildModelSpec modelSpec;
	ControllerSpec controlSpec;

	int index = 1;
	int N = doc_model[0].size() * doc_control[0].size();

	// Loop over models
	for(unsigned i=0;i<doc_model[0].size();i++)
	{
		doc_model[0][i] >> modelSpec;
		//print(modelSpec);

		// Loop over control settings
		for(unsigned j=0;j<doc_control[0].size();j++)
		{
			doc_control[0][j] >> controlSpec;
			//print(controlSpec);

			// Generate experiment name
			int skin_size    = modelSpec.spec.num_elements_x * modelSpec.spec.num_elements_y;
			int tactile_size = modelSpec.spec.tactile_elements_x * modelSpec.spec.tactile_elements_x;
			int tactile_sep  = modelSpec.spec.tactile_separation_x * modelSpec.spec.tactile_separation_y;

			std::ostringstream ss;
			ss << std::setw(3) << std::setfill('0')
			<< index << "_" << skin_size << "-" << tactile_size << "-" << tactile_sep;
			std::string exp_name = "exp_" + ss.str()  ;

			// Print info
			std::cout << "\n# Experiment: "<< exp_name << " ("<< index << " out of " << N << ")\n";

			// Point model world file location
			_worldFilename = pathSkinSim + "/model/worlds/" + modelSpec.name + ".world";

			std::cout << "Initializing World" << std::endl;
			// Create, load, and run the server in its own thread
			this->serverThread = new boost::thread(
					boost::bind(&SkinSimTestingFramework::RunServer, this, _worldFilename,
							_paused, _physics));

			// Initialize transport node
			this->node = transport::NodePtr(new transport::Node());
			this->node->Init();
			this->statsSub = this->node->Subscribe("~/world_stats", &SkinSimTestingFramework::OnStats, this);
			std::cout << "Initialized Transport Node" << std::endl;
			// TODO subscribe to ~/physics/contacts

			// Save ROS topic data to file
			std::string topic = "/skinsim/tactile_data";
			std::string cmd1 = std::string("rostopic echo -p ") + topic.c_str() + std::string(" > ") + pathExp.c_str() + std::string("/") + exp_name.c_str() + std::string(".csv &");
			std::cout<<"$ "<<cmd1.c_str()<<"\n";
			system( cmd1.c_str() );

			std::cout << "Waiting for world completion" << std::endl;
			// Wait for the server to come up
			// Use a 20 second timeout.
			int waitCount = 0, maxWaitCount = 100;
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

			// Stop saving rtp file
			std::string cmd2 = std::string("pkill -9 -f ") + topic.c_str();
			std::cout<<"$ "<<cmd2.c_str()<<"\n";
			system( cmd2.c_str() );

			Unload();

			// Save data
			saveData( exp_name );

			index++;
		}
	}

	delete this->server;

}

};



int main(int argc, char** argv)
{

	std::string exp_name = "exp01";

	// Check the number of command-line parameters
	if (argc == 2)
	{
		// Set model file name
		exp_name = argv[1];
	}
	else
	{
		// Use default file name
		std::cout<<"\n\tUsage: "<<argv[0]<<" [EXPERIMENT NAME]\n\n";
	}

	SkinSimTestingFramework skinSimTestingFrameworkObject;
	skinSimTestingFrameworkObject.runTests(exp_name);

	return 0;

}



