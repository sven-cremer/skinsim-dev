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
 * skinsim_testing_framework.cc
 *
 *  Created on: Aug 31, 2016
 *      Author: Sven Cremer
 *              Isura Ranatunga
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
//#include <map>

// ROS
#include <ros/ros.h>
#include <skinsim_ros_msgs/SetController.h>
#include <skinsim_ros_msgs/ForceFeedback.h>
#include <skinsim_ros_msgs/GetPosition.h>
//#include <skinsim_ros_msgs/PlungerData.h>
//#include <skinsim_ros_msgs/CenterOfPressure.h>

// Boost
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/chrono.hpp>

// Gazebo
#include "gazebo/Server.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

// Utilities
#include <yaml-cpp/yaml.h>
//#include <Eigen/Core>
//#include <sdf/sdf.hh>

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
private: int iterations;
protected: transport::NodePtr node;
protected: transport::SubscriberPtr statsSub;

private: ros::NodeHandle* ros_node_;
private: std::string ros_namespace_;
private: skinsim_ros_msgs::SetController msg_srv_;
private: skinsim_ros_msgs::GetPosition msg_srv_pos_;
private: int iterations_max;

// Calibration
private: double* buffer;
private: int buffer_idx;
private: int buffer_size;
private: double K_sma;
private: double f_target;
private: bool calibration_done;
private: int calibration_counter;

// Color for terminal text
private: static const char RED[];
private: static const char GREEN[];
private: static const char YELLOW[];
private: static const char RESET[];

public:
SkinSimTestingFramework()
{
	std::cout << "SkinSim Testing Framework Constructor" << std::endl;
	this->server = NULL;
}

~SkinSimTestingFramework()
{

}

// ROS subscriber CB
void subscriberCB(const skinsim_ros_msgs::ForceFeedbackConstPtr& msg)
{
	if(msg->force_sensed == 0)
		return;

	double K = msg->force_applied / msg->force_sensed;
	K_sma = K_sma + ( K - buffer[buffer_idx] )/(double)buffer_size;
	buffer[buffer_idx] = K;
	buffer_idx = (buffer_idx + 1)%buffer_size;

	if(fabs(f_target - K_sma*msg->force_sensed ) < 0.01 )
	{
		calibration_counter++;
		if(calibration_counter > 10)
			calibration_done = true;
	}
	else
	{
		calibration_counter=0;
	}

}

void OnStats(ConstWorldStatisticsPtr &_msg)
{
	this->simTime = msgs::Convert(_msg->sim_time());
	this->realTime = msgs::Convert(_msg->real_time());
	this->pauseTime = msgs::Convert(_msg->pause_time());
	this->paused = _msg->paused();
	this->iterations = _msg->iterations();

	this->serverRunning = true;
}

void SetPause(bool _pause)
{
	physics::pause_worlds(_pause);
}

void Unload()
{
	//std::cout<< "ServerFixture::Unload" << std::endl;
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

	//std::cout << "SkinSimTestingFramework::RunServer() - START" << std::endl;

	this->server = new Server();

	std::string ros_api_plugin  = "/opt/ros/jade/lib/libgazebo_ros_api_plugin.so";
	std::string ros_path_plugin = "/opt/ros/jade/lib/libgazebo_ros_paths_plugin.so";
	std::string world_file = _worldFilename;

	// Create fake command-line parameters for Gazebo server
	// This seems to be the best way to set parameters such as sensor plugins
	int numArgs = 8;
	char **v = new char * [numArgs];

	v[0] = new char [strlen( "--verbose"             ) + 1];
	v[1] = new char [strlen( "-s"                    ) + 1];
	v[2] = new char [strlen( ros_path_plugin.c_str() ) + 1];
	v[3] = new char [strlen( "-s"                    ) + 1];
	v[4] = new char [strlen( ros_api_plugin.c_str()  ) + 1];
	v[5] = new char [strlen( world_file.c_str()      ) + 1];
	v[6] = new char [strlen( "--seed"                ) + 1];
	v[7] = new char [strlen( "0.0"                   ) + 1];

	strcpy( v[0] , "--verbose"             );
	strcpy( v[1] , "-s"                    );
	strcpy( v[2] , ros_path_plugin.c_str() );
	strcpy( v[3] , "-s"                    );
	strcpy( v[4] , ros_api_plugin.c_str()  );
	strcpy( v[5] , world_file.c_str()      );
	strcpy( v[6] , "--seed"                );
	strcpy( v[7] , "0.0"                   );

	// TODO --physics ode

	if (!server->ParseArgs(numArgs, v))
	{
		std::cerr<<"Failed parsing server arguments!\n";
	}

	this->server->SetParams(m_gazeboParams);
	this->SetPause(_paused);

	this->server->Run();

	this->server->Fini();

	//std::cout << "SkinSimTestingFramework::RunServer() - DONE" << std::endl;

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
	//std::cout << "TestingFramework::runTests()" << std::endl;

	// Parameters
	std::string filename_model   = "mdlSpecs.yaml";
	std::string filename_control = "ctrSpecs.yaml";

	// Create paths
	std::string pathSkinSim = getenv ("SKINSIM_PATH");
	std::string pathExp     = pathSkinSim + "/data/" + exp_name;
	std::string mdlSpecPath = pathExp + "/" + filename_model;
	std::string ctrSpecPath = pathExp + "/" + filename_control;
	std::string caliPath    = pathExp + "/tactile_calibration.yaml";
	std::string plungerPositionPath = pathExp + "/plunger_position.yaml";
	std::string plungerPositionPathTemp = pathExp + "/tmp_plunger_position.yaml";

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

	// Read YAML calibration file
	std::ifstream fin3(caliPath.c_str());
	YAML::Node doc_cali;
	std::cout<<"Loading file: "<<caliPath<<"\n";
	doc_cali = YAML::LoadAll(fin3);

	// For saving plunger positions
	std::ofstream fout4(plungerPositionPath.c_str());
	YAML::Emitter out;
	out << YAML::BeginMap;

	// Gazebo parameters
	std::string _worldFilename("~");
	bool _paused = true;
	std::string _physics = "ode";

	BuildModelSpec modelSpec;
	ControllerSpec controlSpec;

	int index = 1;
	int N = doc_model[0].size();// * doc_control[0].size();

	// Loop over control settings
	for(unsigned j=0;j<doc_control[0].size();j++)
	{
		doc_control[0][j] >> controlSpec;
		//print(controlSpec);

		// Loop over models
		//for(unsigned i=0;i<doc_model[0].size();i++)
		{
			doc_model[0][j] >> modelSpec;
			//print(modelSpec);

			// Load calibration constant
			double K_cali = 1.0;
			if( doc_cali[0][modelSpec.name])
				K_cali=doc_cali[0][modelSpec.name].as<double>();

			// Generate experiment name
			int skin_size    = modelSpec.spec.num_elements_x * modelSpec.spec.num_elements_y;
			int tactile_size = modelSpec.spec.tactile_elements_x;// * modelSpec.spec.tactile_elements_x;
			int tactile_sep  = modelSpec.spec.tactile_separation_x;// * modelSpec.spec.tactile_separation_y;

			std::ostringstream ss;
			ss << std::setw(2) << std::setfill('0') << index
			<< "_FB_" << controlSpec.feedback_type << "_Kp_" << std::setprecision(1) <<std::fixed << controlSpec.Kp
			<< "_tSize_" << tactile_size << "_tSep_" << tactile_sep;

			std::string exp_name = "exp_" + ss.str()  ;

			// Print info
			std::cout << RED << "\n"<<std::string(70,'#')<<"\n" << RESET;
			std::cout << "# Experiment: "<< exp_name << " ("<< index << " out of " << N << ")\n";
			//std::cout << "  Calibration value: "<<K_cali<<"\n";

			// Point model world file location
			_worldFilename = pathSkinSim + "/model/worlds/" + modelSpec.name + ".world";

			// Compute number of iterations
			iterations_max = modelSpec.spec.max_sim_time/modelSpec.spec.step_size;
			m_gazeboParams["iterations"] =  boost::lexical_cast<std::string>( iterations_max );
			//	for (std::map<std::string,std::string>::iterator it=m_gazeboParams.begin(); it!=m_gazeboParams.end(); ++it)
			//	    std::cout << it->first << " => " << it->second << '\n';

			std::cout << "Initializing World in 3 seconds ..." << std::endl;
			sleep(3.0);	// TODO make sure everything is ready

			// Create, load, and run the server in its own thread
			this->serverThread = new boost::thread(
					boost::bind(&SkinSimTestingFramework::RunServer, this, _worldFilename,
							_paused, _physics));

			std::cout << "Waiting for world completion" << std::endl;
			// Wait for the server to come up (20 second timeout)
			int waitCount = 0, maxWaitCount = 200;
			while ((!this->server || !this->server->GetInitialized()) && (++waitCount < maxWaitCount) )		// TODO error handling if timeout occurs
				common::Time::MSleep(100);

			std::cout << "Gazebo server loaded in "<<(double)waitCount*0.1<<" seconds (timeout after "<<(double)maxWaitCount*0.1<<" seconds)\n";

			// Initialize Gazebo transport node (to get simulation time)
			this->node = transport::NodePtr(new transport::Node());
			this->node->Init();
			this->statsSub = this->node->Subscribe("~/world_stats", &SkinSimTestingFramework::OnStats, this);
			//std::cout << "Initialized Transport Node" << std::endl;
			// TODO subscribe to ~/physics/contacts

			// Make sure the ROS node for Gazebo has already been initialized
			if (!ros::isInitialized())
			{
				std::cerr<<"Error: ROS has not been initialized! \n";
				exit(1);
			}

			// Create ROS service client
			ros_namespace_ = "skinsim";
			this->ros_node_ = new ros::NodeHandle(this->ros_namespace_);
			ros::ServiceClient ros_srv_ = this->ros_node_->serviceClient<skinsim_ros_msgs::SetController>("set_controller");
			ros::ServiceClient ros_srv_p_ = this->ros_node_->serviceClient<skinsim_ros_msgs::GetPosition>("get_plunger_position");

			// Set plunger force message
			msg_srv_.request.type.selected = controlSpec.controller_type;
			msg_srv_.request.fb.selected   = controlSpec.feedback_type;
			msg_srv_.request.f_des = controlSpec.Fd;
			msg_srv_.request.x_des = 0.0;
			msg_srv_.request.v_des = -0.005;
			msg_srv_.request.Kp    = controlSpec.Kp;
			msg_srv_.request.Ki    = controlSpec.Ki;
			msg_srv_.request.Kd    = controlSpec.Kd;
			msg_srv_.request.Kv    = controlSpec.Kv;
			msg_srv_.request.Ts    = controlSpec.Ts;
			msg_srv_.request.Nf    = controlSpec.Nf;
			msg_srv_.request.K_cali = K_cali;
			std::cout<<YELLOW<<"Control message:\n"<<msg_srv_.request<<RESET;

			// Start simulation
			std::cout<<"Starting simulation...\n";
			this->SetPause(false);

			// Save ROS topic data to file (plunger data)
			std::string topic = modelSpec.spec.topic;
			std::string cmd1 = "rostopic echo -p " + topic  + " > " + pathExp + "/" + exp_name + ".csv &";
			std::cout<<"$ "<<cmd1.c_str()<<"\n";
			system( cmd1.c_str() );

			// Save ROS topic data to file (COP data)
			std::string topic2 = "/skinsim/center_of_pressure";
			std::string cmd2 = "rostopic echo -p " + topic2 + " > " + pathExp + "/COP_" + exp_name + ".csv &";
			std::cout<<"$ "<<cmd2.c_str()<<"\n";
			system( cmd2.c_str() );

			// Send control message
			bool message_sent = false;
			while( !message_sent )
			{
				// Send control message to plunger
				if(!message_sent)
				{
					if (ros_srv_.call(msg_srv_))
					{
						message_sent = true;
						std::cout<<"Control message sent!\n";
					}
					//else
					//	std::cerr<<"Failed to send control message, trying again ...\n";
				}
				common::Time::MSleep(10);
			}
			message_sent = false;
			while( !message_sent )
			{
				// Send control message to plunger
				if(!message_sent)
				{
					if (ros_srv_p_.call(msg_srv_pos_))
					{
						message_sent = true;
						std::cout<<"Plunger position: (" << msg_srv_pos_.response.x <<"," << msg_srv_pos_.response.y<< ")\n";
						out << YAML::Key << modelSpec.name << YAML::Value;
						out << YAML::Flow<< YAML::BeginSeq << msg_srv_pos_.response.x << msg_srv_pos_.response.y << YAML::EndSeq;
						// Temporary save results
						YAML::Emitter out_tmp;
						out_tmp << out.c_str() << YAML::EndMap;
						std::ofstream fout_tmp(plungerPositionPathTemp.c_str());
						fout_tmp << out_tmp.c_str();
						fout_tmp.close();
					}
				}
				common::Time::MSleep(10);
			}

			// Wait for simulation to end
			waitCount = 0;
			while( physics::worlds_running() )
			{
				common::Time::MSleep(100);
				waitCount++;

				// Print status
				if(waitCount > 10)
					std::cout << '\r' << GREEN << "Iterations: "<<iterations<<" / "<< iterations_max << std::flush;
			}

			// Simulation finished
			std::cout << RESET;
			std::cout << "\nSimulation completed in "<<(double)waitCount*0.1<<" seconds.\n";
			std::cout << "Simulation run time was " << simTime.Double() << " seconds.\n";

			// Stop saving ROS topic data to file
			std::string cmdA = "pkill -9 -f " + topic;
			std::cout<<"$ "<<cmdA.c_str()<<"\n";
			system( cmdA.c_str() );

			std::string cmdB = "pkill -9 -f " + topic2;
			std::cout<<"$ "<<cmdB.c_str()<<"\n";
			system( cmdB.c_str() );

			// Cleanup
			Unload();
			delete ros_node_;

			index++;
		}
	}

	// Write to YAML file and close
	std::cout<<"Saving: "<<plungerPositionPath<<"\n";
	out << YAML::EndMap;
	fout4 << out.c_str();
	fout4.close();

	delete this->server;

}


void runCalibration(std::string exp_name)
{
	// Parameters
	std::string filename_model   = "mdlSpecs.yaml";

	// Create paths
	std::string pathSkinSim = getenv ("SKINSIM_PATH");
	std::string pathExp     = pathSkinSim + "/data/" + exp_name;
	std::string mdlSpecPath = pathExp + "/" + filename_model;
	std::string calibration_file = pathExp + "/tactile_calibration.yaml";
	std::string calibration_file_tmp = pathExp + "/tmp_tactile_calibration.yaml";

	// Read YAML model file
	std::ifstream fin(mdlSpecPath.c_str());
	YAML::Node doc_model;
	std::cout<<"Loading file: "<<mdlSpecPath<<"\n";
	doc_model = YAML::LoadAll(fin);

	// For saving calibration constants
	std::ofstream fout(calibration_file.c_str());
	YAML::Emitter out;
	out << YAML::BeginMap;

	// Gazebo parameters
	std::string _worldFilename("~");
	bool _paused = true;
	std::string _physics = "ode";

	BuildModelSpec modelSpec;

	int index = 1;
	int N = doc_model[0].size();

	// Loop over models
	for(unsigned i=0;i<doc_model[0].size();i++)
	{
		doc_model[0][i] >> modelSpec;
		//print(modelSpec);

		// Print info
		std::cout << RED << "\n"<<std::string(70,'#')<<"\n" << RESET;
		std::cout << "# Model: "<< modelSpec.name << " ("<< index << " out of " << N << ")\n";

		// Point model world file location
		_worldFilename = pathSkinSim + "/model/worlds/" + modelSpec.name + ".world";

		// Compute number of iterations
		iterations_max = modelSpec.spec.max_sim_time/modelSpec.spec.step_size;
		m_gazeboParams["iterations"] =  boost::lexical_cast<std::string>( iterations_max );

		// Create Gazebo world
		std::cout << "Initializing World in 2 seconds ..." << std::endl;
		sleep(2.0);	// TODO make sure everything is ready

		// Create, load, and run the server in its own thread
		this->serverThread = new boost::thread(
				boost::bind(&SkinSimTestingFramework::RunServer, this, _worldFilename,
						_paused, _physics));

		std::cout << "Waiting for world completion" << std::endl;
		// Wait for the server to come up (20 second timeout)
		int waitCount = 0, maxWaitCount = 200;
		while ((!this->server || !this->server->GetInitialized()) && (++waitCount < maxWaitCount) )		// TODO error handling if timeout occurs
			common::Time::MSleep(100);

		std::cout << "Gazebo server loaded in "<<(double)waitCount*0.1<<" seconds (timeout after "<<(double)maxWaitCount*0.1<<" seconds)\n";

		// Initialize Gazebo transport node (to get simulation time)
		this->node = transport::NodePtr(new transport::Node());
		this->node->Init();
		this->statsSub = this->node->Subscribe("~/world_stats", &SkinSimTestingFramework::OnStats, this);

		// Calibration
		buffer_size = 20;
		buffer = new double[buffer_size];
		for(int i=0;i<buffer_size;i++)
			buffer[i]=1.0;
		buffer_idx = 0;
		K_sma = 1.0;
		f_target = 2.0;
		calibration_done = false;
		calibration_counter=0;

		// Make sure the ROS node for Gazebo has already been initialized
		if (!ros::isInitialized())
		{
			std::cerr<<"Error: ROS has not been initialized! \n";
			exit(1);
		}

		// Create ROS service client
		ros_namespace_ = "skinsim";
		this->ros_node_ = new ros::NodeHandle(this->ros_namespace_);
		ros::ServiceClient ros_srv_ = this->ros_node_->serviceClient<skinsim_ros_msgs::SetController>("set_controller");

		// Create ROS topic subscriber
		std::string topic = "/skinsim/force_feedback";
		ros::Subscriber ros_sub_ = this->ros_node_->subscribe(topic.c_str(), 1, &SkinSimTestingFramework::subscriberCB, this);

		// Set plunger force message
		msg_srv_.request.type.selected = skinsim_ros_msgs::ControllerType::DIGITAL_PID;
		msg_srv_.request.fb.selected   = skinsim_ros_msgs::FeedbackType::TACTILE_APPLIED;
		msg_srv_.request.f_des = f_target;
		msg_srv_.request.x_des = 0.0;
		msg_srv_.request.v_des = -0.005;
		msg_srv_.request.Kp = 2.0   ;
		msg_srv_.request.Ki = 20.0  ;
		msg_srv_.request.Kd = 0.0	  ;
		msg_srv_.request.Kv = 0.0   ;
		msg_srv_.request.Ts = 0.001;
		msg_srv_.request.Nf = 100;
		msg_srv_.request.K_cali = 1.0;
		std::cout<<YELLOW<<"Control message:\n"<<msg_srv_.request<<RESET;

		// Start simulation
		std::cout<<"Starting simulation...\n";
		this->SetPause(false);

		// Send control message
		bool message_sent = false;
		while( !message_sent )
		{
			// Send control message to plunger
			if(!message_sent)
			{
				if (ros_srv_.call(msg_srv_))
				{
					message_sent = true;
					std::cout<<"Control message sent!\n";
				}
				//else
				//	std::cerr<<"Failed to send control message, trying again ...\n";
			}
			common::Time::MSleep(10);
		}

		// Wait for simulation to end
		waitCount = 0;
		int fastCount = 0;
		std::ostringstream ss;
		while( physics::worlds_running() && !calibration_done)
		{
			if(iterations < iterations_max*0.7)
			{
				common::Time::MSleep(100);
				waitCount++;

				// Get calibration data
				ros::spinOnce();

				// Print status
				if(waitCount > 10)
				{
					ss.str(""); ss.clear();
					ss << std::fixed << std::setprecision(6) << K_sma  ;
					std::cout << '\r' << GREEN << "Iterations: "<<iterations<<" / "<< iterations_max << "   (K = "<<ss.str()<<", cc="<<calibration_counter<<"/10) "<< std::flush;
				}
			}
			else
			{
				//common::Time::MSleep(1);
				fastCount++;

				// Get calibration data
				ros::spinOnce();

				// Print status
				if((fastCount % 10000) == 0)
				{
					ss.str(""); ss.clear();
					ss << std::fixed << std::setprecision(6) << K_sma;
					std::cout << '\r' << YELLOW << "Iterations: "<<iterations<<" / "<< iterations_max << "   (K = "<<ss.str()<<", cc="<<calibration_counter<<"/10) "<< std::flush;
				}
			}
		}
		if(physics::worlds_running())
			physics::stop_worlds();

		// Compute calibration constant
		out << YAML::Key << modelSpec.name << YAML::Value << K_sma;

		// Temporary save results
		YAML::Emitter out_tmp;
		out_tmp << out.c_str() << YAML::EndMap;		// TODO how to set out_tmp = out?
		std::ofstream fout_tmp(calibration_file_tmp.c_str());
		fout_tmp << out_tmp.c_str();
		fout_tmp.close();

		// Simulation finished
		std::cout << RESET;
		std::cout << "\nSimulation completed in "<<(double)waitCount*0.1<<" seconds.\n";	// FIXME use timer instead
		std::cout << "Simulation run time was " << simTime.Double() << " seconds.\n";

		// Cleanup
		Unload();
		delete ros_node_;
		delete[] buffer;

		index++;
	}

	delete this->server;

	// Write to YAML file and close
	std::cout<<"Saving: "<<calibration_file<<"\n";
	out << YAML::EndMap;
	fout << out.c_str();	// TODO save before in case of crash
	fout.close();

}


};

// Color for terminal text
const char SkinSimTestingFramework::RED[]    = "\033[0;31m";
const char SkinSimTestingFramework::GREEN[]  = "\033[0;32m";
const char SkinSimTestingFramework::YELLOW[] = "\033[0;33m";
const char SkinSimTestingFramework::RESET[]  = "\033[0m";
