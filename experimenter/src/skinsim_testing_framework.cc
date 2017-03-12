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
#include <skinsim_ros_msgs/GetLayout.h>
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

// Gazebo parameters
private: std::string _worldFilename;
private: bool _paused;
private: std::string _physics;

// Path and file names
private: std::string pathSkinSim;
private: std::string pathExp    ;
private: std::string mdlSpecPath;
private: std::string ctrSpecPath;
private: std::string caliPath   ;
private: std::string plungerPositionPath    ;
private: std::string plungerPositionPathTemp;
private: std::string filename_model  ;
private: std::string filename_control;
private: std::string calibration_file    ;
private: std::string calibration_file_tmp;

// YAML files
private: YAML::Node doc_model;
private: YAML::Node doc_control;
private: YAML::Node doc_cali;

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

	this->_worldFilename = "~";
	this->_paused        = true;
	this->_physics       = "ode";
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

void setPaths(std::string exp_name)
{
	this->filename_model   = "mdlSpecs.yaml";
	this->filename_control = "ctrSpecs.yaml";

	this->pathSkinSim = getenv ("SKINSIM_PATH");
	this->pathExp     = pathSkinSim + "/data/" + exp_name;
	this->mdlSpecPath = pathExp + "/" + filename_model;
	this->ctrSpecPath = pathExp + "/" + filename_control;
	this->caliPath    = pathExp + "/tactile_calibration.yaml";

	this->plungerPositionPath     = pathExp + "/plunger_position.yaml";
	this->plungerPositionPathTemp = pathExp + "/tmp_plunger_position.yaml";

	this->calibration_file        = pathExp + "/tactile_calibration.yaml";
	this->calibration_file_tmp    = pathExp + "/tmp_tactile_calibration.yaml";
}

void loadYAML(std::string fname, YAML::Node& doc_)
{
	std::cout<<"Loading file: "<<fname<<"\n";
	std::ifstream fin(fname.c_str());
	doc_.reset();
	doc_ = YAML::LoadAll(fin);
}

void saveYAML(std::string fname, YAML::Emitter& out_)
{
	std::cout<<"\nSaving file: "<<fname<<"\n";
	std::ofstream fout(fname.c_str());
	fout << out_.c_str();
	fout.close();
}

void saveLayout(std::string exp_name, std::string topic, skinsim_ros_msgs::GetLayout msg_layout_)
{
	std::string cmd1, cmd2;
	if(msg_layout_.request.selected == skinsim_ros_msgs::GetLayout::Request::ELEMENTS)
	{
		cmd1 = "rostopic echo -n 1 -p " + topic + " > " + pathExp + "/layout_elements_" + exp_name + ".csv & ";
		cmd2 = "rosservice call /skinsim/publish_layout \"selected: 0\"";
		std::cout<<"Saving element layout ...\n";
	}
	else
	{
		cmd1 = "rostopic echo -n 1 -p " + topic + " > " + pathExp + "/layout_sensors_" + exp_name + ".csv & ";
		cmd2 = "rosservice call /skinsim/publish_layout \"selected: 1\"";
		std::cout<<"Saving sensor layout ...\n";
	}

	system( cmd1.c_str() );
	system( cmd2.c_str() );		// For some reason this doesn't work using a rosservice
}

std::string getExpName(int index, BuildModelSpec modelSpec, ControllerSpec controlSpec)
{
	// Generate experiment name
	int skin_size    = modelSpec.spec.num_elements_x * modelSpec.spec.num_elements_y;
	int tactile_size = modelSpec.spec.tactile_elements_x;// * modelSpec.spec.tactile_elements_x;
	int tactile_sep  = modelSpec.spec.tactile_separation_x;// * modelSpec.spec.tactile_separation_y;

	std::ostringstream ss;
	ss << std::setw(2) << std::setfill('0') << index
	<< "_FB_" << controlSpec.feedback_type << "_Kp_" << std::setprecision(1) <<std::fixed << controlSpec.Kp
	<< "_tSize_" << tactile_size << "_tSep_" << tactile_sep;

	std::string exp_name = "exp_" + ss.str()  ;

	return exp_name;
}

void runTests(std::string exp_name, bool calibrate, bool saveAllData = false)
{
	//std::cout << "TestingFramework::runTests()" << std::endl;

	this->setPaths(exp_name);

	// Load YAML files
	this->loadYAML(mdlSpecPath, doc_model);
	this->loadYAML(ctrSpecPath, doc_control);

	if(doc_model[0].size() != doc_control[0].size())
	{
		std::cerr<<"Error: number of controllers and models do not match!\n";
		return;
	}
	int N = doc_model[0].size();

	// Load calibration values
	if(!calibrate)
		this->loadYAML(caliPath, doc_cali);

	// Saving YAML files
	YAML::Emitter out_cali_;
	YAML::Emitter out_plunger_;
	out_plunger_ << YAML::BeginMap;
	out_cali_    << YAML::BeginMap;

	// Loop over control settings and models
	BuildModelSpec modelSpec;
	ControllerSpec controlSpec;

	for(int idx=0; idx<N; idx++)
	{
		doc_control[0][idx] >> controlSpec;
		doc_model  [0][idx] >> modelSpec;

		//print(controlSpec);
		//print(modelSpec);

		// Load calibration constant
		double K_cali = 1.0;
		if(!calibrate)
		{
			if( doc_cali[0][modelSpec.name])
				K_cali=doc_cali[0][modelSpec.name].as<double>();
		}

		// Generate experiment name
		std::string exp_name = getExpName(idx+1, modelSpec, controlSpec);

		// Print info
		std::cout << RED << "\n"<<std::string(70,'#')<<"\n" << RESET;
		std::cout << "# Experiment: "<< exp_name << " ("<< idx+1 << " out of " << N << ")\n";

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
				boost::bind(&SkinSimTestingFramework::RunServer, this, _worldFilename, _paused, _physics));

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
			return;
		}

		// Create ROS service client
		ros_namespace_ = "skinsim";
		this->ros_node_ = new ros::NodeHandle(this->ros_namespace_);
		ros::ServiceClient ros_srv_   = this->ros_node_->serviceClient<skinsim_ros_msgs::SetController>("set_plunger_controller");
		ros::ServiceClient ros_srv_plunger_ = this->ros_node_->serviceClient<skinsim_ros_msgs::GetPosition>  ("get_plunger_position");
		//ros::ServiceClient ros_srv_layout_ = this->ros_node_->serviceClient<skinsim_ros_msgs::GetLayout>  ("publish_layout");

		// Create ROS topic subscriber for calibration
		ros::Subscriber ros_sub_;
		if(calibrate)
		{
			std::string topic = "/skinsim/force_feedback";
			ros_sub_ = this->ros_node_->subscribe(topic.c_str(), 1, &SkinSimTestingFramework::subscriberCB, this);
		}

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
		if(calibrate)
		{
			msg_srv_.request.type.selected = skinsim_ros_msgs::ControllerType::DIRECT;
			//msg_srv_.request.fb.selected   = skinsim_ros_msgs::FeedbackType::TACTILE_APPLIED;
			msg_srv_.request.f_des = 1.0;
		}
		std::cout<<YELLOW<<"Control message:\n"<<msg_srv_.request<<RESET;

		// Calibration
		if(calibrate)
		{
			buffer_size = 20;
			buffer = new double[buffer_size];
			for(int i=0;i<buffer_size;i++)
				buffer[i]=1.0;
			buffer_idx = 0;
			K_sma = 1.0;
			f_target = 2.0;
			calibration_done = false;
			calibration_counter=0;
		}

		// Start simulation
		std::cout<<"Starting simulation...\n";
		this->SetPause(false);

		// Save ROS topic data to file (plunger data)
		std::string topic = modelSpec.spec.topic;	// "/skinsim/plunger_data"
		std::string cmd1 = "rostopic echo -p " + topic  + " > " + pathExp + "/plunger_" + exp_name + ".csv &";
		std::cout<<"$ "<<cmd1.c_str()<<"\n";
		system( cmd1.c_str() );

		// Save ROS topic data to file (COP data)
		std::string topic2 = "/skinsim/center_of_pressure";
		std::string cmd2 = "rostopic echo -p " + topic2 + " > " + pathExp + "/cop_" + exp_name + ".csv &";
		std::cout<<"$ "<<cmd2.c_str()<<"\n";
		system( cmd2.c_str() );

		// Save joint and tactile data
		std::string topic4 = "/skinsim/joint_data";
		std::string topic5 = "/skinsim/tactile_data";
		if(saveAllData)
		{
			std::string cmd4 = "rostopic echo -p " + topic4 + " > " + pathExp + "/joint_" + exp_name + ".csv &";
			std::cout<<"$ "<<cmd4.c_str()<<"\n";
			system( cmd4.c_str() );

			std::string cmd5 = "rostopic echo -p " + topic5 + " > " + pathExp + "/tactile_" + exp_name + ".csv &";
			std::cout<<"$ "<<cmd5.c_str()<<"\n";
			system( cmd5.c_str() );
		}

		bool message_sent;

		// Send control message to plunger
		message_sent = false;
		while( !message_sent )
		{
			if (ros_srv_.call(msg_srv_))
			{
				message_sent = true;
				std::cout<<"Control message sent!\n";
			}
			//else
			//	std::cerr<<"Failed to send control message, trying again ...\n";
			common::Time::MSleep(10);
		}

		// Get layout
/*
		skinsim_ros_msgs::GetLayout msg_layout_;
		std::string topic3 = "/skinsim/layout";
		std::string cmd3 = "rostopic echo -p -n 1 " + topic3 + " > " + pathExp + "/layout_elements_" + exp_name + ".csv &";
		std::string cmd4 = "rostopic echo -p -n 1 " + topic3 + " > " + pathExp + "/layout_sensors_" + exp_name + ".csv &";

		msg_layout_.request.selected = skinsim_ros_msgs::GetLayout::Request::ELEMENTS;
		std::cout<<"$ "<<cmd3.c_str()<<"\n";
		system( cmd3.c_str() );
		message_sent = false;
		while( !message_sent )
		{
			if (ros_srv_layout_.call(msg_layout_))
			{
				message_sent = true;
				std::cout<< std::boolalpha<<"-> Layout message sent!\n"<<"              "<<(bool)msg_layout_.response.success<<"\n---\n";
			}
			common::Time::MSleep(10);
		}

		msg_layout_.request.selected = skinsim_ros_msgs::GetLayout::Request::SENSORS;
		std::cout<<"$ "<<cmd4.c_str()<<"\n";
		system( cmd4.c_str() );
		message_sent = false;
		while( !message_sent )
		{
			if (ros_srv_layout_.call(msg_layout_))
			{
				message_sent = true;
				std::cout<< std::boolalpha<<"-> Layout message sent!\n"<<"              "<<(int)msg_layout_.response.success<<"\n---\n";
			}
			common::Time::MSleep(10);
		}
*/
		skinsim_ros_msgs::GetLayout msg_layout_elements_;
		skinsim_ros_msgs::GetLayout msg_layout_sensors_;
		msg_layout_elements_.request.selected = skinsim_ros_msgs::GetLayout::Request::ELEMENTS;
		msg_layout_sensors_.request.selected = skinsim_ros_msgs::GetLayout::Request::SENSORS;
		std::string topic3 = "/skinsim/layout";
		bool layoutSave = false;

		saveLayout(exp_name, topic3, msg_layout_elements_);
		saveLayout(exp_name, topic3, msg_layout_sensors_);


		// Get plunger position
		message_sent = false;
		while( !message_sent )
		{
			if (ros_srv_plunger_.call(msg_srv_pos_))
			{
				message_sent = true;
				std::cout<<"Plunger position: (" << msg_srv_pos_.response.x <<"," << msg_srv_pos_.response.y<< ")\n";
				out_plunger_ << YAML::Key << modelSpec.name << YAML::Value;
				out_plunger_ << YAML::Flow<< YAML::BeginSeq << msg_srv_pos_.response.x << msg_srv_pos_.response.y << YAML::EndSeq;
				// Temporary save results
				YAML::Emitter out_tmp;
				out_tmp << out_plunger_.c_str() << YAML::EndMap;
				saveYAML(plungerPositionPathTemp, out_tmp);
			}
			common::Time::MSleep(10);
		}

		// Wait for simulation to end
		waitCount = 0;
		if(calibrate)
		{
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
		}
		else
		{
			while( physics::worlds_running() )
			{
				common::Time::MSleep(100);
				waitCount++;

				// Print status
				if(waitCount > 10)
					std::cout << '\r' << GREEN << "Iterations: "<<iterations<<" / "<< iterations_max << std::flush;
			}
		}

		// Make sure simulation is stopped
		if(physics::worlds_running())
			physics::stop_worlds();

		// Save calibration constant
		if(calibrate)
		{
			out_cali_ << YAML::Key << modelSpec.name << YAML::Value << K_sma;

			YAML::Emitter out_tmp;
			out_tmp << out_cali_.c_str() << YAML::EndMap;
			saveYAML(calibration_file_tmp, out_tmp);
		}

		// Simulation finished
		std::cout << RESET;
		std::cout << "\nSimulation completed in "<<(double)waitCount*0.1<<" seconds.\n";	// FIXME use timer instead
		std::cout << "Simulation run time was " << simTime.Double() << " seconds.\n";

		// Stop saving ROS topic data to file
		std::string cmdA = "pkill -9 -f " + topic;
		std::cout<<"$ "<<cmdA.c_str()<<"\n";
		system( cmdA.c_str() );

		std::string cmdB = "pkill -9 -f " + topic2;
		std::cout<<"$ "<<cmdB.c_str()<<"\n";
		system( cmdB.c_str() );

		std::string cmdC = "pkill -9 -f " + topic3;
		std::cout<<"$ "<<cmdC.c_str()<<"\n";
		system( cmdC.c_str() );

		if(saveAllData)
		{
			std::string cmdD = "pkill -9 -f " + topic4;
			std::cout<<"$ "<<cmdD.c_str()<<"\n";
			system( cmdD.c_str() );

			std::string cmdE = "pkill -9 -f " + topic5;
			std::cout<<"$ "<<cmdE.c_str()<<"\n";
			system( cmdE.c_str() );
		}

		// Cleanup
		Unload();
		delete ros_node_;
		if(calibrate)
			delete[] buffer;
	}

	// Save YAML files
	out_plunger_ << YAML::EndMap;
	saveYAML(plungerPositionPath, out_plunger_);

	if(calibrate)
	{
		out_cali_ << YAML::EndMap;
		saveYAML(calibration_file, out_cali_);
	}

	delete this->server;

}

};

// Color for terminal text
const char SkinSimTestingFramework::RED[]    = "\033[0;31m";
const char SkinSimTestingFramework::GREEN[]  = "\033[0;32m";
const char SkinSimTestingFramework::YELLOW[] = "\033[0;33m";
const char SkinSimTestingFramework::RESET[]  = "\033[0m";
