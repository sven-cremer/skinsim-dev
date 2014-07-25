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

#include <fstream>
#include <string>
#include <vector>
#include <map>

#include <sdf/sdf.hh>

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

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

using namespace gazebo;

using namespace SkinSim;
using namespace std;

class SkinSimTestingFramework
{

protected: Server *server;
protected: boost::thread *serverThread;

protected: common::Time simTime, realTime, pauseTime;
private: double percentRealTime;
private: bool paused;
private: bool serverRunning;
protected: transport::NodePtr node;
protected: transport::SubscriberPtr statsSub;

public:
  SkinSimTestingFramework()
  {
    this->server = NULL;

    this->node = transport::NodePtr(new transport::Node());
    this->node->Init();
    this->statsSub = this->node->Subscribe("~/world_stats", &SkinSimTestingFramework::OnStats, this);
  }
 
  /////////////////////////////////////////////////
  void OnStats(ConstWorldStatisticsPtr &_msg)
  {
    this->simTime = msgs::Convert(_msg->sim_time());
    this->realTime = msgs::Convert(_msg->real_time());
    this->pauseTime = msgs::Convert(_msg->pause_time());
    this->paused = _msg->paused();

//    if (this->realTime == 0)
//      this->percentRealTime = 0;
//    else
//      this->percentRealTime =
//        (this->simTime / this->realTime).Double();

    this->serverRunning = true;
  }
  
  /////////////////////////////////////////////////
  void SetPause(bool _pause)
  {
    physics::pause_worlds(_pause);
  }
  
  /////////////////////////////////////////////////
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

  /////////////////////////////////////////////////
  void RunServer(const std::string &_worldFilename, bool _paused, const std::string &_physics)
  {
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

    this->SetPause(_paused);

    this->server->Run();

    this->server->Fini();

    delete this->server;
    this->server = NULL;
  }
  
  void runTests()
  {
    // Initialize ROS
  //  ros::init (argc, argv, "skinmodel_automated_tester");
  //  ros::NodeHandle nh;

  //  std::string para_model_name   = "/model_name";
  //  std::string para_sdf_dir_name = "/sdf_dir_name";
  //  std::string para_sdf_filename = "/sdf_filename";
  //  std::string para_jcf_filename = "/joint_config_filename";
  //  std::string para_tid_filename = "/tactile_id_filename";
  //
  //  std::string para_xByX         = "xByX";
  //  std::string para_density      = "density";
  //  std::string para_size_x       = "size_x" ;
  //  std::string para_size_y       = "size_y" ;
  //
  //  std::string para_skin_height  = "skin_height";
  //  std::string para_plane_height = "plane_height";
  //  std::string para_d_pos        = "d_pos"  ;
  //
  //  std::string para_sens_rad     = "sens_rad";
  //  std::string para_space_wid    = "space_wid" ;

    std::string model_name        = "spring_array";

    std::string pathString( getenv ("SKINSIM_PATH") );
    std::string sdf_dir_name      = pathString + "/skinsim_model";
    std::string sdf_filename      = pathString + "/skinsim_model";

    double xByX         = 0.0 ;

    double density      = 0.0 ;
    double size_x       = 1.5 ;
    double size_y       = 1.5 ;

    double skin_height  = 1.3 ;
    double plane_height = 0.4 ;
    double d_pos        = 0.5 ;

    double sens_rad     = 1.0 ;
    double space_wid    = 3.0 ;

  //  if (!nh.getParam( para_model_name   , model_name   )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_model_name.c_str())  ; }
  //  if (!nh.getParam( para_sdf_dir_name , sdf_dir_name )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_sdf_dir_name.c_str()); }
  //  if (!nh.getParam( para_sdf_filename , sdf_filename )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_sdf_filename.c_str()); }
  //
  //  if (!nh.getParam( para_xByX   , xByX    )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_xByX.c_str())   ; }
  //  if (!nh.getParam( para_d_pos  , d_pos   )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_d_pos  .c_str()); }
  //  if (!nh.getParam( para_density, density )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_density.c_str()); }

    // FIXME this assumes square patches
    if( xByX != 0.0 )
    {
      size_x = d_pos*xByX ;
      size_y = d_pos*xByX ;
    }else
    {
  //    if (!nh.getParam(para_size_x , size_x  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_size_x .c_str()); }
  //    if (!nh.getParam(para_size_y , size_y  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_size_y .c_str()); }
    }

  //  if (!nh.getParam(para_skin_height, skin_height )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_skin_height.c_str()); }
  //  if (!nh.getParam(para_plane_height, plane_height )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_plane_height .c_str()); }
  //
  //  if (!nh.getParam(para_sens_rad , sens_rad )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_sens_rad.c_str()); }
  //  if (!nh.getParam(para_space_wid , space_wid  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_space_wid.c_str()); }

    int expNumber = 3;
    std::string _worldFilename("~");
    bool _paused = false;
    std::string _physics = "ode";

    for( int i = 1; i <= expNumber; ++i )
    {
      
      std::cout << std::endl << "Experiment number: " << i << std::endl;
      
      // Create model files
      stringstream ss;
      ss << i;
      std::string expStr = ss.str();

      SkinSimModelBuilder skinSimModelBuilderObject( model_name + expStr ,
                                                     sdf_filename        ,
                                                     xByX                ,
                                                     density             ,
                                                     size_x              ,
                                                     size_y              ,
                                                     skin_height         ,
                                                     plane_height        ,
                                                     d_pos               ,
                                                     sens_rad            ,
                                                     space_wid            );

      //  Load("worlds/box_plane_low_friction_test.world", true);
      //  physics::WorldPtr world = physics::get_world("default");
      //  world->Step(5000);

      delete this->server;
      this->server = NULL;

      // Point to newly created world file location
      _worldFilename = pathString + "/skinsim_model/worlds/" + model_name + expStr + ".world";

      // Create, load, and run the server in its own thread
      this->serverThread = new boost::thread(
         boost::bind(&SkinSimTestingFramework::RunServer, this, _worldFilename,
                     _paused, _physics));

      // Wait for the server to come up
      // Use a 60 second timeout.
      int waitCount = 0, maxWaitCount = 6000;
      while ((!this->server || !this->server->GetInitialized()) &&
             ++waitCount < maxWaitCount)
        common::Time::MSleep(100);
      gzdbg << "ServerFixture load in "
             << static_cast<double>(waitCount)/10.0
             << " seconds, timeout after "
             << static_cast<double>(maxWaitCount)/10.0
             << " seconds\n";
      
      common::Time::MSleep(2000);

      std::cout << std::endl << "Time: " << simTime.sec << " sec " << simTime.nsec << " nsec " << std::endl;


      Unload();

    }

  }

};



int main(int argc, char** argv)
{

  SkinSimTestingFramework skinSimTestingFrameworkObject;
  skinSimTestingFrameworkObject.runTests();

  return 0;

}



