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

/*
 * VisualizeContact.cc
 *
 *  Created on: Feb 8, 2016
 *      Author: sumit
 */


#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/SurfaceParams.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/util/LogRecord.hh>

#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <list>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>

#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/rendering/rendering.hh>

#include "SkinSim/ModelPath.hh"

namespace gazebo
{
	class VisualizeContactPlugin : public SensorPlugin
	{
	public:

		/// \brief Constructor.
		public: VisualizeContactPlugin();

		/// \brief Destructor.
		public: virtual ~VisualizeContactPlugin();

		/// \brief Load the sensor plugin.
		/// \param[in] _sensor Pointer to the sensor that loaded this plugin.
		/// \param[in] _sdf SDF element that describes the plugin.
		public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

		/// \brief Callback that receives the contact sensor's update signal.
		private: virtual void OnUpdate();

		/// \brief Pointer to the contact sensor
		private: sensors::ContactSensorPtr parentSensor;

		/// \brief Connection that maintains a link between the contact sensor's
		/// updated signal and the OnUpdate callback.
		private: event::ConnectionPtr updateConnection;

		private: transport::NodePtr node;


	};

	VisualizeContactPlugin::VisualizeContactPlugin() : SensorPlugin()
	{
	}

	VisualizeContactPlugin::~VisualizeContactPlugin()
	{
	}

	void VisualizeContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
	{
	  // Get the parent sensor.
	  this->parentSensor =
	    boost::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

	  // Make sure the parent sensor is valid.
	  if (!this->parentSensor)
	  {
	    gzerr << "ContactPlugin requires a ContactSensor.\n";
	    return;
	  }

	  // Connect to the sensor update event.
	  this->updateConnection = this->parentSensor->ConnectUpdated(
	      boost::bind(&VisualizeContactPlugin::OnUpdate, this));

	  // Make sure the parent sensor is active.
	  this->parentSensor->SetActive(true);
	}

	void VisualizeContactPlugin::OnUpdate()
	{
	  // Get all the contacts.
	  msgs::Contacts contacts;
	  contacts = this->parentSensor->GetContacts();

//	  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
//	  {

//	    std::cout << "Collision between[" << contacts.contact(i).collision1()
//	              << "] and [" << contacts.contact(i).collision2() << "]\n";

//	    for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
//	    {
//	      std::cout << j << "  Position:"
//	                << contacts.contact(i).position(j).x() << " "
//	                << contacts.contact(i).position(j).y() << " "
//	                << contacts.contact(i).position(j).z() << "\n";
//	      std::cout << "   Normal:"
//	                << contacts.contact(i).normal(j).x() << " "
//	                << contacts.contact(i).normal(j).y() << " "
//	                << contacts.contact(i).normal(j).z() << "\n";
//	      std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
//	    }
//	  }
	}

	// Register this plugin with the simulator
	GZ_REGISTER_SENSOR_PLUGIN (VisualizeContactPlugin)
}













