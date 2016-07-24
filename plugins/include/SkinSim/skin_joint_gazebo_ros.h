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
 * skin_joint_gazebo_ros.h
 *
 *  Created on: Jul 5, 2016
 *      Author: Sven Cremer
 */

#ifndef SKIN_JOINT_GAZEBO_ROS_H_
#define SKIN_JOINT_GAZEBO_ROS_H_

// Gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// ROS messages
#include <geometry_msgs/WrenchStamped.h>
#include <skinsim_ros_msgs/SkinJointDataArray.h>

// Utilities
#include <yaml-cpp/yaml.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <SkinSim/ModelPath.hh>

namespace gazebo
{

/// \brief SkinJointGazeboRos plugin
/// The model plugin simulates each skin joint and publishes the data to ROS
class SkinJointGazeboRos : public ModelPlugin
{
  /// \brief Constructor
  public: SkinJointGazeboRos();

  /// \brief Destructor
  public: virtual ~SkinJointGazeboRos();

  /// \brief Load the plugin
  public: void Load( physics::ModelPtr _model, sdf::ElementPtr _sdf );

  /// \brief Update the joints and publish data
  protected: virtual void UpdateJoints();

  /// \brief Vector containing pointers to all joints inside the model
  private: physics::Joint_V joints_;

  /// \brief Vector containing the joint names
  private: std::vector<std::string> joint_names_;

  /// \brief Number of joints inside the model
  private: int num_joints_;

  /// \brief A pointer to the Gazebo world
  private: physics::WorldPtr world_;

  /// \brief A pointer to the Gazebo model
  private: physics::ModelPtr model_;

  /// \brief Stores the Gazebo model name
  private: std::string model_name_;

  /// \brief A pointer to the ROS node handle
  private: ros::NodeHandle* ros_node_;

  /// \brief A ROS publisher
  private: ros::Publisher ros_pub_;

  /// \brief A toggle for publishing to ROS
  private: bool pub_to_ros_;

  /// \brief A custom ROS message
  private: skinsim_ros_msgs::SkinJointDataArray skin_msg_;

  /// \brief Stores the ROS topic name
  private: std::string topic_name_;

  /// \brief Stores the ROS namespace
  private: std::string ros_namespace_;

  /// \brief Count connections to ROS publisher
  private: int ros_connections_;

  /// \brief: Callback function when subscriber connects
  private: void RosConnect();

  /// \brief Callback function when subscriber disconnects
  private: void RosDisconnect();

  /// \brief Callback queue thread that processes messages
  private: ros::CallbackQueue ros_queue_;
  private: void RosQueueThread();
  private: boost::thread callback_ros_queue_thread_;

  /// \brief A mutex to lock access to fields that are used in message callbacks
  private: boost::mutex lock_;

  /// \brief A pointer to the update event connection
  private: event::ConnectionPtr update_connection_;

  /// \brief For saving last time
  private: common::Time last_time_;

  //  \brief Update rate
  private: double update_rate_;

  /// \brief MSD parameters
  private: double mass_;
  private: double sping_;
  private: double damper_;

  /// \brief For loading YAML parameters
  YAML::Node    doc_;
  std::ifstream input_file_;

};

}

#endif /* SKIN_JOINT_GAZEBO_ROS_H_ */
