/********* ************************************************************
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
 * plunger.h
 *
 *  Created on: Aug 6, 2016
 *      Author: Sven Cremer
 */

#ifndef PLUNGER_H_
#define PLUNGER_H_

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/SensorTypes.hh>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// ROS messages
#include <geometry_msgs/WrenchStamped.h>
#include <skinsim_ros_msgs/SetController.h>
#include <skinsim_ros_msgs/ControllerType.h>
#include <skinsim_ros_msgs/FeedbackType.h>
#include <skinsim_ros_msgs/ForceFeedback.h>

// Utilities
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace gazebo
{

/// \brief Plunger plugin
/// The model plugin simulates each skin joint and publishes the data to ROS
class Plunger : public ModelPlugin
{
  /// \brief Constructor
  public: Plunger();

  /// \brief Destructor
  public: virtual ~Plunger();

  /// \brief Load the plugin
  public: void Load( physics::ModelPtr _model, sdf::ElementPtr _sdf );

  /// \brief Update the joints and publish data
  protected: virtual void UpdateJoints();

  /// \brief Vector containing pointers to all joints inside the model
  private: physics::JointPtr joint_;

  /// \brief Vector containing the joint names
  private: std::string joint_name_;

  /// \brief A pointer to the Gazebo world
  private: physics::WorldPtr world_;

  /// \brief A pointer to the Gazebo model
  private: physics::ModelPtr model_;

  /// \brief Stores the Gazebo model name
  private: std::string model_name_;

  /// \brief A pointer to the ROS node handle
  private: ros::NodeHandle* ros_node_;

  /// \brief A ROS service server
   private: ros::ServiceServer ros_srv_;

   /// \brief Service callback function
   private: bool serviceCB(skinsim_ros_msgs::SetController::Request& req, skinsim_ros_msgs::SetController::Response& res);

   /// \brief A custom ROS message
   private: skinsim_ros_msgs::ControllerType controller_type_;

   /// \brief A custom ROS message
   private: skinsim_ros_msgs::FeedbackType feedback_type_;

   /// \brief A ROS subscriber for the force feedback
   private: ros::Subscriber ros_sub_fb_;

   /// \brief A callback for the force feedback subscriber
   private: void ForceFeedbackCB(const skinsim_ros_msgs::ForceFeedback::ConstPtr& _msg);

   /// \brief A custom ROS message for the force feedback
   skinsim_ros_msgs::ForceFeedback msg_fb_;

  /// \brief A ROS publisher
  private: ros::Publisher ros_pub_;

  /// \brief A toggle for publishing to ROS
  private: bool pub_to_ros_;

  /// \brief A custom ROS message
  private: geometry_msgs::WrenchStamped msg_wrench_;

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

  /// \brief Explicit force controller parameters
  private: double Kp_;		// Proportional force gain
  private: double Kd_;		// Derivative force gain
  private: double Kv_;		// Velocity gain for active damping

  /// \brief Force command
  private: double effort_;

  /// \brief For storing joint state
  private: physics::JointWrench wrench_;
  private: math::Vector3 force_;
  private: math::Vector3 torque_;
  private: double position_current_;
  private: double position_desired_;
  private: double velocity_current_;
  private: double velocity_desired_;
  private: double force_current_;
  private: double force_prev_;
  private: double force_desired_;

  /// \brief Joint axis of rotation in the global frame.
  private: math::Vector3 axis_global_;

  /// \brief Joint axis of rotation in the local frame.
  private: math::Vector3 axis_local_;

  /// \brief Joint PID controller
  private: physics::JointControllerPtr joint_pid_;
  private: common::PID pid_;

  /// \brief Connection that maintains a link between the contact sensor's
  /// updated signal and the OnContactUpdate callback.
  private: event::ConnectionPtr update_contact_connection_;

  /// \brief Callback that receives the contact sensor's update signal.
  private: virtual void OnContactUpdate();

  /// \brief Pointer to the contact sensor
  private: sensors::ContactSensorPtr contact_sensor_ptr_;

  /// \brief Number of contacts
  private: int num_contacts_;

};

}

#endif /* PLUNGER_H_ */
