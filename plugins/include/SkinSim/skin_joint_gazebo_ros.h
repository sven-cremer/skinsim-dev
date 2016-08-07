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
#include <skinsim_ros_msgs/Joint1DArray.h>
#include <skinsim_ros_msgs/GetLayout.h>
#include <skinsim_ros_msgs/PointArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// Utilities
#include <yaml-cpp/yaml.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <SkinSim/ModelPath.hh>
#include <Eigen/Core>

struct Distances
{
	std::vector<int> index;
	std::vector<double> distance;
};
struct Tactile
{
	int index;
	math::Vector3 position;					// Center of sensor in skin_array frame
	std::vector<std::string> joint_names;
	std::vector<int> joint_index;
	double force_sensed;
	double force_applied;
};

bool indexSort(int a, int b, std::vector<double> data)
{
    return data[a]<data[b];
}

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

  /// \brief A pointer to the contact manager of the physics engine
  private: physics::ContactManager* contact_mgr_;

  /// \brief Vector containing the collision names
  private: std::vector<std::string> collision_names_;

  /// \brief Vector containing the collision names
  private: std::vector<bool> collision_index_;

  private: Eigen::VectorXd f_app_;
  private: Eigen::VectorXd f_sen_;
  private: Eigen::VectorXd f_app_prev_;
  private: Eigen::VectorXd f_sen_prev_;

  /// \brief A pointer to the ROS node handle
  private: ros::NodeHandle* ros_node_;

  /// \brief A ROS service server
  private: ros::ServiceServer ros_srv_;

  /// \brief Service callback function that triggers the layout publisher
  /// The layout data could have been returned in the service response,
  /// however it is more convenient to save a rostopic into a CSV file.
  private: bool serviceCB(skinsim_ros_msgs::GetLayout::Request& req, skinsim_ros_msgs::GetLayout::Response& res);

  /// \brief A ROS publisher for the joint data
  private: ros::Publisher ros_pub_joint_;

  /// \brief A ROS publisher for the layout data
  private: ros::Publisher ros_pub_layout_;

  /// \brief A ROS publisher for RVIZ
  private: ros::Publisher ros_pub_rviz_;

  /// \brief A toggle for publishing to ROS
  private: bool pub_to_ros_;

  /// \brief A custom ROS message for the joint data
  private: skinsim_ros_msgs::Joint1DArray joint_msg_;

  /// \brief A custom ROS message for the layout data
  private: skinsim_ros_msgs::PointArray layout_msg_;

  /// \brief A custom ROS message for the layout data
  private: visualization_msgs::MarkerArray msg_rviz_;

  /// \brief Stores the ROS topic name
  private: std::string topic_name_;

  /// \brief Stores the ROS namespace
  private: std::string ros_namespace_;

  /// \brief Count connections to ROS publisher
  private: int ros_connections_;

  /// \brief: Empty callback function
  private: void emptyCB();

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

  /// \brief Distances between each element
  private: std::vector<Distances> layout;

  /// \brief Tactile sensor data
  private: std::vector<Tactile> sensors_;

  /// \brief Number of sensors inside the model
  private: int num_sensors_;

};

}

#endif /* SKIN_JOINT_GAZEBO_ROS_H_ */
