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

#include "gazebo/common/CommonIface.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/gazebo.hh"
#include <gazebo/msgs/msgs.hh>

#include <boost/filesystem.hpp>

#include <SkinSim/ControlSpecYAML.hh>

#include <tactileData.pb.h>

namespace gazebo
{
  typedef const boost::shared_ptr<const msgs::msgs::TactileData> TactileDataPtr;
  typedef const boost::shared_ptr<const gazebo::msgs::Vector3d> VectorThreePtr;

  struct ControllerData
  {
    double time         ;
    double force_sensed ;
    double force        ;
    double joint_force  ;
    double targetForce  ;
  };

  class PlaneJoint : public ModelPlugin
  {

    std::ofstream  saveToFile;
    ControllerSpec ctrSpecs  ;

    transport::SubscriberPtr tactileSub;
    transport::NodePtr       node      ;

    std::string pathString;

    void cb(ConstWorldStatisticsPtr &_msg)
    {
      std::cout << _msg->DebugString();
    }

    void writeCtrDataToFile( ControllerData & ctrData )
    {
      saveToFile << ctrData.time         << ","
                 << ctrData.force_sensed << ","
                 << ctrData.force        << ","
                 << ctrData.joint_force  << ","
                 << ctrData.targetForce  << std::endl ;
    }

    ~PlaneJoint()
    {
      saveToFile.close();
      gazebo::transport::fini();
    }

    void tactileCallback( VectorThreePtr &msg)
    {
      m_lock.lock();
        sensed_force_ = 0;
        ground_force_ = 0;
        sensed_force_ = msg->y();
        ground_force_ = msg->z();
      m_lock.unlock();
    }

    public:

    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      // Get SkinSim path
      pathString = getenv ("SKINSIM_PATH");

      // Read YAML files
      std::string configFilePath = pathString + "/model/config/ctr_config.yaml";
      std::ifstream fin(configFilePath.c_str());

      YAML::Parser parser(fin);
      YAML::Node doc;
      parser.GetNextDocument(doc);
      doc[0] >> ctrSpecs;

      fin.close();

      explFctr_Kp = ctrSpecs.explFctr_Kp ;
      explFctr_Ki = ctrSpecs.explFctr_Ki ;
      explFctr_Kd = ctrSpecs.explFctr_Kd ;
      impCtr_Xnom = ctrSpecs.impCtr_Xnom ;
      impCtr_M    = ctrSpecs.impCtr_M    ;
      impCtr_K    = ctrSpecs.impCtr_K    ;
      impCtr_D    = ctrSpecs.impCtr_D    ;
      controller_type_     = ctrSpecs.ctrType     ;
      target_force_ = ctrSpecs.targetForce ;


      this->model_ = _model;
      this->joint_ = this->model_->GetJoint( "plane_joint" );

      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(model_->GetName());
      this->tactileSub = this->node->Subscribe("~/tacData", &PlaneJoint::tactileCallback, this);


      this->update_connection_ = event::Events::ConnectWorldUpdateBegin( boost::bind(&PlaneJoint::UpdateJoint, this));

      sensed_force_ = 0;
      ground_force_ = 0;
      int_err    = 0;
      a_prev     = 0;

      m_prevTime    = this->model_->GetWorld()->GetSimTime().Double();
      m_currentTime = this->model_->GetWorld()->GetSimTime().Double();

      // TODO improve this
      // Experimental data collection
      std::string filename = pathString + "/data/" + ctrSpecs.name + ".dat";

      saveToFile.open ( filename.c_str() );

      saveToFile << "time,"
                 << "force_sensed,"
                 << "force,"
                 << "joint_force,"
                 << "targetForce," << std::endl ;

    }

    public: void UpdateJoint()
    {
      m_lock.lock();
        double sensed_force = sensed_force_ ;
        double ground_force = ground_force_ ;
      m_lock.unlock();


      // Get current time
      m_currentTime = this->model_->GetWorld()->GetSimTime().Double();

      joint_force_ = this->joint_->GetForce(0);

      if( m_currentTime > 1.0 )
      {
        // Explicit force controller
        if( controller_type_ == 0 )
        {
          double delT = m_currentTime - m_prevTime ;

          m_prevTime = m_currentTime;

          double a = target_force_ - ground_force;
          double der_err = ( a - a_prev )/0.001;
          a_prev   = a ;
          int_err = int_err + a;
          // TODO need antiwindup
          this->joint_->SetForce( 0, a*explFctr_Kp + int_err*explFctr_Ki + der_err*explFctr_Kd);
        }

        // Impedance control
        if( controller_type_ == 1 )
        {
          double rest_angle       = impCtr_Xnom ;
          double stiffness        = impCtr_K    ;
          double damp_coefficient = impCtr_D    ;

          double current_angle = this->joint_->GetAngle(0).Radian();

          double current_velocity = this->joint_->GetVelocity(0);

          this->joint_->SetForce(0, ( rest_angle - current_angle)*stiffness - damp_coefficient*current_velocity );
        }
      }

      controller_data_.time         = m_currentTime;
      controller_data_.force_sensed = sensed_force ;
      controller_data_.force        = ground_force ;
      controller_data_.joint_force  = joint_force_  ;
      controller_data_.targetForce  = target_force_  ;

      // Save exp data
      writeCtrDataToFile( controller_data_ );

    }

    physics::JointPtr joint_;
    physics::ModelPtr model_;  
    event::ConnectionPtr update_connection_;

    // Force sensed
    double sensed_force_; // sensed force
    double ground_force_; // ground truth force
    double joint_force_;
    double int_err;

    ControllerData controller_data_;

    // Time
    double m_currentTime;
    double m_prevTime ;
    double a_prev;
    boost::mutex m_lock ;

    // Explicit force controller
    double explFctr_Kp ;
    double explFctr_Ki ;
    double explFctr_Kd ;

    // Impedance controller
    double impCtr_Xnom ;
    double impCtr_M    ;
    double impCtr_K    ;
    double impCtr_D    ;

    // Controller type selection
    // 0 - explicit force control
    // 1 - impedance control
    int controller_type_ ;

    double target_force_ ;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PlaneJoint)
}
