#include "gazebo/common/CommonIface.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/gazebo.hh"

#include "skinsim_msgs/tactileData.h"
#include "skinsim_msgs/controllerData.h"

#include "ros/ros.h"

namespace gazebo
{
  class PlaneJoint : public ModelPlugin
  {

    void tactileCallback(const skinsim_msgs::tactileData::ConstPtr& msg)
    {
      m_lock.lock();

        sens_force = 0;
        grnd_force = 0;
        for (unsigned int i = 0; i < msg->force.size(); ++i)
        {
          //sens_force = sens_force + msg->force_noisy[i];
          grnd_force = grnd_force + msg->force[i];
        }

      m_lock.unlock();
    }

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      // ROS Nodehandle
      this->ros_node = new ros::NodeHandle("~");

      std::string para_explFctr_Kp = "/plane_explFctr_Kp";
      std::string para_explFctr_Ki = "/plane_explFctr_Ki";
      std::string para_explFctr_Kd = "/plane_explFctr_Kd";
      if (!this->ros_node->getParam(para_explFctr_Kp, explFctr_Kp)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_explFctr_Kp.c_str()); }
      if (!this->ros_node->getParam(para_explFctr_Ki, explFctr_Ki)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_explFctr_Ki.c_str()); }
      if (!this->ros_node->getParam(para_explFctr_Kd, explFctr_Kd)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_explFctr_Kd.c_str()); }

      std::string para_impCtr_Xnom = "/plane_impCtr_Xnom";
      std::string para_impCtr_K    = "/plane_impCtr_K"   ;
      std::string para_impCtr_D    = "/plane_impCtr_D"   ;
      if (!this->ros_node->getParam(para_impCtr_Xnom, impCtr_Xnom)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_impCtr_Xnom.c_str()); }
      if (!this->ros_node->getParam(para_impCtr_K   , impCtr_K   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_impCtr_K   .c_str()); }
      if (!this->ros_node->getParam(para_impCtr_D   , impCtr_D   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_impCtr_D   .c_str()); }

      std::string para_ctrType    = "/plane_ctrType"   ;
      if (!this->ros_node->getParam(para_ctrType, ctrType)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_ctrType.c_str()); }

      std::string para_targetForce    = "/plane_targetForce"   ;
      if (!this->ros_node->getParam(para_targetForce, targetForce)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_targetForce.c_str()); }

      this->tactile_sub = this->ros_node->subscribe( "tacData", 1, &PlaneJoint::tactileCallback, this );
      this->force_pub = this->ros_node->advertise<skinsim_msgs::controllerData>("controller_data", 1);

      this->model_ = _model;
      this->joint_ = this->model_->GetJoint( "plane_joint" );

      this->update_connection_ = event::Events::ConnectWorldUpdateBegin( boost::bind(&PlaneJoint::UpdateJoint, this));

      sens_force = 0;
      grnd_force = 0;
      int_err    = 0;
      a_prev     = 0;

      prev_time  = this->model_->GetWorld()->GetSimTime().Double();

    }

    public: void UpdateJoint()
    {

      m_lock.lock();
        double sensed_force = sens_force ;
        double ground_force = grnd_force ;
      m_lock.unlock();


      if( this->model_->GetWorld()->GetSimTime().Double() > 1.0 )
      {

        // Explicit force controller
        if( ctrType == 0 )
        {
          double delT = this->model_->GetWorld()->GetSimTime().Double() - prev_time ;

          prev_time = this->model_->GetWorld()->GetSimTime().Double();

          double a = targetForce - ground_force;
          double der_err = ( a - a_prev )/0.001;
          a_prev   = a ;
          int_err = int_err + a;

  //        this->joint_->SetForce( 0, a*explFctr_Kp );
  //        this->joint_->SetForce( 0, a*explFctr_Kp + der_err*explFctr_Kd );
          // TODO need antiwindup
          this->joint_->SetForce( 0, a*explFctr_Kp + int_err*explFctr_Ki + der_err*explFctr_Kd);
  //        this->joint_->SetForce( 0, a*explFctr_Kp + int_err*explFctr_Ki );
          current_force = this->joint_->GetForce(0);
        }

        // Impedance control
        if( ctrType == 1 )
        {
          double rest_angle       = impCtr_Xnom ;
          double stiffness        = impCtr_K    ;
          double damp_coefficient = impCtr_D    ;

          double current_angle = this->joint_->GetAngle(0).Radian();

          double current_velocity = this->joint_->GetVelocity(0);
          double current_time = this->model_->GetWorld()->GetSimTime().Double();
          this->joint_->SetForce(0, ( rest_angle - current_angle)*stiffness - damp_coefficient*current_velocity );
          //current_force = this->joint_->GetForce(0);
          
        }

        //std::cout << name<<" Current force: "<<current_force<<"\n";
      }

      ctrData.time         = this->model_->GetWorld()->GetSimTime().Double();
      //ctrData.force_sensed = sensed_force;
      ctrData.force        = ground_force;
      ctrData.joint_force  = current_force;
      ctrData.explFctr_Kp  = explFctr_Kp;
      ctrData.explFctr_Ki  = explFctr_Ki;
      ctrData.explFctr_Kd  = explFctr_Kd;

      ctrData.impCtr_Xnom  = impCtr_Xnom;
      ctrData.impCtr_K     = impCtr_K   ;
      ctrData.impCtr_D     = impCtr_D   ;

      ctrData.ctrType      = ctrType    ;
      ctrData.targetForce  = targetForce;

      force_pub.publish( ctrData );

    }

    physics::JointPtr joint_;
    physics::ModelPtr model_;  
    event::ConnectionPtr update_connection_;

    // ROS
    ros::NodeHandle* ros_node;

    // ROS Subscriber
    ros::Subscriber tactile_sub;

    // ROS Publisher
    ros::Publisher  force_pub;

    // Force sensed
    double sens_force ; // sensed force
    double grnd_force ; // ground truth force
    double current_force;
    double int_err    ;
    skinsim_msgs::controllerData ctrData;

    // Time
    double prev_time ;
    double a_prev;
    boost::mutex m_lock ;

    // Explicit force controller
    double explFctr_Kp ;
    double explFctr_Ki ;
    double explFctr_Kd ;

    // Impedance controller
    double impCtr_Xnom ;
    double impCtr_K    ;
    double impCtr_D    ;

    // Controller type selection
    // 0 - explicit force control
    // 1 - impedance control
    int ctrType ;

    double targetForce ;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PlaneJoint)
}
