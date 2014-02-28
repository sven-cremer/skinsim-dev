#include "gazebo/common/CommonIface.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/gazebo.hh"

#include "spring_array/tactileData.h"
#include "spring_array/controllerData.h"

#include "ros/ros.h"

namespace gazebo
{
  class Plane_Joint : public ModelPlugin
  {

    void tactileCallback(const spring_array::tactileData::ConstPtr& msg)
    {
      m_lock.lock();

        sens_force = 0;
        grnd_force = 0;
        for (unsigned int i = 0; i < msg->force.size(); ++i)
        {
          sens_force = sens_force + msg->force_noisy[i];
          grnd_force = grnd_force + msg->force[i];
        }

      m_lock.unlock();
    }

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      // ROS Nodehandle
      this->ros_node = new ros::NodeHandle("~");

      std::string para_plane_spring = "/plane_spring";
      std::string para_plane_damper = "/plane_damper";

      if (!this->ros_node->getParam(para_plane_spring, plane_spring)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_plane_spring.c_str()); }
      if (!this->ros_node->getParam(para_plane_damper, plane_damper)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_plane_damper.c_str()); }

      std::string para_explFctr_Kp = "/plane_explFctr_Kp";
      std::string para_explFctr_Ki = "/plane_explFctr_Ki";
      std::string para_explFctr_Kd = "/plane_explFctr_Kd";

      if (!this->ros_node->getParam(para_explFctr_Kp, kp)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_explFctr_Kp.c_str()); }
      if (!this->ros_node->getParam(para_explFctr_Ki, ki)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_explFctr_Ki.c_str()); }
      if (!this->ros_node->getParam(para_explFctr_Kd, kd)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_explFctr_Kd.c_str()); }

      this->tactile_sub = this->ros_node->subscribe( "tactile_data", 1, &Plane_Joint::tactileCallback, this );
      this->force_pub = this->ros_node->advertise<spring_array::controllerData>("controller_data", 1);

      this->model_ = _model;
      this->joint_ = this->model_->GetJoint( "plane_joint" );

      this->update_connection_ = event::Events::ConnectWorldUpdateBegin( boost::bind(&Plane_Joint::UpdateJoint, this));

      sens_force = 0;
      grnd_force = 0;
      int_err    = 0;
      a_prev     = 0;

      prev_time  = this->model_->GetWorld()->GetSimTime().Double();

    }

    public: void UpdateJoint()
    {
      double target_force_ = 0.75;


      m_lock.lock();
        double sensed_force = sens_force ;
        double ground_force = grnd_force ;
      m_lock.unlock();


      if( this->model_->GetWorld()->GetSimTime().Double() > 1.0 )
      {

  //      // Impedance control
  //      double rest_angle = 0;
  //      double stiffness = 24;
  //      double damp_coefficient = 10;
  //      double current_angle = this->joint_->GetAngle(0).Radian();
  //      double current_force = this->joint_->GetForce(0);
  //      double current_velocity = this->joint_->GetVelocity(0);
  //      double current_time = this->model_->GetWorld()->GetSimTime().Double();
  //      this->joint_->SetForce(0, ( rest_angle - current_angle)*stiffness - damp_coefficient*current_velocity );

        // Explicit force controller
//        double kp = 0.40      ;
//        double ki = 0.0001    ;
//        double kd = 1*sqrt(kp);

        double delT = this->model_->GetWorld()->GetSimTime().Double() - prev_time ;

        prev_time = this->model_->GetWorld()->GetSimTime().Double();

        double a = target_force_ - ground_force;
        double der_err = ( a - a_prev )/0.001;
        a_prev   = a ;
        int_err = int_err + a;


//        this->joint_->SetForce( 0, a*kp );
//        this->joint_->SetForce( 0, a*kp + der_err*kd );
        // TODO need antiwindup
        this->joint_->SetForce( 0, a*kp + int_err*ki + der_err*kd);
//        this->joint_->SetForce( 0, a*kp + int_err*ki );

        //std::cout << name<<" Current force: "<<current_force<<"\n";
      }

      ctrData.time         = this->model_->GetWorld()->GetSimTime().Double();
      ctrData.force_sensed = sensed_force;
      ctrData.force        = ground_force;

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
    double int_err    ;
    spring_array::controllerData ctrData;

    // Parameters
    double plane_spring ;
    double plane_damper ;

    // Time
    double prev_time ;
    double a_prev;
    boost::mutex m_lock;

    // Explicit force controller
    double kp ;
    double ki ;
    double kd ;



  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Plane_Joint)
}
