#include "gazebo/common/CommonIface.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/PID.hh"
#include "gazebo/gazebo.hh"


//https://github.com/gazebosim/gazebo/blob/master/examples/plugins/pr2_pose_test.cc

double sens_force;

namespace gazebo
{
  class Plane_Joint : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      
      this->model_ = _model;
      this->joint_ = this->model_->GetJoint("plane_joint");
      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Plane_Joint::UpdateJoint, this));

      disp_t = 0.01;
      std::fill(array, array + 35, 0);
      force_sensed = 0;
      sens_force = 0;
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init();
      this->sub = node->Subscribe("/gazebo/force_sensor_info", &Plane_Joint::cb, this);

    }
    public: void UpdateJoint()
    {
      double rest_angle = 0;
      double stiffness = 35;
      double damp_coefficient = 15;
      double current_angle = this->joint_->GetAngle(0).Radian();
      double current_force = this->joint_->GetForce(0);
      double current_velocity = this->joint_->GetVelocity(0);
      double current_time = this->model_->GetWorld()->GetSimTime().Double();
      double force = (rest_angle - current_angle)*stiffness-damp_coefficient*current_velocity;
      this->joint_->SetForce(0, force);
      std::cout<<force<<","<<sens_force<<"\n";
      //std::cout << name<<" Current force: "<<current_force<<"\n";
      
      
    }

    public: void cb(const boost::shared_ptr<const gazebo::msgs::Vector3d> &_msg)
    {
      int n;
      n = _msg->y();
      array[n]=_msg->z();
      if(_msg->x()>disp_t)
      {
        for(int i = 0;i<35; i++)
        {
          force_sensed = force_sensed+array[i];
        }
        //std::cout << "Sensed Force: "<<force_sensed<<"\n";
        disp_t = disp_t + 0.01;
        sens_force = force_sensed;
      }
    }
    physics::JointPtr joint_;
    physics::ModelPtr model_;  
    event::ConnectionPtr update_connection_;
    gazebo::transport::NodePtr node;
    gazebo::transport::SubscriberPtr sub;
    double disp_t;
    double array[35];
    double force_sensed;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Plane_Joint)
}
