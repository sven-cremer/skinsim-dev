#include "gazebo/common/CommonIface.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  class Mass_Joint : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      
      this->model = _model;
      this->joint = this->model->GetJoint("my_mass_joint");
      
      action_t = 3.0;
      a = 1;
      force = 0.0;
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&Mass_Joint::OnUpdate, this));
      
    }

    public: void OnUpdate()
    {
      // Apply a small force to the model.
      
      double current_time = this->model->GetWorld()->GetSimTime().Double();

      double dX = this->joint->GetAngle(0).Radian(); // (rest_angle - current_angle) - rest angle is 0
      double s_force = -20*dX;

      this->joint->SetForce(0, s_force);

//      this->joint->SetForce(0, force);
//
//      double current_force = this->joint->GetForce(0);
//
//      //std::cout<< "current force: "<<current_force<<"\n";
//
//      if(current_time > action_t)
//      {
//        switch(a)
//        {
//          case 1:
//            force = -2.00;
//            break;
//          case 2:
//            force = -1.22;
//            break;
//          case 3:
//            force = -0.50;
//            break;
//          case 4:
//            force = -2.00;
//            break;
//          default:
//            force = -1.50;
//            a = 1;
//          }
//        a++;
//        action_t = action_t + 5;
//      }

    }
    physics::JointPtr joint;
    physics::ModelPtr model;  
    private: event::ConnectionPtr updateConnection;
    double action_t;
    double force;
    int a;

  };


  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Mass_Joint)
}
