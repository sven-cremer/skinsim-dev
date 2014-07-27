#include "gazebo/common/CommonIface.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  class MassJoint : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _model;
      this->joint = this->model->GetJoint("my_mass_joint");
      action_t = 0.5;
      a = 1;
      force = 0.0;
      this->updateConnection = event::Events::ConnectWorldUpdateBegin( boost::bind(&MassJoint::OnUpdate, this) );
      
    }

    public: void OnUpdate()
    {
      // Apply a small force to the model.
      double current_time = this->model->GetWorld()->GetSimTime().Double();

      this->joint->SetForce(0, force);

      double current_force = this->joint->GetForce(0);

      if(current_time > action_t)
      {
        switch(a)
        {
          case 1:
            force = -0.14;
            break;
          case 2:
            force = -1.80;
            break;
          case 3:
            force = -1.0;
            break;
          case 4:
            force = -3.25;
            break;
          default:
            force = -2.50;
            a = 0;
          }
        a++;
        action_t = action_t + 1;
      }
    }

  private:

    physics::JointPtr joint;
    physics::ModelPtr model;  
    event::ConnectionPtr updateConnection;

    double action_t;
    double force;
    int a;

  };


  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MassJoint)
}
