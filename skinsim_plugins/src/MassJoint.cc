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
