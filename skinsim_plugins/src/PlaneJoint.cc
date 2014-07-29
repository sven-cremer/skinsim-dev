#include "gazebo/common/CommonIface.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/gazebo.hh"
#include <gazebo/msgs/msgs.hh>

#include <boost/filesystem.hpp>

#include <SkinSim/ControlSpecYAML.hh>

#include "tactileData.pb.h"

namespace gazebo
{
  typedef const boost::shared_ptr<const skinsim_msgs::msgs::TactileData> TactileDataPtr;
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

//    void tactileCallback( TactileDataPtr &msg)
    void tactileCallback( VectorThreePtr &msg)
    {
      m_lock.lock();

        sens_force = 0;
        grnd_force = 0;
//        for (unsigned int i = 0; i < msg->force.size(); ++i)
//        {
          sens_force = msg->y(); // msg->force_noisy();
          grnd_force = msg->z(); // msg->force();
//        }

      m_lock.unlock();
    }

    public:

    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      // Get SkinSim path
      pathString = getenv ("SKINSIM_PATH");

      // Read YAML files
      std::string configFilePath = pathString + "/skinsim_model/config/ctr_config.yaml";
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
      ctrType     = ctrSpecs.ctrType     ;
      targetForce = ctrSpecs.targetForce ;


      this->model_ = _model;
      this->joint_ = this->model_->GetJoint( "plane_joint" );

      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(model_->GetName());
      this->tactileSub = this->node->Subscribe("~/tacData", &PlaneJoint::tactileCallback, this);


      this->update_connection_ = event::Events::ConnectWorldUpdateBegin( boost::bind(&PlaneJoint::UpdateJoint, this));

      sens_force = 0;
      grnd_force = 0;
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
        double sensed_force = sens_force ;
        double ground_force = grnd_force ;
      m_lock.unlock();


      // Get current time
      m_currentTime = this->model_->GetWorld()->GetSimTime().Double();

      joint_force = this->joint_->GetForce(0);

      if( m_currentTime > 1.0 )
      {
        // Explicit force controller
        if( ctrType == 0 )
        {
          double delT = m_currentTime - m_prevTime ;

          m_prevTime = m_currentTime;

          double a = targetForce - ground_force;
          double der_err = ( a - a_prev )/0.001;
          a_prev   = a ;
          int_err = int_err + a;


          //        this->joint_->SetForce( 0, a*explFctr_Kp );
  //        this->joint_->SetForce( 0, a*explFctr_Kp + der_err*explFctr_Kd );
          // TODO need antiwindup
          this->joint_->SetForce( 0, a*explFctr_Kp + int_err*explFctr_Ki + der_err*explFctr_Kd);
  //        this->joint_->SetForce( 0, a*explFctr_Kp + int_err*explFctr_Ki );
        }

        // Impedance control
        if( ctrType == 1 )
        {
          double rest_angle       = impCtr_Xnom ;
          double stiffness        = impCtr_K    ;
          double damp_coefficient = impCtr_D    ;

          double current_angle = this->joint_->GetAngle(0).Radian();

          double current_velocity = this->joint_->GetVelocity(0);

          this->joint_->SetForce(0, ( rest_angle - current_angle)*stiffness - damp_coefficient*current_velocity );
          //current_force = this->joint_->GetForce(0);
          
        }

        //std::cout << name<<" Current force: "<<current_force<<"\n";
      }

      ctrData.time         = m_currentTime;
      ctrData.force_sensed = sensed_force ;
      ctrData.force        = ground_force ;
      ctrData.joint_force  = joint_force  ;
      ctrData.targetForce  = targetForce  ;

      // Save exp data
      writeCtrDataToFile( ctrData );

    }

    physics::JointPtr joint_;
    physics::ModelPtr model_;  
    event::ConnectionPtr update_connection_;

    // Force sensed
    double sens_force ; // sensed force
    double grnd_force ; // ground truth force
    double joint_force;
    double int_err    ;

    ControllerData ctrData;

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
    int ctrType ;

    double targetForce ;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PlaneJoint)
}
