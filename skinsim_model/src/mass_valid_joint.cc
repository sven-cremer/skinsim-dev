#include <string>
#include <fstream>

#include "ros/ros.h"

#include "gazebo/common/CommonIface.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/gazebo.hh"

#include "skinsim_msgs/inputData.h"

#include <yaml-cpp/yaml.h>


namespace gazebo
{
  class MassValidJoint : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      this->ros_node = new ros::NodeHandle("~");
      this->model = _model;
      this->joint = this->model->GetJoint("my_mass_joint");
      this->input_pub = this->ros_node->advertise<skinsim_msgs::inputData>("inputData",1);
      std::string para_ttl_file   = "/ttl_file";
      count = 1;
      std::string ttl_file; 
      if (!this->ros_node->getParam(para_ttl_file, ttl_file)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_ttl_file.c_str()); }
      fin.open(ttl_file.c_str());

      if (fin)
      {
        ROS_INFO("Load Success!");
      }
      else
      {
        ROS_ERROR("Load Fail!");
      }

      parser.Load(fin);
      parser.GetNextDocument(doc);

      double f_value;
      for (unsigned i = 0; i < doc.size(); i++)
      {
        doc[i]["ttl"] >> f_value;
        this->ttlValues.push_back(f_value);
        //std::cout << "Here's the output YAML:\n---" << scalar << "---\n";
      }
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MassValidJoint::OnUpdate, this));
      
    }

    public: void OnUpdate()
    {
      // Apply a small force to the model.
      skinsim_msgs::inputData inputData;
      double current_time = this->model->GetWorld()->GetSimTime().Double();
      if(count<this->ttlValues.size())
      {
        force = -ttlValues[count];
      }
      else
      {
        force = -0.05;
      }
      this->joint->SetForce(0, force);

      double current_force = this->joint->GetForce(0);

      std::cout<<count<< " ttl_force: "<<this->ttlValues[count]<<"\n";
      count++;
      inputData.input_force = current_force;
      inputData.time = current_time;


      input_pub.publish(inputData); 
    }
private:
    physics::JointPtr joint;
    physics::ModelPtr model;  
    event::ConnectionPtr updateConnection;
    ros::NodeHandle* ros_node;
    double force;
    int count;
    ros::Publisher input_pub;
    std::vector<double> ttlValues;
    
    YAML::Parser parser;
    YAML::Node doc;
    std::ifstream fin;

  };


  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MassValidJoint)
}
