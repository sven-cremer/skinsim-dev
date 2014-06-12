#include <fstream>
#include <string>
#include <sstream>

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>

#include "skinsim_msgs/tactileData.h"
#include "skinsim_msgs/conciseData.h"
#include <yaml-cpp/yaml.h>

/*uint64_t GetTimeStamp()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}*/

namespace gazebo
{
class Skin_Joint : public ModelPlugin
{

public:

  Skin_Joint()
  {
    // Start up ROS
    std::string name = "skin_joint_plugin_node";
    int argc = 0;
    //Initialize ROS. This allows ROS to do name remapping through the command line -- not important for now. This is also where we specify the name of our node. Node names must be unique in a running system. 
    ros::init(argc, NULL, name);
  }

  ~Skin_Joint()
  {
    delete this->ros_node;
  }

  void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
  {

    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    // ROS Nodehandle
    this->ros_node = new ros::NodeHandle("~");

    std::string para_file_name   = "/file_name";

    std::string para_skin_spring = "/skin_spring";
    std::string para_skin_dir_spring = "/skin_dir_spring";
    std::string para_skin_damper = "/skin_damper";

    std::string para_x_size = "/x_size";
    std::string para_y_size = "/y_size";

    std::string para_x_dt   = "/x_dt";
    std::string para_y_dt   = "/y_dt";

    std::string para_skin_max = "/skin_max";

    std::string file_name; // = "/home/isura/joint_name.yaml";

    if (!this->ros_node->getParam(para_file_name, file_name)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_file_name.c_str()); }

    if (!this->ros_node->getParam(para_skin_spring, skin_spring)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_skin_spring.c_str()); }
    if (!this->ros_node->getParam(para_skin_dir_spring, skin_dir_spring)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_skin_dir_spring.c_str()); }
    if (!this->ros_node->getParam(para_skin_damper, skin_damper)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_skin_damper.c_str()); }


    if (!this->ros_node->getParam(para_x_size , x_size)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_x_size.c_str()); }
    if (!this->ros_node->getParam(para_y_size , y_size)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_y_size.c_str()); }

    if (!this->ros_node->getParam(para_x_dt   , x_dt  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_x_dt  .c_str()); }
    if (!this->ros_node->getParam(para_y_dt   , y_dt  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_y_dt  .c_str()); }

    if (!this->ros_node->getParam(para_skin_max , skin_max )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_skin_max  .c_str()); }


    fin.open(file_name.c_str());

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

    std::string scalar;
    for (unsigned i = 0; i < doc.size(); i++)
    {
      doc[i]["Joint"] >> scalar;
      this->jointNames.push_back("spring_" + scalar);
      //std::cout << "Here's the output YAML:\n---" << scalar << "---\n";
    }

    fin.close();

    this->model_ = _model;

    // get pointers to joints from gazebo
    this->joints.resize(this->jointNames.size());
    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      this->joints[i] = this->model_->GetJoint(this->jointNames[i]);
      if (!this->joints[i])
      {
        ROS_ERROR("SkinSim robot expected joint[%s] not present, plugin not loaded", this->jointNames[i].c_str());
        return;
      }
    }

    // Create a new transport node
    transport::NodePtr node(new transport::Node());

    this->image_pub = this->ros_node->advertise<sensor_msgs::Image>("skin_image", 1);
    this->tacData_pub = this->ros_node->advertise<skinsim_msgs::tactileData>("tacData",1);
    this->conc_pub = this->ros_node->advertise<skinsim_msgs::conciseData>("concData",1);
    this->frame_name = "base_link";

    // Initialize the node with the Model name
    node->Init(model_->GetName());

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Skin_Joint::UpdateJoint, this));
  }

  // Called by the world update start event
public:
  void OnUpdate()
  {
    ros::spinOnce();
  }

public:
  void UpdateJoint()
  {
    double rest_angle = 0;

    double current_angle    = 0;
    double current_force    = 0;
    double current_velocity = 0;
    double sens_force = 0;

    skinsim_msgs::tactileData tacData;
    skinsim_msgs::conciseData concData;
    double current_time = this->model_->GetWorld()->GetSimTime().Double();
    math::Vector3 vect;

    tacData.time  = this->model_->GetWorld()->GetSimTime().Double();
    tacData.patchID = 1;

    concData.time = this->model_->GetWorld()->GetSimTime().Double();

    sensor_msgs::Image image_msg;
    image_msg.header.frame_id = this->frame_name;
    image_msg.header.stamp.sec  = ros::Time::now().sec;
    image_msg.header.stamp.nsec = ros::Time::now().nsec;
    image_msg.encoding = sensor_msgs::image_encodings::MONO8;
    image_msg.height   = 2*y_size / y_dt + 1 ;
    image_msg.width    = 2*x_size / x_dt + 1 ;
    image_msg.step     = image_msg.width     ;

    double tactile_data ;
    double tact_sum = 0;
    double tact_displacement = 0;
    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      current_angle = this->joints[i]->GetAngle(0).Radian();
      
      current_velocity = this->joints[i]->GetVelocity(0);

      // This sets the mass-spring-damper dynamics, currently only spring and damper
      this->joints[i]->SetForce(0, (rest_angle - current_angle) * skin_spring - skin_damper * current_velocity);
      
      // TODO test if this is better
      //this->joints[i]->SetStiffnessDamping(0, skin_spring, skin_damper, rest_angle );

      current_force = this->joints[i]->GetForce(0); 	
      sens_force = (rest_angle - current_angle) * skin_dir_spring - skin_damper * current_velocity;

      tacData.sensorID.push_back( i ) ;
      tacData.force.push_back( current_force ) ;

      tactile_data = current_force*255 ;
      tactile_data = tactile_data*skin_max ;

      if( tactile_data > 255 ) { tactile_data = 255; }

      if( tactile_data < 0   ) { tactile_data = 0  ; }

      image_msg.data.push_back( tactile_data ); //this->joints[i]->GetForce(0)

      tact_sum = tact_sum + sens_force;
      tact_displacement = tact_displacement + current_angle;

    }
    tacData.total_force = tact_sum; 
    concData.total_force = tact_sum;
    concData.displacement = tact_displacement/joints.size();
    image_pub.publish(image_msg);
    tacData_pub.publish(tacData);
    conc_pub.publish(concData);  
  }


private:

  // ROS Nodehandle
  ros::NodeHandle* ros_node;

  /// \brief keep a list of hard coded joint names.
  std::vector<std::string> jointNames;

  //physics::JointPtr joint_;

  /// \brief Internal list of pointers to Joints
  physics::Joint_V joints;

  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;


  std::string frame_name;

  // ROS Publisher
  ros::Publisher image_pub;
  ros::Publisher tacData_pub;
  ros::Publisher conc_pub;

  YAML::Parser parser;
  YAML::Node doc;
  std::ifstream fin;

  // Parameters
  double skin_spring ;
  double skin_dir_spring;
  double skin_damper ;

  double x_size ;
  double y_size ;

  double x_dt   ;
  double y_dt   ;

  double skin_max ;

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Skin_Joint)

}
