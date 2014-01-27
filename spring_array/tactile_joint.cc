#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <string>

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace gazebo
{
class Tactile_Joint : public ModelPlugin
{

public:

  Tactile_Joint()
  {
    // Start up ROS
    std::string name = "tactile_joint_plugin_node";
    int argc = 0;
    ros::init(argc, NULL, name);
  }

  ~Tactile_Joint()
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

    std::string para_file_name = "/file_name";

    std::string para_tactile_spring          = "/tactile_spring";
    std::string para_tactile_damper          = "/tactile_damper";
    std::string para_tactile_capacitor_area  = "/tactile_capacitor_area";
    std::string para_tactile_capacitor_depth = "/tactile_capacitor_depth";
    std::string para_tactile_permittivity    = "/tactile_permittivity";

    std::string para_x_size = "/x_size";
    std::string para_y_size = "/y_size";

    std::string para_x_dt   = "/x_dt";
    std::string para_y_dt   = "/y_dt";

    std::string para_tactile_max = "/tactile_max";

    std::string file_name; // = "/home/isura/joint_name.yaml";

    if (!this->ros_node->getParam(para_file_name, file_name)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_file_name.c_str()); }

    if (!this->ros_node->getParam(para_tactile_spring         , tactile_spring         )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_tactile_spring         .c_str()); }
    if (!this->ros_node->getParam(para_tactile_damper         , tactile_damper         )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_tactile_damper         .c_str()); }
    if (!this->ros_node->getParam(para_tactile_capacitor_area , tactile_capacitor_area )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_tactile_capacitor_area .c_str()); }
    if (!this->ros_node->getParam(para_tactile_capacitor_depth, tactile_capacitor_depth)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_tactile_capacitor_depth.c_str()); }
    if (!this->ros_node->getParam(para_tactile_permittivity   , tactile_permittivity   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_tactile_permittivity   .c_str()); }

    if (!this->ros_node->getParam(para_x_size , x_size)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_x_size.c_str()); }
    if (!this->ros_node->getParam(para_y_size , y_size)){ ROS_ERROR("Value not loaded from parameter: %s !)", para_y_size.c_str()); }

    if (!this->ros_node->getParam(para_x_dt   , x_dt  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_x_dt  .c_str()); }
    if (!this->ros_node->getParam(para_y_dt   , y_dt  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_y_dt  .c_str()); }

    if (!this->ros_node->getParam(para_tactile_max , tactile_max )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_tactile_max  .c_str()); }

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
      this->jointNames.push_back("tactile_" + scalar);
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

    // Create a publisher on the ~/factory topic
    pub = node->Advertise<msgs::Vector3d>("~/force_sensor_info");

    this->image_pub = this->ros_node->advertise<sensor_msgs::Image>("tactile_image", 1);
    this->frame_name = "base_link";

    // Initialize the node with the Model name
    node->Init(model_->GetName());

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Tactile_Joint::UpdateJoint, this));
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
    double current_voltage  = 0;


    double current_time = this->model_->GetWorld()->GetSimTime().Double();
    math::Vector3 vect;

    sensor_msgs::Image image_msg;

    image_msg.header.frame_id = this->frame_name;
    image_msg.header.stamp.sec  = ros::Time::now().sec;
    image_msg.header.stamp.nsec = ros::Time::now().nsec;
    image_msg.encoding = sensor_msgs::image_encodings::MONO8;
    image_msg.height   = 2*y_size / y_dt + 1 ;
    image_msg.width    = 2*x_size / x_dt + 1 ;
    image_msg.step     = image_msg.width     ;

    double tactile_data;

    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {

      current_angle = this->joints[i]->GetAngle(0).Radian();
      current_force = this->joints[i]->GetForce(0);
      current_velocity = this->joints[i]->GetVelocity(0);

      current_voltage = sqrt(2*current_force*pow((tactile_capacitor_depth + current_angle),2)/(tactile_permittivity*tactile_capacitor_area));

      // This sets the mass-spring-damper dynamics, currently only spring and damper
      this->joints[i]->SetForce(0, (rest_angle - current_angle) * tactile_spring - tactile_damper * current_velocity);
      vect.x = current_time;
      vect.y = i;
      vect.z = current_force;

//      msgs::Set(&msg, vect);
//      pub->Publish(msg);

      tactile_data = current_force*255 ;
      tactile_data = tactile_data*tactile_max ;

      if( tactile_data > 255 ) { tactile_data = 255; }

      if( tactile_data < 0   ) { tactile_data = 0  ; }

      image_msg.data.push_back( tactile_data ); //this->joints[i]->GetForce(0)

    }

      image_pub.publish(image_msg);
  }


  // ROS Nodehandle
private:
  ros::NodeHandle* ros_node;
  /// \brief keep a list of hard coded joint names.
  std::vector<std::string> jointNames;
  //physics::JointPtr joint_;

  /// \brief Internal list of pointers to Joints
  physics::Joint_V joints;

  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;

  msgs::Vector3d msg;
  transport::PublisherPtr pub;

  // ROS Subscriber
  ros::Subscriber sub;

  std::string frame_name;

  // ROS Publisher
  ros::Publisher image_pub;

  YAML::Parser parser;
  YAML::Node doc;
  std::ifstream fin;

  // Parameters
  double tactile_spring          ;
  double tactile_damper          ;
  double tactile_capacitor_area  ;
  double tactile_capacitor_depth ;
  double tactile_permittivity    ;

  double x_size ;
  double y_size ;

  double x_dt   ;
  double y_dt   ;

  double tactile_max ;

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Tactile_Joint)

}
