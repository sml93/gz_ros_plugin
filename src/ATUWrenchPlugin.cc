#include <iostream>
#include <cmath>
#include <math.h>
#include <thread>
#include <functional>

#include "ros/ros.h"
#include <Eigen/Dense>
#include "ros/console.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Wrench.h"

#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

#define deg2rad (M_PI / 180.0)


namespace gazebo
{
  class ATUWrenchPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {

        this->model = _parent;  

      // Listen to the update event. This event is broadcast every simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ATUWrenchPlugin::onUpdate, this));

        std::string atu_wrench_topic = "/atu_wrench"; 

      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "atu_wrench_plugin_node",
          ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to the Gazebo node.
      this->rosNode.reset(new ros::NodeHandle("atu_wrench_plugin_node"));

      wrench.resize(6);
      wrench.setZero();

      // Angle
      ros::SubscribeOptions so = 
        ros::SubscribeOptions::create<geometry_msgs::Wrench>(
          atu_wrench_topic, 1,
          boost::bind(&ATUWrenchPlugin::onWrenchMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

    }


    public: void onWrenchMsg(const geometry_msgs::Wrench::ConstPtr &_msg)
    
    {

        wrench(0) = _msg->force.x;
        wrench(1) = _msg->force.y;
        wrench(2) = _msg->force.z;
        wrench(3) = _msg->torque.x;
        wrench(4) = _msg->torque.y;
        wrench(5) = _msg->torque.z; 

    }

    public: void onUpdate() 
    
    {

        this->model->GetLink("base_link")->SetForce(ignition::math::Vector3d(wrench(0), wrench(1), wrench(2)));
        this->model->GetLink("base_link")->SetTorque(ignition::math::Vector3d(wrench(3), wrench(4), wrench(5)));

    }

    private: 

        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;

        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::Subscriber rosSub;
        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;
        Eigen::VectorXd wrench;


  };

    GZ_REGISTER_MODEL_PLUGIN(ATUWrenchPlugin)

}