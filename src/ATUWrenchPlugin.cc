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

      force_msg.Set(0.0, 0.0, 0.0);
      torque_msg.Set(0.0, 0.0, 0.0);

      // Angle
      ros::SubscribeOptions so = 
        ros::SubscribeOptions::create<geometry_msgs::Wrench>(
          atu_wrench_topic, 1,
          boost::bind(&ATUWrenchPlugin::onWrenchMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread = 
        std::thread(std::bind(&ATUWrenchPlugin::QueueThread, this));

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

        force_msg.Set(wrench(0), wrench(1), wrench(2));
        torque_msg.Set(wrench(3), wrench(4), wrench(5));
        this->model->GetLink("base_link")->SetForce(force_msg);
        this->model->GetLink("base_link")->AddTorque(torque_msg);
        ROS_INFO("wrench_Fx = %f", force_msg.X());
        ROS_INFO("wrench_Fy = %f", force_msg.Y());
        ROS_INFO("wrench_Fz = %f", force_msg.Z());
        ROS_INFO("wrench_Mx = %f", torque_msg.X());
        ROS_INFO("wrench_My = %f", torque_msg.Y());
        ROS_INFO("wrench_Mz = %f", torque_msg.Z());

    }

    private: 

        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;

        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::Subscriber rosSub;
        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;
        Eigen::VectorXd wrench;
        ignition::math::Vector3d force_msg; 
        ignition::math::Vector3d torque_msg; 

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while(this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }


  };

  

    GZ_REGISTER_MODEL_PLUGIN(ATUWrenchPlugin)

}