#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class ModelQuake : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectorWorldUpdateBegin(
        std::bind(&ModelQuake::OnUpdate, this));

      this->old_secs = ros::Time::now().toSec();
      
      // Create a topic name 
      std::string earthquake_freq_topicName = "/earthquake_freq";
      std::string earthquake_mag_topicName = "/earthquake_mag";

      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "earthquake_rosnode",
          ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to the Gazebo node.
      this->rosNode.reset(new ros::NodeHandle("earthquake_rosnode"));

      // Freq
      ros::SubscribeOptions so = 
        ros::SubscribeOptions::create<std_msgs::Float32>(
          earthquake_freq_topicName, 1,
          boost::bind(&ModelQuake::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);
               
      // Spin up the queue helper thread.
      this->rosQueueThread = 
        std::thread(std::bind(&ModelQuake::QueueThread, this));

      //Magnitude
      ros::SubscribeOptions so2 = 
        ros::SubscribeOptions::create<std_msgs::Float32>(
          earthquake_mag_topicName,
          1,
          boost::bind(&ModelQuake::OnRosMsg_Mag, this, _1),
          ros::VoidPtr(), &this->rosQueue2);
      this->rosSub2 = this->rosNode->subscribe(so2);

      // Spin up the queue helper thread.
      this->rosQueueThread2 = 
        std::thread(std::bind(&ModelQuake::QueueThread2, this));
      
      ROS_WARN("Loaded ModelQuake Plugin with parent...%s, only X Axis Freq Supported in this V-1.0", this->model->GetName().c_str());

    }

    public: void OnUpdate()
    {
      double new_secs = ros::Time::now().toSec();
      double delta = new_secs - this->old_secs;

      double max_delta = 0.0;

      if (this->x_axis_freq != 0.0)
      {
        max_delta = 1.0 / this->x_axis_freq;
      }

      double magnitude_speed = this->x_axis_magn;

      if (delta > max_delta && delta != 0.0)
      {
        // Change Direction
        this->direction = this->direction * -1;
        this->old_secs = new_secs;
        ROS_DEBUG("Changing direction...");
      }

      double speed = magnitude_speed * this->direction;

      // Apply a small linear velocity to the model.
      this->model->SetLinearVel(ignition::math::Vector3d(speed, 0, 0));
      this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
    }

    }
  };
}