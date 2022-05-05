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
#include "std_msgs/Int16.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"

namespace gazebo
{
  class jetForce : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      /// \brief Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&jetForce::OnUpdate, this));

      // ROS_WARN("Loaded jetForce Plugin with parent...%s", this->model->GetName().c_str());
      
      // Create a topic name
      std::string jetForce_angle_topicName = "/jetForce_angle";
      std::string jetForce_mag_topicName = "/jetForce_mag";
      std::string pose_topicName = "/mavros/local_position/pose";

      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "jetForce_rosnode",
          ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to the Gazebo node.
      this->rosNode.reset(new ros::NodeHandle("jetForce_rosnode"));

      // Angle
      ros::SubscribeOptions so = 
        ros::SubscribeOptions::create<std_msgs::Int16>(
          jetForce_angle_topicName, 1,
          boost::bind(&jetForce::OnRosMsg_Ang, this, _1),
          ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread = 
        std::thread(std::bind(&jetForce::QueueThread, this));
      
      // Magnitude
      ros::SubscribeOptions so2 = 
        ros::SubscribeOptions::create<std_msgs::Float32>(
          jetForce_mag_topicName, 1,
          boost::bind(&jetForce::OnRosMsg_Mag, this, _1),
          ros::VoidPtr(), &this->rosQueue2);
      this->rosSub2 = this->rosNode->subscribe(so2);

      // Spin up the queue helper thread.
      this->rosQueueThread2 = 
        std::thread(std::bind(&jetForce::QueueThread2, this));

      // Magnitude
      ros::SubscribeOptions so3 = 
        ros::SubscribeOptions::create<geometry_msgs::PoseStamped>(
          pose_topicName, 1,
          boost::bind(&jetForce::OnRosMsg_Pose, this, _1),
          ros::VoidPtr(), &this->rosQueue3);
      this->rosSub3 = this->rosNode->subscribe(so3);

      // Spin up the queue helper thread.
      this->rosQueueThread3 = 
        std::thread(std::bind(&jetForce::QueueThread3, this));
     

    }
    public: void OnUpdate()
    {
      double jetMag = this->x_axis_mag;
      double jetAng = this->x_axis_angle;
      double jetforce_x = jetMag * sin(jetAng);
      double jetforce_z = jetMag * cos(jetAng);

      // ROS_WARN("jetAng >> %f", jetAng);
      // ROS_WARN("jetMag >> %f", jetMag);

      ROS_WARN("x_current >> %f", this->x_current);
      ROS_WARN("x_current >> %f", this->z_current);

      if (jetMag != 0)
      {
        // ROS_WARN("jetforce_x >> %f", jetforce_x);
        // ROS_WARN("jetforce_z >> %f", jetforce_z);
        // Apply a small linear/angular velocity to the model.
        this->model->SetLinearVel(ignition::math::Vector3d(jetforce_x, 0, jetforce_z));
        // this->model->SetAngularVel(ignition::math::Vector3d(jetforce_x, 0, 0));
      }
      // TODO: Write subscriber to the pose topic to get the current x and z and add into the SetLinearVel
    }

    
    public: void SetAngle(const double &_angle)
    {
      this->x_axis_angle = _angle;
      ROS_WARN("x_axis_angle >> %f", this->x_axis_angle);
    }

    
    public: void SetMag(const double &_mag)
    {
      this->x_axis_mag = _mag;
      ROS_WARN("x_axis_mag >> %f", this->x_axis_mag);
    }

    
    public: void OnRosMsg_Ang(const std_msgs::Int16ConstPtr &_msg)
    {
      this->SetAngle(_msg->data);
    }


    public: void OnRosMsg_Mag(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetMag(_msg->data);
    }

    public: void OnRosMsg_Pose(const geometry_msgs::PoseStamped::ConstPtr &_msg)
    {
      // ROS_INFO_STREAM("Received Pose: " << _msg);
      this->x_current = _msg->pose.position.x;
      this->y_current = _msg->pose.position.y;
      this->z_current = _msg->pose.position.z;
      ROS_WARN("x_current >> %f", x_current);
      ROS_WARN("z_current >> %f", z_current);
    }



    ///\brief ROS Helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while(this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    ///\brief ROS Helper function2 that processes messages
    private: void QueueThread2()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue2.callAvailable(ros::WallDuration(timeout));
      }
    }

    ///\brief ROS Helper function3 that processes messages
    private: void QueueThread3()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue3.callAvailable(ros::WallDuration(timeout));
      }
    }

    

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    double x_axis_angle = 0.0;
    double x_axis_mag = 0.0;

    double x_current;
    double y_current;
    double z_current;

    private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    private: ros::Subscriber rosSub;
    private: ros::CallbackQueue rosQueue;
    private: std::thread rosQueueThread;

    private: ros::Subscriber rosSub2;
    private: ros::CallbackQueue rosQueue2;
    private: std::thread rosQueueThread2;

    private: ros::Subscriber rosSub3;
    private: ros::CallbackQueue rosQueue3;
    private: std::thread rosQueueThread3;

  };

  GZ_REGISTER_MODEL_PLUGIN(jetForce)
}