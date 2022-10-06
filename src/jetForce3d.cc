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

#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

#define deg2rad (M_PI / 180.0)

using namespace Eigen;

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
      std::string time_duration_topicName = "/duration";
      std::string jetSwitch_topicname = "/jetOn";

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

      // Duration
      ros::SubscribeOptions so4 = 
        ros::SubscribeOptions::create<std_msgs::Int16>(
          time_duration_topicName, 1,
          boost::bind(&jetForce::OnRosMsg_Dur, this, _1),
          ros::VoidPtr(), &this->rosQueue);
      this->rosSub4 = this->rosNode->subscribe(so4);

      // Spin up the queue helper thread.
      this->rosQueueThread4 = 
        std::thread(std::bind(&jetForce::QueueThread4, this));

      // Jet switch
      ros::SubscribeOptions so5 = 
        ros::SubscribeOptions::create<std_msgs::Int16>(
          jetSwitch_topicname, 1,
          boost::bind(&jetForce::OnRosMsg_Switch, this, _1),
          ros::VoidPtr(), &this->rosQueue);
      this->rosSub5 = this->rosNode->subscribe(so5);

      // Spin up the queue helper thread.
      this->rosQueueThread5 = 
        std::thread(std::bind(&jetForce::QueueThread5, this));
     

    }
    public: void OnUpdate()
    {
      double reset = 0.0;
      double jetMag = sqrt(((this->x_axis_mag)*0.5)/(0.5*8.0));
      double jetAng = deg2rad*this->x_axis_angle;

      MatrixXf matrixA(3,3);
      matrixA.setZero();

      matrixA(0, 0) = cos(jetAng);
      matrixA(0, 2) = sin(jetAng);
      matrixA(1, 1) = 1;
      matrixA(2, 0) = -sin(jetAng);
      matrixA(2, 2) = cos(jetAng);

      MatrixXf matrixB(3,1);
      matrixB.setZero();
      matrixB(2, 0) = jetMag;

      MatrixXf mat(3, 1);
      mat = matrixA * matrixB;
      // std::cout << mat << std::endl;

      double jetforce_x = mat(0, 0);
      double jetforce_z = mat(2, 0);
      int dur = this->jet_dur;
      int jetOn = this->jet_switch;

      // ROS_WARN("jetAng >> %f", jetAng);
      // ROS_WARN("jetMag >> %f", jetMag);

      // if (jetMag != 0)
      // {
      //   // ROS_WARN("jetforce_x >> %f", jetforce_x);
      //   // ROS_WARN("jetforce_z >> %f", jetforce_z);
        
      //   for (int i=0; i<(dur*100); i++)
      //   {
      //     // Apply a small linear/angular velocity to the model.
      //     this->model->SetLinearVel(ignition::math::Vector3d(-jetforce_x, 0, jetforce_z));    // convert Force to velocity by using W = Fd = 0.5mv**2 where d for alp=0 is 0.9
      //     ROS_INFO("mag >> %f", this-> x_axis_mag);
      //       // this->model->SetAngularVel(ignition::math::Vector3d(jetforce_x, 0, 0));
      //   }
      //   this->x_axis_mag = 0.0;
      // }

      if (jetOn)
      {
        // ROS_WARN("jetforce_x >> %f", jetforce_x);
        // ROS_WARN("jetforce_z >> %f", jetforce_z);
        
        for (int i=0; i<(dur*100); i++)
        {
          // Apply a small linear/angular velocity to the model.
          this->model->SetLinearVel(ignition::math::Vector3d(-jetforce_x, 0, jetforce_z));    // convert Force to velocity by using W = Fd = 0.5mv**2 where d for alp=0 is 0.9
          ROS_INFO("mag >> %f", this-> x_axis_mag);
          ROS_INFO("jet on: >> %i", (jetOn));
            // this->model->SetAngularVel(ignition::math::Vector3d(jetforce_x, 0, 0));
        }
        this->jet_switch = 0;
      }
    }

    
    public: void SetAngle(const double &_angle)
    {
      this->x_axis_angle = _angle;
      ROS_INFO("x_axis_angle >> %f", this->x_axis_angle);
    }

    
    public: void SetMag(const double &_mag)
    {
      this->x_axis_mag = _mag;
      ROS_INFO("x_axis_mag >> %f", this->x_axis_mag);
    }

    public: void SetDuration(const int &_dur)
    {
      this->jet_dur = _dur;
    }

    public: void SetSwitch(const int &_switch)
    {
      this->jet_switch = _switch;
    }

    




    public: void OnRosMsg_Ang(const std_msgs::Int16ConstPtr &_msg)
    {
      this->SetAngle(_msg->data);
    }


    public: void OnRosMsg_Mag(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetMag(_msg->data);
    }

    public: void OnRosMsg_Dur(const std_msgs::Int16ConstPtr &_msg)
    {
      this->SetDuration(_msg->data);
    }

    public: void OnRosMsg_Switch(const std_msgs::Int16ConstPtr &_msg)
    {
      this->SetSwitch(_msg->data);
    }

    public: void OnRosMsg_Pose(const geometry_msgs::PoseStamped::ConstPtr &_msg)
    {
      // ROS_INFO_STREAM("Received Pose: " << _msg);
      this->x_current = _msg->pose.position.x;
      this->y_current = _msg->pose.position.y;
      this->z_current = _msg->pose.position.z;
      ROS_INFO("x_current >> %f", x_current);
      ROS_INFO("z_current >> %f", z_current);
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

  
    ///\brief ROS Helper function4 that processes messages
    private: void QueueThread4()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue4.callAvailable(ros::WallDuration(timeout));
      }
    }

    
    ///\brief ROS Helper function5 that processes messages
    private: void QueueThread5()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue5.callAvailable(ros::WallDuration(timeout));
      }
    }

    

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    double x_axis_angle = 0.0;
    double x_axis_mag = 0.0;
    int jet_dur = 0;
    int jet_switch = 0;

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

    private: ros::Subscriber rosSub4;
    private: ros::CallbackQueue rosQueue4;
    private: std::thread rosQueueThread4;

    private: ros::Subscriber rosSub5;
    private: ros::CallbackQueue rosQueue5;
    private: std::thread rosQueueThread5;

  };

  GZ_REGISTER_MODEL_PLUGIN(jetForce)
}
