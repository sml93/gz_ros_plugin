#include <cmath>
#include <math.h>
#include <thread>
#include <functional>
#include <bits/stdc++.h>

#include "ros/ros.h"
#include <Eigen/Dense>
#include "ros/console.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"

#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

#define deg2rad (M_PI / 180.0)

// using namespace std;
using namespace Eigen;

namespace gazebo
{
  class ceilingEffect : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      /// \brief Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ceilingEffect::OnUpdate, this));

      // ROS_WARN("Loaded jetForce Plugin with parent...%s", this->model->GetName().c_str());
      
      // Create a topic name
      std::string ceiling_dist_topicName = "/lw20_ranger";
      std::string pose_topicName = "/mavros/local_position/pose";

      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "ce_rosnode",
          ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to the Gazebo node.
      this->rosNode.reset(new ros::NodeHandle("ce_rosnode"));

      // Ceiling Distance
      ros::SubscribeOptions so = 
        ros::SubscribeOptions::create<sensor_msgs::LaserScan>(
          ceiling_dist_topicName, 1,
          boost::bind(&ceilingEffect::OnRosMsg_CE, this, _1),
          ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread. (Ceiling Dist)
      this->rosQueueThread = 
        std::thread(std::bind(&ceilingEffect::QueueThread, this));
      

      // UAV pose
      ros::SubscribeOptions so2 = 
        ros::SubscribeOptions::create<geometry_msgs::PoseStamped>(
          pose_topicName, 1,
          boost::bind(&ceilingEffect::OnRosMsg_Pose, this, _1),
          ros::VoidPtr(), &this->rosQueue2);
      this->rosSub2 = this->rosNode->subscribe(so2);

      // Spin up the queue helper thread.  (UAV Pose)
      this->rosQueueThread2 = 
        std::thread(std::bind(&ceilingEffect::QueueThread2, this));

    }

    public: void OnUpdate()
    {
      // double reset = 0.0;
      double ceiling_dist = this->ce_dist;
      double ce_thrust = 0;

      double roll = this->roll;
      double pitch = this->pitch;
      double yaw = this->yaw;

      double forceX = 0;
      double forceY = 0;
      double forceZ = 0;

      double delta = 0;
      double gamma = 0;
      double alpha0 = 0;
      double alpha1 = 1.6;
      double rho = 1.293;
      double area = M_PI ;
      double inputVel = 1;

      double radius = 5*0.0254;

      const float DEG2RAD = M_PI/180;

      float sx, sy, sz, cx, cy, cz, phi, theta, psi;

      ROS_INFO("all good here");

      // ROS_WARN("jetAng >> %f", jetAng);
      // ROS_WARN("jetMag >> %f", jetMag);



      // delta = radius / ceiling_dist;

      // gamma = 0.5*(1-(alpha1*pow(delta,2))) + 0.5*(sqrt(pow(1-(alpha1*(pow(delta,2))),2)+(alpha0/8*(pow(delta,2)))));
      // ce_thrust = 2*rho*area*pow(gamma,2)*(pow(inputVel,2));
      

      // MatrixXf matrixR(3,3);
      
      // phi = roll*DEG2RAD;
      // theta = pitch*DEG2RAD;
      // psi = yaw*DEG2RAD;

      // sx = sin(phi);
      // cx = cos(phi);

      // sy = sin(theta);
      // cy = cos(theta);

      // sz = sin(psi);
      // cz = sin(psi);

      // matrixR(0, 0) = cy*cz;
      // matrixR(0, 1) = (sx*sy*cz)-(cx*sz);
      // matrixR(0, 2) = (cx*sy*cz)-(sx*sz);
      // matrixR(1, 0) = cx*sz;
      // matrixR(1, 1) = (sx*sy*sz) + (cx*cz);
      // matrixR(1, 2) = (cx*sy*sz) - (sx*cz);
      // matrixR(2, 0) = -sx;
      // matrixR(2, 1) = sx*cy;
      // matrixR(2, 2) = cx*cy;

      // MatrixXf matrixF(3, 1);
      // matrixF.setZero();
      // matrixF(2, 0) = ce_thrust;

      // MatrixXf mat(3, 1);
      // mat = matrixR * matrixF;


      // if (ceiling_dist <= 0.5)
      // {
      //   // Apply a small linear/angular velocity to the model.
      //   forceX = mat(0,0);
      //   forceY = mat(1,0);
      //   forceZ = mat(2,0);

      //   this->model->GetLink("base_link")->SetForce(ignition::math::Vector3d(forceX, forceY, forceZ));
      //   ROS_INFO("forceX >> %f", forceX);
      //   ROS_INFO("forceY >> %f", forceY);
      //   ROS_INFO("forceZ >> %f", forceZ);
      //   // this->model->SetLinearVel(ignition::math::Vector3d(-jetforce_x, 0, jetforce_z));
      //   // ROS_INFO("mag >> %f", this-> x_axis_mag);
      //     // this->model->SetAngularVel(ignition::math::Vector3d(jetforce_x, 0, 0));
      //   // this->x_axis_mag = 0.0;
      // }


    }

    // public: void getPitch(const double &_angle)
    // {
    //   this->pitchAngle = _angle;
    //   ROS_INFO("pitch_angle >> %f", this->pitchAngle);
    // }

    //////
    //  Helper functions
    /////
    public: void getCEdist(const double &_dist)
    {
      this->ce_dist = 2.5 - _dist;
      ROS_INFO("ce_dist >> %f", this->ce_dist);
    }

    /////
    // Callback function
    /////
    public: void OnRosMsg_CE(const sensor_msgs::LaserScan::ConstPtr &_msg)
    {
      this->getCEdist(_msg->range_min);
      ROS_INFO("ok");
    }


    public: void OnRosMsg_Pose(const geometry_msgs::PoseStamped::ConstPtr &_msg)
    {
      // ROS_INFO_STREAM("Received Pose: " << _msg);
      this->x_current = _msg->pose.orientation.x;
      this->y_current = _msg->pose.orientation.y;
      this->z_current = _msg->pose.orientation.z;
      this->w_current = _msg->pose.orientation.w;
      ROS_INFO("x: >> %f", x_current);

      // // Roll
      // double sinr_cosp = 2 * (this->w_current * this->x_current + this->y_current * this->z_current);
      // double cosr_cosp = 1 - 2 * (this->x_current * this->x_current + this->y_current * this->y_current);
      // this->roll = std::atan2(sinr_cosp, cosr_cosp);

      // // Pitch
      // double sinp = 2 * (this->w_current * this->y_current - this->z_current * this->x_current);
      // if (std::abs(sinp) >= 1)
      //   this->pitch = std::copysign(M_PI / 2, sinp);
      // else
      //   this->pitch = std::asin(sinp);
      
      // // Yaw
      // double siny_cosp = 2 * (this->w_current * this->z_current + this->x_current * this->y_current);
      // double cosy_cosp = 1 -2 * (this->y_current * this->y_current + this->z_current * this->z_current);
      // this->yaw = std::atan2(siny_cosp, cosy_cosp);

      // ROS_INFO("roll_euler >> %f", this->roll);
      // ROS_INFO("pitch_euler >> %f", this->pitch);
      // ROS_INFO("yaw_euler >> %f", this->yaw);
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


    

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    double ceiling_dist = 2.5;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    double ce_dist = 0.0;

    double x_current;
    double y_current;
    double z_current;
    double w_current;

    private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    private: ros::Subscriber rosSub;
    private: ros::CallbackQueue rosQueue;
    private: std::thread rosQueueThread;

    private: ros::Subscriber rosSub2;
    private: ros::CallbackQueue rosQueue2;
    private: std::thread rosQueueThread2;

  };

  GZ_REGISTER_MODEL_PLUGIN(ceilingEffect)
}