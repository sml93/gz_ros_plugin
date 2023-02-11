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
      std::string ce_dist_topicName = "/lw20_ranger";

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

      // UAV pose
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
          ros::VoidPtr(), &this->rosQueue4);
      this->rosSub4 = this->rosNode->subscribe(so4);

      // Spin up the queue helper thread.
      this->rosQueueThread4 = 
        std::thread(std::bind(&jetForce::QueueThread4, this));

      // Jet switch
      ros::SubscribeOptions so5 = 
        ros::SubscribeOptions::create<std_msgs::Int16>(
          jetSwitch_topicname, 1,
          boost::bind(&jetForce::OnRosMsg_Switch, this, _1),
          ros::VoidPtr(), &this->rosQueue5);
      this->rosSub5 = this->rosNode->subscribe(so5);

      // Spin up the queue helper thread.
      this->rosQueueThread5 = 
        std::thread(std::bind(&jetForce::QueueThread5, this));

      // Ceiling Distance
      ros::SubscribeOptions so6 = 
        ros::SubscribeOptions::create<sensor_msgs::LaserScan>(
          ce_dist_topicName, 1,
          boost::bind(&jetForce::OnRosMsg_CE, this, _1),
          ros::VoidPtr(), &this->rosQueue6);
      this->rosSub = this->rosNode->subscribe(so6);

      // Spin up the queue helper thread. (Ceiling Dist)
      this->rosQueueThread6 = 
        std::thread(std::bind(&jetForce::QueueThread6, this));
     

    }
    public: void OnUpdate()
    {
      double reset = 0.0;
      double jetMag = sqrt(((this->x_axis_mag)*0.5)/(0.5*8.0));
      double jetAng = deg2rad*this->x_axis_angle;

      double ceiling_dist = this->ce_dist;
      double roll = this->roll;
      double pitch = this->pitch;
      double yaw = this->yaw;

      double delta = 0;
      double gamma = 0;
      double ce_thrust = 0;
      double alpha0 = 0;
      double radius = 5*0.0254;
      double alpha1 = 1.6;
      double rho = 1.293;
      double area = M_PI*(pow(radius,2));
      double inputVel = 1;

      float sx, sy, sz, cx, cy, cz, phi, theta, psi;



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


      //////
      // Ceiling Effect model
      /////

      if (ceiling_dist <= 0)
      {
        delta = radius / -2.5;
      }
      else {
        delta = radius / -ceiling_dist;
      }
      


      // Roll
      double sinr_cosp = 2 * (this->w_current * this->rx_current + this->ry_current * this->rz_current);
      double cosr_cosp = 1 - 2 * (this->rx_current * this->rx_current + this->ry_current * this->ry_current);
      this->roll = std::atan2(sinr_cosp, cosr_cosp);

      // Pitch
      double sinp = 2 * (this->w_current * this->ry_current - this->rz_current * this->rx_current);
      if (std::abs(sinp) >= 1)
        this->pitch = std::copysign(M_PI / 2, sinp);
      else
        this->pitch = std::asin(sinp);
      
      // Yaw
      double siny_cosp = 2 * (this->w_current * this->rz_current + this->rx_current * this->ry_current);
      double cosy_cosp = 1 -2 * (this->ry_current * this->ry_current + this->rz_current * this->rz_current);
      this->yaw = std::atan2(siny_cosp, cosy_cosp);


      MatrixXf matrixR(3,3);
      
      phi = this->roll*deg2rad;
      theta = this->pitch*deg2rad;
      psi = this->yaw*deg2rad;

      sx = sin(phi);
      cx = cos(phi);

      sy = sin(theta);
      cy = cos(theta);

      sz = sin(psi);
      cz = sin(psi);

      matrixR(0, 0) = cy*cz;
      matrixR(0, 1) = (sx*sy*cz)-(cx*sz);
      matrixR(0, 2) = (cx*sy*cz)-(sx*sz);
      matrixR(1, 0) = cx*sz;
      matrixR(1, 1) = (sx*sy*sz) + (cx*cz);
      matrixR(1, 2) = (cx*sy*sz) - (sx*cz);
      matrixR(2, 0) = -sx;
      matrixR(2, 1) = sx*cy;
      matrixR(2, 2) = cx*cy;

      MatrixXf matrixF(3, 1);
      matrixF.setZero();
      matrixF(2, 0) = ce_thrust;

      MatrixXf matForce(3, 1);
      matForce = matrixR * matrixF;

      if ((ceiling_dist > 0) && (ceiling_dist <= 0.5))
      {
        inputVel = 2.0; // Do thrust stand experiments to get CE input velocitu model then replace this.
        gamma = 0.5*(1-(alpha1*pow(delta,2))) + 0.5*(sqrt(pow(1-(alpha1*(pow(delta,2))),2)+(alpha0/8*(pow(delta,2)))));
        ce_thrust = 2*rho*area*pow(gamma,2)*(pow(inputVel,2));

        // // Apply a small linear/angular velocity to the model.
        // forceX = matForce(0,0);
        // forceY = matForce(1,0);
        // forceZ = matForce(2,0);

        forceX = 0;
        forceY = 0;
        forceZ = 4*ce_thrust;

        
        // Apply a small linear/angular velocity to the model.
        this->model->GetLink("base_link")->SetForce(ignition::math::Vector3d(forceX, forceY, forceZ));
        ROS_INFO("ForceX: >> %f", forceX);
        ROS_INFO("ForceY: >> %f", forceY);
        ROS_INFO("ForceZ: >> %f", forceZ);
      }

      //////
      // Jet effect model
      //////

      if (jetOn)
      {
        // ROS_WARN("jetforce_x >> %f", jetforce_x);
        // ROS_WARN("jetforce_z >> %f", jetforce_z);
        
        for (int i=0; i<(dur*100); i++)
        {
          // Apply a small linear/angular velocity to the model.
          // this->model->SetLinearVel(ignition::math::Vector3d(-jetforce_x, 0, jetforce_z));    // convert Force to velocity by using W = Fd = 0.5mv**2 where d for alp=0 is 0.9
          this->model->GetLink("base_link")->SetForce(ignition::math::Vector3d(-jetforce_x, 0, jetforce_z));
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


    public: void getCEdist(const double &_dist)
    {
      this->ce_dist = 2.5 - _dist;
      ROS_INFO("dist >> %f", ce_dist);
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
      this->rx_current = _msg->pose.orientation.x;
      this->ry_current = _msg->pose.orientation.y;
      this->rz_current = _msg->pose.orientation.z;
      this->w_current = _msg->pose.orientation.w;

      // ROS_INFO("x_current >> %f", x_current);
      // ROS_INFO("z_current >> %f", z_current);
    }

    public: void OnRosMsg_CE(const sensor_msgs::LaserScan::ConstPtr &_msg)
    {
      this->getCEdist(_msg->ranges[0]);
      // this->ce_dist = _msg->ranges[0];
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


    ///\brief ROS Helper function6 that processes messages
    private: void QueueThread6()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue6.callAvailable(ros::WallDuration(timeout));
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

    double roll;
    double pitch;
    double yaw;
    double forceX;
    double forceY;
    double forceZ;

    double x_current;
    double y_current;
    double z_current;
    double rx_current;
    double ry_current;
    double rz_current;
    double w_current;
    double ce_dist;


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

    private: ros::Subscriber rosSub6;
    private: ros::CallbackQueue rosQueue6;
    private: std::thread rosQueueThread6;

  };

  GZ_REGISTER_MODEL_PLUGIN(jetForce)
}
