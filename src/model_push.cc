#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      /// \brief Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ModelPush::OnUpdate, this));

      ROS_WARN("Loaded ModelPush Plugin with parent...%s", this->model->GetName().c_str());
    }

    public: void OnUpdate()
    {
      this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}