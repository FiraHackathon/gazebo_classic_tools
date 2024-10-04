
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Timer.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/physics.hh>  
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/duration.hpp>

#include <std_msgs/msg/string.hpp>

namespace gazebo
{
  class CollisionDetectorPlugin : public WorldPlugin
  {
  public:
    CollisionDetectorPlugin() : WorldPlugin() {}

    virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
      this->rosNode = gazebo_ros::Node::Get();
      gazebo::transport::NodePtr gzNode;

      this->gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
      this->gzNode->Init();

      this->getSDFParameters(_sdf);
      this->model_is_loaded = false;

      this->world = _parent;
      this->collisionPub = this->rosNode->create_publisher<std_msgs::msg::String>("collisions", 10);

      this->contactSub = this->gzNode->Subscribe("/gazebo/default/physics/contacts", &CollisionDetectorPlugin::OnContactMsg, this);
    }

    void OnContactMsg(ConstContactsPtr& msg) {
      if (this->model_is_loaded)
      {
        for (int i = 0; i < msg->contact_size(); ++i)
          {
            const gazebo::msgs::Contact &contact = msg->contact(i);
            std::string collision_name_1 = contact.collision1();
            std::string collision_name_2 = contact.collision2();

          if (filterContacts(contact.collision1(), contact.collision2())) 
          {
            std_msgs::msg::String msg;
            msg.data = "Collision detected between " + collision_name_1 + " and " + collision_name_2;

            this->collisionPub->publish(msg);
          }
        }
      }
      else 
      {
        if (this->world->ModelByName(this->robotName))
        {
          setCollisionNames();
          this->model_is_loaded = true;
        }
      }
    }

    void setCollisionNames()
    {
      physics::ModelPtr model = this->world->ModelByName(this->robotName);
      for (auto link : model->GetLinks())
      {
        for (auto collision : link->GetCollisions()) {
          this->collisionNames.push_back(collision->GetScopedName());
        } 
      }
    }

    bool filterContacts(const std::string collision1, const std::string collision2)
    {
      // Exactly one body is a robot link and the collision is not with the ground
      return  (((std::find(this->collisionNames.begin() , this->collisionNames.end() , collision1) != this->collisionNames.end())
                != (std::find(this->collisionNames.begin() , this->collisionNames.end() , collision2) != this->collisionNames.end()))
                && collision1 != this->groundCollisionName && collision2 != this->groundCollisionName);
    }

    void getSDFParameters(sdf::ElementPtr _sdf)
    {
      if (_sdf->HasElement("robot_model_name"))
        this->robotName = _sdf->Get<std::string>("robot_model_name");
      else
      {
        this->robotName = "robot";
        gzwarn << "Using default value 'robot' for parameter 'robot_model_name' \n";
      }
      if (_sdf->HasElement("ground_collision_name"))
        this->groundCollisionName = _sdf->Get<std::string>("ground_collision_name");
      else
      {
        this->groundCollisionName = "ground_plane::link";
        gzwarn << "Using default value 'ground_plane::link' for parameter 'ground_collision_name' \n";
      }
    }

    virtual ~CollisionDetectorPlugin()
    {
      rclcpp::shutdown();
    }

  private:
    physics::WorldPtr world;
    transport::NodePtr gzNode;
    transport::SubscriberPtr contactSub;
    
    std::vector<std::string> collisionNames;
    std::string groundCollisionName;
    std::string robotName;
    bool model_is_loaded;


    gazebo_ros::Node::SharedPtr rosNode;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr collisionPub;
    rclcpp::TimerBase::SharedPtr timer;
  };

  GZ_REGISTER_WORLD_PLUGIN(CollisionDetectorPlugin)
}
