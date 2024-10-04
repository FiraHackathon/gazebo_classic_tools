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

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&CollisionDetectorPlugin::onUpdate, this));
    }

    void onUpdate()
    {
        if (this->world->ModelByName(this->robotName) && this->world->EntityByName(this->groundCollisionName))
        {
          setRobotCollisionIds();
          this->groundId = this->world->EntityByName(this->groundCollisionName)->GetId();
          this->model_is_loaded = true;
          
          // Destroy connection to avoid unnecessary checks each gazebo frame
          this->updateConnection.reset();
        }
    }

    void OnContactMsg(ConstContactsPtr& msg) 
    {
      if (this->model_is_loaded)
      {
        for (int i = 0; i < msg->contact_size(); ++i)
        {
          const gazebo::msgs::Contact &contact = msg->contact(i);
          unsigned int collision1_id = contact.wrench()[0].body_1_id();
          unsigned int collision2_id = contact.wrench()[0].body_2_id();

          if (filterContacts(collision1_id, collision2_id)) 
          {
            std_msgs::msg::String msg;
            msg.data = "Collision detected between " + std::to_string(collision1_id) + " and " + std::to_string(collision2_id);

            this->collisionPub->publish(msg);
          }
        }
      }
    }

    void setRobotCollisionIds()
    {
      physics::ModelPtr model = this->world->ModelByName(this->robotName);
      for (auto link : model->GetLinks())
      {
        for (auto collision : link->GetCollisions()) {
          this->collisionIds.push_back(collision->GetId());
        } 
      }
    }

    bool filterContacts(const unsigned int collision1, const unsigned int collision2)
    {
      // Exactly one body is a robot link and the collision is not with the ground
      return  (((std::find(this->collisionIds.begin() , this->collisionIds.end() , collision1) != this->collisionIds.end())
                != (std::find(this->collisionIds.begin() , this->collisionIds.end() , collision2) != this->collisionIds.end()))
                && collision1 != this->groundId && collision2 != this->groundId);
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
    event::ConnectionPtr updateConnection;
    
    std::vector<unsigned int> collisionIds;
    std::string groundCollisionName;
    unsigned int groundId;
    std::string robotName;
    bool model_is_loaded;


    gazebo_ros::Node::SharedPtr rosNode;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr collisionPub;
    rclcpp::TimerBase::SharedPtr timer;
  };

  GZ_REGISTER_WORLD_PLUGIN(CollisionDetectorPlugin)
}
