#include <sstream>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/duration.hpp>

#include <gazebo_msgs/msg/contact_state.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>


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

      this->world = _parent;
      this->collisionPub = this->rosNode->create_publisher<gazebo_msgs::msg::ContactsState>(this->rosPubTopic, 10);

      this->contactSub = this->gzNode->Subscribe(
        "/gazebo/default/physics/contacts",
         &CollisionDetectorPlugin::OnContactMsg,
          this
          );

    }


    void OnContactMsg(ConstContactsPtr& msg)
    {
      gazebo_msgs::msg::ContactsState contact_list_msg;
      for (int i = 0; i < msg->contact_size(); ++i)
      {
        const gazebo::msgs::Contact &contact = msg->contact(i);
        std::string collision1_name = contact.collision1();
        std::string collision2_name = contact.collision2();

        if (filterContacts(collision1_name, collision2_name))
        {
          gazebo_msgs::msg::ContactState contact_msg;
          // msg.info = "Collision between " + collision1_name + " and " + collision2_name;
          contact_msg.collision1_name = contact.collision1();
          contact_msg.collision2_name = contact.collision2();

          for (auto contact_position : contact.position())
          {
            geometry_msgs::msg::Vector3 pos;
            pos.x = contact_position.x();
            pos.y = contact_position.y();
            pos.z = contact_position.z();
            contact_msg.contact_positions.push_back(pos);
          }
          contact_list_msg.states.push_back(contact_msg);
        }
      }
      if (!contact_list_msg.states.empty())
      {
        this->collisionPub->publish(contact_list_msg);
      }
    }

    bool filterContacts(const std::string collision1_name, const std::string collision2_name)
    {
      std::vector<std::string> model_link_coll1;
      std::vector<std::string> model_link_coll2;

      split(collision1_name, ':', model_link_coll1);
      split(collision2_name, ':', model_link_coll2);

      // At least one body is in the monitored models or links, and none is in the ignored models or links
      return ( (is_in(model_link_coll1[0], this->monitoredModels) || is_in(model_link_coll1[1], this->monitoredLinks)
                 || is_in(model_link_coll2[0], this->monitoredModels) || is_in(model_link_coll2[1], this->monitoredLinks))
             && (!is_in(model_link_coll1[0], this->ignoredModels) && !is_in(model_link_coll1[1], this->ignoredLinks)
                && !is_in(model_link_coll2[0], this->ignoredModels) && !is_in(model_link_coll2[1], this->ignoredLinks)));
    }

    // Parameters can be model and/or link names, separated by whitespaces if needed
    void getSDFParameters(sdf::ElementPtr _sdf)
    {
      if (_sdf->HasElement("monitored_models"))
        split(_sdf->Get<std::string>("monitored_models"), ' ', this->monitoredModels);
      if (_sdf->HasElement("monitored_links"))
        split(_sdf->Get<std::string>("monitored_links"), ' ', this->monitoredLinks);
      if (_sdf->HasElement("ignored_models"))
        split(_sdf->Get<std::string>("ignored_models"), ' ', this->ignoredModels);
      if (_sdf->HasElement("ignored_links"))
        split(_sdf->Get<std::string>("ignored_links"), ' ', this->ignoredLinks);
      if (_sdf->HasElement("ros_pub_topic"))
        this->rosPubTopic = _sdf->Get<std::string>("ros_pub_topic");
      else
        this->rosPubTopic = "collisions";
    }

  void split(const std::string &s, const char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (getline(ss, item, delim)) {
        if (item != "") elems.push_back(item);
    }
  }

  // Could be templated but only for std::vector<std::string> for now
  bool is_in(const std::string& item, const std::vector<std::string>& container) {
    return (std::find(container.begin(), container.end(), item) != container.end());
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

    std::vector<std::string> monitoredModels;
    std::vector<std::string> monitoredLinks;
    std::vector<std::string> ignoredModels;
    std::vector<std::string> ignoredLinks;
    std::string rosPubTopic;


    gazebo_ros::Node::SharedPtr rosNode;
    rclcpp::Publisher<gazebo_msgs::msg::ContactsState>::SharedPtr collisionPub;
    rclcpp::TimerBase::SharedPtr timer;
  };

  GZ_REGISTER_WORLD_PLUGIN(CollisionDetectorPlugin)
}
