#include <rcl/time.h>

#include <boost/smart_ptr/make_shared_array.hpp>
#include <chrono>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/PhysicsTypes.hh"

namespace gazebo
{

class PosePublisherPlugin : public WorldPlugin
{
  using PoseMsg = geometry_msgs::msg::PoseStamped;
  using PosePublisher = rclcpp::Publisher<PoseMsg>::SharedPtr;

public:
  virtual void Load(physics::WorldPtr world, sdf::ElementPtr sdf)
  {
    std::string model_name;
    std::string topic_name;
    double update_rate = 10;

    ros_node_ = gazebo_ros::Node::Get(sdf);

    if (sdf->HasElement("model")) {
      model_name = sdf->Get<std::string>("model");
    } else {
      RCLCPP_ERROR(ros_node_->get_logger(), "Missing <model>, cannot proceed");
      return;
    }

    if (sdf->HasElement("ros_pub_topic")) {
      topic_name = sdf->Get<std::string>("ros_pub_topic");
    } else {
      RCLCPP_ERROR(ros_node_->get_logger(), "Missing <ros_pub_topic>, cannot proceed");
      return;
    }

    if (sdf->HasElement("update_rate")) {
      update_rate = sdf->Get<double>("update_rate");
    } else {
      RCLCPP_INFO(ros_node_->get_logger(), "Missing <update_rate>, default to 10 Hz");
    }

    auto model = world->ModelByName(model_name);
    if (!model) {
      RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Model '" << model_name << "' does not exist");
      return;
    }

    // Use link as physic entity or model if link is not specified
    if (sdf->HasElement("link")) {
      auto link_name = sdf->Get<std::string>("link");
      auto link = model->GetLink(link_name);
      if (!link) {
        RCLCPP_ERROR_STREAM(
          ros_node_->get_logger(),
          "Link '" << link_name << "' does not exist in model '" << model_name << "'");
        return;
      }
      entity_ = link;
    } else {
      entity_ = model;
    }

    pose_pub_ = ros_node_->create_publisher<PoseMsg>(topic_name, 1);

    // This timer use ROS time to follow simulation rate and accelerate publishing if needed
    timer_ = rclcpp::create_timer(
      ros_node_,
      std::make_shared<rclcpp::Clock>(RCL_ROS_TIME),
      std::chrono::duration<double>{1. / update_rate},
      std::bind(&PosePublisherPlugin::timer_callback_, this));
  }

  void timer_callback_()
  {
    const auto & pose = entity_->WorldPose();
    const auto & quat = pose.Rot();
    PoseMsg msg;
    msg.pose.position.x = pose.X();
    msg.pose.position.y = pose.Y();
    msg.pose.position.z = pose.Z();
    msg.pose.orientation.x = quat.X();
    msg.pose.orientation.y = quat.Y();
    msg.pose.orientation.z = quat.Z();
    msg.pose.orientation.w = quat.W();
    pose_pub_->publish(std::move(msg));
  }

private:
  physics::EntityPtr entity_;
  gazebo_ros::Node::SharedPtr ros_node_;
  PosePublisher pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

GZ_REGISTER_WORLD_PLUGIN(PosePublisherPlugin)
}  // namespace gazebo
