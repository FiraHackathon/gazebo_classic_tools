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
#include <rclcpp/logging.hpp>
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
    world_ = world;
    ros_node_ = gazebo_ros::Node::Get(sdf);

    std::string topic_name;
    double update_rate = 10;

    if (sdf->HasElement("model")) {
      model_name_ = sdf->Get<std::string>("model");
    } else {
      RCLCPP_ERROR(ros_node_->get_logger(), "Missing <model>, cannot proceed");
      return;
    }

    if (sdf->HasElement("link")) {
      link_name_ = sdf->Get<std::string>("link");
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

    if (sdf->HasElement("world_frame")) {
      frame_ = sdf->Get<std::string>("world_frame");
    } else {
      frame_ = "map";
      RCLCPP_INFO(ros_node_->get_logger(), "Missing <world_frame>, default to 'map'");
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
    if (!entity_ && !load_entity()) {
      return;
    }

    const auto & pose = entity_->WorldPose();
    const auto & quat = pose.Rot();
    PoseMsg msg;
    msg.header.frame_id = frame_;
    msg.header.stamp = ros_node_->now();
    msg.pose.position.x = pose.X();
    msg.pose.position.y = pose.Y();
    msg.pose.position.z = pose.Z();
    msg.pose.orientation.x = quat.X();
    msg.pose.orientation.y = quat.Y();
    msg.pose.orientation.z = quat.Z();
    msg.pose.orientation.w = quat.W();
    pose_pub_->publish(std::move(msg));
  }

  bool load_entity()
  {
    auto model = world_->ModelByName(model_name_);
    if (!model) {
      return false;
    }

    // Use link as physic entity or model if link is not specified
    if (!link_name_.empty()) {
      auto link = model->GetLink(link_name_);
      if (!link) {
        RCLCPP_WARN_STREAM(
          ros_node_->get_logger(),
          "Missing link '" << link_name_ << "' in model '" << model_name_ << "'");
        return false;
      }
      entity_ = link;
      RCLCPP_INFO_STREAM(
        ros_node_->get_logger(), "Loaded entity: " << model_name_ << '.' << link_name_);
    } else {
      entity_ = model;
      RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Loaded entity: " << model_name_);
    }

    return true;
  }

private:
  std::string model_name_;
  std::string link_name_;
  std::string frame_;
  physics::WorldPtr world_;
  physics::EntityPtr entity_;
  gazebo_ros::Node::SharedPtr ros_node_;
  PosePublisher pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

GZ_REGISTER_WORLD_PLUGIN(PosePublisherPlugin)
}  // namespace gazebo
