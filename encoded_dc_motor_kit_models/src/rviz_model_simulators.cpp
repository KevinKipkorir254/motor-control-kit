#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" 
#include "ros2_motor_controller_msgs/msg/velocity_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
  
    volatile double real_position;
    volatile double bj_position;
    volatile double armax_position;
    MinimalSubscriber()
    : Node("velocity_feedback_subscriber")
    {
      visualisation_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
      visualisation2_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker2", 10);
      visualisation3_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker3", 10);
      subscription_ = this->create_subscription<ros2_motor_controller_msgs::msg::VelocityFeedback>("velocity_feedback", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      real_position = 0.0;
      bj_position = 0.0;
      armax_position = 0.0;
    }

  private:
    void topic_callback(const ros2_motor_controller_msgs::msg::VelocityFeedback & msg)
    {
        
      //RCLCPP_INFO(this->get_logger(), "I heard: %f, %f, %f", msg.real_velocity, msg.bj_velocity, msg.armax_velocity);
       visualization_msgs::msg::Marker marker_;

      //set the frame ID and timestamp
         marker_.header.frame_id = "shaft_v2_1";
         marker_.header.stamp = this->now();
         marker_.ns = "basic_shapes";
         marker_.id = 0;

         marker_.type = visualization_msgs::msg::Marker::SPHERE;
         marker_.action = visualization_msgs::msg::Marker::ADD;

         real_position += msg.real_velocity*0.01;
        

         marker_.pose.position.x = -0.028;
         //marker_.pose.position.y = 0.019;
         marker_.pose.position.y = 0.019*cos(real_position);
         //marker_.pose.position.z = 0.019;
         marker_.pose.position.z = 0.019*sin(real_position);
         marker_.pose.orientation.x = 0.0;
         marker_.pose.orientation.y = 0.0;
         marker_.pose.orientation.z = 0.0;
         marker_.pose.orientation.w = 1.0;
 
         // Set the scale of the marker -- 1x1x1 here means 1m on a side
         marker_.scale.x = 0.001;
         marker_.scale.y = 0.001;
         marker_.scale.z = 0.001;
     
         // Set the color -- be sure to set alpha to something non-zero!
         marker_.color.r = 0.0f;
         marker_.color.g = 1.0f;
         marker_.color.b = 0.0f;
         marker_.color.a = 1.0;


         marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
        
      //RCLCPP_INFO(this->get_logger(), "I heard: %f, %f, %f", msg.real_velocity, msg.bj_velocity, msg.armax_velocity);
       visualization_msgs::msg::Marker marker2_;

      //set the frame ID and timestamp
         marker2_.header.frame_id = "shaft_v2_1";
         marker2_.header.stamp = this->now();
         marker2_.ns = "basic_shapes";
         marker2_.id = 1;

         marker2_.type = visualization_msgs::msg::Marker::SPHERE;
         marker2_.action = visualization_msgs::msg::Marker::ADD;

         bj_position += msg.bj_velocity*0.01;

         marker2_.pose.position.x = -0.028;
         //marker2_.pose.position.y = 0.019;
         marker2_.pose.position.y = 0.019*cos(bj_position);
         //marker2_.pose.position.z = 0.019;
         marker2_.pose.position.z = 0.019*sin(bj_position);
         marker2_.pose.orientation.x = 0.0;
         marker2_.pose.orientation.y = 0.0;
         marker2_.pose.orientation.z = 0.0;
         marker2_.pose.orientation.w = 1.0;
 
         // Set the scale of the marker -- 1x1x1 here means 1m on a side
         marker2_.scale.x = 0.001;
         marker2_.scale.y = 0.001;
         marker2_.scale.z = 0.001;
     
         // Set the color -- be sure to set alpha to something non-zero!
         marker2_.color.r = 1.0f;
         marker2_.color.g = 0.0f;
         marker2_.color.b = 0.0f;
         marker2_.color.a = 1.0;


         marker2_.lifetime = rclcpp::Duration::from_seconds(0.1);
        
      //RCLCPP_INFO(this->get_logger(), "I heard: %f, %f, %f", msg.real_velocity, msg.bj_velocity, msg.armax_velocity);
       visualization_msgs::msg::Marker marker3_;

      //set the frame ID and timestamp
         marker3_.header.frame_id = "shaft_v2_1";
         marker3_.header.stamp = this->now();
         marker3_.ns = "basic_shapes";
         marker3_.id = 2;

         marker3_.type = visualization_msgs::msg::Marker::SPHERE;
         marker3_.action = visualization_msgs::msg::Marker::ADD;

         armax_position += msg.armax_velocity*0.01;

         marker3_.pose.position.x = -0.028;
         //marker2_.pose.position.y = 0.019;
         marker3_.pose.position.y = 0.019*cos(armax_position);
         //marker2_.pose.position.z = 0.019;
         marker3_.pose.position.z = 0.019*sin(armax_position);
         marker3_.pose.orientation.x = 0.0;
         marker3_.pose.orientation.y = 0.0;
         marker3_.pose.orientation.z = 0.0;
         marker3_.pose.orientation.w = 1.0;
 
         // Set the scale of the marker -- 1x1x1 here means 1m on a side
         marker3_.scale.x = 0.001;
         marker3_.scale.y = 0.001;
         marker3_.scale.z = 0.001;
     
         // Set the color -- be sure to set alpha to something non-zero!
         marker3_.color.r = 0.0f;
         marker3_.color.g = 0.0f;
         marker3_.color.b = 1.0f;
         marker3_.color.a = 1.0;


         marker3_.lifetime = rclcpp::Duration::from_seconds(0.1);

         while(visualisation_->get_subscription_count() < 1)
         {

            if(!rclcpp::ok())
            {
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("velocity_feedback"), "please add a subscriber");
            rclcpp::sleep_for(std::chrono::milliseconds(100));
         }

        // Log that subscribers are available
        //RCLCPP_INFO(this->get_logger(), "Publishing marker to RVIZ...");
        visualisation_->publish(marker_);
        visualisation2_->publish(marker2_);
        visualisation3_->publish(marker3_);

    }
    rclcpp::Subscription<ros2_motor_controller_msgs::msg::VelocityFeedback>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualisation_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualisation2_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualisation3_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}