//    _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
// ,-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)
// `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-'
// 		     __      __  __  ______  __  __    
// 	(•̀ᴗ•́)و  /\ \    /\ \/\ \/\  ___\/\ \_\ \    (◍＞◡＜◍)
// 	 ᶘ ◕ᴥ◕ᶅ	 \ \ \___\ \ \_\ \ \___  \ \  __ \  【≽ܫ≼】
// 	(ﾟ◥益◤ﾟ)  \ \_____\ \_____\/\_____\ \_\ \_\  (ʘ言ʘ╬)
// 	 ᕙ(⇀‸↼‶)ᕗ \/_____/\/_____/\/_____/\/_/\/_/ (◕‿◕✿)
// 
//    _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _
// ,-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)-(_)
// `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-' `-'


#include <memory>
#include <chrono>
#include <string>
#include <iostream>
#include <cmath>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class FloatSubscriber : public rclcpp::Node
{
  public:
    FloatSubscriber(std::string topic)
    : Node("FloatSubscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      topic, 10, std::bind(&FloatSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: " + std::to_string(msg->data));
    }
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
};

class FloatPublisher : public rclcpp::Node
{
  public:
    FloatPublisher(std::string name)
    : Node(name), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::Float32>(name, 100);
      timer_ = this->create_wall_timer(
      10ms, std::bind(&FloatPublisher::timer_callback, this));
    }

    float input=0.0f;

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::Float32();
      message.data = input;
      //RCLCPP_INFO(this->get_logger(), "Publishing: " + std::tostring(message.data));
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    size_t count_;
};

float cotf (float a){
    return cosf(a)/sinf(a);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto x_publisher = std::make_shared<FloatPublisher>("commandx");
    auto y_publisher = std::make_shared<FloatPublisher>("commandy");

    float theta = 0.0f;
    float width = 0.30f;
    auto timestep = 10ms;

    while(rclcpp::ok()){


        x_publisher->input = cosf(theta) * width * 0.5;
        y_publisher->input = sinf(theta) * width * 0.5;

        rclcpp::spin_some(x_publisher);
        rclcpp::spin_some(y_publisher);

        theta += 0.05;
        std::this_thread::sleep_for(timestep); // [middle finger emoji]

    }

    rclcpp::shutdown();

    return 0;
}