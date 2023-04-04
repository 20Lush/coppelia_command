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
    : Node("FloatPublisher"), count_(0)
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

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto publisher = std::make_shared<FloatPublisher>("commandx");
    auto subscriber = std::make_shared<FloatSubscriber>("calloutx");


    float theta = 0.0f;
    float width = 0.35f;

    while(rclcpp::ok()){


        publisher->input = sinf(theta) * width * 0.5;
        
        std::cout << publisher->input << std::endl;

        rclcpp::spin_some(publisher);
        //rclcpp::spin_some(subscriber);

        theta += 0.05;
        std::this_thread::sleep_for(10ms); // [middle finger emoji]

    }

    rclcpp::shutdown();

    return 0;
}