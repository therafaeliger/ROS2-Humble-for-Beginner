#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/sphere.hpp"

using namespace std::chrono_literals;

class NewPublisher : public rclcpp::Node
{
  public:
    NewPublisher()
    : Node("new_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<tutorial_interfaces::msg::Sphere>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&NewPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = tutorial_interfaces::msg::Sphere();
      message.radius = 3;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.radius);
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<tutorial_interfaces::msg::Sphere>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NewPublisher>());
  rclcpp::shutdown();
  return 0;
}