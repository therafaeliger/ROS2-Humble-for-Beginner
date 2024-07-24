#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/sphere.hpp"
using std::placeholders::_1;

class NewSubscriber : public rclcpp::Node
{
  public:
    NewSubscriber()
    : Node("new_subscriber")
    {
      subscription_ = this->create_subscription<tutorial_interfaces::msg::Sphere>(
      "topic", 10, std::bind(&NewSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const tutorial_interfaces::msg::Sphere & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Sphere radius: '%f'", msg.radius);
    }
    rclcpp::Subscription<tutorial_interfaces::msg::Sphere>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NewSubscriber>());
  rclcpp::shutdown();
  return 0;
}