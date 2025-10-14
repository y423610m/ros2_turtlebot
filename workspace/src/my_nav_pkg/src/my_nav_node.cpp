#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// using std::placeholders::_1;

class MyNavNodeCpp : public rclcpp::Node
{
public:
  MyNavNodeCpp() : Node("my_nav_node_cpp"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter_cpp", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MyNavNodeCpp::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello from C++ node " + std::to_string(count_++);
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNavNodeCpp>());
  rclcpp::shutdown();
  return 0;
}
