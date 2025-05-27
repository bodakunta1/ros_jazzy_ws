#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::chrono_literals;

class NumberPublisher : public rclcpp::Node
{
public:
    NumberPublisher(): Node("number_publisher")
    {
        publisher_ = create_publisher<example_interfaces::msg::Int64>("number", 10);
        timer_ = create_timer(1s, std::bind(&NumberPublisher::NumberPublish, this));
        RCLCPP_INFO(get_logger(), "Numbers started");
    }
private:
    void NumberPublish()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = 7;
        RCLCPP_INFO(get_logger(), "Publishing: %ld", msg.data);
        publisher_->publish(msg);
    }
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}