#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::chrono_literals;

class NumberPublisher : public rclcpp::Node
{
public:
    NumberPublisher(): Node("number_publisher")
    {
        this->declare_parameter("number", 2);
        this->declare_parameter("timer_period", 1.0);

        number_ = this->get_parameter("number").as_int();
        double timer_period = this->get_parameter("timer_period").as_double();

        publisher_ = create_publisher<example_interfaces::msg::Int64>("number", 10);
        timer_ = create_timer(std::chrono::duration<double>(timer_period), std::bind(&NumberPublisher::NumberPublish, this));
        RCLCPP_INFO(get_logger(), "Numbers started");
    }
private:
    void NumberPublish()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = number_;
        RCLCPP_INFO(get_logger(), "Publishing: %ld", msg.data);
        publisher_->publish(msg);
    }
    int number_;
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