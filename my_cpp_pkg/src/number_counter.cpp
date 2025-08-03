#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
using namespace std::chrono_literals;
using namespace std::placeholders;
class NumberCounter : public rclcpp::Node
{
public:
    NumberCounter(): Node("number_counter"), total_count_(0)
    {
        publisher_ = create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        subscription_ = create_subscription<example_interfaces::msg::Int64>("number", 10, std::bind(&NumberCounter::numbering_callback, this, _1));
        RCLCPP_INFO(get_logger(), "Numbering Started");
    }
private:
    void numbering_callback(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        total_count_ += msg->data;
        RCLCPP_INFO(get_logger(), "received number: %ld", total_count_);
        auto total_msg = example_interfaces::msg::Int64();
        total_msg.data = total_count_;
        publisher_->publish(total_msg);
    }
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscription_;
    int64_t total_count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}