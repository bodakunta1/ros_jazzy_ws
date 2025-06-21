#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::chrono_literals;

class RobotNewsStationNode : public rclcpp::Node 
{
public:
    RobotNewsStationNode() : Node("robot_news_station")
    {
        this->declare_parameter("robot_name_", "Dalle");
        this->declare_parameter("timer_", 1.0);

        robot_name_ = this->get_parameter("robot_name_").as_string();
        double timer_period = this->get_parameter("timer_").as_double();

        publisher_ = this->create_publisher<example_interfaces::msg::String>("topic_name_string", 10);
        timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_period), std::bind(&RobotNewsStationNode::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "Robot News Station has been started");
    }
private:
    void publishNews()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = "Hi, this is "+ robot_name_ + std::string(" from the robot news station.");
        publisher_->publish(msg);
    }

    std::string robot_name_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}