#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;
class BatteryNode : public rclcpp::Node
{
public:
    double now_;
    double first_time_;
    std::string battery_state_;
    BatteryNode(): Node("battery_node")
    {
        RCLCPP_INFO(get_logger(), "Battery Node started as service client CPP");
        client_ = create_client<my_robot_interfaces::srv::SetLed>("set_led");
        timer_ = create_wall_timer(1s, std::bind(&BatteryNode::battery_status_client, this));
        first_time_ = current_time();
        battery_state_ = "full";
    }
private:
    void battery_status_client()
    {
        now_ = current_time();
        RCLCPP_INFO(get_logger(), "current time: %f", now_);
        if (battery_state_ == "full" && (now_ - first_time_ > 4.0)) {
            RCLCPP_INFO(get_logger(), "Battery is full, changing leds to full");
            call_set_led(2, 1);
            call_set_led(1, 1);
            call_set_led(0, 1);
            battery_state_ = "empty";
            now_ = current_time();
        } else if (battery_state_ == "empty" && (now_ - first_time_ > 6.0)) {
            RCLCPP_INFO(get_logger(), "Battery is empty, changing leds to empty");
            call_set_led(0, 0);
            call_set_led(1, 0);
            call_set_led(2, 0);
            battery_state_ = "full";
            now_ = current_time();
        }

    }

    void call_set_led(int64_t led_number, int64_t led_state)
    {
        auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
        request->led_number = led_number;
        request->led_state = led_state;

        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
            return;
        }

        auto future = client_->async_send_request(request, [this, led_number, led_state](rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedFuture response) {
            if (response.get()->success) {
                RCLCPP_INFO(get_logger(), "set_led service call successfull for led %ld with state %ld", led_number, led_state);
            } else {
                RCLCPP_INFO(get_logger(), "set_led service call failed for led %ld with state %ld", led_number, led_state);
            }
            }
        );
    }

    double current_time()
    {
        rclcpp::Time now = get_clock()->now();
        return now.seconds();
    }
    rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
