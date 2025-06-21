#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/ledstatus.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode(): Node("led_panel_node")
    {
        this->declare_parameter("led_states_", std::vector<int64_t>{0, 0, 0});
        this->get_parameter("led_states_", led_states_);
        
        pub_ = create_publisher<my_robot_interfaces::msg::Ledstatus>("led_status_state", 10);
        timer_ = create_wall_timer(5s, std::bind(&LedPanelNode::PublishLedStates, this));
        server_ = create_service<my_robot_interfaces::srv::SetLed>("set_led", std::bind(&LedPanelNode::SetLedService, this, _1, _2));
        RCLCPP_INFO(get_logger(), "Led panel node has been started.");    
    }
private:
    void PublishLedStates()
    {
        auto msg = my_robot_interfaces::msg::Ledstatus();
        msg.led_states = led_states_;
        //RCLCPP_INFO(get_logger(), "Publishing LED states: [%d, %d, %d]", led_states_[0], led_states_[1], led_states_[2]);
        pub_->publish(msg);
    }

    void SetLedService(const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
                        const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
    {
        int64_t led_number = request->led_number;
        int64_t led_state =  request->led_state;
        if (led_number < 0 || led_number >= static_cast<int64_t>(led_states_.size())){
            RCLCPP_ERROR(get_logger(), "Invalid LED number: %ld", led_number);
            response->success = false;
            return;
        }
        if (led_state != 0 && led_state != 1) {
            RCLCPP_ERROR(get_logger(), "Invalid LED state: %ld", led_state);
            response->success = false;
            return;
        }
        led_states_[led_number] = led_state;
        RCLCPP_INFO(get_logger(), "LED %ld set to state %ld", led_number, led_state);
        response->success = true;
        PublishLedStates();
        return;
        
    }
    std::vector<int64_t> led_states_;
    rclcpp::Publisher<my_robot_interfaces::msg::Ledstatus>::SharedPtr pub_;
    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

