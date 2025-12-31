#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::chrono_literals;

class RobotNode : public rclcpp::Node
{
    public:
        RobotNode() : Node("robot_publisher"), robot_name_("R1")
        {
            publisher_ = this->create_publisher<example_interfaces::msg::String>("topic", 10);
            timer_ = this->create_wall_timer(0.5s, std::bind(&RobotNode::publish_cb, this));
            RCLCPP_INFO(this->get_logger(), "publisher started");
        }

    private:
        void publish_cb()
        {
            auto message = example_interfaces::msg::String();
            message.data = robot_name_ + " is moving";
            publisher_->publish(message);
        }
        std::string robot_name_;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}