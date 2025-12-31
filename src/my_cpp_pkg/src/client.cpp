#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class ClientNode : public rclcpp::Node
{
    public:
        ClientNode() : Node("client")
        {
            client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        }
        void send_request(int a, int b)
        {
            while (!client_->wait_for_service(1s))
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for the server to be available");
            }
            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            request->a = a;
            request->b = b;
            client_->async_send_request(
                request,
                std::bind(&ClientNode::response_callback, this, _1)
            );
        }
    private:
        void response_callback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
        {
            RCLCPP_INFO(this->get_logger(), "Sum: %ld", future.get()->sum);
        }
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClientNode>();
    node->send_request(1, 2);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}