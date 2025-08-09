#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/count_until.hpp"

using namespace std::placeholders;

class Clienttest: public rclcpp::Node
{
public:
    Clienttest():Node("Node_name")
    {
        client_ = rclcpp_action::create_client<robot_interfaces::action::CountUntil>(this, "server_name");
    }

    void send_goal()
    {
        client_->wait_for_action_server();

        auto goal = robot_interfaces::action::CountUntil::Goal();


        auto options = rclcpp_action::Client<robot_interfaces::action::CountUntil>::SendGoalOptions();
        options.goal_response_callback = std::bind(&);


        client_->async_send_goal(goal);
    }

private:

    rclcpp_action::Client<robot_interfaces::action::CountUntil>::SharedPtr client_;

   
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Clienttest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

