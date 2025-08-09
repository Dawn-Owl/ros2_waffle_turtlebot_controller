#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/count_until.hpp"

using namespace std::placeholders;

class CountUntilClient: public rclcpp::Node
{
public:
    CountUntilClient(): Node("client_name")
    {
        count_until_client_ = rclcpp_action::create_client<robot_interfaces::action::CountUntil>(this,"server_node");

    }

    void send_goal()
    {
        count_until_client_ -> wait_for_action_server();

        auto goal = robot_interfaces::action::CountUntil::Goal();
        goal.period = 2.0;
        goal.target_number = 3;

        auto options = rclcpp_action::Client<robot_interfaces::action::CountUntil>::SendGoalOptions();
        options.result_callback = std::bind(&CountUntilClient::result_callback, this, _1);
        options.goal_response_callback = std::bind(&CountUntilClient::goal_response_callback, this, _1);
        options.feedback_callback = std::bind(&CountUntilClient::feedback_callback, this, _1,_2);

        count_until_client_->async_send_goal(goal, options);
    }
    

private:

    void goal_response_callback(const rclcpp_action::ClientGoalHandle<robot_interfaces::action::CountUntil>::SharedPtr &goal_handle)
    {
        
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<robot_interfaces::action::CountUntil>::WrappedResult &result)
    {

    }

    void feedback_callback(const rclcpp_action::ClientGoalHandle<robot_interfaces::action::CountUntil>::SharedPtr &goal_handle,
        const std::shared_ptr<const robot_interfaces::action::CountUntil::Feedback> feedback    
    )
    {

    }

    rclcpp_action::Client<robot_interfaces::action::CountUntil>::SharedPtr count_until_client_;

   
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<CountUntilClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}