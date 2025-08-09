#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/count_until.hpp"

using namespace std::placeholders;

class Servertest: public rclcpp::Node
{
public:
    Servertest():Node("Node_name")
    {
        server_= rclcpp_action::create_server<robot_interfaces::action::CountUntil>(
            this,
            "server_name",
            std::bind(&Servertest::goal_callback,this, _1, _2),
            std::bind(&Servertest::cancel_callback,this,_1),
            std::bind(&Servertest::handle_accepted_callback,this,_1)
        );
    }

private:

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const robot_interfaces::action::CountUntil::Goal> goal
    )
    {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::CountUntil>> goal_handle
    )
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::CountUntil>> goal_handle)
    {
        execute_goal(goal_handle);
    }

    void execute_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::CountUntil>> goal_handle)
    {
        auto result = std::make_shared<robot_interfaces::action::CountUntil::Result>();
        goal_handle->succeed(result);
    }

    rclcpp_action::Server<robot_interfaces::action::CountUntil>::SharedPtr server_;
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Servertest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

