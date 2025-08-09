#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/move_robot.hpp"
#include "example_interfaces/msg/empty.hpp"


using namespace std::placeholders;

class MoveRobotClientNode: public rclcpp::Node
{
public:
    MoveRobotClientNode(): Node("move_robot_client_node")
    {
        move_robot_client_ = rclcpp_action::create_client<robot_interfaces::action::MoveRobot>(this, "move_robot");

        cancel_subscriber_ = this->create_subscription<example_interfaces::msg::Empty>(
            "cancel_move", 10, std::bind(&MoveRobotClientNode::cancel_callback, this, _1));

    }

    void send_goal(int position, int velocity)
    {
        move_robot_client_->wait_for_action_server();

        auto goal = robot_interfaces::action::MoveRobot::Goal();
        goal.position = position;
        goal.velocity = velocity;

        auto options = rclcpp_action::Client<robot_interfaces::action::MoveRobot>::SendGoalOptions();
        options.goal_response_callback = std::bind(&MoveRobotClientNode::goal_response_callback, this, _1);
        options.result_callback = std::bind(&MoveRobotClientNode::goal_result_callback, this, _1);
        options.feedback_callback = std::bind(&MoveRobotClientNode::feedback_callback, this, _1,_2);

        RCLCPP_INFO(this->get_logger(), "Send position: %d , velocity: %d", position, velocity);
        move_robot_client_->async_send_goal(goal,options);
    }
    
private:

    void goal_response_callback(const rclcpp_action::ClientGoalHandle<robot_interfaces::action::MoveRobot>::SharedPtr &goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal got rejected");
        }
        else {
            this->goal_handle_ = goal_handle;
            RCLCPP_INFO(this->get_logger(), "Goal got accepted");
        }
    }

    void goal_result_callback(const rclcpp_action::ClientGoalHandle<robot_interfaces::action::MoveRobot>::WrappedResult &result)
    {
        auto status = result.code;        
        if (status == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Succeeded");
        }
        else if (status == rclcpp_action::ResultCode::ABORTED) {
            RCLCPP_ERROR(this->get_logger(), "Aborted");
        }
        else if (status == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_WARN(this->get_logger(), "Canceled");
        }

        int position = result.result->position;
        std::string message = result.result->message;
        
        RCLCPP_INFO(this->get_logger(), "Position: %d", position);
        RCLCPP_INFO(this->get_logger(), "Message: %s", message.c_str());
    }

    void feedback_callback(const rclcpp_action::ClientGoalHandle<robot_interfaces::action::MoveRobot>::SharedPtr &goal_handle, 
        const std::shared_ptr<const robot_interfaces::action::MoveRobot::Feedback> feedback
    )
    {
        (void)goal_handle;
        int position = feedback->current_position;
        RCLCPP_INFO(this->get_logger(), "Feedback position: %d", position);
    }

    void cancel_callback(const example_interfaces::msg::Empty::SharedPtr msg)
    {
        (void)msg;
        cancel_goal();
    }

    void cancel_goal()
    {
        if (this->goal_handle_) {
            this->move_robot_client_->async_cancel_goal(goal_handle_);
            goal_handle_.reset();
        }
    }

    rclcpp_action::Client<robot_interfaces::action::MoveRobot>::SharedPtr move_robot_client_;
    rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr cancel_subscriber_;
    rclcpp_action::ClientGoalHandle<robot_interfaces::action::MoveRobot>::SharedPtr goal_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MoveRobotClientNode>();
    node->send_goal(92, 3);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}