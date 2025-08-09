#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/move_robot.hpp"

using namespace std::placeholders;

class MoveRobotServerNode: public rclcpp::Node
{
public:
    MoveRobotServerNode(): Node("move_robot_node")
    {
        robot_position_ = 50;
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        move_robot_server_ = rclcpp_action::create_server<robot_interfaces::action::MoveRobot>(
            this,
            "move_robot",
            std::bind(&MoveRobotServerNode::goal_callback, this, _1, _2),
            std::bind(&MoveRobotServerNode::cancel_callback, this, _1),
            std::bind(&MoveRobotServerNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_
            );
        RCLCPP_INFO(this->get_logger(), "Action server start!!!");
        RCLCPP_INFO(this->get_logger(), "Robot position: %d", robot_position_);
    }

private:

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const robot_interfaces::action::MoveRobot::Goal> goal
    )
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received a new goal");

        if ((goal->position < 0) || (goal->position > 100) || (goal->velocity <= 0)) {
            RCLCPP_INFO(this->get_logger(), "Invalid value Rejected!!!");
            return rclcpp_action::GoalResponse::REJECT;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle_) {
                if (goal_handle_->is_active()) {
                    preempted_goal_id_ = goal_handle_->get_goal_id();
                }
            }
        }


        // Accept goal
        RCLCPP_INFO(this->get_logger(), "Accepted goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::MoveRobot>> goal_handle)
    {
        (void)goal_handle;


        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::MoveRobot>> goal_handle)
    {
        execute_goal(goal_handle);
    }

    void execute_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::MoveRobot>> goal_handle)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }


        int goal_position = goal_handle->get_goal()->position;
        int velocity = goal_handle->get_goal()->velocity;


        auto result = std::make_shared<robot_interfaces::action::MoveRobot::Result>();
        auto feedback = std::make_shared<robot_interfaces::action::MoveRobot::Feedback>();
        rclcpp::Rate loop_rate(1.0);

        RCLCPP_INFO(this->get_logger(), "Execute goal");
        while (rclcpp::ok()) {
            // Check preempt goal
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (goal_handle->get_goal_id() == preempted_goal_id_) {
                    result->position = robot_position_;
                    result->message = "Another goal accepted!!!";
                    goal_handle->abort(result);
                    return;
                }
            }

            // Check cancel request
            if (goal_handle->is_canceling()) {
                result->position = robot_position_;
                if (goal_position == robot_position_) {
                   result->message = "Success";
                   goal_handle->succeed(result); 
                }
                else {
                    result->message = "Canceled";
                    goal_handle->canceled(result);
                }
                return;
            }

            int gap = goal_position - robot_position_;

            if (gap == 0) {
                result->position = robot_position_;
                result->message = "Success";
                goal_handle->succeed(result);
                return;
            } else if (gap > 0) {
                robot_position_ += std::min(gap, velocity);
            } else {  
                robot_position_ -= std::min(abs(gap), velocity);
            }   


            RCLCPP_INFO(this->get_logger(), "Robot position: %d", robot_position_);
            feedback->current_position = robot_position_;
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        
        //feedback->current_position
            }
        }        
    int robot_position_;
    rclcpp_action::Server<robot_interfaces::action::MoveRobot>::SharedPtr move_robot_server_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::MoveRobot>> goal_handle_;
    rclcpp_action::GoalUUID preempted_goal_id_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::mutex mutex_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MoveRobotServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}