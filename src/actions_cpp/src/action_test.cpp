#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/count_until.hpp"

using namespace std::placeholders;

class CountUntilServer: public rclcpp::Node
{
public:
    CountUntilServer(): Node("server_name")
    {  
        goal_queue_thread_ = std::thread(&CountUntilServer::run_goal_queue_thread, this);
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        count_until_server_= rclcpp_action::create_server<robot_interfaces::action::CountUntil>(
            this,
            "server_node",
            std::bind(&CountUntilServer::goal_callback, this, _1, _2),
            std::bind(&CountUntilServer::cancel_callback, this, _1),
            std::bind(&CountUntilServer::handle_accepted_goal, this,_1),
            rcl_action_server_get_default_options(),
            cb_group_
           
        );
    }

private:

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const robot_interfaces::action::CountUntil::Goal> goal_handle
    )
    {
        {
            //std::lock_guard<std::mutex> lock(mutex_);
            // if(goal_handle_->is_active())
            // {
            //     RCLCPP_INFO(this->get_logger(), "Already goal spining");
            //     return rclcpp_action::GoalResponse::REJECT;
            // }
            // return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            std::lock_guard<std::mutex> lock(mutex_);
            // if(goal_handle_&& goal_handle_->is_active())
            // {
            //     preempted_goal_id_= goal_handle_->get_goal_id();
            // }

            // return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;



        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::CountUntil>> goal_handle
    )
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::CountUntil>> goal_handle)
    {
        //execute_goal(goal_handle);
        {
            std::lock_guard<std::mutex> lock(mutex_);
             goal_queue_.push(goal_handle);
        }
       
    }

    void run_goal_queue_thread()
    {
        std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::CountUntil>> next_goal;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(goal_queue_.size()>0)
            {
                next_goal = goal_queue_.front();
                goal_queue_.pop();
            }

            if(next_goal)
            {
                execute_goal(next_goal);
            }

        }
    }

    void execute_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::CountUntil>> goal_handle)
    {
       

        // 가져오기
        int target_number = goal_handle->get_goal()->target_number;
        int period = goal_handle->get_goal()->period;

        //진짜 execute
        int counter = 0;
        auto feedback = std::make_shared<robot_interfaces::action::CountUntil::Feedback>();
        auto result = std::make_shared<robot_interfaces::action::CountUntil::Result>();
         for(int i = 0; i < target_number; i++)
         {}

        //result->reached_number 

    }

    rclcpp_action::Server<robot_interfaces::action::CountUntil>::SharedPtr count_until_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::mutex mutex_;
    std::queue<std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::CountUntil>>> goal_queue_;
    std::thread goal_queue_thread_;


    /*
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
     
    rcl_action_server_get_default_options(),
     cb_group_

    rclcpp::CallbackGroup::SharedPtr cb_group_;
     
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    */
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<CountUntilServer>();
    //rclcpp::spin(node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}