#ifndef CARTESIAN_MOTION_TASKS_HPP__
#define CARTESIAN_MOTION_TASKS_HPP__

// std
#include <stdlib.h>
#include <sstream>
#include <thread>
#include <cstring>
#include "cartesian_motion_base/cartesian_motion_type.hpp"

#define TaskPtr std::make_shared<cartesian_motion_base::MotionTask>
#define TaskPtrConst std::make_shared<const cartesian_motion_base::MotionTask>

namespace cartesian_motion_base
{
/**
 * @brief Abstract base class for robot motion tasks.
 *
 * This class defines the interface for robot motion tasks. Derived classes should implement
 * the execute, cancel, isFinished, isActive, getName, getStatus, and getErrorMessage methods.
 */
class MotionTask
{
    public:

    /// @brief Initialize the task with task_name and execute function
    /// @param task_name 
    /// @param execute 
    explicit MotionTask(std::string task_name, std::function<void()> execute)
        : task_name_(task_name), execute_(execute){
            init_ = [](){}; // empty init function
        };
    
    /// @brief Initialize the task with task_name, init function, and execute function
    /// @param task_name 
    /// @param init 
    /// @param execute 
    explicit MotionTask(std::string task_name, std::function<void()> init, std::function<void()> execute)
        : task_name_(task_name), init_(init), execute_(execute){};

    void init(){
        init_();
    }

    void execute(){
        execute_();
    }

    std::string get_name(){
        return task_name_;
    }

    TaskState get_state(){
        return state_;
    }

    void set_state(TaskState state){
        state_ = state;
    }

private:
    std::string task_name_;
    std::function<void()> init_;
    std::function<void()> execute_;

    TaskState state_ = TaskState::INIT;
};



} // namespace cartesian_motion_base



#endif // CARTESIAN_MOTION_TASKS_HPP__