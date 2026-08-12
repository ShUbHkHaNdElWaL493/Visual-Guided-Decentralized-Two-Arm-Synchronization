#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/task.h>

using namespace moveit::task_constructor;

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("ur_loop_node", node_options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    geometry_msgs::msg::PoseStamped p1, p2, p3;
    p1.header.frame_id = "world";
    p1.pose.position.x = 0.4;
    p1.pose.position.y = 0.2;
    p1.pose.position.z = 1.4;
    p1.pose.orientation.w = 0.7071;
    p1.pose.orientation.y = -0.7071;
    p2.header.frame_id = "world";
    p2.pose.position.x = 0.4;
    p2.pose.position.y = -0.2;
    p2.pose.position.z = 1.4;
    p2.pose.orientation.w = 0.7071;
    p2.pose.orientation.y = -0.7071;
    p3.header.frame_id = "world";
    p3.pose.position.x = 0.4;
    p3.pose.position.y = 0.0;
    p3.pose.position.z = 1.4;
    p3.pose.orientation.w = 0.7071;
    p3.pose.orientation.y = -0.7071;
    geometry_msgs::msg::Pose ref1, ref2, ref3, ref4, ref5, ref6;
    ref1.position.x = 0.4;
    ref1.position.y = 0.0;
    ref1.position.z = 1.2;
    ref1.orientation.w = 0.7071;
    ref1.orientation.y = -0.7071;
    ref2.position.x = 0.4;
    ref2.position.y = 0.0;
    ref2.position.z = 1.6;
    ref2.orientation.w = 0.7071;
    ref2.orientation.y = -0.7071;
    ref3.position.x = 0.4;
    ref3.position.y = 0.1;
    ref3.position.z = 1.3;
    ref3.orientation.w = 0.7071;
    ref3.orientation.y = -0.7071;
    ref4.position.x = 0.4;
    ref4.position.y = -0.1;
    ref4.position.z = 1.5;
    ref4.orientation.w = 0.7071;
    ref4.orientation.y = -0.7071;
    ref5.position.x = 0.4;
    ref5.position.y = -0.1;
    ref5.position.z = 1.3;
    ref5.orientation.w = 0.7071;
    ref5.orientation.y = -0.7071;
    ref6.position.x = 0.4;
    ref6.position.y = 0.1;
    ref6.position.z = 1.5;
    ref6.orientation.w = 0.7071;
    ref6.orientation.y = -0.7071;

    Task loop_task;
    loop_task.stages()->setName("ur_loop");
    loop_task.loadRobotModel(node);
    
    auto loop_planner = std::make_shared<solvers::PipelinePlanner>(
        node,
        "pilz_industrial_motion_planner",
        "CIRC"
    );
    loop_planner->setProperty("max_velocity_scaling_factor", 0.05);
    loop_planner->setProperty("max_acceleration_scaling_factor", 0.05);

    /***************************************************
    *                                                  *
    *                  Current State                   *
    *                                                  *
    ***************************************************/
    {
        auto stage = std::make_unique<stages::CurrentState>("Current state");
        loop_task.add(std::move(stage));
    }

    /***************************************************
    *                                                  *
    *                   Loop Stage 1                   *
    *                                                  *
    ***************************************************/
    {

        moveit_msgs::msg::PositionConstraint position_constraint;
        position_constraint.header.frame_id = "world"; 
        position_constraint.link_name = "ur_tool0";
        position_constraint.constraint_region.primitive_poses.resize(1);
        position_constraint.constraint_region.primitive_poses[0] = ref1;
        position_constraint.weight = 1.0;

        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.name = "interim"; 
        path_constraints.position_constraints.push_back(position_constraint);

        auto stage = std::make_unique<stages::MoveTo>("Loop stage 1", loop_planner);
        stage->setGroup("ur_arm");
        stage->setIKFrame("ur_tool0");
        stage->setGoal(p2);
        stage->setPathConstraints(path_constraints);
        loop_task.add(std::move(stage));

    }

    /***************************************************
    *                                                  *
    *                   Loop Stage 2                   *
    *                                                  *
    ***************************************************/
    {

        moveit_msgs::msg::PositionConstraint position_constraint;
        position_constraint.header.frame_id = "world"; 
        position_constraint.link_name = "ur_tool0";
        position_constraint.constraint_region.primitive_poses.resize(1);
        position_constraint.constraint_region.primitive_poses[0] = ref2;
        position_constraint.weight = 1.0;

        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.name = "interim"; 
        path_constraints.position_constraints.push_back(position_constraint);

        auto stage = std::make_unique<stages::MoveTo>("Loop stage 2", loop_planner);
        stage->setGroup("ur_arm");
        stage->setIKFrame("ur_tool0");
        stage->setGoal(p1);
        stage->setPathConstraints(path_constraints);
        loop_task.add(std::move(stage));

    }

    /***************************************************
    *                                                  *
    *                   Loop Stage 3                   *
    *                                                  *
    ***************************************************/
    {

        moveit_msgs::msg::PositionConstraint position_constraint;
        position_constraint.header.frame_id = "world"; 
        position_constraint.link_name = "ur_tool0";
        position_constraint.constraint_region.primitive_poses.resize(1);
        position_constraint.constraint_region.primitive_poses[0] = ref3;
        position_constraint.weight = 1.0;

        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.name = "interim"; 
        path_constraints.position_constraints.push_back(position_constraint);

        auto stage = std::make_unique<stages::MoveTo>("Loop stage 3", loop_planner);
        stage->setGroup("ur_arm");
        stage->setIKFrame("ur_tool0");
        stage->setGoal(p3);
        stage->setPathConstraints(path_constraints);
        loop_task.add(std::move(stage));

    }

    /***************************************************
    *                                                  *
    *                   Loop Stage 4                   *
    *                                                  *
    ***************************************************/
    {

        moveit_msgs::msg::PositionConstraint position_constraint;
        position_constraint.header.frame_id = "world"; 
        position_constraint.link_name = "ur_tool0";
        position_constraint.constraint_region.primitive_poses.resize(1);
        position_constraint.constraint_region.primitive_poses[0] = ref4;
        position_constraint.weight = 1.0;

        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.name = "interim"; 
        path_constraints.position_constraints.push_back(position_constraint);

        auto stage = std::make_unique<stages::MoveTo>("Loop stage 4", loop_planner);
        stage->setGroup("ur_arm");
        stage->setIKFrame("ur_tool0");
        stage->setGoal(p2);
        stage->setPathConstraints(path_constraints);
        loop_task.add(std::move(stage));

    }

    /***************************************************
    *                                                  *
    *                   Loop Stage 5                   *
    *                                                  *
    ***************************************************/
    {

        moveit_msgs::msg::PositionConstraint position_constraint;
        position_constraint.header.frame_id = "world"; 
        position_constraint.link_name = "ur_tool0";
        position_constraint.constraint_region.primitive_poses.resize(1);
        position_constraint.constraint_region.primitive_poses[0] = ref5;
        position_constraint.weight = 1.0;

        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.name = "interim"; 
        path_constraints.position_constraints.push_back(position_constraint);

        auto stage = std::make_unique<stages::MoveTo>("Loop stage 5", loop_planner);
        stage->setGroup("ur_arm");
        stage->setIKFrame("ur_tool0");
        stage->setGoal(p3);
        stage->setPathConstraints(path_constraints);
        loop_task.add(std::move(stage));

    }

    /***************************************************
    *                                                  *
    *                   Loop Stage 6                   *
    *                                                  *
    ***************************************************/
    {

        moveit_msgs::msg::PositionConstraint position_constraint;
        position_constraint.header.frame_id = "world"; 
        position_constraint.link_name = "ur_tool0";
        position_constraint.constraint_region.primitive_poses.resize(1);
        position_constraint.constraint_region.primitive_poses[0] = ref6;
        position_constraint.weight = 1.0;

        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.name = "interim"; 
        path_constraints.position_constraints.push_back(position_constraint);

        auto stage = std::make_unique<stages::MoveTo>("Loop stage 6", loop_planner);
        stage->setGroup("ur_arm");
        stage->setIKFrame("ur_tool0");
        stage->setGoal(p1);
        stage->setPathConstraints(path_constraints);
        loop_task.add(std::move(stage));

    }

    try
    {
        loop_task.init();
    } catch (InitStageException& e)
    {
        RCLCPP_ERROR(node->get_logger(), "ur_loop initialization failed.");
        return -1;
    }
    loop_task.plan();
    if (loop_task.solutions().empty())
    {
        RCLCPP_ERROR(node->get_logger(), "ur_loop planning failed.");
    }
    while (rclcpp::ok())
    {
        auto loop_task_result = loop_task.execute(*(loop_task.solutions().front()));
        if (loop_task_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "ur_loop execution failed.");
            return -1;
        }
    }

    rclcpp::shutdown();
    return 0;

}