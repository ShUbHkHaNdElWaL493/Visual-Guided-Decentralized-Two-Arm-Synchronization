#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/task.h>

using namespace moveit::task_constructor;

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ur_pickup_node", node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() {executor.spin();}).detach();

  moveit_msgs::msg::CollisionObject cube;
  cube.header.frame_id = "world";
  cube.id = "target";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {0.05, 0.05, 0.05};
  geometry_msgs::msg::Pose primitive_pose;
  primitive_pose.position.x = 1.0;
  primitive_pose.position.y = 0.3;
  primitive_pose.position.z = 0.875;
  primitive_pose.orientation.w = 1.0;
  cube.primitives.push_back(primitive);
  cube.primitive_poses.push_back(primitive_pose);
  cube.operation = cube.ADD;
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(cube);

  geometry_msgs::msg::PoseStamped loop_start_fer;
  loop_start_fer.header.frame_id = "world";
  loop_start_fer.pose.position.x = -0.3;
  loop_start_fer.pose.position.y = 0.2;
  loop_start_fer.pose.position.z = 1.5;
  loop_start_fer.pose.orientation.w = 0.7071;
  loop_start_fer.pose.orientation.y = 0.7071;

  Task pickup_task;
  pickup_task.stages()->setName("ur_pickup");
  pickup_task.loadRobotModel(node);
  pickup_task.setProperty("group", "ur_arm");
  pickup_task.setProperty("eef", "ur_eef");
  pickup_task.setProperty("hand", "ur_gripper");
  pickup_task.setProperty("ik_frame", "ur_tool0");

  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(
        node,
        "ompl",
        "RRTConnectkConfigDefault"
  );

  auto cartesian_planner = std::make_shared<solvers::PipelinePlanner>(
        node,
        "pilz_industrial_motion_planner",
        "LIN"
  );
  cartesian_planner->setProperty("max_velocity_scaling_factor", 0.3);
  cartesian_planner->setProperty("max_acceleration_scaling_factor", 0.3);

    /***************************************************
    *                                                  *
    *                  Current State                   *
    *                                                  *
    ***************************************************/
  {
    auto stage = std::make_unique<stages::CurrentState>("Current state");
    pickup_task.add(std::move(stage));
  }

    /***************************************************
    *                                                  *
    *                  Open Gripper                    *
    *                                                  *
    ***************************************************/
  Stage * initial_state_ptr = nullptr;
  {
    auto stage = std::make_unique<stages::MoveTo>("Open gripper", sampling_planner);
    stage->setGroup("ur_gripper");
    stage->setGoal("open");
    initial_state_ptr = stage.get();
    pickup_task.add(std::move(stage));
  }

        /***************************************************
        *                                                  *
        *                 Move To Pickup                   *
        *                                                  *
        ***************************************************/
  {
    stages::Connect::GroupPlannerVector planners = {
      {"ur_arm", sampling_planner},
      {"ur_gripper", sampling_planner}
    };
    auto stage = std::make_unique<stages::Connect>("Move to pickup", planners);
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(Stage::PARENT);
    pickup_task.add(std::move(stage));
  }

    /***************************************************
        *                                                  *
        *                   Pick Object                    *
        *                                                  *
        ***************************************************/
  {
    auto grasp = std::make_unique<SerialContainer>("Pick object");
    pickup_task.properties().exposeTo(grasp->properties(), {"eef", "hand", "group", "ik_frame"});
    grasp->properties().configureInitFrom(Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

                /***************************************************
    --- *                 Approach Object                  *
                ***************************************************/
    {
      auto stage = std::make_unique<stages::MoveRelative>("Approach object", cartesian_planner);
      stage->properties().set("link", "ur_tool0");
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.15);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "ur_tool0";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

                /***************************************************
    --- *               Generate Grasp Pose                *
                ***************************************************/
    {

      auto stage = std::make_unique<stages::GenerateGraspPose>("Generate grasp pose");
      stage->properties().configureInitFrom(Stage::PARENT);
      stage->setPreGraspPose("open");
      stage->setObject("target");
      stage->setAngleDelta(M_PI / 2);
      stage->setMonitoredStage(initial_state_ptr);

      auto wrapper = std::make_unique<stages::ComputeIK>("Compute grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(
                Eigen::Translation3d(0, 0, 0.16) *
                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()),
                "ur_tool0"
      );
      wrapper->properties().configureInitFrom(Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
      grasp->insert(std::move(wrapper));
    }

                /***************************************************
    --- *          Allow Collision (Hand, Object)          *
                ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("Allow collision (hand, object)");
      stage->allowCollisions(
                            "target",
                            pickup_task.getRobotModel()->getJointModelGroup(
        "ur_gripper")->getLinkModelNamesWithCollisionGeometry(),
                            true
      );
      grasp->insert(std::move(stage));
    }

                /***************************************************
    --- *                   Close Hand                     *
                ***************************************************/
    {
      auto stage = std::make_unique<stages::MoveTo>("Close hand", sampling_planner);
      stage->setGroup("ur_gripper");
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

                /***************************************************
    --- *                 Attach Object                    *
                ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("Attach object");
      stage->attachObject("target", "ur_tool0");
      grasp->insert(std::move(stage));
    }

                /***************************************************
    --- *        Allow Collision (Object, Surface)         *
                ***************************************************/
    {
      auto stage =
        std::make_unique<stages::ModifyPlanningScene>("Allow collision (object, surface)");
      stage->allowCollisions("target", "ur_table", true);
      grasp->insert(std::move(stage));
    }

                /***************************************************
    --- *                  Lift Object                     *
                ***************************************************/
    {
      auto stage = std::make_unique<stages::MoveRelative>("Lift object", cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.05, 0.1);
      stage->setIKFrame("ur_tool0");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

                /***************************************************
    --- *         Forbid Collision (Object, Surface)       *
                ***************************************************/
    {
      auto stage =
        std::make_unique<stages::ModifyPlanningScene>("Forbid collision (object, surface)");
      stage->allowCollisions("target", "ur_table", false);
      grasp->insert(std::move(stage));
    }

    pickup_task.add(std::move(grasp));
  }

    /***************************************************
    *                                                  *
    *              Move To loop_start_ur               *
    *                                                  *
    ***************************************************/
  {
    auto stage = std::make_unique<stages::MoveTo>("Move to loop_start_ur", sampling_planner);
    stage->setGroup("ur_arm");
    stage->setGoal("loop_start");
    pickup_task.add(std::move(stage));
  }

    /***************************************************
    *                                                  *
    *              Move To loop_start_fer              *
    *                                                  *
    ***************************************************/
  {
    auto stage = std::make_unique<stages::MoveTo>("Move to loop_start_fer", sampling_planner);
    stage->setGroup("fer_arm");
    stage->setIKFrame("fer_link8");
    stage->setGoal(loop_start_fer);
    pickup_task.add(std::move(stage));
  }

  try {
    pickup_task.init();
  } catch (InitStageException & e) {
    RCLCPP_ERROR(node->get_logger(), "pickup_task initialization failed.");
    return -1;
  }
  while (pickup_task.solutions().empty()) {
    pickup_task.plan();
    if (pickup_task.solutions().empty()) {
      RCLCPP_ERROR(node->get_logger(), "pickup_task planning failed.");
    }
  }
  auto pickup_task_result = pickup_task.execute(*(pickup_task.solutions().front()));
  if (pickup_task_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "pickup_task execution failed.");
    return -1;
  }

  rclcpp::shutdown();
  return 0;

}
