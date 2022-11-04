//
// Created by jay on 3/31/20.
//

#include "../include/core_planning_tutorial/planning_tutorial_sp.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

PlanningTutorial::PlanningTutorial(std::string node_name)
    : BaseNode(node_name)
{
}

bool PlanningTutorial::initialize()
{
    ROS_INFO("Planner Initializing...");

    ros::NodeHandle *nh = get_node_handle();
    ros::NodeHandle *pnh = get_private_node_handle();

    std::string map_representation = pnh->param("map_representation", std::string("PointCloudMapRepresentation"));

    path_pub = nh->advertise<visualization_msgs::Marker>("solved_path", 10);

    pluginlib::ClassLoader<MapRepresentation> map_representation_loader("core_map_representation_interface", "MapRepresentation");
    try
    {
        map = map_representation_loader.createInstance(map_representation);
    }
    catch (pluginlib::PluginlibException &ex)
    {
        ROS_ERROR("The MapRepresentation plugin failed to load. Error: %s", ex.what());
    }

    return true;
}

bool PlanningTutorial::execute()
{
    // ! 1. configuration space definition
    // bound object to define configuration space size
    ompl::base::RealVectorBounds bounds(3);

    // define map or space representation limits
    double x(0), y(0), z(0);

    // get map dimensions and set bounds
    map->getLowBounds(x, y, z);
    bounds.setLow(0, x);
    bounds.setLow(1, y);
    bounds.setLow(2, z);

    map->getHighBounds(x, y, z);
    bounds.setHigh(0, x);
    bounds.setHigh(1, y);
    bounds.setHigh(2, 100);

    // set SpaceInformation
    si_xyzpsi = GetStandardXYZPsiSpacePtr();
    // set bounds to space information
    si_xyzpsi->getStateSpace()->as<XYZPsiStateSpace>()->setBounds(bounds);

    // ! define simple setup
    // give spaceinformation to simplesetup
    simple_setup_ = og::SimpleSetupPtr(new og::SimpleSetup(si_xyzpsi));

    // ! 2. planner
    // define which planner is desired
    auto planner = std::make_shared<og::RRTstar>(si_xyzpsi);

    // pass planner to simple setup
    simple_setup_->setPlanner(planner);

    // ! 3. define start and goals
    auto start = getStartState();
    auto goal = getGoalState();

    // set start and goal
    simple_setup_->setStartState(start);
    simple_setup_->setGoalState(goal);

    // ! 4. set state validity checker
    simple_setup_->setStateValidityChecker(std::bind(&PlanningTutorial::isvalid, this, std::placeholders::_1));

    // ! 5. set optimization objective
    simple_setup_->getProblemDefinition()->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si_xyzpsi)));

    simple_setup_->setup();

    // ! 6. attempt to solve
    ob::PlannerStatus solved = simple_setup_->solve(0.5);

    // ==================================

    ROS_INFO("Planner Initialized");

    if (solved == ob::PlannerStatus::EXACT_SOLUTION)
    {
        ROS_INFO("Solved");

        og::PathGeometric path = simple_setup_->getSolutionPath();

        path.interpolate(std::max(20, (int)path.getStateCount()));
        path_pub.publish(GetMarker(path, 5, 0, 1, 0, 1));
    }

    return true;
}

ob::ScopedState<XYZPsiStateSpace> PlanningTutorial::getStartState()
{

    ob::ScopedState<XYZPsiStateSpace> state(si_xyzpsi);
    // state->SetXYZ(Eigen::Vector3d(562,-584,50));//TODO add tf
    state->setXYZ(Eigen::Vector3d(-834, -437, 50)); // TODO add tf
    state->setPsi(0.0);

    return state;
}

ob::ScopedState<XYZPsiStateSpace> PlanningTutorial::getGoalState()
{

    ob::ScopedState<XYZPsiStateSpace> state(si_xyzpsi);
    // state->SetXYZ(Eigen::Vector3d(-2290,-206,50));
    state->setXYZ(Eigen::Vector3d(-1553, -390, 50));
    state->setPsi(0.0);

    return state;
}

bool PlanningTutorial::isvalid(const ompl::base::State *state)
{

    Eigen::Vector3d pos = state->as<XYZPsiStateSpace::StateType>()->getXYZ();
    geometry_msgs::Point point;
    point.x = pos.x();
    point.y = pos.y();
    point.z = pos.z(); // TODO Push in utils

    return map->isvalid(point);
}

PlanningTutorial::~PlanningTutorial()
{
}

BaseNode *BaseNode::get()
{
    PlanningTutorial *planning_tutorial = new PlanningTutorial("PlanningTutorial");

    return planning_tutorial;
}