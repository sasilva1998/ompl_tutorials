//
// Created by jay on 3/31/20.
//

#include "../include/core_planning_tutorial/planning_tutorial.h"

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

    ompl::base::RealVectorBounds bounds(3);

    double x(0), y(0), z(0);

    map->getLowBounds(x, y, z);
    bounds.setLow(0, x);
    bounds.setLow(1, y);
    bounds.setLow(2, z);

    map->getHighBounds(x, y, z);
    bounds.setHigh(0, x);
    bounds.setHigh(1, y);
    bounds.setHigh(2, 100);

    si_xyzpsi = GetStandardXYZPsiSpacePtr();
    si_xyzpsi->getStateSpace()->as<XYZPsiStateSpace>()->setBounds(bounds);
    si_xyzpsi->setStateValidityCheckingResolution(1.0 / si_xyzpsi->getMaximumExtent());
    si_xyzpsi->setStateValidityChecker(std::bind(&PlanningTutorial::isvalid, this, std::placeholders::_1));
    si_xyzpsi->setup();

    pdef = std::make_shared<ob::ProblemDefinition>(si_xyzpsi);
    pdef->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si_xyzpsi)));

    auto start = getStartState();
    auto goal = getGoalState();
    pdef->setStartAndGoalStates(start, goal);

    auto planner = std::make_shared<og::RRTstar>(si_xyzpsi);
    planner->setProblemDefinition(pdef);
    planner->setup();

    ROS_INFO("Planner Initialized");

    auto solved = planner->ob::Planner::solve(30.0); // seconds (edited)

    if (solved == ob::PlannerStatus::EXACT_SOLUTION)
    {
        ROS_INFO("Solved");

        auto path = std::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
        path->interpolate(std::max(20, (int)path->getStateCount()));
        path_pub.publish(GetMarker(*path, 5, 0, 1, 0, 1));
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