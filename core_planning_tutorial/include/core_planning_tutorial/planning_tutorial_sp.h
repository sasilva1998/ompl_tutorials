//
// Created by jay on 3/31/20.
//

#ifndef CORE_OCTOMAP_MAP_REPRESENTATION_PLANNING_TUTORIAL_H
#define CORE_OCTOMAP_MAP_REPRESENTATION_PLANNING_TUTORIAL_H

#include <base/BaseNode.h>
#include <string>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include "core_planning_state_space/state_spaces/xyzpsi_state_space.h"
#include "core_planning_state_space/state_space_utils/xyzpsi_state_space_utils.h"

#include <core_map_representation_interface/map_representation.h>
#include <pluginlib/class_loader.h>

class PlanningTutorial : public BaseNode
{

public:
    PlanningTutorial(std::string node_name);

    virtual bool initialize();
    virtual bool execute();
    virtual ~PlanningTutorial();

private:
    bool isvalid(const ompl::base::State *state); // TODO Make a class to handle this

    ompl::base::ScopedState<XYZPsiStateSpace> getStartState();

    ompl::base::ScopedState<XYZPsiStateSpace> getGoalState();

    ompl::base::ProblemDefinitionPtr pdef;

    ompl::base::SpaceInformationPtr si_xyzpsi;

    ompl::geometric::SimpleSetupPtr simple_setup_;

    boost::shared_ptr<MapRepresentation> map;

    ros::Publisher path_pub;
};

#endif // CORE_OCTOMAP_MAP_REPRESENTATION_PLANNING_TUTORIAL_H
