# ompl_tutorials

The tutorial found in this repository has been established taking another tutorial as base from: [https://bitbucket.org/castacks/core_planning_tutorial/src/master/](https://bitbucket.org/castacks/core_planning_tutorial/src/master/)

To know all the api information from ompl check: [https://ompl.kavrakilab.org/api_overview.html](https://ompl.kavrakilab.org/api_overview.html)

To run the planning example using the OMPL, follow these steps:

1. Download this repo inside your own workspace (can be `cucr-env`).
2. Compile it either with `catkin build`, `catkin_make` or `cucr-make`.
3. To run the planning example:

   - Using `SimpleSetup`:

   > roslaunch core_planning_tutorial plan_sp.launch

   The last launch runs the C++ code in: `ompl_tutorials/core_planning_tutorial/src/planning_tutorial_sp.cpp`

   - Using only `Planner` and `ProblemDefinition`:

   > roslaunch core_planning_tutorial plan.launch

   The last launch runs the C++ code in: `ompl_tutorials/core_planning_tutorial/src/planning_tutorial.cpp`

To understand in depth, you should scheme the `C++` code looking for the functions' definition.
