/*********************************************************************
 * Rice University Software Distribution License
 *
 * Copyright (c) 2012, Rice University
 * All Rights Reserved.
 *
 * For a full description see the file named LICENSE.
 *
 *********************************************************************/

/* Author: Ryan Luna */

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/util/Console.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/config.h>
#include <omplapp/geometry/detail/FCLContinuousMotionValidator.h>
#include <vector>

using namespace ompl;

void configureApartmentProblem(app::SE3RigidBodyPlanning &setup)
{
    std::string problem = "Apartment";
    std::string environment_mesh = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/" + problem + "_env.dae";
    std::string robot_mesh = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/" + problem + "_robot.dae";

    // load the robot and the environment
    setup.setEnvironmentMesh(environment_mesh);
    setup.setRobotMesh(robot_mesh);

    // Bounds for Apartment environment
    base::RealVectorBounds bounds(3);
    bounds.low[0] = -73.76;
    bounds.low[1] = -179.59;
    bounds.low[2] = -0.03;
    bounds.high[0] = 295.77;
    bounds.high[1] = 168.26;
    bounds.high[2] = 90.39;

    // Start/Goal pair for the Apartment environment
    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(-31.19);
    start->setY(-99.85);
    start->setZ(36.46);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(setup.getSpaceInformation());
    goal->setX(140.0);
    goal->setY(0.0);
    goal->setZ(36.46);
    goal->rotation().setIdentity();

    // Set start and goal
    setup.setStartAndGoalStates(start, goal);

    // Bound the state space
    setup.getSpaceInformation()->getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);
}

void configureEasyProblem(app::SE3RigidBodyPlanning &setup)
{
    std::string problem = "Easy";
    std::string environment_mesh = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/" + problem + "_env.dae";
    std::string robot_mesh = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/" + problem + "_robot.dae";

    // load the robot and the environment
    setup.setEnvironmentMesh(environment_mesh);
    setup.setRobotMesh(robot_mesh);

    // Bounds for Easy/TwistyCool environment
    base::RealVectorBounds bounds(3);
    bounds.low[0] = 14.46;
    bounds.low[1] = -24.25;
    bounds.low[2] = -504.86;
    bounds.high[0] = 457.96;
    bounds.high[1] = 321.25;
    bounds.high[2] = -72.86;

    // Start/Goal pair for the Easy/TwistyCool environment
    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(270.4);
    start->setY(50.0);
    start->setZ(-406.82);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(setup.getSpaceInformation());
    goal->setX(270.4);
    goal->setY(50.0);
    goal->setZ(-186.82);
    goal->rotation().setIdentity();

    // Set start and goal
    setup.setStartAndGoalStates(start, goal);

    // Bound the state space
    setup.getSpaceInformation()->getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);
}

void configureCubiclesProblem(app::SE3RigidBodyPlanning &setup)
{
    std::string problem = "cubicles";
    std::string environment_mesh = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/" + problem + "_env.dae";
    std::string robot_mesh = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/" + problem + "_robot.dae";

    // load the robot and the environment
    setup.setEnvironmentMesh(environment_mesh);
    setup.setRobotMesh(robot_mesh);

    // Bounds for cubicles environment
    base::RealVectorBounds bounds(3);
    bounds.low[0] = -508.88;
    bounds.low[1] = -230.13;
    bounds.low[2] = -123.75;
    bounds.high[0] = 319.62;
    bounds.high[1] = 531.87;
    bounds.high[2] = 101.00;

    // Start/Goal pair for cubicles environment
    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(-4.96);
    start->setY(-40.62);
    start->setZ(70.57);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(setup.getSpaceInformation());
    goal->setX(200.00);
    goal->setY(-40.62);
    goal->setZ(70.57);
    goal->rotation().setIdentity();

    // Set start and goal
    setup.setStartAndGoalStates(start, goal);

    // Bound the state space
    setup.getSpaceInformation()->getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);
}

void test(unsigned int tries, std::vector<std::vector<double>> &times, std::vector<int> &attempts, bool fcl = false,
          bool continuous = false)
{
    std::cout << "Evaluating " << (continuous ? "continuous" : "discrete") << " " << (fcl ? "FCL" : "PQP") << " checker"
              << std::endl;

    std::vector<double> time;
    unsigned int successful = 0;
    unsigned int problem = 0;

    while (problem != 3)
    {
        unsigned int nr_attempts = 0;

        if (problem == 0)
            std::cout << "- Apartment problem " << std::flush;
        else if (problem == 1)
            std::cout << "- Cubicles problem " << std::flush;
        else if (problem == 2)
            std::cout << R"(- 'Easy' problem )" << std::flush;

        // plan in SE3
        app::SE3RigidBodyPlanning setup;

        switch (problem)
        {
            case 0:
                configureApartmentProblem(setup);
                break;
            case 1:
                configureCubiclesProblem(setup);
                break;
            case 2:
                configureEasyProblem(setup);
                break;
        }

        setup.setStateValidityCheckerType(fcl ? app::FCL : app::PQP);

        if (continuous)
        {
            setup.setup();  // FCLContinuousMotionValidator extracts goodies from the state validity checker.
                            // Instantiate the svc here.
            setup.getSpaceInformation()->setMotionValidator(std::make_shared<app::FCLContinuousMotionValidator>(
                setup.getSpaceInformation(), setup.getMotionModel()));
        }

        setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
        setup.setPlanner(std::make_shared<geometric::RRT>(setup.getSpaceInformation()));
        setup.setup();

        while (successful < tries)
        {
            setup.clear();
            setup.solve(30.0);

            // Retry if the planner failed, except for the continuous collision checker case.
            if (setup.haveExactSolutionPath() || continuous)
            {
                successful++;
                time.push_back(setup.getLastPlanComputationTime());
                std::cout << '.' << std::flush;
            }
            else
                std::cout << 'x' << std::flush;
            nr_attempts++;
        }

        times.push_back(time);
        time.clear();
        successful = 0;
        problem++;
        attempts.push_back(nr_attempts);
        std::cout << std::endl;
    }
}

// Compares discrete and continuous collision checkers in OMPL.app
int main(int argc, char **argv)
{
#if OMPL_HAS_PQP
    // User can supply number of tries as 2nd command line argument.  Otherwise, use default NR_TRIES.
    unsigned int nr_tries;
    if (argc == 2)
    {
        nr_tries = atoi(argv[1]);

        // make sure command line input is valid
        if (nr_tries == std::numeric_limits<unsigned int>::min() ||
            nr_tries == std::numeric_limits<unsigned int>::max())
            nr_tries = 20u;
    }
    else
        nr_tries = 20u;

    ompl::msg::noOutputHandler();  // Disable console output from OMPL.

    std::vector<std::vector<double>> pqp_times;
    std::vector<std::vector<double>> dfcl_times;
    std::vector<std::vector<double>> cfcl_times;
    std::vector<int> pqp_attempts;
    std::vector<int> dfcl_attempts;
    std::vector<int> cfcl_attempts;

    std::cout << "Comparing collision checkers:" << std::endl;
    std::cout << "Each problem is executed until " << nr_tries << " attempts are successful (30 sec limit)"
              << std::endl;

    // PQP Test (discrete)
    test(nr_tries, pqp_times, pqp_attempts);

    // Discrete FCL Test
    test(nr_tries, dfcl_times, dfcl_attempts, true);

    // Continuous FCL Test
    test(nr_tries, cfcl_times, cfcl_attempts, true, true);

    std::cout << std::endl << "Analysis:" << std::endl;

    // Assume size of all 3 time vectors are the same
    for (size_t i = 0; i < pqp_times.size(); ++i)
    {
        bool isOdd;
        auto pqp_time = pqp_times[i].begin() + pqp_times[i].size() / 2u;
        auto dfcl_time = dfcl_times[i].begin() + dfcl_times[i].size() / 2u;
        auto cfcl_time = cfcl_times[i].begin() + cfcl_times[i].size() / 2u;

        std::nth_element(pqp_times[i].begin(), pqp_time, pqp_times[i].end());
        std::nth_element(dfcl_times[i].begin(), dfcl_time, dfcl_times[i].end());
        std::nth_element(cfcl_times[i].begin(), cfcl_time, cfcl_times[i].end());

        std::cout << std::endl;
        if (i == 0)
            std::cout << " Apartment problem - Median Time (s)" << std::endl;
        else if (i == 1)
            std::cout << " Cubicles problem - Median Time (s)" << std::endl;
        else if (i == 2)
            std::cout << R"( 'Easy' problem - Median Time (s))" << std::endl;

        isOdd = pqp_times[i].size() % 2 == 1;
        std::cout << "    Discrete PQP: " << (isOdd ? *pqp_time : .5 * (*pqp_time + *(pqp_time + 1))) << "  "
                  << nr_tries << "/" << pqp_attempts[i] << " planning attempts successful" << std::endl;
        isOdd = dfcl_times[i].size() % 2 == 1;
        std::cout << "    Discrete FCL: " << (isOdd ? *dfcl_time : .5 * (*dfcl_time + *(dfcl_time + 1))) << "  "
                  << nr_tries << "/" << dfcl_attempts[i] << " planning attempts successful" << std::endl;
        isOdd = cfcl_times[i].size() % 2 == 1;
        std::cout << "  Continuous FCL: " << (isOdd ? *cfcl_time : .5 * (*cfcl_time + *(cfcl_time + 1))) << "  "
                  << nr_tries << " total attempts" << std::endl;
    }

#else
    std::cerr << "ERROR: PQP collision checker is not installed" << std::endl;
#endif

    return 0;
}
