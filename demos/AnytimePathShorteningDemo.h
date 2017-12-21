/*********************************************************************
 * Rice University Software Distribution License
 *
 * Copyright (c) 2010, Rice University
 * All Rights Reserved.
 *
 * For a full description see the file named LICENSE.
 *
 *********************************************************************/

/* Author: Ryan Luna */

#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/sbl/SBL.h>

#include <fstream>
#include <iostream>

using namespace ompl;

enum ProblemType
{
    BARRIERS = 0,
    CUBICLES,
    EASY,
    INVALID_PROBLEM
};

static const char *ProblemTypeStrings[3] = {"Barriers", "Cubicles", "Easy"};

enum OptimizationType
{
    SHORTCUT = 0,
    HYBRIDIZE,
    ALTERNATE,
    NONE,
    INVALID_OPTIMIZATION
};

static const char *OptTypeStrings[4] = {"Shortcut", "Hybridize", "Alternating", "None"};

enum PlannerType
{
    RRT = 0,
    RRTSTAR,
    EST,
    SBL,
    RRTCONNECT,
    PRM,
    PRMSTAR,
    KPIECE,
    BKPIECE,
    LBKPIECE,
    INVALID_PLANNER
};

static const char *PlannerTypeStrings[10] = {"RRT", "RRT*", "EST",    "SBL",     "RRTConnect",
                                             "PRM", "PRM*", "KPIECE", "BKPIECE", "LBKPIECE"};

void readArgsFromFile(const char *filename, ProblemType &probType, OptimizationType &optType, double &runtime,
                      std::vector<PlannerType> &planners)
{
    std::cout << __FUNCTION__ << "  " << filename << std::endl;

    std::ifstream fin(filename);
    if (!fin)
    {
        OMPL_ERROR("FATAL: Failed to open %s", filename);
        return;
    }

    std::string problem, optimization;
    fin >> problem;
    fin >> optimization;

    if (problem == "barriers")
        probType = BARRIERS;
    else if (problem == "cubicles")
        probType = CUBICLES;
    else if (problem == "easy")
        probType = EASY;

    if (optimization == "shortcut")
        optType = SHORTCUT;
    else if (optimization == "hybridize")
        optType = HYBRIDIZE;
    else if (optimization == "alternate")
        optType = ALTERNATE;

    fin >> runtime;

    std::string plannerArg;
    while (!fin.eof() && fin.good())
    {
        plannerArg = "";
        fin >> plannerArg;
        if (plannerArg == "")
            continue;

        if (plannerArg == "rrt")
            planners.push_back(RRT);
        else if (plannerArg == "rrtstar")
            planners.push_back(RRTSTAR);
        else if (plannerArg == "est")
            planners.push_back(EST);
        else if (plannerArg == "sbl")
            planners.push_back(SBL);
        else if (plannerArg == "rrtconnect")
            planners.push_back(RRTCONNECT);
        else if (plannerArg == "prm")
            planners.push_back(PRM);
        else if (plannerArg == "prmstar")
            planners.push_back(PRMSTAR);
        else if (plannerArg == "kpiece")
            planners.push_back(KPIECE);
        else if (plannerArg == "bkpiece")
            planners.push_back(BKPIECE);
        else if (plannerArg == "lbkpiece")
            planners.push_back(LBKPIECE);
        else
            std::cout << "Unknown planner argument: " << plannerArg << ".  Skipping..." << std::endl;
    }

    fin.close();
}

void parseArguments(int argc, char **argv, ProblemType &probType, OptimizationType &optType, double &runtime,
                    std::vector<PlannerType> &planners)
{
    if (argc == 2)
    {
        readArgsFromFile(argv[1], probType, optType, runtime, planners);
        return;
    }

    std::string problem(argv[1]);
    std::string optimization(argv[2]);
    runtime = atof(argv[3]);

    probType = INVALID_PROBLEM;
    optType = INVALID_OPTIMIZATION;

    if (problem == "barriers")
        probType = BARRIERS;
    else if (problem == "cubicles")
        probType = CUBICLES;
    else if (problem == "easy")
        probType = EASY;

    if (optimization == "shortcut")
        optType = SHORTCUT;
    else if (optimization == "hybridize")
        optType = HYBRIDIZE;
    else if (optimization == "alternate")
        optType = ALTERNATE;
    else if (optimization == "none")
        optType = NONE;

    for (int i = 4; i < argc; ++i)
    {
        std::string plannerArg(argv[i]);
        if (plannerArg == "rrt")
            planners.push_back(RRT);
        else if (plannerArg == "rrtstar")
            planners.push_back(RRTSTAR);
        else if (plannerArg == "est")
            planners.push_back(EST);
        else if (plannerArg == "sbl")
            planners.push_back(SBL);
        else if (plannerArg == "rrtconnect")
            planners.push_back(RRTCONNECT);
        else if (plannerArg == "prm")
            planners.push_back(PRM);
        else if (plannerArg == "prmstar")
            planners.push_back(PRMSTAR);
        else if (plannerArg == "kpiece")
            planners.push_back(KPIECE);
        else if (plannerArg == "bkpiece")
            planners.push_back(BKPIECE);
        else if (plannerArg == "lbkpiece")
            planners.push_back(LBKPIECE);
        else
            std::cout << "Unknown planner argument: " << plannerArg << ".  Skipping..." << std::endl;
    }
}

bool validArguments(const ProblemType &probType, const OptimizationType &optType, const double &runtime,
                    const std::vector<PlannerType> &planners)
{
    bool valid = true;
    if (probType == INVALID_PROBLEM)
    {
        std::cout << "FATAL: Problem type is invalid!" << std::endl;
        valid = false;
    }

    if (optType == INVALID_OPTIMIZATION)
    {
        std::cout << "FATAL: Optimization type is invalid!" << std::endl;
        valid = false;
    }

    if (runtime <= 0.0)
    {
        std::cout << "FATAL: Runtime must be strictly positive!" << std::endl;
        valid = false;
    }

    if (planners.size() == 0)
    {
        std::cout << "FATAL: At least one planner must be declared!" << std::endl;
        valid = false;
    }

    if (valid)
    {
        std::cout << "Solving the " << ProblemTypeStrings[probType] << " problem using " << OptTypeStrings[optType]
                  << " optimization for " << runtime << " seconds with " << std::endl;
        for (const auto &planner : planners)
            std::cout << PlannerTypeStrings[planner] << " ";
        std::cout << std::endl;
    }

    return valid;
}

base::PlannerPtr allocPlanner(PlannerType type, const base::SpaceInformationPtr &si)
{
    assert(type != INVALID_PLANNER);

    base::PlannerPtr planner;
    planner.reset();

    switch (type)
    {
        case RRT:
            planner = std::make_shared<geometric::RRT>(si);
            break;

        case RRTSTAR:
            planner = std::make_shared<geometric::RRTstar>(si);
            break;

        case EST:
            planner = std::make_shared<geometric::EST>(si);
            break;

        case SBL:
            planner = std::make_shared<geometric::SBL>(si);
            break;

        case RRTCONNECT:
            planner = std::make_shared<geometric::RRTConnect>(si);
            break;

        case PRM:
            planner = std::make_shared<geometric::PRM>(si);
            break;

        case PRMSTAR:
            planner = std::make_shared<geometric::PRM>(si, true);
            break;

        case KPIECE:
            planner = std::make_shared<geometric::KPIECE1>(si);
            break;

        case BKPIECE:
            planner = std::make_shared<geometric::BKPIECE1>(si);
            break;

        case LBKPIECE:
            planner = std::make_shared<geometric::LBKPIECE1>(si);
            break;

        default:
            OMPL_ERROR("UNKNOWN PLANNER TYPE");
    }

    return planner;
}
