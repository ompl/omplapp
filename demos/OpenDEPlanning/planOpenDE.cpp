/*********************************************************************
 * Rice University Software Distribution License
 *
 * Copyright (c) 2010, Rice University
 * All Rights Reserved.
 *
 * For a full description see the file named LICENSE.
 *
 *********************************************************************/

/* Author: Ioan Sucan */

#include "OpenDEWorld.inc"
#include "OMPLEnvironment.inc"
#include "OMPLSetup.inc"

#include "displayOpenDE.h"
#include "omplapp/config.h"

#include <drawstuff/drawstuff.h>

#include <thread>

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

static DisplayOpenDESpaces DISP;
static std::vector<std::pair<double, double>> POINTS;
static bool drawTree = true;

static void start()
{
    dAllocateODEDataForThread(dAllocateMaskAll);
    static float xyz[3] = {3.8548f, 9.0843f, 7.5900f};
    static float hpr[3] = {-145.5f, -22.5f, 0.25f};

    dsSetViewpoint(xyz, hpr);
}

static void command(int cmd)
{
    if ((char)cmd == 't')
        drawTree = !drawTree;
}

static void simLoop(int /*pause*/)
{
    DISP.displaySpaces();

    if (drawTree)
    {
        glPointSize(2.0);
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_POINTS);
        for (auto &i : POINTS)
            glVertex3d(i.first, i.second, 0.05);
        glEnd();
    }

    static ompl::time::duration d = ompl::time::seconds(0.001);
    std::this_thread::sleep_for(d);
}

static void playPath(oc::OpenDESimpleSetup *ss)
{
    while (true)
    {
        ss->playSolutionPath(0.005);
        static ompl::time::duration d = ompl::time::seconds(1);
        std::this_thread::sleep_for(d);
    }
}

int main(int argc, char **argv)
{
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = nullptr;

    auto *textures = (char *)alloca(strlen(OMPLAPP_RESOURCE_DIR) + 10);
    strcpy(textures, OMPLAPP_RESOURCE_DIR);
    strcat(textures, "/textures");
    fn.path_to_textures = textures;

    dInitODE2(0);

    bodies = 0;
    joints = 0;
    boxes = 0;
    spheres = 0;
    resetSimulation();
    DISP.addSpace(space, 0.9, 0.9, 0.5);
    DISP.setGeomColor(avoid_box_geom, 0.9, 0.0, 0.0);
    DISP.setGeomColor(movable_box_geom[0], 0.1, 0.8, 0.8);
    DISP.setGeomColor(movable_box_geom[1], 0.1, 0.8, 0.8);
    DISP.setGeomColor(movable_box_geom[2], 0.1, 0.8, 0.8);
    DISP.setGeomColor(movable_box_geom[3], 0.1, 0.8, 0.8);
    DISP.setGeomColor(goal_geom, 0.0, 0.9, 0.1);

    oc::OpenDEEnvironmentPtr ce = std::make_shared<CarEnvironment>();
    ob::StateSpacePtr sm = std::make_shared<CarStateSpace>(ce);

    oc::ControlSpacePtr cm = std::make_shared<CarControlSpace>(sm);

    oc::OpenDESimpleSetup ss(cm);
    ss.setGoal(std::make_shared<CarGoal>(ss.getSpaceInformation(), GOAL_X, GOAL_Y));
    ob::RealVectorBounds vb(3);
    vb.low[0] = -50;
    vb.low[1] = -50;
    vb.low[2] = -5;
    vb.high[0] = 50;
    vb.high[1] = 50;
    vb.high[2] = 10;
    ss.setVolumeBounds(vb);

    ss.setup();
    ss.print();
    std::shared_ptr<std::thread> th;

    std::cout << "Planning for at most 60 seconds ..." << std::endl;

    if (ss.solve(60))
    {
        std::cout << "Solved!" << std::endl;
        ob::ScopedState<oc::OpenDEStateSpace> last(ss.getSpaceInformation());
        last = ss.getSolutionPath().getStates().back();
        std::cout << "Reached: " << last->getBodyPosition(0)[0] << " " << last->getBodyPosition(0)[1] << std::endl;

        POINTS.clear();
        ob::PlannerData pd(ss.getSpaceInformation());
        ss.getPlannerData(pd);
        for (unsigned int i = 0; i < pd.numVertices(); ++i)
        {
            const double *pos = pd.getVertex(i).getState()->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
            POINTS.push_back(std::make_pair(pos[0], pos[1]));
        }

        th = std::make_shared<std::thread>([&ss] { return playPath(&ss); });
    }

    // run simulation
    dsSimulationLoop(argc, argv, 640, 480, &fn);
    if (th)
    {
        // th->interrupt();
        th->join();
    }

    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
