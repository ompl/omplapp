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

#include "ODEWorld.inc"
#include "OMPLEnvironment.inc"
#include "OMPLSetup.inc"

#include "displayODE.h"
#include "omplapp/config.h"

#include <drawstuff/drawstuff.h>

#include <boost/thread.hpp>

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

static DisplayODESpaces                         DISP;
static std::vector< std::pair<double, double> > POINTS;
static bool                                     drawTree = true;

static void start()
{
    dAllocateODEDataForThread(dAllocateMaskAll);
    static float xyz[3] = { 3.8548f,9.0843f,7.5900f} ;
    static float hpr[3] = { -145.5f,-22.5f,0.25f } ;

    dsSetViewpoint (xyz,hpr);
}

static void command (int cmd)
{
    if ((char)cmd == 't')
        drawTree = !drawTree;
}

static void simLoop (int pause)
{
    DISP.displaySpaces();

    if (drawTree)
    {
        glPointSize(2.0);
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_POINTS);
        for (unsigned int i = 0 ; i < POINTS.size() ; ++i)
            glVertex3d(POINTS[i].first, POINTS[i].second, 0.05);
        glEnd();
    }

    static ompl::time::duration d = ompl::time::seconds(0.001);
    boost::this_thread::sleep(d);
}

static void playPath(oc::ODESimpleSetup *ss)
{
    while (1)
    {
        ss->playSolutionPath(0.005);
        static ompl::time::duration d = ompl::time::seconds(1);
        boost::this_thread::sleep(d);
    }
}

int main(int argc, char **argv)
{
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = 0;

    char *textures = (char*)alloca(strlen(OMPLAPP_RESOURCE_DIR) + 10);
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

    oc::ODEEnvironmentPtr ce(new CarEnvironment());
    ob::StateSpacePtr sm(new CarStateSpace(ce));

    oc::ControlSpacePtr cm(new CarControlSpace(sm));

    oc::ODESimpleSetup ss(cm);
    ss.setGoal(ob::GoalPtr(new CarGoal(ss.getSpaceInformation(), GOAL_X, GOAL_Y)));
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
    boost::thread *th = NULL;

    std::cout << "Planning for at most 60 seconds ..." << std::endl;

    if (ss.solve(60))
    {
        std::cout << "Solved!" << std::endl;
        ob::ScopedState<oc::ODEStateSpace> last(ss.getSpaceInformation());
        last = ss.getSolutionPath().states.back();
        std::cout << "Reached: " << last->getBodyPosition(0)[0] << " " << last->getBodyPosition(0)[1] << std::endl;

        POINTS.clear();
        const ob::PlannerData &pd = ss.getPlannerData();
        for (unsigned int i = 0 ; i < pd.states.size() ; ++i)
        {
            const double *pos = pd.states[i]->as<oc::ODEStateSpace::StateType>()->getBodyPosition(0);
            POINTS.push_back(std::make_pair(pos[0], pos[1]));
        }

        th = new boost::thread(boost::bind(&playPath, &ss));
    }

    // run simulation
    dsSimulationLoop(argc, argv, 640, 480, &fn);
    if (th)
    {
        th->interrupt();
        th->join();
    }

    dSpaceDestroy (space);
    dWorldDestroy (world);
    dCloseODE();

    return 0;
}
