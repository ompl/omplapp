// this code is taken from OpenDE's demos

#include <ode/ode.h>

#define LENGTH 3.5		// chassis length
#define WIDTH 2.5		// chassis width
#define HEIGHT 1.0		// chassis height
#define RADIUS 0.5		// wheel radius
#define STARTZ 1.0		// starting height of chassis
#define CMASS 1			// chassis mass
#define WMASS 1			// wheel mass
#define COMOFFSET -5		// center of mass offset
#define BOXMASS 1		// wall box mass
#define FMAX 25			// car engine fmax
#define ITERS 20		// number of iterations
#define BOXSIZE 3.0		// size of wall boxes

static dWorldID world;
static dSpaceID space;
static dBodyID body[10000];
static int bodies;
static dJointID joint[100000];
static int joints;
static dGeomID ground;
static dGeomID box[10000];
static int boxes;
static dGeomID sphere[10000];
static int spheres;

static dGeomID movable_box_geom;
static dBodyID movable_box_body;
static dMass m;

void makeCar(dReal x, dReal y, int &bodyI, int &jointI, int &boxI, int &sphereI)
{
    int i;
    dMass m;
    
    // chassis body
    body[bodyI] = dBodyCreate (world);
    dBodySetPosition (body[bodyI],x,y,STARTZ);
    dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
    dMassAdjust (&m,CMASS/2.0);
    dBodySetMass (body[bodyI],&m);
    box[boxI] = dCreateBox (space,LENGTH,WIDTH,HEIGHT);
    dGeomSetBody (box[boxI],body[bodyI]);
    
    // wheel bodies
    for (i=1; i<=4; i++) {
        body[bodyI+i] = dBodyCreate (world);
        dQuaternion q;
        dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
        dBodySetQuaternion (body[bodyI+i],q);
        dMassSetSphere (&m,1,RADIUS);
        dMassAdjust (&m,WMASS);
        dBodySetMass (body[bodyI+i],&m);
        sphere[sphereI+i-1] = dCreateSphere (space,RADIUS);
        dGeomSetBody (sphere[sphereI+i-1],body[bodyI+i]);
    }
    dBodySetPosition (body[bodyI+1],x+0.4*LENGTH-0.5*RADIUS,y+WIDTH*0.5,STARTZ-HEIGHT*0.5);
    dBodySetPosition (body[bodyI+2],x+0.4*LENGTH-0.5*RADIUS,y-WIDTH*0.5,STARTZ-HEIGHT*0.5);
    dBodySetPosition (body[bodyI+3],x-0.4*LENGTH+0.5*RADIUS,y+WIDTH*0.5,STARTZ-HEIGHT*0.5);
    dBodySetPosition (body[bodyI+4],x-0.4*LENGTH+0.5*RADIUS,y-WIDTH*0.5,STARTZ-HEIGHT*0.5);
    
    // front and back wheel hinges
    for (i=0; i<4; i++) {
        joint[jointI+i] = dJointCreateHinge2 (world,0);
        dJointAttach (joint[jointI+i],body[bodyI],body[bodyI+i+1]);
        const dReal *a = dBodyGetPosition (body[bodyI+i+1]);
        dJointSetHinge2Anchor (joint[jointI+i],a[0],a[1],a[2]);
        dJointSetHinge2Axis1 (joint[jointI+i],0,0,(i<2 ? 1 : -1));
        dJointSetHinge2Axis2 (joint[jointI+i],0,1,0);
        dJointSetHinge2Param (joint[jointI+i],dParamSuspensionERP,0.8);
        dJointSetHinge2Param (joint[jointI+i],dParamSuspensionCFM,1e-5);
        dJointSetHinge2Param (joint[jointI+i],dParamVel2,0);
        dJointSetHinge2Param (joint[jointI+i],dParamFMax2,FMAX);
    }
    
    //center of mass offset body. (hang another copy of the body COMOFFSET units below it by a fixed joint)
    dBodyID b = dBodyCreate (world);
    body[bodyI + 5] = b;
    dBodySetPosition (b,x,y,STARTZ+COMOFFSET);
    dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
    dMassAdjust (&m,CMASS/2.0);
    dBodySetMass (b,&m);
    dJointID j = dJointCreateFixed(world, 0);
    dJointAttach(j, body[bodyI], b);
    dJointSetFixed(j);
    
    bodyI	+= 6;
    jointI	+= 4;
    boxI	+= 1;
    sphereI	+= 4;
}

void resetSimulation(void)
{
    int i;
    i = 0;
    
    // destroy world if it exists
    if (bodies)
    {
        dSpaceDestroy (space);
        dWorldDestroy (world);
    }
    
    // recreate world
    
    world = dWorldCreate();
    
    //	space = dHashSpaceCreate( 0 );
    //	space = dSimpleSpaceCreate( 0 );
    space = dSweepAndPruneSpaceCreate( 0, dSAP_AXES_XYZ );
    
    dWorldSetGravity (world,0,0,-1.5);
    dWorldSetCFM (world, 1e-5);
    dWorldSetERP (world, 0.8);
    dWorldSetQuickStepNumIterations (world,ITERS);
    ground = dCreatePlane (space,0,0,1,0);
    
    bodies = 0;
    joints = 0;
    boxes = 0;
    spheres = 0;
    
    makeCar(0, 0, bodies, joints, boxes, spheres);
        
    movable_box_body = dBodyCreate(world);
    dBodySetPosition(movable_box_body, -25, 0, BOXSIZE/2);
    dMassSetBox (&m, 1, BOXSIZE, BOXSIZE, BOXSIZE);
    dMassAdjust (&m, BOXMASS);
    dBodySetMass (movable_box_body ,&m);
    movable_box_geom = dCreateBox (space,BOXSIZE,BOXSIZE,BOXSIZE);
    dGeomSetBody (movable_box_geom, movable_box_body);    
}
