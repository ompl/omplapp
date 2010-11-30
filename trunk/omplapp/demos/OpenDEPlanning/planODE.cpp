#include "displayODE.h"

#include <ompl/extensions/ode/ODESimpleSetup.h>
#include <ompl/base/GoalRegion.h>
#include <ompl/config.h>

#include <drawstuff/drawstuff.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif


#define LENGTH 3.5		// chassis length
#define WIDTH 2.5		// chassis width
#define HEIGHT 1.0		// chassis height
#define RADIUS 0.5		// wheel radius
#define STARTZ 1.0		// starting height of chassis
#define CMASS 1			// chassis mass
#define WMASS 1			// wheel mass
#define COMOFFSET -5		// center of mass offset
#define WALLMASS 1		// wall box mass
#define BALLMASS 1		// ball mass
#define FMAX 25			// car engine fmax
#define ROWS 1			// rows of cars
#define COLS 1			// columns of cars
#define ITERS 20		// number of iterations
#define WBOXSIZE 1.0		// size of wall boxes
#define WALLWIDTH 12		// width of wall
#define WALLHEIGHT 10		// height of wall
#define DISABLE_THRESHOLD 0.008	// maximum velocity (squared) a body can have and be disabled
#define DISABLE_STEPS 10	// number of steps a box has to have been disable-able before it will be disabled

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
static dGeomID wall_boxes[10000];
static dBodyID wall_bodies[10000];
static int wb_stepsdis[10000];
static int wb;
static bool doFast;
static dBodyID b;
static dMass m;

static DisplayODESpaces disp;

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
	dBodySetPosition (b,x,y,STARTZ+COMOFFSET);
	dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
	dMassAdjust (&m,CMASS/2.0);
	dBodySetMass (b,&m);
	dJointID j = dJointCreateFixed(world, 0);
	dJointAttach(j, body[bodyI], b);
	dJointSetFixed(j);
	//box[boxI+1] = dCreateBox(space,LENGTH,WIDTH,HEIGHT);
	//dGeomSetBody (box[boxI+1],b);
	
	bodyI	+= 5;
	jointI	+= 4;
	boxI	+= 1;
	sphereI	+= 4;
}

void resetSimulation()
{
	int i;
	i = 0;

	disp.clear();

	// destroy world if it exists
	if (bodies)
	{
		dSpaceDestroy (space);
		dWorldDestroy (world);
	}
	
	for (i = 0; i < 1000; i++)
		wb_stepsdis[i] = 0;

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
	wb = 0;
	
	//	for (dReal x = 0.0; x < COLS*(LENGTH+RADIUS); x += LENGTH+RADIUS)
	//		for (dReal y = -((ROWS-1)*(WIDTH/2+RADIUS)); y <= ((ROWS-1)*(WIDTH/2+RADIUS)); y += WIDTH+RADIUS*2)
	makeCar(0, 0, bodies, joints, boxes, spheres);

	bool offset = false;
	for (dReal z = WBOXSIZE/2.0; z <= WALLHEIGHT; z+=WBOXSIZE)
	{
		offset = !offset;
		for (dReal y = (-WALLWIDTH+z)/2; y <= (WALLWIDTH-z)/2; y+=WBOXSIZE)
		{
			wall_bodies[wb] = dBodyCreate (world);
			dBodySetPosition (wall_bodies[wb],-20,y,z);
			dMassSetBox (&m,1,WBOXSIZE,WBOXSIZE,WBOXSIZE);
			dMassAdjust (&m, WALLMASS);
			dBodySetMass (wall_bodies[wb],&m);
			wall_boxes[wb] = dCreateBox (space,WBOXSIZE,WBOXSIZE,WBOXSIZE);
			dGeomSetBody (wall_boxes[wb],wall_bodies[wb]);
			//dBodyDisable(wall_bodies[wb++]);
			wb++;
		}
	}
	
	disp.addSpace(space);
	
	dMessage(0,"wall boxes: %i", wb);
}

static void start()
{
    dAllocateODEDataForThread(dAllocateMaskAll);
    static float xyz[3] = { 3.8548f,9.0843f,7.5900f} ;
    static float hpr[3] = { -145.5f,-22.5f,0.25f } ;
    
    dsSetViewpoint (xyz,hpr);
}

static void command (int cmd)
{
}

static void simLoop (int pause)
{
    disp.displaySpaces();
    
}

/// @cond IGNORE 

class CarEnvironment : public oc::ODEEnvironment
{
public:

    CarEnvironment(void) : oc::ODEEnvironment()
    {
	setPlanningParameters();
    }
    
    virtual ~CarEnvironment(void)
    {
    }
    
    /**************************************************
     * Implementation of functions needed by planning *
     **************************************************/

    virtual unsigned int getControlDimension(void) const
    {
	return 2;
    }
    
    virtual void getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const
    {
	static double maxForce = 0.2;
	lower.resize(2);
	lower[0] = -0.3;
	lower[1] = -1.2;
	
	upper.resize(2);
	upper[0] = 0.3;
	upper[1] = 1.2;
    }
    
    virtual void applyControl(const double *control) const
    {
	dReal speed = control[0];
	dReal turn = control[1];
	for (int j = 0; j < joints; j++)
	{
	    dReal curturn = dJointGetHinge2Angle1 (joint[j]);
	    //dMessage (0,"curturn %e, turn %e, vel %e", curturn, turn, (turn-curturn)*1.0);
	    dJointSetHinge2Param(joint[j],dParamVel,(turn-curturn)*1.0);
	    dJointSetHinge2Param(joint[j],dParamFMax,dInfinity);
	    dJointSetHinge2Param(joint[j],dParamVel2,speed);
	    dJointSetHinge2Param(joint[j],dParamFMax2,FMAX);
	}		
    }
    
    virtual bool isValidCollision(dGeomID geom1, dGeomID geom2, const dContact& contact) const
    {
	return false;
    }
    
    virtual void setupContact(dGeomID geom1, dGeomID geom2, dContact &contact) const
    {
	contact.surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
	if (dGeomGetClass(geom1) == dSphereClass || dGeomGetClass(geom2) == dSphereClass)
	    contact.surface.mu = 20;
	else
	    contact.surface.mu = 0.5;
	contact.surface.slip1 = 0.0;
	contact.surface.slip2 = 0.0;
	contact.surface.soft_erp = 0.8;
	contact.surface.soft_cfm = 0.01;
    }

    /**************************************************/
    
    // Set parameters needed by the base class (such as the bodies
    // that make up to state of the system we are planning for)
    void setPlanningParameters(void)
    {
	world_ = world;
	collisionSpaces_.push_back(space);
	for (int i = 0 ; i < bodies ; ++i)
	    stateBodies_.push_back(body[i]);
	for (int i = 0 ; i < boxes ; ++i)
	    stateBodies_.push_back(wall_bodies[i]);
    }
    
};


// Define the goal we want to reach
class RigidBodyGoal : public ob::GoalRegion
{
public:
    
    RigidBodyGoal(const ob::SpaceInformationPtr &si) : ob::GoalRegion(si)
    {
	threshold_ = 0.5;
    }
    
    virtual double distanceGoal(const ob::State *st) const
    {
	const double *pos = st->as<oc::ODEStateManifold::StateType>()->getBodyPosition(0);
	double dx = fabs(pos[0] - 30);
	double dy = fabs(pos[1] - 55);
	double dz = fabs(pos[2] - 35);
	return sqrt(dx * dx + dy * dy + dz * dz);
    }
    
};  


// Define how we project a state 
class RigidBodyStateProjectionEvaluator : public ob::ProjectionEvaluator
{
public: 

    RigidBodyStateProjectionEvaluator(const ob::StateManifold *manifold) : ob::ProjectionEvaluator(manifold)
    {
	cellDimensions_.resize(3);
	cellDimensions_[0] = 1;
	cellDimensions_[1] = 1;
	cellDimensions_[2] = 1;
    }
    
    virtual unsigned int getDimension(void) const
    {
	return 3;
    }
    
    virtual void project(const ob::State *state, ob::EuclideanProjection &projection) const
    {
	const double *pos = state->as<oc::ODEStateManifold::StateType>()->getBodyPosition(0);
	projection[0] = pos[0];
	projection[1] = pos[1];
	projection[2] = pos[2];
    }
    
};

// Define our own manifold, to include a distance function we want and register a default projection
class RigidBodyStateManifold : public oc::ODEStateManifold
{
public:

    RigidBodyStateManifold(const oc::ODEEnvironmentPtr &env) : oc::ODEStateManifold(env)
    {
    }
    
    virtual double distance(const ob::State *s1, const ob::State *s2) const
    {
	const double *p1 = s1->as<oc::ODEStateManifold::StateType>()->getBodyPosition(0);
	const double *p2 = s1->as<oc::ODEStateManifold::StateType>()->getBodyPosition(0);
	double dx = fabs(p1[0] - p2[0]);
	double dy = fabs(p1[1] - p2[1]);
	double dz = fabs(p1[2] - p2[2]);
	return sqrt(dx * dx + dy * dy + dz * dz);
    }

    virtual void registerProjections(void)
    {
    	registerDefaultProjection(ob::ProjectionEvaluatorPtr(new RigidBodyStateProjectionEvaluator(this)));
    }
    
};

/// @endcond

int main(int argc, char **argv)
{
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = 0;
    //fn.path_to_textures = ".";

    dInitODE2(0);

    bodies = 0;
    joints = 0;
    boxes = 0;
    spheres = 0;
    resetSimulation();

    // run simulation                                                       
    dsSimulationLoop (argc,argv,640,480,&fn);
    dSpaceDestroy (space);
    dWorldDestroy (world);
    dCloseODE();

    return 0;
}
