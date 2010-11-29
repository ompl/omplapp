#ifndef OMPLAPP_COMMON_RIGID_BODY_PLANNING_
#define OMPLAPP_COMMON_RIGID_BODY_PLANNING_

#include "omplapp/config.h"
#include <ompl/geometric/SimpleSetup.h>
#include <boost/shared_ptr.hpp>
#include <aiScene.h>
#include <assimp.hpp>
#include <string>

namespace ompl
{

    /** \brief Namespace containing code specific to OMPL.app */
    namespace app
    {
	
	/** \brief This is a wrapper around
	    ompl::geometric::SimpleSetup that allows loading CAD files
	    (using assimp), automatically sets the collision checker
	    (PQP) and automatically infers bounds for the space to
	    plan in, depending on the objects in the environment. */
	class RigidBodyPlanning : public geometric::SimpleSetup
	{
	public:

	    /** \brief Constructor expects a manifold that can represent a rigid body */
	    RigidBodyPlanning(const base::StateManifoldPtr &manifold) : geometric::SimpleSetup(manifold), factor_(1.0), add_(0.0), start_(manifold)
	    {
		start_.random();
		robotCenter_.Set(0, 0, 0);
	    }

	    virtual ~RigidBodyPlanning(void)
	    {
	    }

	    /** \brief The CAD file for the robot often has a root
		transform. This root transform is undone when the
		robot is loaded, but the value of that transform is
		kept as the robot's starting state. This function
		returns that starting state. */
	    const base::ScopedState<>& getEnvStartState(void) const
	    {
		return start_;
	    }
	    
	    /** \brief Set the starting state for the robot */
	    void setEnvStartState(const base::ScopedState<> &state)
	    {
		start_ = state;
	    }
	    
	    /** \brief This function specifies the name of the CAD
		file representing the environment (\e
		env). Optionally, this environment can also be redered
		with OpenGL (\e useOpenGL is true). The function
		returns the GL list id of the rendered objects (0 if
		nothing is rendered). */
	    virtual int setEnvironmentMesh(const std::string &env, bool useOpenGL = false);

	     /** \brief This function specifies the name of the CAD
		file representing the robot (\e robot). Optionally,
		this environment can also be redered with OpenGL (\e
		useOpenGL is true). The function returns the GL list
		id of the rendered objects (0 if nothing is
		rendered). */
	    virtual int setRobotMesh(const std::string &robot, bool useOpenGL = false);
	    
	    /** \brief The bounds of the environment are inferred
		based on the axis-aligned bounding box for the objects
		in the environment. The inferred size is multiplied by
		\e factor. By default \e factor = 1, */
	    void setBoundsFactor(double factor)
	    {
		factor_ = factor;
	    }
	    
	    /** \brief Get the data set by setBoundsFactor() */
	    double getBoundsFactor(void) const
	    {
		return factor_;
	    }	    

	    /** \brief The bounds of the environment are inferred
		based on the axis-aligned bounding box for the objects
		in the environment. \e add is added to the inferred
		size. By default \e add = 0, */
	    void setBoundsAddition(double add)
	    {
		add_ = add;
	    }
	    
	    /** \brief Get the data set by setBoundsAddition() */
	    double getBoundsAddition(void) const
	    {
		return add_;
	    }
	    
	    virtual void setup(void);
	    
	protected:
	    
	    /** \brief Given the representation of an environment, infer its bounds. */ 
	    virtual void inferEnvironmentBounds(const aiScene *scene) = 0;

	    /** \brief Update environment bounds such that start and goal states are included within bounds. */ 
	    virtual void inferProblemDefinitionBounds(void) = 0;

	    /** \brief Extract the robot center and the starting state based on the root transform of the robot. */
	    virtual void getRobotCenterAndStartState(const aiScene *robot) = 0;
	    
	    /** \brief Allocate default stae validity checker using PQP. */
	    virtual base::StateValidityCheckerPtr allocStateValidityChecker(const aiScene *env, const aiScene *robot) const = 0;
	    
	    /** \brief The factor to multiply inferred environment bounds by (default 1) */
	    double              factor_;

	    /** \brief The value to add to inferred environment bounds (default 0) */
	    double              add_;
	    
	    /** \brief The default start state of the robot */
	    base::ScopedState<> start_;

	    /** \brief The center of the robot, as loaded from the CAD file */
	    aiVector3D          robotCenter_;
	    
	    /** \brief Instance of assimp importer used to load environment */
	    boost::shared_ptr<Assimp::Importer> importerEnv_;

	    /** \brief Instance of assimp importer used to load robot */
	    boost::shared_ptr<Assimp::Importer> importerRobot_;
	};
	
    }
}

#endif
