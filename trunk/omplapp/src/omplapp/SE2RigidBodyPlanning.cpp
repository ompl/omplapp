#include "omplapp/SE2RigidBodyPlanning.h"
#include "omplapp/detail/PQPSE2StateValidityChecker.h"
#include "omplapp/detail/assimpUtil.h"
#include <limits>

void ompl::app::SE2RigidBodyPlanning::inferEnvironmentBounds(const aiScene *scene)
{
    base::RealVectorBounds bounds = getStateManifold()->as<base::SE2StateManifold>()->getBounds();
    
    // if bounds are not valid
    if (bounds.getVolume() < std::numeric_limits<double>::epsilon())
    {
	std::vector<aiVector3D> vertices;
	scene::extractVertices(scene, vertices);
	base::RealVectorBounds b(3);
	scene::inferBounds(b, vertices, factor_, add_);
	bounds.low[0] = b.low[0]; bounds.low[1] = b.low[1];
	bounds.high[0] = b.high[0]; bounds.high[1] = b.high[1];
	getStateManifold()->as<base::SE2StateManifold>()->setBounds(bounds);
    }
}

void ompl::app::SE2RigidBodyPlanning::inferProblemDefinitionBounds(void)
{
    // update the bounds based on start states, if needed
    base::RealVectorBounds bounds = getStateManifold()->as<base::SE2StateManifold>()->getBounds();
    
    std::vector<const base::State*> states;
    getProblemDefinition()->getInputStates(states);
    
    double minX = std::numeric_limits<double>::infinity();
    double minY = minX;
    double maxX = -minX;
    double maxY = maxX;
    for (unsigned int i = 0 ; i < states.size() ; ++i)
    {
	double x = states[i]->as<base::SE2StateManifold::StateType>()->getX();
	double y = states[i]->as<base::SE2StateManifold::StateType>()->getY();
	if (minX > x) minX = x;
	if (maxX < x) maxX = x;
	if (minY > y) minY = y;
	if (maxY < y) maxY = y;
    }
    double dx = (maxX - minX) * factor_ + add_;
    double dy = (maxY - minY) * factor_ + add_;
    
    if (bounds.low[0] > minX - dx) bounds.low[0] = minX - dx;
    if (bounds.low[1] > minY - dy) bounds.low[1] = minY - dy;
    
    if (bounds.high[0] < maxX + dx) bounds.high[0] = maxX + dx;
    if (bounds.high[1] < maxY + dy) bounds.high[1] = maxY + dy;
    
    getStateManifold()->as<base::SE2StateManifold>()->setBounds(bounds);
}

void ompl::app::SE2RigidBodyPlanning::getRobotCenterAndStartState(const aiScene *robot)
{
    scene::sceneCenter(robot, robotCenter_);
    robotCenter_.z = 0.0;
    start_->as<base::SE2StateManifold::StateType>()->setYaw(0.0);
    start_->as<base::SE2StateManifold::StateType>()->setX(robotCenter_.x);
    start_->as<base::SE2StateManifold::StateType>()->setY(robotCenter_.y);
}

ompl::base::StateValidityCheckerPtr ompl::app::SE2RigidBodyPlanning::allocStateValidityChecker(const aiScene *env, const aiScene *robot) const
{   
    PQPSE2StateValidityChecker *svc = new PQPSE2StateValidityChecker(getSpaceInformation());
    svc->configure(robot, env, robotCenter_);
    return base::StateValidityCheckerPtr(svc);
}
