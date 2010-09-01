#include "common/SE3RigidBodyPlanning.h"
#include "common/detail/PQPSE3StateValidityChecker.h"
#include "common/detail/assimpUtil.h"
#include <limits>

void ompl::app::SE3RigidBodyPlanning::inferProblemDefinitionBounds(void)
{
    // update the bounds based on start states, if needed
    base::RealVectorBounds bounds = getStateManifold()->as<base::SE3StateManifold>()->getBounds();
    
    std::vector<const base::State*> states;
    getProblemDefinition()->getInputStates(states);
    
    double minX = std::numeric_limits<double>::infinity();
    double minY = minX;
    double minZ = minX;
    double maxX = -minX;
    double maxY = maxX;
    double maxZ = maxX;
    for (unsigned int i = 0 ; i < states.size() ; ++i)
    {
	double x = states[i]->as<base::SE3StateManifold::StateType>()->getX();
	double y = states[i]->as<base::SE3StateManifold::StateType>()->getY();
	double z = states[i]->as<base::SE3StateManifold::StateType>()->getZ();
	if (minX > x) minX = x;
	if (maxX < x) maxX = x;
	if (minY > y) minY = y;
	if (maxY < y) maxY = y;
	if (minZ > z) minZ = z;
	if (maxZ < z) maxZ = z;
    }
    double dx = (maxX - minX) * factor_ + add_;
    double dy = (maxY - minY) * factor_ + add_;
    double dz = (maxZ - minZ) * factor_ + add_;
    
    if (bounds.low[0] > minX - dx) bounds.low[0] = minX - dx;
    if (bounds.low[1] > minY - dy) bounds.low[1] = minY - dy;
    if (bounds.low[2] > minZ - dz) bounds.low[2] = minZ - dz;
    
    if (bounds.high[0] < maxX + dx) bounds.high[0] = maxX + dx;
    if (bounds.high[1] < maxY + dy) bounds.high[1] = maxY + dy;
    if (bounds.high[2] < maxZ + dz) bounds.high[2] = maxZ + dz;
    
    getStateManifold()->as<base::SE3StateManifold>()->setBounds(bounds);
}

void ompl::app::SE3RigidBodyPlanning::inferEnvironmentBounds(const aiScene *scene)
{
    base::RealVectorBounds bounds = getStateManifold()->as<base::SE3StateManifold>()->getBounds();
    
    // if bounds are not valid
    if (bounds.getVolume() < std::numeric_limits<double>::epsilon())
    {
	std::vector<aiVector3D> vertices;
	scene::extractVertices(scene, vertices);
	scene::inferBounds(bounds, vertices, factor_, add_);
	getStateManifold()->as<base::SE3StateManifold>()->setBounds(bounds);
    }
}

void ompl::app::SE3RigidBodyPlanning::getRobotCenterAndStartState(const aiScene *robot)
{
    scene::sceneCenter(robot, robotCenter_);
    start_->as<base::SE3StateManifold::StateType>()->rotation().setIdentity();
    start_->as<base::SE3StateManifold::StateType>()->setX(robotCenter_.x);
    start_->as<base::SE3StateManifold::StateType>()->setY(robotCenter_.y);
    start_->as<base::SE3StateManifold::StateType>()->setZ(robotCenter_.z);
}

ompl::base::StateValidityCheckerPtr ompl::app::SE3RigidBodyPlanning::allocStateValidityChecker(const aiScene *env, const aiScene *robot) const
{
    PQPSE3StateValidityChecker *svc = new PQPSE3StateValidityChecker(getSpaceInformation());
    svc->configure(robot, env, robotCenter_);
    return base::StateValidityCheckerPtr(svc);
}
