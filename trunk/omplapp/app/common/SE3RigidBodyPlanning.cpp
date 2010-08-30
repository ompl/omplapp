#include "common/SE3RigidBodyPlanning.h"
#include "common/detail/PQPSE3StateValidityChecker.h"
#include "common/detail/assimpUtil.h"
#include <limits>

namespace ompl
{
    namespace app
    {
	
	static void inferBounds(const base::StateManifoldPtr &m, const aiScene* scene, double factor, double add)
	{
	    base::RealVectorBounds bounds = m->as<base::SE3StateManifold>()->getBounds();
	    
	    // if bounds are not valid
	    if (bounds.getVolume() < std::numeric_limits<double>::epsilon())
	    {
		std::vector<aiVector3D> vertices;
		extractVertices(scene, vertices);
		inferBounds(bounds, vertices, factor, add);
		m->as<base::SE3StateManifold>()->setBounds(bounds);
	    }
	}
	
	static void inferBounds(const base::StateManifoldPtr &m, const base::ProblemDefinitionPtr &pdef, double factor, double add)
	{
	    // update the bounds based on start states, if needed
	    base::RealVectorBounds bounds = m->as<base::SE3StateManifold>()->getBounds();
	    
	    std::vector<const base::State*> states;
	    pdef->getInputStates(states);
	    
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
	    double dx = (maxX - minX) * factor + add;
	    double dy = (maxY - minY) * factor + add;
	    double dz = (maxZ - minZ) * factor + add;
	    
	    if (bounds.low[0] > minX - dx) bounds.low[0] = minX - dx;
	    if (bounds.low[1] > minY - dy) bounds.low[1] = minY - dy;
	    if (bounds.low[2] > minZ - dz) bounds.low[2] = minZ - dz;
	    
	    if (bounds.high[0] < maxX + dx) bounds.high[0] = maxX + dx;
	    if (bounds.high[1] < maxY + dy) bounds.high[1] = maxY + dy;
	    if (bounds.high[2] < maxZ + dz) bounds.high[2] = maxZ + dz;
	    
	    m->as<base::SE3StateManifold>()->setBounds(bounds);
	}
    }
}

int ompl::app::SE3RigidBodyPlanning::setMeshes(const std::string &robot, const std::string &env, bool useOpenGL)
{
    // load environment 
    Assimp::Importer importerE;
    
    assert(!robot.empty());
    assert(!env.empty());
    const aiScene* envScene = importerE.ReadFile(env.c_str(),
						 aiProcess_Triangulate            |
						 aiProcess_JoinIdenticalVertices  |
						 aiProcess_SortByPType);
    if (envScene)
    {
	if (envScene->HasMeshes())
	{
	    inferBounds(getStateManifold(), envScene, factor_, add_);
	    si_->setStateValidityCheckingResolution(shortestEdge(envScene));
	}
	else
	    msg_.error("There is no mesh specified in the indicated environment resource: %s", env.c_str());
    }
    else
	msg_.error("Unable to load environment scene: %s", env.c_str());
    
    // load robot 
    Assimp::Importer importerR;
    const aiScene* robotScene = importerR.ReadFile(robot.c_str(),
						   aiProcess_Triangulate            |
						   aiProcess_JoinIdenticalVertices  |
						   aiProcess_SortByPType);
    if (robotScene)
    {
	if (!robotScene->HasMeshes())
	    msg_.error("There is no mesh specified in the indicated robot resource: %s", robot.c_str());
    }
    else
	msg_.error("Unable to load robot scene: %s", robot.c_str());
    
    // create state validity checker
    if (robotScene->HasMeshes())
	setStateValidityChecker(base::StateValidityCheckerPtr(new PQPSE3StateValidityChecker(getSpaceInformation(), robotScene, envScene)));
    
    return useOpenGL ? assimpRender(robotScene, envScene) : 0;
}

void ompl::app::SE3RigidBodyPlanning::setup(void)
{
    inferBounds(getStateManifold(), getProblemDefinition(), factor_, add_);
    geometric::SimpleSetup::setup();    
}
