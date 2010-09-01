#include "common/RigidBodyPlanning.h"
#include "common/detail/assimpUtil.h"
#include <limits>

int ompl::app::RigidBodyPlanning::setMeshes(const std::string &robot, const std::string &env, bool useOpenGL)
{
    // load environment 
    Assimp::Importer importerE;
    
    assert(!robot.empty());
    assert(!env.empty());
    const aiScene* envScene = importerE.ReadFile(env.c_str(),
						 aiProcess_Triangulate            |
						 aiProcess_JoinIdenticalVertices  |
						 aiProcess_SortByPType            |
						 aiProcess_OptimizeGraph          |
						 aiProcess_OptimizeMeshes);
    if (envScene)
    {
	if (envScene->HasMeshes())
	{
	    inferEnvironmentBounds(envScene);
	    si_->setStateValidityCheckingResolution(scene::shortestEdge(envScene));
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
						   aiProcess_SortByPType            |
						   aiProcess_OptimizeGraph          |
						   aiProcess_OptimizeMeshes);
    if (robotScene)
    {
	if (!robotScene->HasMeshes())
	    msg_.error("There is no mesh specified in the indicated robot resource: %s", robot.c_str());
    }
    else
	msg_.error("Unable to load robot scene: %s", robot.c_str());
    
    // create state validity checker
    if (robotScene->HasMeshes())
    {
	getRobotCenterAndStartState(robotScene);
	setStateValidityChecker(allocStateValidityChecker(envScene, robotScene));
	msg_.debug("Start state based on loaded robot:");
	start_.print();
    }
    
    return useOpenGL ? scene::assimpRender(robotScene, envScene, robotCenter_) : 0;
}

void ompl::app::RigidBodyPlanning::setup(void)
{
    inferProblemDefinitionBounds();
    geometric::SimpleSetup::setup();    
}
