#include "common/RigidBodyPlanning.h"
#include "common/detail/assimpUtil.h"
#include <limits>
#include <sstream>

int ompl::app::RigidBodyPlanning::setRobotMesh(const std::string &robot, bool useOpenGL)
{
    assert(!robot.empty());
    importerRobot_.reset(new Assimp::Importer());

    const aiScene* robotScene = importerRobot_->ReadFile(robot.c_str(),
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
    if (robotScene && robotScene->HasMeshes())
    {
	getRobotCenterAndStartState(robotScene);
	
	msg_.debug("Start state based on loaded robot:");
	std::stringstream ss;
	start_.print(ss);
	msg_.debug(ss.str());

	if (importerEnv_)
	    setStateValidityChecker(allocStateValidityChecker(importerEnv_->GetScene(), robotScene));
    }

    return (useOpenGL && robotScene) ? scene::assimpRender(robotScene, robotCenter_) : 0;
}

int ompl::app::RigidBodyPlanning::setEnvironmentMesh(const std::string &env, bool useOpenGL)
{
    assert(!env.empty());
    importerEnv_.reset(new Assimp::Importer());
   
    const aiScene* envScene = importerEnv_->ReadFile(env.c_str(),
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
	    si_->setStateValidityCheckingResolution(std::max(0.01, scene::shortestEdge(envScene) / getStateManifold()->getMaximumExtent()));
	}
	else
	    msg_.error("There is no mesh specified in the indicated environment resource: %s", env.c_str());
    }
    else
	msg_.error("Unable to load environment scene: %s", env.c_str());

    if (importerRobot_)
	setStateValidityChecker(allocStateValidityChecker(envScene, importerRobot_->GetScene()));
	
    return (useOpenGL && envScene) ? scene::assimpRender(envScene) : 0;
}

void ompl::app::RigidBodyPlanning::setup(void)
{
    inferProblemDefinitionBounds();
    geometric::SimpleSetup::setup();    
}
