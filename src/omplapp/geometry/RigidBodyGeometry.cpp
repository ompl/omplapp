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

#include "omplapp/geometry/RigidBodyGeometry.h"
#include "omplapp/geometry/detail/PQPStateValidityChecker.h"
#include <limits>
#include <sstream>

int ompl::app::RigidBodyGeometry::setRobotMesh(const std::string &robot)
{
    importerRobot_.clear();
    return addRobotMesh(robot);
}

int ompl::app::RigidBodyGeometry::addRobotMesh(const std::string &robot)
{
    assert(!robot.empty());
    std::size_t p = importerRobot_.size();
    importerRobot_.resize(p + 1);
    importerRobot_[p].reset(new Assimp::Importer());

    const aiScene* robotScene = importerRobot_[p]->ReadFile(robot.c_str(),
                                                            aiProcess_Triangulate            |
                                                            aiProcess_JoinIdenticalVertices  |
                                                            aiProcess_SortByPType            |
                                                            aiProcess_OptimizeGraph          |
                                                            aiProcess_OptimizeMeshes);
    if (robotScene)
    {
        if (!robotScene->HasMeshes())
        {
            msg_.error("There is no mesh specified in the indicated robot resource: %s", robot.c_str());
            importerRobot_.resize(p);
        }
    }
    else
    {
        msg_.error("Unable to load robot scene: %s", robot.c_str());
        importerRobot_.resize(p);
    }

    if (p < importerRobot_.size())
    {
        pqp_svc_.reset();
        return 1;
    }
    else
        return 0;
}

int ompl::app::RigidBodyGeometry::setEnvironmentMesh(const std::string &env)
{
    importerEnv_.clear();
    return addEnvironmentMesh(env);
}

int ompl::app::RigidBodyGeometry::addEnvironmentMesh(const std::string &env)
{
    assert(!env.empty());
    std::size_t p = importerEnv_.size();
    importerEnv_.resize(p + 1);
    importerEnv_[p].reset(new Assimp::Importer());

    const aiScene* envScene = importerEnv_[p]->ReadFile(env.c_str(),
                                                        aiProcess_Triangulate            |
                                                        aiProcess_JoinIdenticalVertices  |
                                                        aiProcess_SortByPType            |
                                                        aiProcess_OptimizeGraph          |
                                                        aiProcess_OptimizeMeshes);
    if (envScene)
    {
        if (!envScene->HasMeshes())
        {
            msg_.error("There is no mesh specified in the indicated environment resource: %s", env.c_str());
            importerEnv_.resize(p);
        }
    }
    else
    {
        msg_.error("Unable to load environment scene: %s", env.c_str());
        importerEnv_.resize(p);
    }

    if (p < importerEnv_.size())
    {
        pqp_svc_.reset();
        return 1;
    }
    else
        return 0;
}

void ompl::app::RigidBodyGeometry::inferEnvironmentBounds(const base::StateManifoldPtr &manifold) const
{
    base::RealVectorBounds bounds = mtype_ == Motion_2D ? manifold->as<base::SE2StateManifold>()->getBounds() : manifold->as<base::SE3StateManifold>()->getBounds();
    
    // if bounds are not valid
    if (bounds.getVolume() < std::numeric_limits<double>::epsilon())
    {
        if (mtype_ == Motion_2D)
            manifold->as<base::SE2StateManifold>()->setBounds(inferEnvironmentBounds());
        else
            manifold->as<base::SE3StateManifold>()->setBounds(inferEnvironmentBounds());
    }    
}

ompl::base::RealVectorBounds ompl::app::RigidBodyGeometry::inferEnvironmentBounds(void) const
{
    base::RealVectorBounds bounds(3);

    for (unsigned int i = 0 ; i < importerEnv_.size() ; ++i)
    {
        std::vector<aiVector3D> vertices;
        scene::extractVertices(importerEnv_[i]->GetScene(), vertices);
        scene::inferBounds(bounds, vertices, factor_, add_);
    }

    if (mtype_ == Motion_2D)
    {
        bounds.low.resize(2);
        bounds.high.resize(2);
    }

    return bounds;
}

void ompl::app::RigidBodyGeometry::inferProblemDefinitionBounds(const base::ProblemDefinitionPtr &pdef, const GeometricStateExtractor &se, unsigned int robotCount, 
                                                                const base::StateManifoldPtr &manifold)
{
    // update the bounds based on start states, if needed
    base::RealVectorBounds bounds = mtype_ == Motion_2D ? manifold->as<base::SE2StateManifold>()->getBounds() : manifold->as<base::SE3StateManifold>()->getBounds();
    
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
        for (unsigned int r = 0 ; r < robotCount ; ++r)
        {
            const base::State *s = se(states[i], r);
            double x = mtype_ == Motion_2D ? s->as<base::SE2StateManifold::StateType>()->getX() : s->as<base::SE3StateManifold::StateType>()->getX();
            double y = mtype_ == Motion_2D ? s->as<base::SE2StateManifold::StateType>()->getY() : s->as<base::SE3StateManifold::StateType>()->getY();
            double z = mtype_ == Motion_2D ? 0.0 : s->as<base::SE3StateManifold::StateType>()->getZ();
            if (minX > x) minX = x;
            if (maxX < x) maxX = x;
            if (minY > y) minY = y;
            if (maxY < y) maxY = y;
            if (minZ > z) minZ = z;
            if (maxZ < z) maxZ = z;
        }
    }
    double dx = (maxX - minX) * (factor_ - 1.0) + add_;
    double dy = (maxY - minY) * (factor_ - 1.0) + add_;
    double dz = (maxZ - minZ) * (factor_ - 1.0) + add_;
    
    if (bounds.low[0] > minX - dx) bounds.low[0] = minX - dx;
    if (bounds.low[1] > minY - dy) bounds.low[1] = minY - dy;
    
    if (bounds.high[0] < maxX + dx) bounds.high[0] = maxX + dx;
    if (bounds.high[1] < maxY + dy) bounds.high[1] = maxY + dy;
    
    if (mtype_ == Motion_3D)
    {
        if (bounds.high[2] < maxZ + dz) bounds.high[2] = maxZ + dz;
        if (bounds.low[2] > minZ - dz) bounds.low[2] = minZ - dz;
        
        manifold->as<base::SE3StateManifold>()->setBounds(bounds);
    }
    else
        manifold->as<base::SE2StateManifold>()->setBounds(bounds);
}

void ompl::app::RigidBodyGeometry::getEnvStartState(base::ScopedState<>& state) const
{
    aiVector3D s = getRobotCenter();

    if (mtype_ == Motion_2D)
    {
        state->as<base::SE2StateManifold::StateType>()->setX(s.x);
        state->as<base::SE2StateManifold::StateType>()->setY(s.y);
        state->as<base::SE2StateManifold::StateType>()->setYaw(0.0);
    }
    else
    {
        state->as<base::SE3StateManifold::StateType>()->setX(s.x);
        state->as<base::SE3StateManifold::StateType>()->setY(s.y);
        state->as<base::SE3StateManifold::StateType>()->setZ(s.z);
        state->as<base::SE3StateManifold::StateType>()->rotation().setIdentity();
    }
}

aiVector3D ompl::app::RigidBodyGeometry::getRobotCenter(void) const
{
    aiVector3D s(0.0, 0.0, 0.0);
    for (unsigned int i = 0 ; i < importerRobot_.size() ; ++i)
    {
        aiVector3D c;
        scene::sceneCenter(importerRobot_[i]->GetScene(), c);
        s = s + c;
    }

    if (mtype_ == Motion_2D)
        s.z = 0.0;

    if (!importerRobot_.empty())
        s = s / (double)importerRobot_.size();
    return s;
}

const ompl::base::StateValidityCheckerPtr& ompl::app::RigidBodyGeometry::allocStateValidityChecker(const base::SpaceInformationPtr &si, const GeometricStateExtractor &se, bool selfCollision)
{
    if (pqp_svc_)
        return pqp_svc_;

    GeometrySpecification geom;

    for (unsigned int i = 0 ; i < importerEnv_.size() ; ++i)
        geom.obstacles.push_back(importerEnv_[i]->GetScene());

    for (unsigned int i = 0 ; i < importerRobot_.size() ; ++i)
    {
        geom.robot.push_back(importerRobot_[i]->GetScene());
        aiVector3D c;
        scene::sceneCenter(geom.robot.back(), c);
        geom.robotShift = geom.robotShift + c;
    }
    if (!importerRobot_.empty())
        geom.robotShift = geom.robotShift / (double)importerRobot_.size();

    if (mtype_ == Motion_2D)
        geom.robotShift.z = 0.0;

    if (mtype_ == Motion_2D)
        pqp_svc_.reset(new PQPStateValidityChecker<Motion_2D>(si, geom, se, selfCollision));
    else
        pqp_svc_.reset(new PQPStateValidityChecker<Motion_3D>(si, geom, se, selfCollision));

    return pqp_svc_;
}
