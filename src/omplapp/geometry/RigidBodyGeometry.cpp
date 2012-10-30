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
#if OMPL_HAS_PQP
#include "omplapp/geometry/detail/PQPStateValidityChecker.h"
#endif
#include "omplapp/geometry/detail/FCLStateValidityChecker.h"
#include <boost/lexical_cast.hpp>

bool ompl::app::RigidBodyGeometry::setRobotMesh(const std::string &robot)
{
    importerRobot_.clear();
    computeGeometrySpecification();
    return addRobotMesh(robot);
}

bool ompl::app::RigidBodyGeometry::addRobotMesh(const std::string &robot)
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
            OMPL_ERROR("There is no mesh specified in the indicated robot resource: %s", robot.c_str());
            importerRobot_.resize(p);
        }
    }
    else
    {
        OMPL_ERROR("Unable to load robot scene: %s", robot.c_str());
        importerRobot_.resize(p);
    }

    if (p < importerRobot_.size())
    {
        computeGeometrySpecification();
        return true;
    }
    else
        return false;
}

bool ompl::app::RigidBodyGeometry::setEnvironmentMesh(const std::string &env)
{
    importerEnv_.clear();
    computeGeometrySpecification();
    return addEnvironmentMesh(env);
}

bool ompl::app::RigidBodyGeometry::addEnvironmentMesh(const std::string &env)
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
            OMPL_ERROR("There is no mesh specified in the indicated environment resource: %s", env.c_str());
            importerEnv_.resize(p);
        }
    }
    else
    {
        OMPL_ERROR("Unable to load environment scene: %s", env.c_str());
        importerEnv_.resize(p);
    }

    if (p < importerEnv_.size())
    {
        computeGeometrySpecification();
        return true;
    }
    else
        return false;
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

const ompl::app::GeometrySpecification& ompl::app::RigidBodyGeometry::getGeometrySpecification(void) const
{
    return geom_;
}

void ompl::app::RigidBodyGeometry::computeGeometrySpecification(void)
{
    validitySvc_.reset();
    geom_.obstacles.clear();
    geom_.obstaclesShift.clear();
    geom_.robot.clear();
    geom_.robotShift.clear();

    for (unsigned int i = 0 ; i < importerEnv_.size() ; ++i)
        geom_.obstacles.push_back(importerEnv_[i]->GetScene());

    for (unsigned int i = 0 ; i < importerRobot_.size() ; ++i)
    {
        geom_.robot.push_back(importerRobot_[i]->GetScene());
        aiVector3D c = getRobotCenter(i);
        if (mtype_ == Motion_2D)
            c[2] = 0.0;
        geom_.robotShift.push_back(c);
    }
}

aiVector3D ompl::app::RigidBodyGeometry::getRobotCenter(unsigned int robotIndex) const
{
    aiVector3D s(0.0, 0.0, 0.0);
    if (robotIndex >= importerRobot_.size())
        throw Exception("Robot " + boost::lexical_cast<std::string>(robotIndex) + " not found.");

    scene::sceneCenter(importerRobot_[robotIndex]->GetScene(), s);
    return s;
}

void ompl::app::RigidBodyGeometry::setStateValidityCheckerType (CollisionChecker ctype)
{
    if (ctype != ctype_)
    {
        ctype_ = ctype;
        if (validitySvc_)
        {
            validitySvc_.reset ();
        }

        assert (!validitySvc_);
    }
}

const ompl::base::StateValidityCheckerPtr& ompl::app::RigidBodyGeometry::allocStateValidityChecker(const base::SpaceInformationPtr &si, const GeometricStateExtractor &se, bool selfCollision)
{
    if (validitySvc_)
        return validitySvc_;

    GeometrySpecification geom = getGeometrySpecification();

    switch (ctype_)
    {
#if OMPL_HAS_PQP
        case PQP:
            if (mtype_ == Motion_2D)
                validitySvc_.reset (new PQPStateValidityChecker<Motion_2D>(si, geom, se, selfCollision));
            else
                validitySvc_.reset (new PQPStateValidityChecker<Motion_3D>(si, geom, se, selfCollision));
            break;
#endif
        case FCL:
            if (mtype_ == Motion_2D)
                validitySvc_.reset (new FCLStateValidityChecker<Motion_2D>(si, geom, se, selfCollision));
            else
                validitySvc_.reset (new FCLStateValidityChecker<Motion_3D>(si, geom, se, selfCollision));
            break;

        default:
            OMPL_ERROR("Unexpected collision checker type (%d) encountered", ctype_);
    };

    return validitySvc_;
}
