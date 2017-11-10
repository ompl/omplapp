/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2011, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ryan Luna */

#ifndef OMPLAPP_GEOMETRY_DETAIL_FCL_METHOD_WRAPPER_
#define OMPLAPP_GEOMETRY_DETAIL_FCL_METHOD_WRAPPER_

#include "omplapp/config.h"

// OMPL and OMPL.app headers
#include "omplapp/geometry/GeometrySpecification.h"
#include "omplapp/geometry/detail/assimpUtil.h"

// FCL Headers
#include <fcl/config.h>
#if FCL_MAJOR_VERSION==0 && FCL_MINOR_VERSION<6
#include <fcl/collision.h>
#include <fcl/distance.h>
#include <fcl/collision_node.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/continuous_collision.h>
#else
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/narrowphase/continuous_collision.h>
#endif

// STL headers
#include <memory>
#include <utility>
#include <vector>
#include <limits>
#include <cmath>

namespace ob = ompl::base;

namespace ompl
{
    namespace app
    {
        OMPL_CLASS_FORWARD(FCLMethodWrapper);

        /// \brief Wrapper for FCL discrete and continuous collision checking and distance queries
        class FCLMethodWrapper
        {
        public:

#if FCL_MAJOR_VERSION==0 && FCL_MINOR_VERSION<6
            using Vector3 = fcl::Vec3f;
            using Quaternion = fcl::Quaternion3f;
            using Transform = fcl::Transform3f;
            using BVType = fcl::OBBRSS;
            using Model = fcl::BVHModel<BVType>;
            using MeshDistanceTraversalNodeOBBRSS = fcl::MeshDistanceTraversalNodeOBBRSS;
            using CollisionRequest = fcl::CollisionRequest;
            using CollisionResult = fcl::CollisionResult;
            using ContinuousCollisionRequest = fcl::ContinuousCollisionRequest;
            using ContinuousCollisionResult = fcl::ContinuousCollisionResult;
            using DistanceRequest = fcl::DistanceRequest;
            using DistanceResult = fcl::DistanceResult;
#else
            using Vector3 = fcl::Vector3d;
            using Quaternion = fcl::Quaterniond;
            using Transform = fcl::Transform3d;
            using BVType = fcl::OBBRSS<double>;
            using Model = fcl::BVHModel<BVType>;
            using MeshDistanceTraversalNodeOBBRSS = fcl::detail::MeshDistanceTraversalNodeOBBRSS<double>;
            using CollisionRequest = fcl::CollisionRequest<double>;
            using CollisionResult = fcl::CollisionResult<double>;
            using ContinuousCollisionRequest = fcl::ContinuousCollisionRequest<double>;
            using ContinuousCollisionResult = fcl::ContinuousCollisionResult<double>;
            using DistanceRequest = fcl::DistanceRequest<double>;
            using DistanceResult = fcl::DistanceResult<double>;
#endif

            using FCLPoseFromStateCallback = std::function<void(Transform &, const base::State *)>;

            FCLMethodWrapper(const GeometrySpecification &geom,
                             GeometricStateExtractor se,
                             bool selfCollision,
                             FCLPoseFromStateCallback poseCallback)
                : extractState_(std::move(se)), selfCollision_(selfCollision),
                  poseFromStateCallback_(std::move(poseCallback))
            {
                configure(geom);
            }

            virtual ~FCLMethodWrapper()
            {
                for (auto & robotPart : robotParts_)
                    delete robotPart;
            }

            /// \brief Checks whether the given robot state collides with the
            /// environment or itself.
            virtual bool isValid(const base::State *state) const
            {
#if FCL_MAJOR_VERSION==0 && FCL_MINOR_VERSION<6
                static Transform identity;
#else
                static Transform identity(Transform::Identity());
#endif
                CollisionRequest collisionRequest;
                CollisionResult collisionResult;
                Transform transform;

                if (environment_.num_tris > 0)
                {
                    // Performing collision checking with environment.
                    for (std::size_t i = 0; i < robotParts_.size(); ++i)
                    {
                        poseFromStateCallback_(transform, extractState_(state, i));
                        if (fcl::collide(robotParts_[i], transform, &environment_,
                            identity, collisionRequest, collisionResult) > 0)
                            return false;
                    }
                }

                // Checking for self collision
                if (selfCollision_)
                {
                    Transform trans_i, trans_j;
                    for (std::size_t i = 0 ; i < robotParts_.size(); ++i)
                    {
                        poseFromStateCallback_(trans_i, extractState_(state, i));

                        for (std::size_t j  = i + 1 ; j < robotParts_.size(); ++j)
                        {
                            poseFromStateCallback_(trans_j, extractState_(state, j));
                            if (fcl::collide(robotParts_[i], trans_i, robotParts_[j], trans_j,
                                collisionRequest, collisionResult) > 0)
                                return false;
                        }
                    }
                }

                return true;
            }

            /// \brief Check the continuous motion between s1 and s2.  If there is a collision
            /// collisionTime will contain the parameterized time to collision in the range [0,1).
            virtual bool isValid(const base::State *s1, const base::State *s2, double &collisionTime) const
            {
                Transform transi_beg, transi_end, trans;
                ContinuousCollisionRequest collisionRequest(10, 0.0001, fcl::CCDM_SCREW,
                    fcl::GST_LIBCCD, fcl::CCDC_CONSERVATIVE_ADVANCEMENT);
                ContinuousCollisionResult collisionResult;

                // Checking for collision with environment
                if (environment_.num_tris > 0)
                {
                    for (size_t i = 0; i < robotParts_.size(); ++i)
                    {
                        // Getting the translation and rotation from s1 and s2
                        poseFromStateCallback_(transi_beg, extractState_(s1, i));
                        poseFromStateCallback_(transi_end, extractState_(s2, i));

                        // Checking for collision
                        fcl::continuousCollide(robotParts_[i], transi_beg, transi_end,
                            &environment_, trans, trans,
                            collisionRequest, collisionResult);
                        if (collisionResult.is_collide)
                        {
                            collisionTime = collisionResult.time_of_contact;
                            return false;
                        }
                    }
                }

                // Checking for self collision
                if (selfCollision_)
                {
                    Transform transj_beg, transj_end;
                    for (std::size_t i = 0 ; i < robotParts_.size(); ++i)
                    {
                        poseFromStateCallback_(transi_beg, extractState_(s1, i));
                        poseFromStateCallback_(transi_end, extractState_(s2, i));

                        for (std::size_t j = i+1; j < robotParts_.size(); ++j)
                        {
                            poseFromStateCallback_(transj_beg, extractState_(s1, j));
                            poseFromStateCallback_(transj_end, extractState_(s2, j));

                            // Checking for collision
                            fcl::continuousCollide(robotParts_[i], transi_beg, transi_end,
                                 robotParts_[j], transj_beg, transj_end,
                                 collisionRequest, collisionResult);
                            if (collisionResult.is_collide)
                            {
                                collisionTime = collisionResult.time_of_contact;
                                return false;
                            }
                        }
                    }
                }
                return true;
            }

            /// \brief Returns the minimum distance from the given robot state and the environment
            virtual double clearance(const base::State *state) const
            {
#if FCL_MAJOR_VERSION==0 && FCL_MINOR_VERSION<6
                static Transform identity;
#else
                static Transform identity(Transform::Identity());
#endif
                double minDist = std::numeric_limits<double>::infinity ();
                if (environment_.num_tris > 0)
                {
                    DistanceRequest distanceRequest(true);
                    DistanceResult distanceResult;
                    Transform trans;
                    for (size_t i = 0; i < robotParts_.size (); ++i)
                    {
                        poseFromStateCallback_(trans, extractState_(state, i));
                        fcl::distance(robotParts_[i], trans, &environment_, identity, distanceRequest, distanceResult);
                        if (distanceResult.min_distance < minDist)
                            minDist = distanceResult.min_distance;
                    }
                }

                return minDist;
            }

         protected:

            /// \brief Configures the geometry of the robot and the environment
            /// to setup validity checking.
            void configure(const GeometrySpecification &geom)
            {
                // Configuring the model of the environment
                environment_.beginModel ();
                std::pair<std::vector <Vector3>, std::vector<fcl::Triangle>> tri_model;
                tri_model = getFCLModelFromScene(geom.obstacles, geom.obstaclesShift);
                environment_.addSubModel(tri_model.first, tri_model.second);

                environment_.endModel ();
                environment_.computeLocalAABB();

                if (environment_.num_tris == 0)
                    OMPL_INFORM("Empty environment loaded");
                else
                    OMPL_INFORM("Loaded environment model with %d triangles.", environment_.num_tris);

                // Configuring the model of the robot, composed of one or more pieces
                for (size_t rbt = 0; rbt < geom.robot.size(); ++rbt)
                {
                    auto* model = new Model();
                    model->beginModel();
                    aiVector3D shift(0.0, 0.0, 0.0);
                    if (geom.robotShift.size() > rbt)
                        shift = geom.robotShift[rbt];

                    tri_model = getFCLModelFromScene(geom.robot[rbt], shift);
                    model->addSubModel(tri_model.first, tri_model.second);

                    model->endModel();
                    model->computeLocalAABB();

                    OMPL_INFORM("Robot piece with %d triangles loaded", model->num_tris);
                    robotParts_.push_back(model);
                }
            }

            /// \brief Convert a mesh to a FCL BVH model
            std::pair<std::vector <Vector3>, std::vector<fcl::Triangle>> getFCLModelFromScene(const aiScene *scene, const aiVector3D &center) const
            {
                std::vector<const aiScene*> scenes(1, scene);
                std::vector<aiVector3D>     centers(1, center);
                return getFCLModelFromScene(scenes, centers);
            }

            /// \brief Convert a mesh to a FCL BVH model
            std::pair<std::vector<Vector3>, std::vector<fcl::Triangle>> getFCLModelFromScene(const std::vector<const aiScene*> &scenes, const std::vector<aiVector3D> &center) const
            {
                // Model consists of a set of points, and a set of triangles
                // that connect those points
                std::vector<fcl::Triangle> triangles;
                std::vector<Vector3> pts;

                for (unsigned int i = 0; i < scenes.size(); ++i)
                {
                    if (scenes[i] != nullptr)
                    {
                        std::vector<aiVector3D> t;
                        // extractTriangles is a misleading name.  this extracts the set of points,
                        // where each set of three contiguous points consists of a triangle
                        scene::extractTriangles(scenes[i], t);

                        if (center.size() > i)
                            for (auto & j : t)
                                j -= center[i];

                        assert(t.size() % 3 == 0);

                        for (auto & j : t)
                        {
                            pts.emplace_back(j[0], j[1], j[2]);
                        }

                        for (unsigned int j = 0; j < t.size(); j+=3)
                            triangles.emplace_back(j, j+1, j+2);
                    }
                }
                return std::make_pair(pts, triangles);
            }

            /// \brief Geometric model used for the environment
            Model environment_;

            /// \brief List of components for the geometric model of the robot
            mutable std::vector<Model*> robotParts_;

            /// \brief Callback to get the geometric portion of a specific state
            GeometricStateExtractor     extractState_;

            /// \brief Flag indicating whether the geometry is checked for self collisions
            bool                        selfCollision_;

            /// \brief Callback to extract translation and rotation from a state
            FCLPoseFromStateCallback    poseFromStateCallback_;
        };
    }
}

#endif
