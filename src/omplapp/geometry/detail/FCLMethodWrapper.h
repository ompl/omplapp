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
#include <fcl/collision.h>
#include <fcl/collision_node.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/continuous_collision.h>

// Boost and STL headers
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <vector>
#include <limits>
#include <cmath>

namespace ob = ompl::base;

namespace ompl
{
    namespace app
    {
        OMPL_CLASS_FORWARD (FCLMethodWrapper);

        /// \brief Wrapper for FCL discrete and continuous collision checking and distance queries
        class FCLMethodWrapper
        {
        public:

            typedef boost::function<void(fcl::Vec3f&, fcl::Quaternion3f&, const base::State*)> FCLPoseFromStateCallback;

            FCLMethodWrapper (const GeometrySpecification &geom,
                              const GeometricStateExtractor &se,
                              bool selfCollision,
                              FCLPoseFromStateCallback poseCallback) : extractState_(se), selfCollision_(selfCollision),
                                                                       poseFromStateCallback_ (poseCallback)
            {
                configure (geom);
            }

            virtual ~FCLMethodWrapper (void)
            {
                for (unsigned int i=0; i<robotParts_.size(); ++i)
                    delete robotParts_[i];
            }

            /// \brief Checks whether the given robot state collides with the
            /// environment or itself.
            virtual bool isValid (const base::State *state) const
            {
                fcl::CollisionRequest collisionRequest;
                fcl::CollisionResult collisionResult;
                fcl::Quaternion3f rot;
                fcl::Vec3f pos;
                fcl::Transform3f transform;

                if (environment_.num_tris > 0)
                {
                    // Performing collision checking with environment.
                    for (std::size_t i = 0; i < robotParts_.size(); ++i)
                    {
                        poseFromStateCallback_(pos, rot, extractState_(state, i));
                        transform.setTransform(rot, pos);
                        if (fcl::collide(robotParts_[i], transform, &environment_,
                            fcl::Transform3f(), collisionRequest, collisionResult) > 0)
                            return false;
                    }
                }

                // Checking for self collision
                if (selfCollision_)
                {
                    fcl::Transform3f trans_i, trans_j;
                    for (std::size_t i = 0 ; i < robotParts_.size(); ++i)
                    {
                        poseFromStateCallback_(pos, rot, extractState_(state, i));
                        trans_i.setTransform(rot, pos);

                        for (std::size_t j  = i + 1 ; j < robotParts_.size (); ++j)
                        {
                            poseFromStateCallback_(pos, rot, extractState_(state, j));
                            trans_j.setTransform(rot, pos);
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
            virtual bool isValid (const base::State *s1, const base::State *s2, double &collisionTime) const
            {
                fcl::Transform3f transi_beg, transi_end, trans;
                fcl::Quaternion3f rot;
                fcl::Vec3f pos;
                fcl::ContinuousCollisionRequest collisionRequest(10, 0.0001, fcl::CCDM_SCREW,
                    fcl::GST_LIBCCD, fcl::CCDC_CONSERVATIVE_ADVANCEMENT);
                fcl::ContinuousCollisionResult collisionResult;

                // Checking for collision with environment
                if (environment_.num_tris > 0)
                {
                    for (size_t i = 0; i < robotParts_.size(); ++i)
                    {
                        // Getting the translation and rotation from s1 and s2
                        poseFromStateCallback_(pos, rot, extractState_(s1, i));
                        transi_beg.setTransform(rot, pos);
                        poseFromStateCallback_(pos, rot, extractState_(s2, i));
                        transi_end.setTransform(rot, pos);

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
                    fcl::Transform3f transj_beg, transj_end;
                    for (std::size_t i = 0 ; i < robotParts_.size(); ++i)
                    {
                        poseFromStateCallback_(pos, rot, extractState_(s1, i));
                        transi_beg.setTransform(rot, pos);
                        poseFromStateCallback_(pos, rot, extractState_(s2, i));
                        transi_end.setTransform(rot, pos);

                        for (std::size_t j = i+1; j < robotParts_.size(); ++j)
                        {
                            poseFromStateCallback_(pos, rot, extractState_(s1, j));
                            transj_beg.setTransform(rot, pos);
                            poseFromStateCallback_(pos, rot, extractState_(s2, j));
                            transj_end.setTransform(rot, pos);

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
            virtual double clearance (const base::State *state) const
            {
                double minDist = std::numeric_limits<double>::infinity ();
                if (environment_.num_tris > 0)
                {
                    fcl::DistanceRequest distanceRequest(true);
                    fcl::DistanceResult distanceResult;
                    fcl::Transform3f trans, trans_env;
                    fcl::Quaternion3f rot;
                    fcl::Vec3f pos;
                    fcl::MeshDistanceTraversalNodeOBBRSS distanceNode;
                    for (size_t i = 0; i < robotParts_.size (); ++i)
                    {
                        poseFromStateCallback_(pos, rot, extractState_(state, i));
                        trans.setTransform(rot, pos);
                        fcl::initialize(distanceNode, *robotParts_[i], trans, environment_, trans_env, distanceRequest, distanceResult);
                        fcl::distance (&distanceNode);
                        if (distanceResult.min_distance < minDist)
                            minDist = distanceResult.min_distance;
                    }
                }

                return minDist;
            }

         protected:

            /// \brief Configures the geometry of the robot and the environment
            /// to setup validity checking.
            void configure (const GeometrySpecification &geom)
            {
                // Configuring the model of the environment
                environment_.beginModel ();
                std::pair <std::vector <fcl::Vec3f>, std::vector<fcl::Triangle> > tri_model;
                tri_model = getFCLModelFromScene (geom.obstacles, geom.obstaclesShift);
                environment_.addSubModel (tri_model.first, tri_model.second);

                environment_.endModel ();
                environment_.computeLocalAABB ();

                if (environment_.num_tris == 0)
                    OMPL_INFORM("Empty environment loaded");
                else
                    OMPL_INFORM("Loaded environment model with %d triangles.", environment_.num_tris);

                // Configuring the model of the robot, composed of one or more pieces
                for (size_t rbt = 0; rbt < geom.robot.size (); ++rbt)
                {
                    Model* model = new Model();
                    model->beginModel ();
                    aiVector3D shift(0.0, 0.0, 0.0);
                    if (geom.robotShift.size () > rbt)
                        shift = geom.robotShift[rbt];

                    tri_model = getFCLModelFromScene (geom.robot[rbt], shift);
                    model->addSubModel (tri_model.first, tri_model.second);

                    model->endModel ();
                    model->computeLocalAABB ();

                    OMPL_INFORM("Robot piece with %d triangles loaded", model->num_tris);
                    robotParts_.push_back (model);
                }
            }

            /// \brief Convert a mesh to a FCL BVH model
            std::pair <std::vector <fcl::Vec3f>, std::vector<fcl::Triangle> > getFCLModelFromScene (const aiScene *scene, const aiVector3D &center) const
            {
                std::vector<const aiScene*> scenes(1, scene);
                std::vector<aiVector3D>     centers(1, center);
                return getFCLModelFromScene(scenes, centers);
            }

            /// \brief Convert a mesh to a FCL BVH model
            std::pair <std::vector <fcl::Vec3f>, std::vector<fcl::Triangle> >getFCLModelFromScene (const std::vector<const aiScene*> &scenes, const std::vector<aiVector3D> &center) const
            {
                // Model consists of a set of points, and a set of triangles
                // that connect those points
                std::vector<fcl::Triangle> triangles;
                std::vector <fcl::Vec3f> pts;

                for (unsigned int i = 0; i < scenes.size (); ++i)
                {
                    if (scenes[i])
                    {
                        std::vector<aiVector3D> t;
                        // extractTriangles is a misleading name.  this extracts the set of points,
                        // where each set of three contiguous points consists of a triangle
                        scene::extractTriangles (scenes[i], t);

                        if (center.size () > i)
                            for (unsigned int j = 0; j < t.size (); ++j)
                                t[j] -= center[i];

                        assert (t.size () % 3 == 0);

                        for (unsigned int j = 0; j < t.size (); ++j)
                        {
                            pts.push_back (fcl::Vec3f (t[j][0], t[j][1], t[j][2]));
                        }

                        for (unsigned int j = 0; j < t.size (); j+=3)
                            triangles.push_back (fcl::Triangle (j, j+1, j+2));
                    }
                }
                return std::make_pair (pts, triangles);
            }

            /// \brief The type of geometric bounding done for the robot and environment
            typedef fcl::OBBRSS  BVType;
            /// \brief The type geometric model used for the meshes
            typedef fcl::BVHModel <BVType> Model;

            /// \brief Geometric model used for the environment
            Model environment_;

            /// \brief List of components for the geometric model of the robot
            mutable std::vector <Model*> robotParts_;

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
