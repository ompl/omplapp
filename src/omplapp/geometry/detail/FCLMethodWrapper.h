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
#include <fcl/BVH/BVH_model.h>
#include <fcl/collision.h>
#include <fcl/collision_node.h>
#include <fcl/math/transform.h>
#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/ccd/conservative_advancement.h>

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
        ClassForward (FCLMethodWrapper);

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
            }

            /// \brief Checks whether the given robot state collides with the
            /// environment or itself.
            virtual bool isValid (const base::State *state) const
            {
                bool valid = true;
                fcl::CollisionRequest collisionRequest;
                fcl::CollisionResult collisionResult;

                if (environment_.num_tris > 0)
                {
                    // Performing collision checking with environment.
                    for (size_t i = 0; i < robotParts_.size () && valid; ++i)
                    {
                        fcl::Quaternion3f quaternion;
                        fcl::Vec3f translation;
                        poseFromStateCallback_(translation, quaternion, extractState_(state, i));

                        fcl::Transform3f transform;
                        transform.setTransform (quaternion, translation);

                        valid &= fcl::collide (&robotParts_[i], transform, &environment_, fcl::Transform3f(),
                            collisionRequest, collisionResult) == 0;
                    }
                }

                // Checking for self collision
                if (selfCollision_ && valid)
                {
                    for (std::size_t i = 0 ; i < robotParts_.size () && valid; ++i)
                    {
                        fcl::Quaternion3f qi;
                        fcl::Vec3f ti;
                        poseFromStateCallback_(ti, qi, extractState_(state, i));

                        fcl::Transform3f trans_i;
                        trans_i.setTransform (qi, ti);

                        for (std::size_t j  = i + 1 ; j < robotParts_.size () && valid; ++j)
                        {
                            fcl::Quaternion3f qj;
                            fcl::Vec3f tj;
                            poseFromStateCallback_(tj, qj, extractState_(state, j));

                            fcl::Transform3f trans_j;
                            trans_i.setTransform (qj, tj);

                            valid &= fcl::collide (&robotParts_[i], trans_i, &robotParts_[j], trans_j,
                                collisionRequest, collisionResult) == 0;
                        }
                    }
                }

                return valid;
            }

            /// \brief Check the continuous motion between s1 and s2.  If there is a collision
            /// collisionTime will contain the parameterized time to collision in the range [0,1).
            virtual bool isValid (const base::State *s1, const base::State *s2, double &collisionTime) const
            {
                bool valid (true);
                collisionTime = 1.0;

                fcl::Quaternion3f quat1, quat2;
                fcl::Vec3f trans1, trans2;
                fcl::Matrix3f rot1, rot2;
                fcl::CollisionRequest collisionRequest;
                fcl::CollisionResult collisionResult;

                // Checking for collision with environment
                if (environment_.num_tris > 0)
                {
                    for (size_t i = 0; i < robotParts_.size () && valid; ++i)
                    {
                        // Getting the translation and rotation from s1 and s2
                        poseFromStateCallback_(trans1, quat1, extractState_(s1, i));
                        poseFromStateCallback_(trans2, quat2, extractState_(s2, i));

                        quat1.toRotation (rot1);
                        quat2.toRotation (rot2);

                        // Interpolating part i from s1 to s2
                        fcl::InterpMotion motion1 (rot1, trans1, rot2, trans2);
                        // The environment does not move
                        fcl::InterpMotion motion2;

                        // Checking for collision
                        valid &= (fcl::conservativeAdvancement <BVType, fcl::MeshConservativeAdvancementTraversalNodeOBBRSS, fcl::MeshCollisionTraversalNodeOBBRSS>
                            (&robotParts_[i], &motion1, &environment_, &motion2,
                            collisionRequest, collisionResult, collisionTime) == 0);
                    }
                }

                // Checking for self collision
                if (selfCollision_ && valid)
                {
                    for (std::size_t i = 0 ; i < robotParts_.size () && valid; ++i)
                    {
                        poseFromStateCallback_(trans1, quat1, extractState_(s1, i));
                        poseFromStateCallback_(trans2, quat2, extractState_(s2, i));

                        quat1.toRotation (rot1);
                        quat2.toRotation (rot2);

                        // Interpolating part i from s1 to s2
                        fcl::InterpMotion motion_i (rot1, trans1, rot2, trans2);

                        for (std::size_t j = i+1; j < robotParts_.size () && valid; ++j)
                        {
                            poseFromStateCallback_(trans1, quat1, extractState_(s1, j));
                            poseFromStateCallback_(trans2, quat2, extractState_(s2, j));

                            quat1.toRotation (rot1);
                            quat2.toRotation (rot2);

                            // Interpolating part j from s1 to s2
                            fcl::InterpMotion motion_j (rot1, trans1, rot2, trans2);

                            // Checking for collision
                            valid &= (fcl::conservativeAdvancement <BVType, fcl::MeshConservativeAdvancementTraversalNodeOBBRSS, fcl::MeshCollisionTraversalNodeOBBRSS>
                                (&robotParts_[i], &motion_i, &robotParts_[j], &motion_j,
                                collisionRequest, collisionResult, collisionTime) == 0);
                        }
                    }
                }

                return valid;
            }

            /// \brief Returns the minimum distance from the given robot state and the environment
            virtual double clearance (const base::State *state) const
            {
                double dist = std::numeric_limits<double>::infinity ();
                fcl::DistanceRequest distanceRequest;
                fcl::DistanceResult distanceResult;
                if (environment_.num_tris > 0)
                {
                    for (size_t i = 0; i < robotParts_.size (); ++i)
                    {
                        fcl::Quaternion3f q1;
                        fcl::Vec3f t1;
                        poseFromStateCallback_(t1, q1, extractState_(state, i));

                        fcl::Transform3f tr1, tr2;
                        tr1.setTransform (q1, t1);

                        fcl::MeshDistanceTraversalNodeOBBRSS distanceNode;
                        fcl::initialize (distanceNode, robotParts_[i], tr1, environment_, tr2, distanceRequest, distanceResult);

                        if (distanceResult.min_distance < dist)
                            dist = distanceResult.min_distance;
                    }
                }

                return dist;
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
                    logInform("Empty environment loaded");
                else
                    logInform("Loaded environment model with %d triangles.", environment_.num_tris);

                // Configuring the model of the robot, composed of one or more pieces
                for (size_t rbt = 0; rbt < geom.robot.size (); ++rbt)
                {
                    Model model;
                    model.beginModel ();
                    aiVector3D shift(0.0, 0.0, 0.0);
                    if (geom.robotShift.size () > rbt)
                        shift = geom.robotShift[rbt];

                    tri_model = getFCLModelFromScene (geom.robot[rbt], shift);
                    model.addSubModel (tri_model.first, tri_model.second);

                    model.endModel ();
                    model.computeLocalAABB ();

                    logInform("Robot piece with %d triangles loaded", model.num_tris);
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
            mutable std::vector <Model> robotParts_;

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
