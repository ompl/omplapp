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

#ifndef OMPLAPP_GEOMETRY_DETAIL_FCL_COLLISION_CHECKER_
#define OMPLAPP_GEOMETRY_DETAIL_FCL_COLLISION_CHECKER_

#ifdef USE_FCL

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "omplapp/geometry/GeometrySpecification.h"
#include "omplapp/geometry/detail/assimpUtil.h"

// FCL Headers
#include <fcl/BVH_model.h>
#include <fcl/collision.h>
#include <fcl/collision_node.h>
#include <fcl/transform.h>
#include <fcl/traversal_node_bvhs.h>
#include <fcl/simple_setup.h>
#include <fcl/conservative_advancement.h>

// Boost and STL headers
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <vector>
#include <limits>
#include <cmath>

namespace ompl
{
    namespace app
    {
        /// @cond IGNORE
        template<MotionModel T>
        struct OMPL_FCL_StateType
        {
            typedef base::SE3StateSpace::StateType type;

            void FCLPoseFromState(fcl::Vec3f &trans, fcl::SimpleQuaternion &quat, const type &state) const
            {
                trans.setValue (state.getX (), state.getY (), state.getZ ());
                quat.getW () = state.rotation ().w;
                quat.getX () = state.rotation ().x;
                quat.getY () = state.rotation ().y;
                quat.getZ () = state.rotation ().z;
            }
        };

        template<>
        struct OMPL_FCL_StateType<Motion_2D>
        {
            typedef base::SE2StateSpace::StateType type;

            void FCLPoseFromState (fcl::Vec3f &trans, fcl::SimpleQuaternion &quat, const type &state) const
            {
                trans.setValue (state.getX (), state.getY (), 0.0);

                const double ca = cos (state.getYaw ());
                const double sa = sin (state.getYaw ());

                fcl::Vec3f rot[3];
                rot[0].setValue (ca, -sa,  0.0);
                rot[1].setValue (sa,  ca,  0.0);
                rot[2].setValue (0.0, 0.0, 1.0);

                quat.fromRotation (rot);
            }
        };
        /// @endcond

        /// \brief Wrapper for FCL collision and distance checking
        template<MotionModel T>
        class FCLStateValidityChecker : public base::StateValidityChecker
        {
        public:
            FCLStateValidityChecker (const base::SpaceInformationPtr &si, const GeometrySpecification &geom,
                                     const GeometricStateExtractor &se, bool selfCollision) : base::StateValidityChecker(si), extractState_(se),
                                                                                              selfCollision_(selfCollision), msg_("FCL Collision Checker")
            {
                configure(geom);
            }

            virtual ~FCLStateValidityChecker (void)
            {
            }

            /// \brief Checks whether the given robot state collides with the
            /// environment or itself.
            virtual bool isValid (const base::State *state) const
            {
                bool valid = si_->satisfiesBounds (state) && environment_.num_tris > 0;

                if (valid)
                {
                    boost::mutex::scoped_lock slock(mutex_);

                    // Need to adjust robotParts_ for the state configuration.
                    transformRobot (state);

                    // Performing collision checking with environment.
                    std::vector <fcl::Contact> contacts;
                    for (size_t i = 0; i < robotParts_.size () && valid; ++i)
                    {
                        valid &= fcl::collide (&robotParts_[i], &environment_, 1, false, false, contacts) == 0;
                    }

                    // Checking for self collision
                    if (selfCollision_ && valid)
                    {
                        for (std::size_t i = 0 ; i < robotParts_.size () && valid; ++i)
                        {
                            for (std::size_t j  = i + 1 ; j < robotParts_.size () && valid; ++j)
                            {
                                valid &= fcl::collide (&robotParts_[i], &robotParts_[j], 1, false, false, contacts) == 0;
                            }
                        }
                    }
                }
                return valid;
            }

            /// \brief Check the continuous motion between s1 and s2.  If there is a collision
            /// collisionTime will contain the parameterized time to collision in the range [0,1).
            virtual bool isValid (const base::State *s1, const base::State *s2, double &collisionTime) const
            {
                typedef typename OMPL_FCL_StateType<T>::type StateType;

                // It is assumed that s1 is valid.  Make sure s2 is within bounds
                bool valid = si_->satisfiesBounds (s2) && environment_.num_tris > 0;

                collisionTime = std::numeric_limits<double>::infinity ();
                boost::mutex::scoped_lock slock(mutex_);
                
                transformRobot (s1);

                for (size_t i = 0; i < robotParts_.size () && valid; ++i)
                {
                    fcl::SimpleQuaternion quaternion;
                    fcl::Vec3f translation;
                    stateConvertor_.FCLPoseFromState (translation, quaternion, *static_cast<const StateType*>(extractState_(s2, i)));

                    fcl::Vec3f rotation[3];
                    quaternion.toRotation (rotation);
                    fcl::InterpMotion<BVType> robot_motion (robotParts_[i].getRotation (), robotParts_[i].getTranslation (),
                                                            rotation, translation);

                    fcl::InterpMotion<BVType> env_motion;
                    std::vector <fcl::Contact> contacts;

                    valid &= (fcl::conservativeAdvancement <BVType> (&robotParts_[i], &robot_motion, &environment_, &env_motion,
                                                           1, false, false, contacts, collisionTime) == 0);
                }

                // Checking for self collision
                if (selfCollision_ && valid)
                {
                    std::vector <fcl::Contact> contacts;
                    for (std::size_t i = 0 ; i < robotParts_.size () && valid; ++i)
                    {
                        fcl::SimpleQuaternion quaternion_i;
                        fcl::Vec3f translation_i;
                        stateConvertor_.FCLPoseFromState (translation_i, quaternion_i, *static_cast<const StateType*>(extractState_(s2, i)));

                        fcl::Vec3f rotation_i[3];
                        quaternion_i.toRotation (rotation_i);

                        fcl::InterpMotion<BVType> i_motion (robotParts_[i].getRotation (), robotParts_[i].getTranslation (),
                                                            rotation_i, translation_i);

                        for (std::size_t j  = i + 1 ; j < robotParts_.size () && valid; ++j)
                        {
                            fcl::SimpleQuaternion quaternion_j;
                            fcl::Vec3f translation_j;
                            stateConvertor_.FCLPoseFromState (translation_j, quaternion_j, *static_cast<const StateType*>(extractState_(s2, j)));

                            fcl::Vec3f rotation_j[3];
                            quaternion_j.toRotation (rotation_j);

                            fcl::InterpMotion<BVType> j_motion (robotParts_[j].getRotation (), robotParts_[j].getTranslation (),
                                                                rotation_j, translation_j);

                            valid &= (fcl::conservativeAdvancement <BVType> (&robotParts_[i], &i_motion, &robotParts_[j], &j_motion,
                                                                             1, false, false, contacts, collisionTime) == 0);
                        }
                    }
                }

                return valid;
            }

            /// \brief Returns the minimum distance from the given robot state and the environment
            virtual double clearance (const base::State *state) const
            {
                double dist = std::numeric_limits<double>::infinity ();

                if (environment_.num_tris > 0)
                {
                    boost::mutex::scoped_lock slock(mutex_);

                    // Need to adjust robotParts_ for the state configuration.
                    transformRobot (state);

                    for (size_t i = 0; i < robotParts_.size (); ++i)
                    {
                        fcl::MeshDistanceTraversalNodeRSS distanceNode;
                        initialize (distanceNode, environment_, robotParts_[i]);

                        // computing minimum distance
                        fcl::distance (&distanceNode);

                        if (distanceNode.min_distance < dist)
                            dist = distanceNode.min_distance;
                    }
                }

                return dist;
            }

         protected:

            /// \brief Transforms (translate and rotate) the components of the
            /// robot to correspond to the given state.
            void transformRobot (const base::State *state) const
            {
                typedef typename OMPL_FCL_StateType<T>::type StateType;
                for (size_t i = 0; i < robotParts_.size (); ++i)
                {
                    fcl::SimpleQuaternion quaternion;
                    fcl::Vec3f translation;
                    stateConvertor_.FCLPoseFromState (translation, quaternion, *static_cast<const StateType*>(extractState_(state, i)));
                    robotParts_[i].setTransform (quaternion, translation);
                }
            }

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
                    msg_.inform("Empty environment loaded");
                else
                    msg_.inform("Loaded environment model with %d triangles.", environment_.num_tris);

                // Configuring the model of the robot, composed of one or more pieces
                for (size_t rbt = 0; rbt < geom.robot.size (); ++rbt)
                {
                    RobotModel model;
                    model.beginModel ();
                    aiVector3D shift(0.0, 0.0, 0.0);
                    if (geom.robotShift.size () > rbt)
                        shift = geom.robotShift[rbt];

                    tri_model = getFCLModelFromScene (geom.robot[rbt], shift);
                    model.addSubModel (tri_model.first, tri_model.second);

                    model.endModel ();
                    model.computeLocalAABB ();

                    msg_.inform("Robot piece with %d triangles loaded", model.num_tris);
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

            // The type of geometric bounding done for the robot and environment
            typedef fcl::RSS  BVType;
            // The type geometric model used for the robot
            typedef fcl::BVHModel <BVType> RobotModel;

            /// \brief Geometric model used for the environment
            fcl::BVHModel <BVType> environment_;

            /// \brief List of components for the geometric model of the robot
            mutable std::vector <RobotModel> robotParts_;

            /// \brief Object to convert a configuration of the robot to a type desirable for FCL
            OMPL_FCL_StateType<T>           stateConvertor_;

            /// \brief Callback to get the geometric portion of a specific state
            GeometricStateExtractor     extractState_;

            /// \brief Flag indicating whether the geometry is checked for self collisions
            bool                        selfCollision_;

            /// \brief Interface used for reporting errors
            msg::Interface              msg_;

            /// \brief Mutex for thread safety.
            mutable boost::mutex        mutex_;
        };
    }
}

#endif // USE_FCL

#endif
