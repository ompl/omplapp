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

#ifndef OMPLAPP_GEOMETRY_DETAIL_PQP_COLLISION_CHECKER_
#define OMPLAPP_GEOMETRY_DETAIL_PQP_COLLISION_CHECKER_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/manifolds/SE2StateManifold.h>
#include <ompl/base/manifolds/SE3StateManifold.h>

#include "omplapp/geometry/GeometrySpecification.h"
#include "omplapp/geometry/detail/assimpUtil.h"

#include <PQP.h>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <vector>
#include <cmath>

namespace ompl
{
    namespace app
    {

        /// @cond IGNORE

        template<MotionModel T>
        struct OMPL_StateType
        {
            typedef base::SE3StateManifold::StateType type;

            /** \brief  Convert a quaternion to a 3x3 rotation matrix; based on code from
                http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm */
            void quaternionToMatrix(PQP_REAL m[3][3], const base::SO3StateManifold::StateType &q) const
            {
                double sqw = q.w*q.w;
                double sqx = q.x*q.x;
                double sqy = q.y*q.y;
                double sqz = q.z*q.z;

                m[0][0] =  sqx - sqy - sqz + sqw;
                m[1][1] = -sqx + sqy - sqz + sqw;
                m[2][2] = -sqx - sqy + sqz + sqw;

                double tmp1 = q.x*q.y;
                double tmp2 = q.z*q.w;
                m[1][0] = 2.0 * (tmp1 + tmp2);
                m[0][1] = 2.0 * (tmp1 - tmp2);
                tmp1 = q.x*q.z;
                tmp2 = q.y*q.w;
                m[2][0] = 2.0 * (tmp1 - tmp2);
                m[0][2] = 2.0 * (tmp1 + tmp2);
                tmp1 = q.y*q.z;
                tmp2 = q.x*q.w;
                m[2][1] = 2.0 * (tmp1 + tmp2);
                m[1][2] = 2.0 * (tmp1 - tmp2);
            }

            void PQP_pose_from_state(PQP_REAL robTrans[3], PQP_REAL robRot[3][3], const type &s) const
            {
                robTrans[0] = s.getX();
                robTrans[1] = s.getY();
                robTrans[2] = s.getZ();
                quaternionToMatrix(robRot, s.rotation());
            }
        };

        template<>
        struct OMPL_StateType<Motion_2D>
        {
            typedef base::SE2StateManifold::StateType type;

            void PQP_pose_from_state(PQP_REAL robTrans[3], PQP_REAL robRot[3][3], const type &s) const
            {
                robTrans[0] = s.getX();
                robTrans[1] = s.getY();
                robTrans[2] = 0.0;

                const double ca = cos(s.getYaw());
                const double sa = sin(s.getYaw());

                robRot[0][0] = ca;
                robRot[0][1] = -sa;
                robRot[0][2] = 0.0;

                robRot[1][0] = sa;
                robRot[1][1] = ca;
                robRot[1][2] = 0.0;

                robRot[2][0] = 0.0;
                robRot[2][1] = 0.0;
                robRot[2][2] = 1.0;
            }
        };

        /// @endcond


        /** \brief Define an ompl::base::StateValidityChecker that can
            construct PQP models internally.  The instance is still
            abstract however, as the isValid() function is not
            implemented (knowledge of the manifold is needed for this
            function to be implemented) */
        template<MotionModel T>
        class PQPStateValidityChecker : public base::StateValidityChecker
        {
        public:

            PQPStateValidityChecker(const base::SpaceInformationPtr &si, const GeometrySpecification &geom,
                                    const GeometricStateExtractor &se, bool selfCollision) : base::StateValidityChecker(si), extractState_(se),
                                                                                             selfCollision_(selfCollision), msg_("Collision Checker")
            {
                configure(geom);
            }

            virtual bool isValid(const base::State *state) const
            {
                typedef typename OMPL_StateType<T>::type StateType;

                if (!environment_)
                    return true;

                static PQP_REAL identityTranslation[3] = { 0.0, 0.0, 0.0 };
                static PQP_REAL identityRotation[3][3] = { { 1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0} };

                boost::mutex::scoped_lock slock(mutex_);

                PQP_REAL robTrans[3];
                PQP_REAL robRot[3][3];

                for (std::size_t i = 0 ; i < robotParts_.size() ; ++i)
                {
                    stateConvertor_.PQP_pose_from_state(robTrans, robRot, *static_cast<const StateType*>(extractState_(state, i)));
                    PQP_CollideResult cr;
                    PQP_Collide(&cr, robRot, robTrans, robotParts_[i].get(),
                                identityRotation, identityTranslation, environment_.get(), PQP_FIRST_CONTACT);
                    if (cr.Colliding())
                        return false;
                }

                if (selfCollision_)
                {
                    PQP_REAL robTrans2[3];
                    PQP_REAL robRot2[3][3];
                    for (std::size_t i  = 0 ; i < robotParts_.size() ; ++i)
                    {
                        stateConvertor_.PQP_pose_from_state(robTrans, robRot, *static_cast<const StateType*>(extractState_(state, i)));
                        for (std::size_t j  = i + 1 ; j < robotParts_.size() ; ++j)
                        {
                            stateConvertor_.PQP_pose_from_state(robTrans2, robRot2, *static_cast<const StateType*>(extractState_(state, j)));
                            PQP_CollideResult cr;
                            PQP_Collide(&cr, robRot, robTrans, robotParts_[i].get(),
                                        robRot2, robTrans2, robotParts_[j].get(), PQP_FIRST_CONTACT);
                            if (cr.Colliding())
                                return false;
                        }
                    }
                }

                return true;
            }

        protected:

            /** \brief Shared pointer wrapper for PQP_Model */
            typedef boost::shared_ptr<PQP_Model>     PQPModelPtr;

            void configure(const GeometrySpecification &geom)
            {
                environment_ = getPQPModelFromScene(geom.obstacles, geom.obstaclesShift);
                if (!environment_)
                    msg_.inform("Empty environment loaded");
                else
                    msg_.inform("Loaded environment model with %d triangles", environment_->num_tris);

                for (unsigned int i = 0 ; i < geom.robot.size() ; ++i)
                {
                    aiVector3D shift(0.0, 0.0, 0.0);
                    if (geom.robotShift.size() > i)
                        shift = geom.robotShift[i];
                    PQPModelPtr m = getPQPModelFromScene(geom.robot[i], shift);
                    if (!m)
                        throw Exception("Invalid robot mesh");
                    else
                        msg_.inform("Loaded robot model with %d triangles", m->num_tris);
                    robotParts_.push_back(m);
                }
            }

            PQPModelPtr getPQPModelFromScene(const aiScene *scene, const aiVector3D &center) const
            {
                std::vector<const aiScene*> scenes(1, scene);
                std::vector<aiVector3D>     centers(1, center);
                return getPQPModelFromScene(scenes, centers);
            }

            /** \brief Convert a mesh to a PQP model */
            PQPModelPtr getPQPModelFromScene(const std::vector<const aiScene*> &scenes, const std::vector<aiVector3D> &center) const
            {
                std::vector<aiVector3D> triangles;
                for (unsigned int i = 0 ; i < scenes.size() ; ++i)
                    if (scenes[i])
                    {
                        std::vector<aiVector3D> t;
                        scene::extractTriangles(scenes[i], t);
                        if (center.size() > i)
                            for (unsigned int j = 0 ; j < t.size() ; ++j)
                                t[j] -= center[i];
                        triangles.insert(triangles.end(), t.begin(), t.end());
                    }
                return getPQPModelFromTris(triangles);
            }

            /** \brief Convert a set of triangles to a PQP model */
            PQPModelPtr getPQPModelFromTris(const std::vector<aiVector3D> &triangles) const
            {
                PQPModelPtr model;

                if (triangles.empty())
                    return model;

                // create the PQP model
                model.reset(new PQP_Model());
                model->BeginModel();
                int id = 0;
                const int N = triangles.size() / 3;
                for (int j = 0 ; j < N ; ++j)
                {
                    const aiVector3D &v0 = triangles[j * 3];
                    const aiVector3D &v1 = triangles[j * 3 + 1];
                    const aiVector3D &v2 = triangles[j * 3 + 2];
                    PQP_REAL dV0[3] = {v0.x, v0.y, v0.z};
                    PQP_REAL dV1[3] = {v1.x, v1.y, v1.z};
                    PQP_REAL dV2[3] = {v2.x, v2.y, v2.z};
                    model->AddTri(dV0, dV1, dV2, id++);
                }

                model->EndModel();

                return model;
            }

            OMPL_StateType<T>           stateConvertor_;

            GeometricStateExtractor     extractState_;

            bool                        selfCollision_;

            /** \brief Model of the robot */
            std::vector<PQPModelPtr>    robotParts_;

            /** \brief Model of the environment */
            PQPModelPtr                 environment_;

            mutable boost::mutex        mutex_;

            /** \brief Interface used for reporting errors */
            msg::Interface              msg_;

        };

    }
}

#endif
