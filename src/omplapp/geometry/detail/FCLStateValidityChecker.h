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

#include "omplapp/config.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "omplapp/geometry/detail/FCLMethodWrapper.h"
#include "omplapp/geometry/GeometrySpecification.h"

// Boost and STL headers
#include <memory>

namespace ob = ompl::base;

namespace ompl
{
    namespace app
    {
        /// @cond IGNORE
        template<MotionModel T>
        struct OMPL_FCL_StateType
        {
            using type = ob::SE3StateSpace::StateType;

            void FCLPoseFromState(FCLMethodWrapper::Transform &tf, const ob::State *state) const
            {
                const auto *derived = static_cast <const type*>(state);
                const auto &q = derived->rotation();
#if FCL_MAJOR_VERSION==0 && FCL_MINOR_VERSION<6
                tf = FCLMethodWrapper::Transform(
                        FCLMethodWrapper::Quaternion(q.w, q.x, q.y, q.z),
                        FCLMethodWrapper::Vector3(derived->getX(), derived->getY(), derived->getZ())
                    );
#else
                tf = fcl::Translation3d(derived->getX(), derived->getY(), derived->getZ()) *
                    FCLMethodWrapper::Quaternion(q.w, q.x, q.y, q.z);
#endif
            }
        };

        template<>
        struct OMPL_FCL_StateType<Motion_2D>
        {
            using type = ob::SE2StateSpace::StateType;

            void FCLPoseFromState(FCLMethodWrapper::Transform &tf, const ob::State *state) const
            {
                static const FCLMethodWrapper::Vector3 zaxis(0., 0., 1.);
                const auto * derived = static_cast <const type*>(state);
#if FCL_MAJOR_VERSION==0 && FCL_MINOR_VERSION<6
                FCLMethodWrapper::Quaternion q;
                q.fromAxisAngle(zaxis, derived->getYaw());
                tf = FCLMethodWrapper::Transform(q,
                        FCLMethodWrapper::Vector3(derived->getX(), derived->getY(), 0.)
                    );
#else
                tf = fcl::Translation3d(derived->getX(), derived->getY(), 0.) *
                            fcl::AngleAxisd(derived->getYaw(), zaxis);
#endif
            }
        };
        /// @endcond

        /// \brief Wrapper for FCL collision and distance checking
        template<MotionModel T>
        class FCLStateValidityChecker : public ob::StateValidityChecker
        {
        public:
            FCLStateValidityChecker(const ob::SpaceInformationPtr &si, const GeometrySpecification &geom,
                                    const GeometricStateExtractor &se, bool selfCollision)
            : ob::StateValidityChecker(si),
              fclWrapper_(std::make_shared<FCLMethodWrapper>(geom, se, selfCollision,
                [this](FCLMethodWrapper::Transform &tf, const ob::State *state)
                {
                    stateConvertor_.FCLPoseFromState(tf, state);
                }))
            {
                specs_.clearanceComputationType = base::StateValidityCheckerSpecs::EXACT;
            }

            ~FCLStateValidityChecker() override = default;

            /// \brief Checks whether the given robot state collides with the
            /// environment or itself.
            bool isValid(const ob::State *state) const override
            {
                return si_->satisfiesBounds(state) && fclWrapper_->isValid(state);
            }

            /// \brief Returns the minimum distance from the given robot state and the environment
            double clearance(const ob::State *state) const override
            {
                return fclWrapper_->clearance(state);
            }

            const FCLMethodWrapperPtr& getFCLWrapper() const
            {
                return fclWrapper_;
            }

         protected:

            /// \brief Object to convert a configuration of the robot to a type desirable for FCL
            OMPL_FCL_StateType<T>       stateConvertor_;

            /// \brief Wrapper for FCL collision and distance methods
            FCLMethodWrapperPtr         fclWrapper_;

        };
    }
}

#endif
