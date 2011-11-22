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

#ifdef OMPL_HAS_FCL

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "omplapp/geometry/detail/FCLMethodWrapper.h"
#include "omplapp/geometry/GeometrySpecification.h"

// Boost and STL headers
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

namespace ob = ompl::base;

namespace ompl
{
    namespace app
    {
        /// @cond IGNORE
        template<MotionModel T>
        struct OMPL_FCL_StateType
        {
            typedef ob::SE3StateSpace::StateType type;

            void FCLPoseFromState(fcl::Vec3f &trans, fcl::SimpleQuaternion &quat, const ob::State *state) const
            {
                const type * derived = static_cast <const type*> (state);

                trans.setValue (derived->getX (), derived->getY (), derived->getZ ());
                quat.getW () = derived->rotation ().w;
                quat.getX () = derived->rotation ().x;
                quat.getY () = derived->rotation ().y;
                quat.getZ () = derived->rotation ().z;
            }
        };

        template<>
        struct OMPL_FCL_StateType<Motion_2D>
        {
            typedef ob::SE2StateSpace::StateType type;

            void FCLPoseFromState (fcl::Vec3f &trans, fcl::SimpleQuaternion &quat, const ob::State *state) const
            {
                const type * derived = static_cast <const type*> (state);

                trans.setValue (derived->getX (), derived->getY (), 0.0);
                const double ca = cos (derived->getYaw ());
                const double sa = sin (derived->getYaw ());

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
        class FCLStateValidityChecker : public ob::StateValidityChecker
        {
        public:
            FCLStateValidityChecker (const ob::SpaceInformationPtr &si, const GeometrySpecification &geom,
                                     const GeometricStateExtractor &se, bool selfCollision) : ob::StateValidityChecker(si),
                                                                                              msg_("FCL Collision Checker"),
                                                                                              fclWrapper_(new FCLMethodWrapper (geom, se, selfCollision, boost::bind (&OMPL_FCL_StateType<T>::FCLPoseFromState, stateConvertor_, _1, _2, _3)))
            {
            }

            virtual ~FCLStateValidityChecker (void)
            {
            }

            /// \brief Checks whether the given robot state collides with the
            /// environment or itself.
            virtual bool isValid (const ob::State *state) const
            {
                return si_->satisfiesBounds (state) && fclWrapper_->isValid (state);
            }

            /// \brief Returns the minimum distance from the given robot state and the environment
            virtual double clearance (const ob::State *state) const
            {
                return fclWrapper_->clearance (state);
            }

            const FCLMethodWrapperPtr& getFCLWrapper (void) const
            {
                return fclWrapper_;
            }

         protected:

            /// \brief Interface used for reporting errors
            msg::Interface              msg_;

            /// \brief Object to convert a configuration of the robot to a type desirable for FCL
            OMPL_FCL_StateType<T>       stateConvertor_;

            /// \brief Wrapper for FCL collision and distance methods
            FCLMethodWrapperPtr         fclWrapper_;

        };
    }
}

#endif // OMPL_HAS_FCL

#endif
