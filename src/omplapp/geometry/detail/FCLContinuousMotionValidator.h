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

#ifndef OMPLAPP_GEOMETRY_DETAIL_FCL_CONTINUOUS_MOTION_VALIDATOR_
#define OMPLAPP_GEOMETRY_DETAIL_FCL_CONTINUOUS_MOTION_VALIDATOR_

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>

#include "omplapp/geometry/detail/FCLMethodWrapper.h"
#include "omplapp/geometry/detail/FCLStateValidityChecker.h"
#include "omplapp/geometry/GeometrySpecification.h"

namespace ob = ompl::base;

namespace ompl
{
    namespace app
    {
        /// \brief A motion validator that utilizes FCL's continuous collision checking
        class FCLContinuousMotionValidator : public ob::MotionValidator
        {
        public:

            /// \brief Constructor
            FCLContinuousMotionValidator(ob::SpaceInformation* si, MotionModel mm) : ob::MotionValidator(si)
            {
                defaultSettings(mm);
            }

            /// \brief Constructor
            FCLContinuousMotionValidator(const ob::SpaceInformationPtr &si, MotionModel mm) : ob::MotionValidator(si)
            {
                defaultSettings(mm);
            }

            /// \brief Destructor
            ~FCLContinuousMotionValidator() override = default;

            /// \brief Returns true if motion between s1 and s2 is collision free.
            bool checkMotion(const ob::State *s1, const ob::State *s2) const override
            {
                double unused;

                // assume motion starts in a valid configuration so s1 is valid
                // Must check validity of s2 before performing collision check between s1 and s2
                bool valid = si_->isValid(s2) && fclWrapper_->isValid (s1, s2, unused);

                // Increment valid/invalid motion counters
                valid ? valid_++ : invalid_++;

                return valid;
            }

            /// \brief Checks the motion between s1 and s2. If the motion is
            /// invalid, lastValid contains the last valid state and the
            /// parameterized time [0,1) when this state occurs.
            bool checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State*, double> &lastValid) const override
            {
                bool valid = false;

                // if there is a collision, collisionTime will contain the time to collision,
                // parameterized from [0,1), where s1 is 0 and s2 is 1.
                double collisionTime;
                valid = fclWrapper_->isValid (s1, s2, collisionTime);

                // Find the last valid state before collision...
                // NOTE: This should probably be refactored so that the continuous checker
                // returns the last valid time.  Last valid transformation may also be
                // possible, but that introduces a dependency on the geometry.
                if (!valid)
                {
                    ob::State *lastValidState;
                    if (lastValid.first != nullptr)
                        lastValidState = lastValid.first;
                    else
                        lastValidState = si_->allocState ();

                    collisionTime -= 0.01;
                    stateSpace_->interpolate (s1, s2, collisionTime, lastValidState);

                    while (!si_->isValid (lastValidState) && collisionTime > 0)
                    {
                        collisionTime -= 0.01;
                        stateSpace_->interpolate (s1, s2, collisionTime, lastValidState);
                    }

                    // ensure that collisionTime is greater than zero
                    if (collisionTime < 0.01)
                    {
                        collisionTime = 0.0;
                        si_->copyState (lastValidState, s1);
                    }

                    lastValid.second = collisionTime;

                    if (lastValid.first == nullptr)
                        si_->freeState (lastValidState);
                }

                // Increment valid/invalid motion counters
                valid ? valid_++ : invalid_++;

                return valid;
            }

        protected:

            /// \brief Restore settings to default values.
            void defaultSettings(MotionModel mm)
            {
                stateSpace_ = si_->getStateSpace().get();
                if (stateSpace_ == nullptr)
                    throw Exception("No state space for motion validator");

                // Extract FCLWrapper from FCLStateValidityChecker.
                switch (mm)
                {
                    case app::Motion_2D:
                        const app::FCLStateValidityChecker<app::Motion_2D> *fcl_2d_state_checker;
                        fcl_2d_state_checker = dynamic_cast <const app::FCLStateValidityChecker<app::Motion_2D>* > (si_->getStateValidityChecker ().get ());

                        if (fcl_2d_state_checker == nullptr)
                        {
                            // Be extra verbose in this fatal error
                            OMPL_ERROR("Unable to cast state validity checker to FCLStateValidityChecker.");
                            assert (fcl_2d_state_checker != 0);
                        }

                        fclWrapper_ = fcl_2d_state_checker->getFCLWrapper ();
                        break;

                    case app::Motion_3D:
                        const app::FCLStateValidityChecker<app::Motion_3D> *fcl_3d_state_checker;
                        fcl_3d_state_checker = dynamic_cast <const app::FCLStateValidityChecker<app::Motion_3D>* > (si_->getStateValidityChecker ().get ());

                        if (fcl_3d_state_checker == nullptr)
                        {
                            // Be extra verbose in this fatal error
                            OMPL_ERROR("Unable to cast state validity checker to FCLStateValidityChecker.");
                            assert (fcl_3d_state_checker != 0);
                        }

                        fclWrapper_ = fcl_3d_state_checker->getFCLWrapper ();
                        break;

                    default:
                        OMPL_WARN("Unknown motion model specified: %u", mm);
                        break;
                }

                if (!fclWrapper_)
                {
                    // Be extra verbose in this fatal error
                    OMPL_ERROR("FCLWrapper object is not valid.");
                    assert (fclWrapper_ != 0);
                }
            }

            /// \brief Wrapper for FCL collision and distance methods
            FCLMethodWrapperPtr         fclWrapper_;

            /// \brief Handle to the statespace that this motion validator operates in.
            ob::StateSpace*             stateSpace_;
        };
    }
}

#endif
