/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Mark Moll */

#ifndef OMPLAPP_KINEMATIC_CAR_PLANNING_
#define OMPLAPP_KINEMATIC_CAR_PLANNING_

#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ompl
{
    namespace app
    {
        /** \brief A class to facilitate planning for a generic kinematic car
            model

            The dynamics of the kinematic car are described by the following
            equations:
            \f{eqnarray*}{
            \dot x &=& u_0 \cos\theta,\\
            \dot y &=& u_0\sin\theta,\\
            \dot\theta &=& \frac{u_0}{L}\tan u_1,\f}
            where the control inputs \f$(u_0,u_1)\f$ are the translational
            velocity and the steering angle, respectively, and \f$L\f$ is the
            distance between the front and rear axle of the car (set to 1 by
            default).
        */
        class KinematicCarPlanning : public AppBase<AppType::CONTROL>
        {
        public:
            KinematicCarPlanning();
            KinematicCarPlanning(const control::ControlSpacePtr &controlSpace);
            ~KinematicCarPlanning() override = default;

            bool isSelfCollisionEnabled() const override
            {
                return false;
            }
            unsigned int getRobotCount() const override
            {
                return 1;
            }
            base::ScopedState<> getDefaultStartState() const override;
            base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const override
            {
                return state;
            }
            const base::StateSpacePtr& getGeometricComponentStateSpace() const override
            {
                return getStateSpace();
            }
            double getVehicleLength()
            {
                return 1./lengthInv_;
            }
            void setVehicleLength(double length)
            {
                lengthInv_ = 1./length;
            }
            virtual void setDefaultControlBounds();

        protected:

            const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/) const override
            {
                return state;
            }

            virtual void ode(const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot);

            virtual void postPropagate(const base::State* state, const control::Control* control, double duration, base::State* result);

            static control::ControlSpacePtr constructControlSpace()
            {
                return std::make_shared<control::RealVectorControlSpace>(constructStateSpace(), 2);
            }
            static base::StateSpacePtr constructStateSpace()
            {
                return std::make_shared<base::SE2StateSpace>();
            }

            double timeStep_{1e-2};
            double lengthInv_{1.};
            control::ODESolverPtr odeSolver;
        };
    }
}

#endif
