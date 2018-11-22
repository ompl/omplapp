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

#ifndef OMPLAPP_DYNAMIC_CAR_PLANNING_
#define OMPLAPP_DYNAMIC_CAR_PLANNING_

#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ompl
{
    namespace app
    {
        /** \brief A class to facilitate planning for a generic second-order
            car model

            The dynamics of the second-order car are described by the following
            equations:
            \f{eqnarray*}{
            \dot x &=& v\cos\theta,\\
            \dot y &=& v\sin\theta,\\
            \dot\theta &=& \frac{vm}{L}\tan \phi,\\
            \dot v &=& u_0,\\
            \dot\phi &=& u_1,\f}
            where \f$v\f$ is the speed, \f$\phi\f$ the steering angle, the
            controls \f$(u_0,u_1)\f$ control their rate of change, \f$m\f$ is
            the mass of the car, and \f$L\f$ is the distance between the front
            and rear axle of the car. Both \f$m\f$ and \f$L\f$ are set to 1 by
            default.
        */
        class DynamicCarPlanning : public AppBase<AppType::CONTROL>
        {
        public:
            DynamicCarPlanning()
                : AppBase<AppType::CONTROL>(constructControlSpace(), Motion_2D),
                  odeSolver(std::make_shared<control::ODEBasicSolver<>>(si_, [this](const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot)
                      {
                          ode(q, ctrl, qdot);
                      }))
            {
                name_ = std::string("Dynamic car");
                setDefaultBounds();

                si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver,
                    [this](const base::State* state, const control::Control* control, const double duration, base::State* result)
                    {
                        postPropagate(state, control, duration, result);
                    }));
            }
            ~DynamicCarPlanning() override = default;

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
                base::ScopedState<> r(si_);
                r = 0.0;
                r[0] = state[0];
                r[1] = state[1];
                r[2] = state[2];
                return r;
            }

            const base::StateSpacePtr& getGeometricComponentStateSpace() const override
            {
                return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(0);
            }

            double getVehicleLength()
            {
                return 1./lengthInv_;
            }
            void setVehicleLength(double length)
            {
                lengthInv_ = 1./length;
            }
            double getMass()
            {
                return mass_;
            }
            void setMass(double mass)
            {
                mass_ = mass;
            }
            virtual void setDefaultBounds();

        protected:

            const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/) const override
            {
                return state->as<base::CompoundState>()->components[0];
            }

            virtual void ode(const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot);

            virtual void postPropagate(const base::State* state, const control::Control* control, double duration, base::State* result);

            static control::ControlSpacePtr constructControlSpace()
            {
                return std::make_shared<control::RealVectorControlSpace>(constructStateSpace(), 2);
            }
            static base::StateSpacePtr constructStateSpace()
            {
                auto stateSpace(std::make_shared<base::CompoundStateSpace>());
                stateSpace->addSubspace(std::make_shared<base::SE2StateSpace>(), 1.);
                stateSpace->addSubspace(std::make_shared<base::RealVectorStateSpace>(2), .3);
                stateSpace->lock();
                return stateSpace;
            }

            double timeStep_{1e-2};
            double lengthInv_{1.};
            double mass_{1.};
            control::ODESolverPtr odeSolver;
        };

    }
}

#endif
