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
#include <ompl/base/manifolds/SE2StateManifold.h>
#include <ompl/control/manifolds/RealVectorControlManifold.h>

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
        class DynamicCarPlanning : public AppBase<CONTROL>
        {
        public:
            DynamicCarPlanning()
                : AppBase<CONTROL>(constructControlManifold(), Motion_2D), timeStep_(1e-3), lengthInv_(1.), mass_(1.)
            {
                name_ = std::string("Dynamic car");
                setDefaultBounds();
                getControlManifold()->setPropagationFunction(boost::bind(&DynamicCarPlanning::propagate, this, _1, _2, _3, _4));
            }
            ~DynamicCarPlanning()
            {
            }

            bool isSelfCollisionEnabled(void) const
            {
                return false;
            }
            virtual unsigned int getRobotCount(void) const
            {
                return 1;
            }
            virtual base::ScopedState<> getDefaultStartState(void) const;
            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const
            {
                return state;
            }
            virtual const base::StateManifoldPtr& getGeometricComponentStateManifold(void) const
            {
                return getStateManifold()->as<base::CompoundStateManifold>()->getSubManifold(0);
            }
            virtual const base::State* getGeometricComponentState(const base::State* state, unsigned int index) const
            {
                return state->as<base::CompoundState>()->components[0];
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
            void propagate(const base::State *from, const control::Control *ctrl,
                const double duration, base::State *result);
            virtual void ode(const base::State *q, const control::Control *ctrl, base::State *qdot);

            static control::ControlManifoldPtr constructControlManifold(void)
            {
                return control::ControlManifoldPtr(new control::RealVectorControlManifold(constructStateManifold(), 2));
            }
            static base::StateManifoldPtr constructStateManifold(void)
            {
                return base::StateManifoldPtr(new base::SE2StateManifold())
                    + base::StateManifoldPtr(new base::RealVectorStateManifold(2));
            }

            double timeStep_;
            double lengthInv_;
            double mass_;
        };

    }
}

#endif
