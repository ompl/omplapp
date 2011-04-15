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
#include <ompl/base/manifolds/SE2StateManifold.h>
#include <ompl/control/manifolds/RealVectorControlManifold.h>

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
        class KinematicCarPlanning : public AppBase<CONTROL>
        {
        public:
            KinematicCarPlanning()
                : AppBase<CONTROL>(constructControlManifold(), Motion_2D), timeStep_(1e-2), lengthInv_(20.)
            {
                name_ = std::string("Kinematic car");
                setDefaultControlBounds();
                getControlManifold()->setPropagationFunction(boost::bind(&KinematicCarPlanning::propagate, this, _1, _2, _3, _4));
            }
            KinematicCarPlanning(control::ControlManifoldPtr controlManifold)
                : AppBase<CONTROL>(controlManifold, Motion_2D), timeStep_(1e-2), lengthInv_(1.)
            {
                setDefaultControlBounds();
                getControlManifold()->setPropagationFunction(boost::bind(&KinematicCarPlanning::propagate, this, _1, _2, _3, _4));
            }
            ~KinematicCarPlanning()
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
                return getStateManifold();
            }
            virtual const base::State* getGeometricComponentState(const base::State* state, unsigned int index) const
            {
                return state;
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
            void propagate(const base::State *from, const control::Control *ctrl,
                const double duration, base::State *result);
            virtual void ode(const base::State *q, const control::Control *ctrl, base::State *qdot);

            static control::ControlManifoldPtr constructControlManifold(void)
            {
                return control::ControlManifoldPtr(new control::RealVectorControlManifold(constructStateManifold(), 2));
            }
            static base::StateManifoldPtr constructStateManifold(void)
            {
                return base::StateManifoldPtr(new base::SE2StateManifold());
            }

            double timeStep_;
            double lengthInv_;
        };

        /** \brief \brief A class to facilitate planning for a Dubins car model

            The Dubins car allows only the velocity controls { 0, max }
            to be used.
        */
        class DubinsCarPlanning : public KinematicCarPlanning
        {
        public:
            class DubinsControlSampler : public control::ControlSampler
            {
            public:
                DubinsControlSampler(const control::ControlManifold *manifold) : control::ControlSampler(manifold)
                {
                }
                void sample(control::Control* control);
            };

            class DubinsControlManifold : public control::RealVectorControlManifold
            {
            public:
                DubinsControlManifold(const base::StateManifoldPtr &stateManifold)
                    : control::RealVectorControlManifold(stateManifold, 2)
                {
                }
                control::ControlSamplerPtr allocControlSampler(void) const
                {
                    return control::ControlSamplerPtr(new DubinsControlSampler(this));
                }
            };

            DubinsCarPlanning() : KinematicCarPlanning(constructControlManifold())
            {
                name_ = std::string("Dubins car");
            }

        protected:
            static control::ControlManifoldPtr constructControlManifold(void)
            {
                return control::ControlManifoldPtr(new DubinsControlManifold(constructStateManifold()));
            }
        };

        /** \brief \brief A class to facilitate planning for a Reeds-Shepp car model

            The Reeds-Shepp car allows only the velocity controls { min, 0, max }
            to be used.
        */
        class ReedsSheppCarPlanning : public KinematicCarPlanning
        {
        public:
            class ReedsSheppControlSampler : public control::ControlSampler
            {
            public:
                ReedsSheppControlSampler(const control::ControlManifold *manifold) : control::ControlSampler(manifold)
                {
                }
                void sample(control::Control* control);
            };

            class ReedsSheppControlManifold : public control::RealVectorControlManifold
            {
            public:
                ReedsSheppControlManifold(const base::StateManifoldPtr &stateManifold)
                    : control::RealVectorControlManifold(stateManifold, 2)
                {
                }
                control::ControlSamplerPtr allocControlSampler(void) const
                {
                    return control::ControlSamplerPtr(new ReedsSheppControlSampler(this));
                }
            };

            ReedsSheppCarPlanning() : KinematicCarPlanning(constructControlManifold())
            {
                name_ = std::string("Reeds-Shepp car");
            }

        protected:
            static control::ControlManifoldPtr constructControlManifold(void)
            {
                return control::ControlManifoldPtr(new ReedsSheppControlManifold(constructStateManifold()));
            }
        };

    }
}

#endif
