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

#ifndef OMPLAPP_COMMON_SE3_CONTROL_PLANNING_
#define OMPLAPP_COMMON_SE3_CONTROL_PLANNING_

#include "omplapp/graphics/GRigidBodyGeometry.h"
#include <ompl/base/manifolds/SE3StateManifold.h>
#include <ompl/control/manifolds/RealVectorControlManifold.h>
#include <ompl/control/SimpleSetup.h>

namespace ompl
{
    namespace app
    {

        /** \brief Wrapper for ompl::app::RigidBodyPlanning that plans
            for rigid bodies in SE2. */
        class SE3ControlPlanning : public control::SimpleSetup,
                                   public GRigidBodyGeometry
        {
        public:
            
            SE3ControlPlanning(void) : control::SimpleSetup(constructControlManifold()), GRigidBodyGeometry(Motion_3D)
            {
                getControlManifold()->setPropagationFunction(boost::bind(&SE3ControlPlanning::propagateForward, this, _1, _2, _3, _4));
            }
            
            virtual ~SE3ControlPlanning(void)
            {
            }
            
            const base::StateManifoldPtr& getSE3StateManifold(void)
            {
                return getStateManifold()->as<base::CompoundStateManifold>()->getSubManifold(0);
            }
            
            void inferEnvironmentBounds(void);
            
            void inferProblemDefinitionBounds(void);

            int renderPlannerData(void) const;

            virtual void setup(void);
            
        private:
            
            void propagateForward(const base::State *from, const control::Control *ctrl, const double duration, base::State *result)
            {
                // random dynamics; please fill in something reasonable

                const double *c = ctrl->as<control::RealVectorControlManifold::ControlType>()->values;
                
                result->as<base::CompoundState>()->as<base::SE3StateManifold::StateType>(0)->
                    setX(from->as<base::CompoundState>()->as<base::SE3StateManifold::StateType>(0)->getZ() + c[0]);
                
                result->as<base::CompoundState>()->as<base::SE3StateManifold::StateType>(0)->
                    setY(from->as<base::CompoundState>()->as<base::SE3StateManifold::StateType>(0)->getZ() + c[0]);
                
                result->as<base::CompoundState>()->as<base::SE3StateManifold::StateType>(0)->
                    setZ(from->as<base::CompoundState>()->as<base::SE3StateManifold::StateType>(0)->getZ() + c[0]);
                
                result->as<base::CompoundState>()->as<base::SE3StateManifold::StateType>(0)->rotation().setAxisAngle(c[1], c[2], c[3], 0.1);
                result->as<base::CompoundState>()->as<base::RealVectorStateManifold::StateType>(1)->values[0] = c[0];
            }
            
            static control::ControlManifoldPtr constructControlManifold(void)
            {
                return control::ControlManifoldPtr(new control::RealVectorControlManifold(constructStateManifold(), 4));
            }
            
            static base::StateManifoldPtr constructStateManifold(void)
            {
                return base::StateManifoldPtr(new base::SE3StateManifold()) + base::StateManifoldPtr(new base::RealVectorStateManifold(1));
            }
            
        };

    }
}

#endif
