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

#ifndef OMPLAPP_APP_BASE_
#define OMPLAPP_APP_BASE_

#include "omplapp/geometry/RigidBodyGeometry.h"
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/SimpleSetup.h>
#include "omplapp/apps/detail/appUtil.h"

namespace ompl
{
    namespace app
    {
        
        enum AppType
            { GEOMETRIC, CONTROL };
        
        template<AppType T>
        struct AppTypeSelector
        {
            typedef geometric::SimpleSetup SimpleSetup;
            typedef base::StateManifoldPtr ManifoldType;
        };
        
        template<>
        struct AppTypeSelector<CONTROL>
        {
            typedef control::SimpleSetup        SimpleSetup;
            typedef control::ControlManifoldPtr ManifoldType;            
        };
        
        template<AppType T>
        class AppBase : public AppTypeSelector<T>::SimpleSetup,
                        public RigidBodyGeometry
        {
        public:
            
            AppBase(const typename AppTypeSelector<T>::ManifoldType &manifold, MotionModel model) : AppTypeSelector<T>::SimpleSetup(manifold), RigidBodyGeometry(model)
            {
            }

            virtual ~AppBase(void)
            {
            }

            virtual bool isSelfCollisionEnabled(void) const = 0;
            
            virtual base::ScopedState<> getDefaultStartState(void) const = 0;
            
            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const = 0;
            
            virtual const base::StateManifoldPtr& getGeometricComponentStateManifold(void) const = 0;
            
            virtual const base::State* getGeometricComponentState(const base::State* state, unsigned int index) const = 0;

            virtual unsigned int getRobotCount(void) const = 0;
            
            void inferEnvironmentBounds(void)
            {
                InferEnvironmentBounds(getGeometricComponentStateManifold(), *static_cast<RigidBodyGeometry*>(this));
            }
            
            void inferProblemDefinitionBounds(void)
            {    
                InferProblemDefinitionBounds(AppTypeSelector<T>::SimpleSetup::getProblemDefinition(),
                                             boost::bind(&AppBase::getGeometricComponentState, this, _1, _2), factor_, add_,
                                             getRobotCount(), getGeometricComponentStateManifold(), mtype_);
            }
            
            virtual void setup(void)
            {
                inferEnvironmentBounds();
                inferProblemDefinitionBounds();
                
                const base::StateValidityCheckerPtr &svc = allocStateValidityChecker(AppTypeSelector<T>::SimpleSetup::si_, 
                                                                                     boost::bind(&AppBase::getGeometricComponentState, this, _1, _2), 
                                                                                     isSelfCollisionEnabled());
                if (AppTypeSelector<T>::SimpleSetup::si_->getStateValidityChecker() != svc)
                    AppTypeSelector<T>::SimpleSetup::si_->setStateValidityChecker(svc);
                
                if (AppTypeSelector<T>::SimpleSetup::getProblemDefinition()->getStartStateCount() == 0)
                {
                    AppTypeSelector<T>::SimpleSetup::msg_.inform("Adding default start state");
                    AppTypeSelector<T>::SimpleSetup::addStartState(getDefaultStartState());
                }
                
                AppTypeSelector<T>::SimpleSetup::setup();
            }
        };

    }
}

#endif
