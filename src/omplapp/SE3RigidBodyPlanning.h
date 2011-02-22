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

#ifndef OMPLAPP_COMMON_SE3_RIGID_BODY_PLANNING_
#define OMPLAPP_COMMON_SE3_RIGID_BODY_PLANNING_

#include "omplapp/graphics/GRigidBodyGeometry.h"
#include <ompl/base/manifolds/SE3StateManifold.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ompl
{
    namespace app
    {

        /** \brief Wrapper for ompl::app::RigidBodyPlanning that plans
            for rigid bodies in SE2. */
        class SE3RigidBodyPlanning : public geometric::SimpleSetup,
				     public GRigidBodyGeometry
        {
        public:

            SE3RigidBodyPlanning(void) : geometric::SimpleSetup(base::StateManifoldPtr(new base::SE3StateManifold())), GRigidBodyGeometry(Motion_3D)
					 
            {
            }
	    
            virtual ~SE3RigidBodyPlanning(void)
            {
            }
	
	    void inferEnvironmentBounds(void);
	    void inferProblemDefinitionBounds(void);
	    
            virtual void setup(void);

        };

    }
}

#endif
