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

#ifndef OMPLAPP_G_SE3_RIGID_BODY_PLANNING_
#define OMPLAPP_G_SE3_RIGID_BODY_PLANNING_

#include "omplapp/apps/SE3RigidBodyPlanning.h"
#include "omplapp/graphics/RenderGeometry.h"

namespace ompl
{
    namespace app
    {

        /** \brief Wrapper for ompl::app::RigidBodyPlanning that plans
            for rigid bodies in SE2. */
        class GSE3RigidBodyPlanning : public SE3RigidBodyPlanning,
                                      public RenderGeometry
        {
        public:

            GSE3RigidBodyPlanning(const GeometricStateExtractor &gse) : SE3RigidBodyPlanning(),
                                                                        RenderGeometry(*dynamic_cast<const RigidBodyGeometry*>(this), gse)
            {
            }

            virtual ~GSE3RigidBodyPlanning(void)
            {
            }
        };
        
    }
}

#endif
