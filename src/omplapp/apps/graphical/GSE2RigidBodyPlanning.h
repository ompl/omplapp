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

#ifndef OMPLAPP_G_SE2_RIGID_BODY_PLANNING_
#define OMPLAPP_G_SE2_RIGID_BODY_PLANNING_

#include "omplapp/apps/SE2RigidBodyPlanning.h"
#include "omplapp/graphics/RenderGeometry.h"

namespace ompl
{
    namespace app
    {

        class GSE2RigidBodyPlanning : public SE2RigidBodyPlanning,
                                      public RenderGeometry
        {
        public:

            GSE2RigidBodyPlanning(void) : SE2RigidBodyPlanning(),
                                          RenderGeometry(*dynamic_cast<const RigidBodyGeometry*>(this), getGeometricStateExtractor())
            {
            }

            virtual ~GSE2RigidBodyPlanning(void)
            {
            }
        };

    }
}

#endif
