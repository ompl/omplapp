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

        class GSE3RigidBodyPlanning : public SE3RigidBodyPlanning,
                                      public RenderGeometry
        {
        public:

            GSE3RigidBodyPlanning(void) : SE3RigidBodyPlanning(),
                                          RenderGeometry(*dynamic_cast<const RigidBodyGeometry*>(this), getGeometricStateExtractor())
            {
            }

            virtual ~GSE3RigidBodyPlanning(void)
            {
            }
        };

    }
}

#endif
