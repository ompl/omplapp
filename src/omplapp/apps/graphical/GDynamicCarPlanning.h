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

#ifndef OMPLAPP_G_DYNAMIC_CAR_PLANNING_
#define OMPLAPP_G_DYNAMIC_CAR_PLANNING_

#include "omplapp/apps/DynamicCarPlanning.h"
#include "omplapp/graphics/RenderGeometry.h"

namespace ompl
{
    namespace app
    {

        class GDynamicCarPlanning : public DynamicCarPlanning,
                                    public RenderGeometry
        {
        public:

            GDynamicCarPlanning(void) : DynamicCarPlanning(),
                                        RenderGeometry(*dynamic_cast<const RigidBodyGeometry*>(this), getGeometricStateExtractor())
            {
            }

            virtual ~GDynamicCarPlanning(void)
            {
            }
        };
    }
}

#endif
