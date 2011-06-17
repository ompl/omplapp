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

#ifndef OMPLAPP_G_BLIMP_PLANNING_
#define OMPLAPP_G_BLIMP_PLANNING_

#include "omplapp/apps/BlimpPlanning.h"
#include "omplapp/graphics/RenderGeometry.h"

namespace ompl
{
    namespace app
    {

        class GBlimpPlanning : public BlimpPlanning,
                               public RenderGeometry
        {
        public:

            GBlimpPlanning(void) : BlimpPlanning(),
                                   RenderGeometry(*dynamic_cast<const RigidBodyGeometry*>(this), getGeometricStateExtractor())
            {
            }

            virtual ~GBlimpPlanning(void)
            {
            }
        };
    }
}

#endif
