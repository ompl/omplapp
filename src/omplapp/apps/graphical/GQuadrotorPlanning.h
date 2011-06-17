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

#ifndef OMPLAPP_G_QUADROTOR_PLANNING_
#define OMPLAPP_G_QUADROTOR_PLANNING_

#include "omplapp/apps/QuadrotorPlanning.h"
#include "omplapp/graphics/RenderGeometry.h"

namespace ompl
{
    namespace app
    {

        class GQuadrotorPlanning : public QuadrotorPlanning,
                                   public RenderGeometry
        {
        public:

            GQuadrotorPlanning(void) : QuadrotorPlanning(),
                                       RenderGeometry(*dynamic_cast<const RigidBodyGeometry*>(this), getGeometricStateExtractor())
            {
            }

            virtual ~GQuadrotorPlanning(void)
            {
            }
        };
    }
}

#endif
