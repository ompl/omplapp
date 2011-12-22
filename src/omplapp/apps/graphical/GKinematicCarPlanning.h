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

#ifndef OMPLAPP_G_KINEMATIC_CAR_PLANNING_
#define OMPLAPP_G_KINEMATIC_CAR_PLANNING_

#include "omplapp/apps/KinematicCarPlanning.h"
#include "omplapp/graphics/RenderGeometry.h"

namespace ompl
{
    namespace app
    {

        class GKinematicCarPlanning : public KinematicCarPlanning,
                                    public RenderGeometry
        {
        public:

            GKinematicCarPlanning(void) : KinematicCarPlanning(),
                                        RenderGeometry(*dynamic_cast<const RigidBodyGeometry*>(this), getGeometricStateExtractor())
            {
            }

            virtual ~GKinematicCarPlanning(void)
            {
            }
        };
    }
}

#endif
