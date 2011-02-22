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

#ifndef OMPLAPP_GEOMETRY_G_RIGID_BODY_GEOMETRY_
#define OMPLAPP_GEOMETRY_G_RIGID_BODY_GEOMETRY_

#include "omplapp/geometry/RigidBodyGeometry.h"

namespace ompl
{
    namespace app
    {

        class GRigidBodyGeometry : public RigidBodyGeometry
        {
        public:

            /** \brief Constructor expects a manifold that can represent a rigid body */
	    explicit 
            GRigidBodyGeometry(MotionModel mtype) : RigidBodyGeometry(mtype)
            {
            }

	    virtual ~GRigidBodyGeometry(void)
            {
            }

            /** \brief This function specifies the name of the CAD
                file representing the environment (\e
                env). This environment is also redered
                with OpenGL. The function returns the GL list id of
                the rendered objects (0 if nothing is rendered). */
	    virtual int setEnvironmentMesh(const std::string &env);

            /** \brief This function specifies the name of the CAD
                file representing a part of the environment (\e
                env). This environment is also redered
                with OpenGL. The function returns the GL list id of
                the rendered objects (0 if nothing is rendered). */
	    virtual int addEnvironmentMesh(const std::string &env);

             /** \brief This function specifies the name of the CAD
		 file representing the robot (\e robot). This
		 environment is also redered with OpenGL. The function
		 returns the GL list id of the rendered objects (0 if
		 nothing is rendered). */
	    virtual int setRobotMesh(const std::string &robot);

             /** \brief This function specifies the name of the CAD
                file representing a part of the robot (\e robot). This
                environment is also redered with OpenGL. The function
                returns the GL list id of the rendered objects (0 if
                nothing is rendered). */
	    virtual int addRobotMesh(const std::string &robot);
	    
        };

    }
}

#endif
