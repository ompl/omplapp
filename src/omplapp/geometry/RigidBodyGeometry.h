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

#ifndef OMPLAPP_GEOMETRY_RIGID_BODY_GEOMETRY_
#define OMPLAPP_GEOMETRY_RIGID_BODY_GEOMETRY_

#include "omplapp/geometry/GeometrySpecification.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/manifolds/RealVectorBounds.h>
#include <boost/shared_ptr.hpp>
#include <aiScene.h>
#include <assimp.hpp>
#include <string>
#include <vector>

namespace ompl
{

    /** \brief Namespace containing code specific to OMPL.app */
    namespace app
    {

        class RigidBodyGeometry
        {
        public:

            /** \brief Constructor expects a manifold that can represent a rigid body */
            explicit
            RigidBodyGeometry(MotionModel mtype) : mtype_(mtype), factor_(1.0), add_(0.0), msg_("Geometry")
            {
            }

            virtual ~RigidBodyGeometry(void)
            {
            }

            /** \brief The CAD file for the robot often has a root
                transform. This root transform is undone when the
                robot is loaded, but the value of that transform is
                kept as the robot's starting state. This function
                returns that starting state. \e state is assumed to be
                part of ompl::base::SE2StateManifold if motion is in
                2D, and ompl::base::SE3StateManifold if motion is in
                3D.*/
            void getEnvStartState(base::ScopedState<>& state) const;

            /** \brief This function specifies the name of the CAD
                file representing the environment (\e
                env). Returns 1 on success, 0 on failure. */
            virtual int setEnvironmentMesh(const std::string &env);

            /** \brief This function specifies the name of the CAD
                file representing a part of the environment (\e
                env). Returns 1 on success, 0 on failure. */
            virtual int addEnvironmentMesh(const std::string &env);

             /** \brief This function specifies the name of the CAD
                 file representing the robot (\e robot). Returns 1 on success, 0 on failure. */
            virtual int setRobotMesh(const std::string &robot);

             /** \brief This function specifies the name of the CAD
                file representing a part of the robot (\e robot). Returns 1 on success, 0 on failure. */
            virtual int addRobotMesh(const std::string &robot);

            /** \brief Get the robot's center (average of all the vertices of all its parts) */
            aiVector3D getRobotCenter(void) const;

            /** \brief Allocate default state validity checker using PQP. */
            const base::StateValidityCheckerPtr& allocStateValidityChecker(const base::SpaceInformationPtr &si, const GeometricStateExtractor &se, bool selfCollision);

            /** \brief The bounds of the environment are inferred
                based on the axis-aligned bounding box for the objects
                in the environment. The inferred size is multiplied by
                \e factor. By default \e factor = 1, */
            void setBoundsFactor(double factor)
            {
                factor_ = factor;
            }

            /** \brief Get the data set by setBoundsFactor() */
            double getBoundsFactor(void) const
            {
                return factor_;
            }

            /** \brief The bounds of the environment are inferred
                based on the axis-aligned bounding box for the objects
                in the environment. \e add is added to the inferred
                size. By default \e add = 0, */
            void setBoundsAddition(double add)
            {
                add_ = add;
            }

            /** \brief Get the data set by setBoundsAddition() */
            double getBoundsAddition(void) const
            {
                return add_;
            }

            /** \brief Given the representation of an environment,
                infer its bounds. The bounds will be 2-dimensional
                when planning in 2D and 3-dimensional when planning in
                3D. */
            base::RealVectorBounds inferEnvironmentBounds(void) const;

        protected:

                  MotionModel         mtype_;

            /** \brief The factor to multiply inferred environment bounds by (default 1) */
            double              factor_;

            /** \brief The value to add to inferred environment bounds (default 0) */
            double              add_;

            /** \brief Instance of assimp importer used to load environment */
            std::vector< boost::shared_ptr<Assimp::Importer> > importerEnv_;

            /** \brief Instance of assimp importer used to load robot */
            std::vector< boost::shared_ptr<Assimp::Importer> > importerRobot_;

            base::StateValidityCheckerPtr pqp_svc_;

            msg::Interface                msg_;

        };

    }
}

#endif
