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

#include "omplapp/config.h"
#include "omplapp/geometry/GeometrySpecification.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <memory>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <string>
#include <vector>

namespace ompl
{

    /** \brief Namespace containing code specific to OMPL.app */
    namespace app
    {
        /** \brief Enumeration of the possible collision checker types */
        enum CollisionChecker
            { PQP, FCL };

        class RigidBodyGeometry
        {
        public:

            /** \brief Constructor expects a state space that can represent a rigid body */
            /// \param mtype The motion model (2D or 3D) for the rigid body.
            /// \param ctype The type of collision checker to use for rigid body planning.
            explicit
            RigidBodyGeometry(MotionModel mtype, CollisionChecker ctype) : mtype_(mtype), factor_(1.0), add_(0.0), ctype_(ctype)
            {
            }

            /// \brief Constructor expects a state space that can represent a rigid body
            /// \param mtype The motion model (2D or 3D) for the rigid body.
            /// \remarks This constructor defaults to a PQP state validity checker
            explicit
            RigidBodyGeometry(MotionModel mtype) : mtype_(mtype), factor_(1.0), add_(0.0), ctype_(FCL)
            {
            }

            virtual ~RigidBodyGeometry() = default;

            MotionModel getMotionModel() const
            {
                return mtype_;
            }

            CollisionChecker getCollisionCheckerType() const
            {
                return ctype_;
            }

            bool hasEnvironment() const
            {
                return !importerEnv_.empty();
            }

            bool hasRobot() const
            {
                return !importerRobot_.empty();
            }

            unsigned int getLoadedRobotCount() const
            {
                return importerRobot_.size();
            }

            /** \brief Get the robot's center (average of all the vertices of all its parts) */
            aiVector3D getRobotCenter(unsigned int robotIndex) const;

            /** \brief This function specifies the name of the CAD
                file representing the environment (\e
                env). Returns 1 on success, 0 on failure. */
            virtual bool setEnvironmentMesh(const std::string &env);

            /** \brief This function specifies the name of the CAD
                file representing a part of the environment (\e
                env). Returns 1 on success, 0 on failure. */
            virtual bool addEnvironmentMesh(const std::string &env);

             /** \brief This function specifies the name of the CAD
                 file representing the robot (\e robot). Returns 1 on success, 0 on failure. */
            virtual bool setRobotMesh(const std::string &robot);

             /** \brief This function specifies the name of the CAD
                file representing a part of the robot (\e robot). Returns 1 on success, 0 on failure. */
            virtual bool addRobotMesh(const std::string &robot);

            /** \brief Change the type of collision checking for the rigid body */
            virtual void setStateValidityCheckerType (CollisionChecker ctype);

            /** \brief Allocate default state validity checker using FCL. */
            const base::StateValidityCheckerPtr& allocStateValidityChecker(const base::SpaceInformationPtr &si, const GeometricStateExtractor &se, bool selfCollision);

            const GeometrySpecification& getGeometrySpecification() const;

            /** \brief The bounds of the environment are inferred
                based on the axis-aligned bounding box for the objects
                in the environment. The inferred size is multiplied by
                \e factor. By default \e factor = 1, */
            void setBoundsFactor(double factor)
            {
                factor_ = factor;
            }

            /** \brief Get the data set by setBoundsFactor() */
            double getBoundsFactor() const
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
            double getBoundsAddition() const
            {
                return add_;
            }

            /** \brief Given the representation of an environment,
                infer its bounds. The bounds will be 2-dimensional
                when planning in 2D and 3-dimensional when planning in
                3D. */
            base::RealVectorBounds inferEnvironmentBounds() const;

        protected:

            void computeGeometrySpecification();

            MotionModel         mtype_;

            /** \brief The factor to multiply inferred environment bounds by (default 1) */
            double              factor_;

            /** \brief The value to add to inferred environment bounds (default 0) */
            double              add_;

            /** \brief Instance of assimp importer used to load environment */
            std::vector< std::shared_ptr<Assimp::Importer> > importerEnv_;

            /** \brief Instance of assimp importer used to load robot */
            std::vector< std::shared_ptr<Assimp::Importer> > importerRobot_;

            /** \brief Object containing mesh data for robot and environment */
            GeometrySpecification         geom_;

            /** \brief Instance of the state validity checker for collision checking */
            base::StateValidityCheckerPtr validitySvc_;

            /** \brief Value containing the type of collision checking to use */
            CollisionChecker              ctype_;

        };

    }
}

#endif
