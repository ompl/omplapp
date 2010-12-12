#ifndef OMPLAPP_COMMON_SE2_RIGID_BODY_PLANNING_
#define OMPLAPP_COMMON_SE2_RIGID_BODY_PLANNING_

#include "omplapp/RigidBodyPlanning.h"
#include <ompl/base/manifolds/SE2StateManifold.h>

namespace ompl
{
    namespace app
    {

        /** \brief Wrapper for ompl::app::RigidBodyPlanning that plans
            for rigid bodies in SE2. */
        class SE2RigidBodyPlanning : public RigidBodyPlanning
        {
        public:

            SE2RigidBodyPlanning(void) : RigidBodyPlanning(base::StateManifoldPtr(new base::SE2StateManifold()))
            {
            }

            virtual ~SE2RigidBodyPlanning(void)
            {
            }

        protected:

            virtual void inferEnvironmentBounds(const aiScene *scene);
            virtual void inferProblemDefinitionBounds(void);
            virtual void getRobotCenterAndStartState(const aiScene *robot);

            virtual base::StateValidityCheckerPtr allocStateValidityChecker(const aiScene *env, const aiScene *robot) const;

        };


    }
}

#endif
