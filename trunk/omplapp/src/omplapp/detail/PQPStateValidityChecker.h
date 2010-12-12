#include <ompl/base/SpaceInformation.h>
#include <ompl/base/manifolds/SO3StateManifold.h>
#include <PQP.h>
#include <aiScene.h>
#include <boost/shared_ptr.hpp>
#include <vector>

namespace ompl
{
    namespace app
    {

        /** \brief Define an ompl::base::StateValidityChecker that can
            construct PQP models internally.  The instance is still
            abstract however, as the isValid() function is not
            implemented (knowledge of the manifold is needed for this
            function to be implemented) */
        class PQPStateValidityChecker : public base::StateValidityChecker
        {
        public:

            PQPStateValidityChecker(const base::SpaceInformationPtr &si) : base::StateValidityChecker(si)
            {
            }

            /** \brief This function @b must be called after construction, to initialize the collision checker. */
            virtual void configure(const aiScene *robot, const aiScene *obstacles, const aiVector3D &robotCenter);

        protected:

            /** \brief Shared pointe wrapper for PQP_Model */
            typedef boost::shared_ptr<PQP_Model> PQPModelPtr;

            /** \brief  Convert a quaternion to a 3x3 rotation matrix; based on code from
                http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm */
            void quaternionToMatrix(const base::SO3StateManifold::StateType &q, PQP_REAL m[3][3]) const;

            /** \brief Convert a mesh to a PQP model */
            PQPModelPtr getPQPModelFromScene(const aiScene *scene) const;

            /** \brief Convert a mesh to a PQP model, but subtract center from it first. */
            PQPModelPtr getPQPModelFromScene(const aiScene *scene, const aiVector3D &center) const;

            /** \brief Convert a set of triangles to a PQP model */
            PQPModelPtr getPQPModelFromTris(const std::vector<aiVector3D> &triangles) const;

            /** \brief Model of the robot */
            PQPModelPtr    robot_;

            /** \brief Model of the environment */
            PQPModelPtr    environment_;

            /** \brief Interfaca used for reporting errors */
            msg::Interface msg_;

        };

    }
}
