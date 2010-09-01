#include <assimp.hpp>
#include <assimp.h>
#include <aiScene.h>
#include <aiPostProcess.h>
#include <ompl/base/manifolds/RealVectorBounds.h>
#include <vector>

namespace ompl
{
    namespace app
    {

	namespace scene
	{
	    
	    void inferBounds(base::RealVectorBounds &bounds, const std::vector<aiVector3D> &vertices, double multiply = 1.1, double add = 0.0);
	    void extractTriangles(const aiScene *scene, std::vector<aiVector3D> &triangles);
	    void extractVertices(const aiScene *scene, std::vector<aiVector3D> &vertices);
	    double shortestEdge(const aiScene *scene);
	    void sceneCenter(const aiScene *scene, aiVector3D &center);
	    int assimpRender(const aiScene* scene);
	    int assimpRender(const aiScene* scene, const aiVector3D &robotCenter);
	}
	
    }
}
