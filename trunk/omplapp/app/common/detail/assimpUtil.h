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

	void inferBounds(base::RealVectorBounds &bounds, const std::vector<aiVector3D> &vertices, double multiply = 1.1, double add = 0.0);
	void extractTriangles(const aiScene *scene, std::vector<aiVector3D> &triangles);
	void extractVertices(const aiScene *scene, std::vector<aiVector3D> &vertices);
	void extractMeshes(const aiScene *scene, std::vector<aiMesh*> &meshes);
	int assimpRender(const aiScene* envScene, const aiScene* robotScene);
    }
}
