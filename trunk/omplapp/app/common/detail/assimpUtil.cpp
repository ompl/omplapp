#include "common/detail/assimpUtil.h"
#include <ompl/util/Console.h>
#include <limits>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

void ompl::app::scene::inferBounds(base::RealVectorBounds &bounds, const std::vector<aiVector3D> &vertices, double multiply, double add)
{
    double minX = std::numeric_limits<double>::infinity();
    double minY = minX;
    double minZ = minX;
    double maxX = -minX;
    double maxY = maxX;
    double maxZ = maxX;
    for (unsigned int i = 0 ; i < vertices.size() ; ++i)
    {
	if (minX > vertices[i].x) minX = vertices[i].x;
	if (maxX < vertices[i].x) maxX = vertices[i].x;
	if (minY > vertices[i].y) minY = vertices[i].y;
	if (maxY < vertices[i].y) maxY = vertices[i].y;
	if (minZ > vertices[i].z) minZ = vertices[i].z;
	if (maxZ < vertices[i].z) maxZ = vertices[i].z;
    }
    
    multiply -= 1.0;
    if (multiply < 0.0)
    {
	multiply = 0.0;
	msg::Interface msg;
	msg.warn("The multiplicative factor in the bounds computation process should be larger than 1.0");
    }
    if (add < 0.0)
    {
	add = 0.0;
	msg::Interface msg;
	msg.warn("The additive factor in the bounds computation process should be larger than 0.0");
    }
    
    double dx = (maxX - minX) * multiply + add;
    double dy = (maxY - minY) * multiply + add;
    double dz = (maxZ - minZ) * multiply + add;
    bounds.low[0] = minX - dx; bounds.low[1] = minY - dy; bounds.low[2] = minZ - dz;
    bounds.high[0] = maxX + dx; bounds.high[1] = maxY + dy; bounds.high[2] = maxZ + dz;	    
}


namespace ompl
{
    namespace app
    {
	
	namespace scene
	{
	    
	    void extractVerticesAux(const aiScene *scene, const aiNode *node, aiMatrix4x4 transform,
				    std::vector<aiVector3D> &vertices)
	    {
		transform *= node->mTransformation;
		for (unsigned int i = 0 ; i < node->mNumMeshes; ++i)
		{
		    const aiMesh* a = scene->mMeshes[node->mMeshes[i]];
		    for (unsigned int i = 0 ; i < a->mNumVertices ; ++i)
			vertices.push_back(transform * a->mVertices[i]);
		}
		
		for (unsigned int n = 0; n < node->mNumChildren; ++n)
		    extractVerticesAux(scene, node->mChildren[n], transform, vertices);
	    }
	    
	    void extractTrianglesAux(const aiScene *scene, const aiNode *node, aiMatrix4x4 transform,
				     std::vector<aiVector3D> &triangles)
	    {
		transform *= node->mTransformation;
		for (unsigned int i = 0 ; i < node->mNumMeshes; ++i)
		{
		    const aiMesh* a = scene->mMeshes[node->mMeshes[i]];
		    for (unsigned int i = 0 ; i < a->mNumFaces ; ++i)
			if (a->mFaces[i].mNumIndices == 3)
			{
			    triangles.push_back(transform * a->mVertices[a->mFaces[i].mIndices[0]]);
			    triangles.push_back(transform * a->mVertices[a->mFaces[i].mIndices[1]]);
			    triangles.push_back(transform * a->mVertices[a->mFaces[i].mIndices[2]]);
			}
		}
		
		for (unsigned int n = 0; n < node->mNumChildren; ++n)
		    extractTrianglesAux(scene, node->mChildren[n], transform, triangles);
	    }
	}
    }
}


void ompl::app::scene::extractVertices(const aiScene *scene, std::vector<aiVector3D> &vertices)
{
    vertices.clear();
    if (scene && scene->HasMeshes())
	extractVerticesAux(scene, scene->mRootNode, aiMatrix4x4(), vertices);
}

void ompl::app::scene::sceneCenter(const aiScene *scene, aiVector3D &center)
{
    std::vector<aiVector3D> vertices;
    extractVertices(scene, vertices);
    center.Set(0, 0, 0);
    for (unsigned int i = 0 ; i < vertices.size() ; ++i)
	center += vertices[i];
    center /= (float)vertices.size();
}

void ompl::app::scene::extractTriangles(const aiScene *scene, std::vector<aiVector3D> &triangles)
{
    triangles.clear();
    if (scene && scene->HasMeshes())
	extractTrianglesAux(scene, scene->mRootNode, aiMatrix4x4(), triangles);
}

double ompl::app::scene::shortestEdge(const aiScene *scene)
{
    std::vector<aiVector3D> triangles;
    extractTriangles(scene, triangles);
    double s = std::numeric_limits<double>::infinity();
    for (unsigned int i = 0 ; i < triangles.size() / 3 ; ++i)
    {
	double d = (triangles[3 * i] - triangles[3 * i + 1]).Length();
	if (d < s) s = d;
	d = (triangles[3 * i] - triangles[3 * i + 2]).Length();
	if (d < s) s = d;
	d = (triangles[3 * i + 1] - triangles[3 * i + 2]).Length();
	if (d < s) s = d;
    }
    return s;
}


// Most of the code below is taken from external/assimp/samples/SimpleOpenGL/Sample_SimpleOpenGL.c
namespace ompl
{
    namespace app
    {
	
	namespace scene
	{
	    
	    void color4_to_float4(const aiColor4D *c, float f[4])
	    {
		f[0] = c->r;
		f[1] = c->g;
		f[2] = c->b;
		f[3] = c->a;
	    }
	    
	    void set_float4(float f[4], float a, float b, float c, float d)
	    {
		f[0] = a;
		f[1] = b;
		f[2] = c;
		f[3] = d;
	    }
	    
	    void apply_material(const aiMaterial *mtl)
	    {
		float c[4];
		GLenum fill_mode;
		int ret1, ret2;
		aiColor4D diffuse;
		aiColor4D specular;
		aiColor4D ambient;
		aiColor4D emission;
		float shininess, strength;
		int two_sided;
		int wireframe;
		unsigned int max;
		
		set_float4(c, 0.8f, 0.8f, 0.8f, 1.0f);
		if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
		    color4_to_float4(&diffuse, c);
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, c);
		
		set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
		if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular))
		    color4_to_float4(&specular, c);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);
		
		set_float4(c, 0.2f, 0.2f, 0.2f, 1.0f);
		if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient))
		    color4_to_float4(&ambient, c);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, c);
		
		set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
		if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission))
		    color4_to_float4(&emission, c);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, c);
		
		max = 1;
		ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
		max = 1;
		ret2 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
		if((ret1 == AI_SUCCESS) && (ret2 == AI_SUCCESS))
		    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess * strength);
		else 
		{
		    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
		    set_float4(c, 0.0f, 0.0f, 0.0f, 0.0f);
		    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);
		}
		
		max = 1;
		if(AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_ENABLE_WIREFRAME, &wireframe, &max))
		    fill_mode = wireframe ? GL_LINE : GL_FILL;
		else
		    fill_mode = GL_FILL;
		glPolygonMode(GL_FRONT_AND_BACK, fill_mode);
		
		max = 1;
		if((AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_TWOSIDED, &two_sided, &max)) && two_sided)
		    glEnable(GL_CULL_FACE);
		else 
		    glDisable(GL_CULL_FACE);
	    }
	    
	    // Can't send color down as a pointer to aiColor4D because AI colors are ABGR.
	    void Color4f(const struct aiColor4D *color)
	    {
		glColor4f(color->r, color->g, color->b, color->a);
	    }
	    
	    void recursive_render(const aiScene *scene, const aiScene *sc, const aiNode* nd)
	    {
		int i;
		unsigned int n = 0, t;
		aiMatrix4x4 m = nd->mTransformation;
		
		// update transform
		aiTransposeMatrix4(&m);
		glPushMatrix();
		glMultMatrixf((float*)&m);
		
		// draw all meshes assigned to this node
		for (; n < nd->mNumMeshes; ++n) {
		    const aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];
		    
		    apply_material(sc->mMaterials[mesh->mMaterialIndex]);
		    if(mesh->mNormals == NULL)
			glDisable(GL_LIGHTING);
		    else
			glEnable(GL_LIGHTING);
		    if(mesh->mColors[0] == NULL)
			glDisable(GL_COLOR_MATERIAL);
		    else
			glEnable(GL_COLOR_MATERIAL);
		    
		    for (t = 0; t < mesh->mNumFaces; ++t)
		    {
			const aiFace* face = &mesh->mFaces[t];
			GLenum face_mode;
			
			switch(face->mNumIndices) 
			{
			case 1: face_mode = GL_POINTS; break;
			case 2: face_mode = GL_LINES; break;
			case 3: face_mode = GL_TRIANGLES; break;
			default: face_mode = GL_POLYGON; break;
			}
			
			glBegin(face_mode);
			for(i = 0; i < (int)face->mNumIndices; i++)
			{
			    int index = face->mIndices[i];
			    if(mesh->mColors[0] != NULL)
				Color4f(&mesh->mColors[0][index]);
			    if(mesh->mNormals != NULL) 
				glNormal3fv(&mesh->mNormals[index].x);
			    glVertex3fv(&mesh->mVertices[index].x);
			}
			glEnd();
		    }
		}
		// draw all children
		for (n = 0; n < nd->mNumChildren; ++n)
		    recursive_render(scene, sc, nd->mChildren[n]);
		glPopMatrix();
	    }
	    
	    int assimpRender(const aiScene* robotScene, const aiScene* envScene, const aiVector3D &robotCenter)
	    {
		int result = glGenLists(2);
		
		// create display list for robot; we undo the translation of the robot
		glNewList(result, GL_COMPILE);
		aiMatrix4x4 t;
		aiMatrix4x4::Translation(-robotCenter, t);
		aiTransposeMatrix4(&t);
		glPushMatrix();
		glMultMatrixf((float*)&t);
		recursive_render(robotScene, robotScene, robotScene->mRootNode);
		glPopMatrix();
		glEndList();
		
		// create display list for environment
		glNewList(result+1, GL_COMPILE);
		recursive_render(envScene, envScene, envScene->mRootNode);
		glEndList();
		
		return result;
	    }
	    
	}
    }
}
