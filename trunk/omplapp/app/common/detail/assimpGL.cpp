#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include "assimp.h"
#include "assimpGL.h"

// Most of the code below is taken from external/assimp/samples/SimpleOpenGL/Sample_SimpleOpenGL.c


namespace ompl
{
	namespace app
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
                    case 1: face_mode = GL_POINTS; printf("point\n"); break;
                    case 2: face_mode = GL_LINES; printf("line\n"); break;
                    case 3: face_mode = GL_TRIANGLES; printf("triangle\n"); break;
                    default: face_mode = GL_POLYGON; printf("polygon\n"); break;
                }

                glBegin(face_mode);
                for(i = 0; i < face->mNumIndices; i++)
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

    int assimpRender(const aiScene* robotScene, const aiScene* envScene)
    {
        int result = glGenLists(2);

        // create display list for robot
        glNewList(result, GL_COMPILE);
        recursive_render(robotScene, robotScene, robotScene->mRootNode);
        glEndList();

        // create display list for environment
        glNewList(result+1, GL_COMPILE);
        recursive_render(envScene, envScene, envScene->mRootNode);
        glEndList();

        return result;
    }

    }
}