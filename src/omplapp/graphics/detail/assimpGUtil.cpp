/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Mark Moll, Ioan Sucan */

#include "omplapp/graphics/detail/assimpGUtil.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif


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
                    fill_mode = wireframe != 0 ? GL_LINE : GL_FILL;
                else
                    fill_mode = GL_FILL;
                glPolygonMode(GL_FRONT_AND_BACK, fill_mode);

                max = 1;
                if((AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_TWOSIDED, &two_sided, &max)) && (two_sided != 0))
                    glEnable(GL_CULL_FACE);
                else
                    glDisable(GL_CULL_FACE);
            }

            // Can't send color down as a pointer to aiColor4D because AI colors are ABGR.
            void Color4f(const aiColor4D *color)
            {
                glColor4f(color->r, color->g, color->b, color->a);
            }

            void recursive_render(const aiScene *scene, const aiNode* nd)
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

                    apply_material(scene->mMaterials[mesh->mMaterialIndex]);
                    if(mesh->mNormals == nullptr)
                        glDisable(GL_LIGHTING);
                    else
                        glEnable(GL_LIGHTING);
                    if(mesh->mColors[0] == nullptr)
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
                            if(mesh->mColors[0] != nullptr)
                                Color4f(&mesh->mColors[0][index]);
                            if(mesh->mNormals != nullptr)
                                glNormal3fv(&mesh->mNormals[index].x);
                            glVertex3fv(&mesh->mVertices[index].x);
                        }
                        glEnd();
                    }
                }
                // draw all children
                for (n = 0; n < nd->mNumChildren; ++n)
                    recursive_render(scene, nd->mChildren[n]);
                glPopMatrix();
            }

            int assimpRender(const std::vector<const aiScene*> &scenes, const std::vector<aiVector3D> &robotCenter)
            {
                int result = glGenLists(1);

                // create display list for robot; we undo the translation of the robot
                glNewList(result, GL_COMPILE);

                aiMatrix4x4 t;
                for (unsigned int i = 0 ; i < scenes.size() ; ++i)
                {
                    bool tr = robotCenter.size() > i;
                    if (tr)
                    {
                        aiMatrix4x4::Translation(-robotCenter[i], t);
                        aiTransposeMatrix4(&t);
                        glPushMatrix();
                        glMultMatrixf((float*)&t);
                    }

                    recursive_render(scenes[i], scenes[i]->mRootNode);

                    if (tr)
                        glPopMatrix();
                }

                glEndList();

                return result;
            }


            int assimpRender(const aiScene* scene, const aiVector3D &robotCenter)
            {
                std::vector<const aiScene*> scenes(1, scene);
                std::vector<aiVector3D>    centers(1, robotCenter);
                return assimpRender(scenes, centers);
            }

            int assimpRender(const std::vector<const aiScene*> &scenes)
            {
                std::vector<aiVector3D> empty;
                return assimpRender(scenes, empty);
            }

            int assimpRender(const aiScene* scene)
            {
                std::vector<const aiScene*> scenes(1, scene);
                return assimpRender(scenes);
            }

        }
    }
}
