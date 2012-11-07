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

#include "omplapp/geometry/detail/assimpUtil.h"
#include <ompl/util/Console.h>
#include <limits>

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
        OMPL_WARN("The multiplicative factor in the bounds computation process should be larger than 1.0");
    }
    if (add < 0.0)
    {
        add = 0.0;
        OMPL_WARN("The additive factor in the bounds computation process should be larger than 0.0");
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
