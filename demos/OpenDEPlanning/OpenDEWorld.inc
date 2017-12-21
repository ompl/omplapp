// this code is taken from OpenDE's demos

/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

// This is a demo of the QuickStep and StepFast methods,
// originally by David Whittaker.

#include <ode/ode.h>

#define LENGTH 3.5    // chassis length
#define WIDTH 2.5     // chassis width
#define HEIGHT 1.0    // chassis height
#define RADIUS 0.5    // wheel radius
#define STARTZ 1.0    // starting height of chassis
#define CMASS 1       // chassis mass
#define WMASS 1       // wheel mass
#define COMOFFSET -5  // center of mass offset
#define BOXMASS 1     // wall box mass
#define FMAX 25       // car engine fmax
#define ITERS 20      // number of iterations
#define BOXSIZE 3.0   // size of wall boxes

static dWorldID world;
static dSpaceID space;
static dBodyID body[10000];
static int bodies;
static dJointID joint[100000];
static int joints;
static dGeomID ground;
static dGeomID box[10000];
static int boxes;
static dGeomID sphere[10000];
static int spheres;

static dGeomID movable_box_geom[4];
static dBodyID movable_box_body[4];
static dMass movable_mass[4];

static dGeomID avoid_box_geom;
static dGeomID goal_geom;
static dBodyID goal_body;
static dMass goal_mass;

static double GOAL_X = -25;
static double GOAL_Y = 0;

void makeCar(dReal x, dReal y, int &bodyI, int &jointI, int &boxI, int &sphereI)
{
    int i;
    dMass m;

    // chassis body
    body[bodyI] = dBodyCreate(world);
    dBodySetPosition(body[bodyI], x, y, STARTZ);
    dMassSetBox(&m, 1, LENGTH, WIDTH, HEIGHT);
    dMassAdjust(&m, CMASS / 2.0);
    dBodySetMass(body[bodyI], &m);
    box[boxI] = dCreateBox(space, LENGTH, WIDTH, HEIGHT);
    dGeomSetBody(box[boxI], body[bodyI]);

    // wheel bodies
    for (i = 1; i <= 4; i++)
    {
        body[bodyI + i] = dBodyCreate(world);
        dQuaternion q;
        dQFromAxisAndAngle(q, 1, 0, 0, M_PI * 0.5);
        dBodySetQuaternion(body[bodyI + i], q);
        dMassSetSphere(&m, 1, RADIUS);
        dMassAdjust(&m, WMASS);
        dBodySetMass(body[bodyI + i], &m);
        sphere[sphereI + i - 1] = dCreateSphere(space, RADIUS);
        dGeomSetBody(sphere[sphereI + i - 1], body[bodyI + i]);
    }
    dBodySetPosition(body[bodyI + 1], x + 0.4 * LENGTH - 0.5 * RADIUS, y + WIDTH * 0.5, STARTZ - HEIGHT * 0.5);
    dBodySetPosition(body[bodyI + 2], x + 0.4 * LENGTH - 0.5 * RADIUS, y - WIDTH * 0.5, STARTZ - HEIGHT * 0.5);
    dBodySetPosition(body[bodyI + 3], x - 0.4 * LENGTH + 0.5 * RADIUS, y + WIDTH * 0.5, STARTZ - HEIGHT * 0.5);
    dBodySetPosition(body[bodyI + 4], x - 0.4 * LENGTH + 0.5 * RADIUS, y - WIDTH * 0.5, STARTZ - HEIGHT * 0.5);

    // front and back wheel hinges
    for (i = 0; i < 4; i++)
    {
        joint[jointI + i] = dJointCreateHinge2(world, 0);
        dJointAttach(joint[jointI + i], body[bodyI], body[bodyI + i + 1]);
        const dReal *a = dBodyGetPosition(body[bodyI + i + 1]);
        dJointSetHinge2Anchor(joint[jointI + i], a[0], a[1], a[2]);
        dJointSetHinge2Axis1(joint[jointI + i], 0, 0, (i < 2 ? 1 : -1));
        dJointSetHinge2Axis2(joint[jointI + i], 0, 1, 0);
        dJointSetHinge2Param(joint[jointI + i], dParamSuspensionERP, 0.8);
        dJointSetHinge2Param(joint[jointI + i], dParamSuspensionCFM, 1e-5);
        dJointSetHinge2Param(joint[jointI + i], dParamVel2, 0);
        dJointSetHinge2Param(joint[jointI + i], dParamFMax2, FMAX);
    }

    // center of mass offset body. (hang another copy of the body COMOFFSET units below it by a fixed joint)
    dBodyID b = dBodyCreate(world);
    body[bodyI + 5] = b;
    dBodySetPosition(b, x, y, STARTZ + COMOFFSET);
    dMassSetBox(&m, 1, LENGTH, WIDTH, HEIGHT);
    dMassAdjust(&m, CMASS / 2.0);
    dBodySetMass(b, &m);
    dJointID j = dJointCreateFixed(world, 0);
    dJointAttach(j, body[bodyI], b);
    dJointSetFixed(j);

    bodyI += 6;
    jointI += 4;
    boxI += 1;
    sphereI += 4;
}

void resetSimulation(void)
{
    // destroy world if it exists
    if (bodies)
    {
        dSpaceDestroy(space);
        dWorldDestroy(world);
    }

    // recreate world

    world = dWorldCreate();

    space = dSweepAndPruneSpaceCreate(0, dSAP_AXES_XYZ);

    dWorldSetGravity(world, 0, 0, -1.5);
    dWorldSetCFM(world, 1e-5);
    dWorldSetERP(world, 0.8);
    dWorldSetQuickStepNumIterations(world, ITERS);
    ground = dCreatePlane(space, 0, 0, 1, 0);

    bodies = 0;
    joints = 0;
    boxes = 0;
    spheres = 0;

    makeCar(0, 0, bodies, joints, boxes, spheres);

    movable_box_body[0] = dBodyCreate(world);
    dBodySetPosition(movable_box_body[0], -18, 0, BOXSIZE / 2);
    dMassSetBox(&movable_mass[0], 1, BOXSIZE / 2, BOXSIZE * 4, BOXSIZE);
    dMassAdjust(&movable_mass[0], BOXMASS);
    dBodySetMass(movable_box_body[0], &movable_mass[0]);
    movable_box_geom[0] = dCreateBox(space, BOXSIZE / 2, BOXSIZE * 4, BOXSIZE);
    dGeomSetBody(movable_box_geom[0], movable_box_body[0]);

    movable_box_body[1] = dBodyCreate(world);
    dBodySetPosition(movable_box_body[1], -32, 0, BOXSIZE / 2);
    dMassSetBox(&movable_mass[1], 1, BOXSIZE / 2, BOXSIZE * 4, BOXSIZE);
    dMassAdjust(&movable_mass[1], BOXMASS);
    dBodySetMass(movable_box_body[1], &movable_mass[1]);
    movable_box_geom[1] = dCreateBox(space, BOXSIZE / 2, BOXSIZE * 4, BOXSIZE);
    dGeomSetBody(movable_box_geom[1], movable_box_body[1]);

    movable_box_body[2] = dBodyCreate(world);
    dBodySetPosition(movable_box_body[2], -25, -7, BOXSIZE / 2);
    dMassSetBox(&movable_mass[2], 1, BOXSIZE * 4, BOXSIZE / 2, BOXSIZE);
    dMassAdjust(&movable_mass[2], BOXMASS);
    dBodySetMass(movable_box_body[2], &movable_mass[2]);
    movable_box_geom[2] = dCreateBox(space, BOXSIZE * 4, BOXSIZE / 2, BOXSIZE);
    dGeomSetBody(movable_box_geom[2], movable_box_body[2]);

    movable_box_body[3] = dBodyCreate(world);
    dBodySetPosition(movable_box_body[3], -25, 7, BOXSIZE / 2);
    dMassSetBox(&movable_mass[3], 1, BOXSIZE * 4, BOXSIZE / 2, BOXSIZE);
    dMassAdjust(&movable_mass[3], BOXMASS);
    dBodySetMass(movable_box_body[3], &movable_mass[3]);
    movable_box_geom[3] = dCreateBox(space, BOXSIZE * 4, BOXSIZE / 2, BOXSIZE);
    dGeomSetBody(movable_box_geom[3], movable_box_body[3]);

    avoid_box_geom = dCreateBox(space, BOXSIZE, BOXSIZE * 2, BOXSIZE);
    dGeomSetPosition(avoid_box_geom, -10, 0, BOXSIZE / 2);

    goal_geom = dCreateBox(space, BOXSIZE / 2, BOXSIZE / 2, BOXSIZE / 2);
    goal_body = dBodyCreate(world);
    dMassSetBox(&goal_mass, 1, BOXSIZE / 2, BOXSIZE / 2, BOXSIZE / 2);
    dMassAdjust(&goal_mass, (double)BOXMASS / 16.0);
    dBodySetMass(goal_body, &goal_mass);
    dGeomSetBody(goal_geom, goal_body);
    dBodySetPosition(goal_body, GOAL_X, GOAL_Y, BOXSIZE / 4);
}
