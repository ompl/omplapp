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

#include <ompl/extensions/opende/OpenDEEnvironment.h>

/// @cond IGNORE

class CarEnvironment : public ompl::control::OpenDEEnvironment
{
public:
    CarEnvironment(void) : ompl::control::OpenDEEnvironment()
    {
        setPlanningParameters();
    }

    virtual ~CarEnvironment(void)
    {
    }

    /**************************************************
     * Implementation of functions needed by planning *
     **************************************************/

    virtual unsigned int getControlDimension(void) const
    {
        return 2;
    }

    virtual void getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const
    {
        lower.resize(2);
        lower[0] = -0.3;
        lower[1] = -3.5;

        upper.resize(2);
        upper[0] = 0.3;
        upper[1] = 1.5;
    }

    virtual void applyControl(const double *control) const
    {
        dReal turn = control[0];
        dReal speed = control[1];
        for (int j = 0; j < joints; j++)
        {
            dReal curturn = dJointGetHinge2Angle1(joint[j]);
            dJointSetHinge2Param(joint[j], dParamVel, (turn - curturn) * 1.0);
            dJointSetHinge2Param(joint[j], dParamFMax, dInfinity);
            dJointSetHinge2Param(joint[j], dParamVel2, speed);
            dJointSetHinge2Param(joint[j], dParamFMax2, FMAX);
        }
    }

    virtual bool isValidCollision(dGeomID geom1, dGeomID geom2, const dContact & /*contact*/) const
    {
        if (geom1 == box[0] || geom2 == box[0])
            if (geom1 == avoid_box_geom || geom2 == avoid_box_geom)
                return false;
        return true;
    }

    virtual void setupContact(dGeomID geom1, dGeomID geom2, dContact &contact) const
    {
        contact.surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
        if (dGeomGetClass(geom1) == dSphereClass || dGeomGetClass(geom2) == dSphereClass)
            contact.surface.mu = 20;
        else
            contact.surface.mu = 0.5;
        contact.surface.slip1 = 0.0;
        contact.surface.slip2 = 0.0;
        contact.surface.soft_erp = 0.8;
        contact.surface.soft_cfm = 0.01;
    }

    /**************************************************/

    // Set parameters needed by the base class (such as the bodies
    // that make up to state of the system we are planning for)
    void setPlanningParameters(void)
    {
        world_ = world;
        collisionSpaces_.push_back(space);
        for (int i = 0; i < bodies; ++i)
            stateBodies_.push_back(body[i]);
        stateBodies_.push_back(movable_box_body[0]);
        stateBodies_.push_back(movable_box_body[1]);
        stateBodies_.push_back(movable_box_body[2]);
        stateBodies_.push_back(movable_box_body[3]);
        stateBodies_.push_back(goal_body);
        minControlSteps_ = 10;
        maxControlSteps_ = 50;
        stepSize_ = 0.2;
    }
};

/// @endcond
