#!/bin/env python

from ompl import base as ob
from ompl import control as oc

def isStateValid(spaceInformation, state):
	return True #spaceInformation.satiesfiesBounds(state)

def propagate(start, control, duration, state):
	state = start
	return oc.PROPAGATION_START_UNKNOWN
	
def plan():
	manifold = ob.SE2StateManifold()
	
	bounds = ob.RealVectorBounds(2)
	bounds.setLow(-1)
	bounds.setHigh(1)
	
	manifold.setBounds(bounds)
	
	cmanifold = oc.RealVectorControlManifold(manifold, 2)
	
	cbounds = oc.RealVectorBounds(2)
	cbounds.setLow(-.3)
	cbounds.setHigh(.3)
	
	cmanifold.setBounds(cbounds)
	
	cmanifold.setPropagationFunction(propagate)
	
	ss = oc.SimpleSetup(cmanifold)
	ss.setStateValidityChecker(isStateValid)
	
	start = ob.State(manifold)
	mapper = ob.SE2StateManifold.Mapper(start)
	mapper.setX(-0.5);
	mapper.setY(0.0);
	mapper.setYaw(0.0);
	
	goal = ob.State(start);
	mapper.use(goal.reference());
	mapper.setX(0.5);
	
	ss.setStartAndGoalStates(start, goal, 0.05)
	
	solved = ss.solve(10.0)
	
	if solved:
		solution = ss.getSolutionPath()
		path = solution.asGeometric()
		print "Found solution:", path
	
if __name__ == "__main__":
	plan()
