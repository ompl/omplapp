#!/bin/env python

from os.path import basename, abspath, dirname
import sys
try:
	from ompl import base as ob
	from ompl import geometric as og
except:
	sys.path.insert(0, dirname(dirname(abspath(__file__))))
	from ompl import base as ob
	from ompl import geometric as og
	
def isStateValid(spaceInformation, state):
	return state.getX() < .6
	
def plan():
	manifold = ob.SE2StateManifold()
	
	bounds = ob.RealVectorBounds(2)
	bounds.setLow(-1)
	bounds.setHigh(1)
	
	manifold.setBounds(bounds)
	
	ss = og.SimpleSetup(manifold)
	
	ss.setStateValidityChecker(isStateValid)
	
	start = ob.SE2State(manifold)
	start.random()
	start().setX(.5)
	
	goal = ob.SE2State(manifold)
	goal.random()
	goal().setY(-.5)
	
	ss.setStartAndGoalStates(ob.State(start), ob.State(goal))
	
	solved = ss.solve(1.0)
	
	if solved:
		ss.simplifySolution()
		print ss.getSolutionPath()


if __name__ == "__main__":
	plan()
