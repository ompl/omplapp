#!/bin/env python

from ompl import base as ob
from ompl import geometric as og

def isStateValid(spaceInformation, state):
	return True
	
def plan():
	manifold = ob.SE3StateManifold()
	
	bounds = ob.RealVectorBounds(3)
	bounds.setLow(-1)
	bounds.setHigh(1)
	
	manifold.setBounds(bounds)
	
	ss = og.SimpleSetup()
	
	ss.setStateValidityChecker(isStateValid)
	
	start = ob.State(manifold)
	start.random()
	
	goal = ob.State(manifold)
	goal.random()
	
	ss.setStartAndGoalStates(start, goal)
	
	solved = ss.solve(1.0)
	
	if solved:
		ss.simplifySolution()
		print ss.getSolutionPath()


if __name__ == "__main__":
	plan()
