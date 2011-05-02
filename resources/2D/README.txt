This directory contains a few environments (and corresponding robots)
that are suitable for planning in SE2. Planning in SE3 is of course
possible, but these environments were designed with SE2 in mind.

* The following environments:

   BoundingBox_planar_env.dae
   BugTrap_planar_env.dae
   H_planar_env.dae
   Maze_planar_env.dae
   Passage_planar_env.dae
   RandomPolygons_planar_env.dae
 
  and following robot descriptions:

   car1_planar_robot.dae
   car2_planar_robot.dae
   hook_planar_robot.dae
   StraightC_planar_robot.dae

 Were originally created by Erion Plaku, for use with the OOPSMP
 library, and ported to OMPL by Amit Bhatia and Mark Moll. The scales
 of the robots & the environments match, so different combinations are
 possible.

* The maze problem (UniqueSolutionMaze_env.dae and
  UniqueSolutionMaze_robot.dae) was created by Marius Șucan.
