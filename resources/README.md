# OMPL.app Resources

The `resources` directory contains a number of resources for the OMPL.app GUI and for benchmarking, as well as a few textures for one of the OMPL.app demo programs. The 2D and 3D directories contain a number of robot and environment models that can be used as input for ompl_app.py or to benchmark motion planning algorithms. Below are short descriptions of each benchmark. For some problems there are SketchUp (.skp) files available, so that it is easy to modify them in [SketchUp](http://www.sketchup.com), a free 3D modeling program. The SketchUp files cannot be read directly by OMPL, but need to be exported as COLLADA files. From each SketchUp file, two COLLADA files are exported: <problem>_env.dae and <problem>_robot.dae; these files represent the environment and robot geometries. The *.cfg files specify a motion planning problem and can be loaded with the GUI. The *.path files contain sample solutions for the corresponding .cfg file.


# 2D

- The **BoundingBox**, **BugTrap**, **H**, **Maze**, **Passage**, and **RandomPolygons** environment models as well as the **car1**, **car2**, **hook**, and **StraightC** robot models were originally created by Erion Plaku, for use with the OOPSMP library, and ported to OMPL by Amit Bhatia and Mark Moll. The scales of the robots and the environments match, so different combinations are possible.
- The **UniqueSolutionMaze** problem was created by Marius Șucan.
- The **Barriers_easy** problem was created by Ryan Luna. The environment contains many different homotopy classes of solutions paths, which makes it a good test case for optimizing planners.


# 3D

- **Abstract.** Another example of an environment where there exist several homotopy classes of solution paths. Contributed by Andrew Dobson at Rutgers University.
- **Alpha puzzle.** Provided by Boris Yamrom, GE Corporate Research & Development Center (see the [Parasol Lab, Texas A&M University][parasol]). The alpha puzzle benchmark is a motion planning problem containing a narrow passage. The puzzle consists of two tubes, each twisted into an alpha shape; one tube is the obstacle and the other the moving object (robot). The objective is to separate the intertwined tubes. There are three variants of this benchmark. The 1.0 is the original problem and is the hardest to solve. In the 1.2 and 1.5 versions one of the alpha shapes has been stretched along one axis to make the narrow passage 20% and 50% wider, respectively.
- **Apartment.** This is a real piano mover's problem. Try to move the piano to the hallway near the door entrance. The models were obtained from Google 3D warehouse. They contain very detailed meshes, so planners that rely heavily on collision checking do not perform as well.
- **Bug trap.** Created by the [Parasol Lab, Texas A&M University][parasol]. The objective is to take the `bug' robot (a rod) outside of the trap obstacle through the narrow passage.
- **Cubicles.** A simple office-like environment. Depending on the start and goal state, the robot may have to fly through the basement. Created by Nick Bridle and Nicolas Feltman at Rice University.
- **Escape.** There are two versions of this problem. Both involve a long twisty narrow passage between start and goal. The **Pipedream.** and **Spirelli** benchmarks are along the same lines of the **Escape** benchmarks. All were created by Bryant Gipson.
- **Home.** A version of the piano mover's problem with several homotopy classes.
  Most planners will find the long path, but not the one going through the
  narrow passages. Contributed by Andrew Dobson at Rutgers University.
- **Twistycool.** The standard narrow passage problem. Created by Nick Bridle and Nicolas Feltman at Rice University. The **Easy** problem is the same as Twistycool, except that the narrow passage has been widened. This makes it very easy to solve.


# Textures

The files in the `textures` directory are copied from the drawstuff library, included in the [Open Dynamics Engine](http://sourceforge.net/projects/opende/). They are used in the Open Dynamics Engine demo program.

[parasol]: http://parasol.tamu.edu/groups/amatogroup/benchmarks/mp/