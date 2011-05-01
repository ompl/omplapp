This directory contains a number of robot and environment models that
can be used as input for ompl_app.py or to benchmark motion planning
algorithms. Below is a short description of each benchmark. For each
problem there is a SketchUp (.skp) file available, so that it is easy
to modify them in Google SketchUp, a free 3D modeling program. The
SketchUp files cannot be read directly by OMPL, but need to be
exported as COLLADA files. From each SkethcUp file, two COLLADA files
are exported: <problem>_env.dae and <problem>_robot.dae; these files
represent the environment and robot geometries. 

* Cubicles. A simple office-like environment. Depending on the start and goal
  state, the robot may have to fly through the basement. Created by Nick Bridle
  and Nicolas Feltman at Rice University.

* Twistycool. The standard narrow passage problem. Created by Nick Bridle and
  Nicolas Feltman at Rice University.

* Easy. The same problem as Twistycool, except that the narrow passage has been
  widened. This makes it very easy to solve.

* Apartment. This is a real piano mover's problem. Try to move the piano to the
  hallway near the door entrance. The models were obtained from Google 3D
  warehouse. They contain very detailed meshes, so planners that rely heavily
  on collision checking do not perform as well.

* Alpha. Provided by Boris Yamrom, GE Corporate Research & Development Center
  (see http://parasol.tamu.edu/groups/amatogroup/benchmarks/mp/). The alpha
  puzzle benchmark is a motion planning problem containing a narrow passage.
  The puzzle consists of two tubes, each twisted into an alpha shape; one tube
  is the obstacle and the other the moving object (robot). The objective is to
  separate the intertwined tubes.

* Bug trap. Created by Parasol Lab, Texas A&M University (see
  http://parasol.tamu.edu/groups/amatogroup/benchmarks/mp/). The objective is
  to take the `bug' robot (a rod) outside of the trap obstacle through the
  narrow passage.
