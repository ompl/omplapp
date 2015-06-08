# OMPL Web
A web front end for sample based motion planning and benchmarking using the Open Motion Planning Library.

[ompl.kavrakilab.org](http://ompl.kavrakilab.org)


## Development Plan

### ~~Phase 1: simplesetup --> master~~
...

~~Preparing to merge branch simplesetup into master.~~
- ~~Front end testing~~
- ~~Pull out styling from html to css~~
- ~~Add ability to pick a problem (including robot, envrionment, and
  configuration) from the server, instead of having to upload your own~~

### Phase 2: master (current)
- Create planners and planner parameters dynamically using existing python
  bindings
- Clean up and optimize existing code
- Create mock ups of a new interface, including visualization and benchmarking
- Implement a front end template of new interface

### Phase 3: branch master --> benchmarking
- Implement server side benchmarking code
- Implement user-facing benchmarking code
- Test benchmarking
- Connect ompl\_web to [Planner Arena](http://www.plannerarena.org)
- Test ompl\_web and Planner Arena interactions

***Merge benchmarking --> master***

### Phase 4: branch master --> visualization
- Implement loading of 3D models
- Implement static visualization of solution paths, where all states of the
  robot within the environment are displayed, without animation
- Test static visualization
- Implement animated visualization
- Test animated visualization

***Merge visualization --> master***
