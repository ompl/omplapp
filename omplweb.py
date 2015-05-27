import json

from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from ompl import app as oa

import flask
from flask import request
app = flask.Flask(__name__)
app.config.from_object(__name__)


########## OMPL Code ##########


class Problem(object):

    def __init__(self):
        # planning problem #

        # start problem info stuff #    
        self.name = "apartment_piano"
        self.location = "default_location" #TODO: not accurate, needs to be problem location

        self.date_modified = "2/15/2015"

        # start filename stuff #
        self.env_path = "../../../robots/3D/Apartment_env.dae"
        self.robot_path = "../../../robots/3D/Apartment_robot.dae"

        # start problem specific stuff #
        self.start_x = 241.81
        self.start_y = 106.15
        self.start_z = 36.46
        self.start_theta = 3.12413936107
        self.start_axis_x = 0.0
        self.start_axis_y = 0.0
        self.start_axis_z = -1.0
        self.goal_x = -31.19
        self.goal_y = -99.85
        self.goal_z = 36.46
        self.goal_theta = 3.12413936107
        self.goal_axis_x = 0.0
        self.goal_axis_y = 0.0
        self.goal_axis_z = -1.0
        self.bounds_min_x = -73.76
        self.bounds_min_y = -179.59
        self.bounds_min_z = -0.03
        self.bounds_max_x = 295.77
        self.bounds_max_y = 168.26
        self.bounds_max_z = 90.39
        
        # benchmark specific stuff #
        self.time_limit = 5.0
        self.mem_limit = 10000.0
        self.run_count = 1
        
        # planners for benchmarks #
        self.planners = 'rrt'



def parse(data):
    settings = json.loads(data)
    
    problem = Problem()
    problem.name = settings['name']
    problem.location = settings['location']
    problem.date_modified = settings['date_modified']
    
    # start filename stuff #
    problem.env_path = settings['env_path']
    problem.robot_path = settings['robot_path']
    
    # start problem specific stuff #
    problem.start_x = settings['start_x']
    problem.start_y = settings['start_y']
    problem.start_z = settings['start_z']
    problem.start_theta = settings['start_theta']
    problem.start_axis_x = settings['start_axis_x']
    problem.start_axis_y = settings['start_axis_y']
    problem.start_axis_z = settings['start_axis_z']
    problem.goal_x = settings['goal_x']
    problem.goal_y = settings['goal_y']
    problem.goal_z = settings['goal_z']
    problem.goal_theta = settings['goal_theta']
    problem.goal_axis_x = settings['goal_axis_x']
    problem.goal_axis_y = settings['goal_axis_y']
    problem.goal_axis_z = settings['goal_axis_z']
    problem.bounds_min_x = settings['bounds_min_x']
    problem.bounds_min_y = settings['bounds_min_y']
    problem.bounds_min_z = settings['bounds_min_z']
    problem.bounds_max_x = settings['bounds_max_x']
    problem.bounds_max_y = settings['bounds_max_y']
    problem.bounds_max_z = settings['bounds_max_z']
    
    # benchmark specific stuff #
    problem.time_limit = settings['time_limit']
    problem.mem_limit = settings['mem_limit']
    problem.run_count = settings['run_count']
    
    # planners for benchmarks #
    problem.planners = settings['planners']
 
    return problem







def convert_path_to_json(path, solved=True):
    """
    Given a list of list states from ompl, generates a json string with the
    following form. Will set status to didSolve.

    status: true
    path: [ [x,y,z,q1,q2,q3,q4], ... ]

    status: true
    0: ( (x,y,z), (angle,angle,angle,angle) ) //this is a quaternion
    1: ( (x,y,z), (angle,angle,angle,angle) )
    ...

    """

    solved_json = 'true' if solved else 'false'

    # first, create an ordered dictionary (if possible)

    # TODO: actually implement this

    dictionary = {}
    dictionary['solved'] = solved_json
    dictionary['raw_path'] = str(path)

    dictionary['path'] = path

    #TODO: do string conversion here if possible - right now expects string path

    #print 'warning: path is not a GeometricPath, only sending raw string'
    #    print pathlist[0]

    return json.dumps(dictionary)

def solve(problem):
    """
    Given a Django ProblemModel,loads the problem into OMPL, solves it, unloads.
    
    Returns a tuple (bool didSolve, string path).

    NOTE: not very efficient, but I do not want any statefulness on server side
    """

    # load problem information from Django.
    #problem = Problem.objects.get(pk=problem_id)
    if problem == None:
        print 'ompl.solve: problem failed to load'
        return (False, '')

    # initialize ompl problem with the values in problem
    # NOTE: just do super easy rigid body in 3D, SE(3) for now
    print 'ompl: setting up problem'

    #space = ob.SE3StateSpace()
    #omplSetup = og.SimpleSetup(space)
    omplSetup = oa.SE3RigidBodyPlanning()

    #TODO: set validity checker? how did OMPLApp handle this?
    print 'Is self collision enabled? %s' % omplSetup.isSelfCollisionEnabled()


    #TODO: some debug prints to check that the paths are getting set, remove later 
    print problem.env_path,problem
    
    # load and set meshes
    omplSetup.setEnvironmentMesh( str(problem.env_path) )
    omplSetup.setRobotMesh( str(problem.robot_path) )
    
    # set start and goal
    start = ob.State(omplSetup.getGeometricComponentStateSpace())
    start().setX( float(problem.start_x) )
    start().setY( float(problem.start_y) )
    start().setZ( float(problem.start_z) )

    start().rotation().setAxisAngle( float(problem.start_axis_x), float(problem.start_axis_y), float(problem.start_axis_z), float(problem.start_theta) )

    goal = ob.State(omplSetup.getGeometricComponentStateSpace())
    goal().setX( float(problem.goal_x) )
    goal().setY( float(problem.goal_y) )
    goal().setZ( float(problem.goal_z) )  

    # use Axis-Angle description to set axis and angle
    goal().rotation().setAxisAngle( float(problem.goal_axis_x), float(problem.goal_axis_y), float(problem.goal_axis_z), float(problem.goal_theta) )    

    omplSetup.setStartAndGoalStates(start,goal)

    # set bounds
    bounds = ob.RealVectorBounds(3)
    bounds.low[0] = float(problem.bounds_min_x)
    bounds.low[1] = float(problem.bounds_min_y)
    bounds.low[2] = float(problem.bounds_min_z)
    bounds.high[0] = float(problem.bounds_max_x)
    bounds.high[1] = float(problem.bounds_max_y)
    bounds.high[2] = float(problem.bounds_max_z)
    omplSetup.getGeometricComponentStateSpace().setBounds(bounds)


    # grab planner
    #TODO: grab this from the user's preference, but for now use the first planner in problem's list
    #TODO: um get a planner and use it instead of the default!


    # finally, solve!
    didSolve = omplSetup.solve( float(problem.time_limit) )
    path_string = ''

    statesList = []
    if didSolve:

        path = omplSetup.getSolutionPath()
        didSolve = path.check()
        if didSolve == False:
            print("Path reported by planner seems to be invalid!")
        else:
            print "in ompl: solve seems successful"

            #path_string = str(path)

            # convert to json string
            ns = path.getStateCount()
            pathlist = [ omplSetup.getGeometricComponentState( ob.State(omplSetup.getGeometricComponentStateSpace(),path.getState(i)), 0) for i in range(ns) ]
                
            #BIG TODO: PROPERLY access states from ompl. 
            # For states that don't let us access them like this WILL segfault
            for state in pathlist:
                stateList = [str(state[0]),
                             str(state[1]),
                             str(state[2]),
                             str(state[3]),
                             str(state[4]),
                             str(state[5]),
                             str(state[6])]
                statesList.append(stateList)

            print 'successful state to path (despite segfault risk here)'

    path_string = convert_path_to_json(statesList,didSolve)
    # return "Test"
    return path_string


    # TODO: return path properly, right now just return it all and the status as a string





########## Flask Code ##########

@app.route("/")
def index():
	return flask.render_template("index.html")

@app.route("/<name>")
def hello(name):
	return flask.render_template("name.html", name=name)

@app.route("/webgl")
def viewWebGl():
	return flask.render_template("viewWebGL.html")

@app.route("/simple")
def simpleGL():
	return flask.render_template("simpleGL.html")

@app.route("/<usrname>/<int:uid>")
def printName(usrname, uid):
	return "Name: %s, ID: %d" % (usrname, uid)

@app.route("/omplapp", methods=['GET'])
def omplapp():
	return flask.render_template("omplweb.html")    
    
@app.route("/omplapp/solve", methods=['POST'])
def solveProblem():
	# jsondata = request.get_json(False, False, True)
	return solve(parse(request.form.get('settings', str)))
    # return "Received data."

if __name__ == "__main__":
	app.debug = True
	app.run()



