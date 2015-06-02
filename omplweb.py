import json
import os

from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from ompl import app as oa

import flask
from flask import request
from werkzeug import secure_filename

# Location of user uploaded .dae files
UPLOAD_FOLDER = '/Users/prudhvi/Dropbox/School/Research/KavrakiLab/OMPL/flask/omplweb/uploads'

app = flask.Flask(__name__)
app.config.from_object(__name__)


########## OMPL Code ##########


class Problem(object):
	"""
	Defines the motion planning problem.

	Attributes:
		The class attributes are the configuration options that define the
		motion planing problem; they include general information and the start
		and goal poses of the robot.
	"""

	def __init__(self):
		"""
		Initializes attributes to default values.
		"""

		self.name = ''
		self.location = ''
		self.date_modified = ''
		self.env_path = ''
		self.robot_path = ''
		self.start_x = 0
		self.start_y = 0
		self.start_z = 0
		self.start_theta = 0
		self.start_axis_x = 0
		self.start_axis_y = 0
		self.start_axis_z = 0
		self.goal_x = 0
		self.goal_y = 0
		self.goal_z = 0
		self.goal_theta = 0
		self.goal_axis_x = 0
		self.goal_axis_y = 0
		self.goal_axis_z = 0
		self.bounds_min_x = 0
		self.bounds_min_y = 0
		self.bounds_min_z = 0
		self.bounds_max_x = 0
		self.bounds_max_y = 0
		self.bounds_max_z = 0
		self.time_limit = 0
		self.mem_limit = 0
		self.run_count = 0
		self.planners = ''












		###########################
		##	 Hard coded values	 ##
		###########################


		# self.name = "apartment_piano"
		# self.location = "default_location" #TODO: not accurate, needs to be problem location

		# self.date_modified = "2/15/2015"

		# self.env_path = "../../../robots/3D/Apartment_env.dae"
		# self.robot_path = "../../../robots/3D/Apartment_robot.dae"

		# self.start_x = 241.81
		# self.start_y = 106.15
		# self.start_z = 36.46
		# self.start_theta = 3.12413936107
		# self.start_axis_x = 0.0
		# self.start_axis_y = 0.0
		# self.start_axis_z = -1.0
		# self.goal_x = -31.19
		# self.goal_y = -99.85
		# self.goal_z = 36.46
		# self.goal_theta = 3.12413936107
		# self.goal_axis_x = 0.0
		# self.goal_axis_y = 0.0
		# self.goal_axis_z = -1.0
		# self.bounds_min_x = -73.76
		# self.bounds_min_y = -179.59
		# self.bounds_min_z = -0.03
		# self.bounds_max_x = 295.77
		# self.bounds_max_y = 168.26
		# self.bounds_max_z = 90.39

		# self.time_limit = 5.0
		# self.mem_limit = 10000.0
		# self.run_count = 1

		# self.planners = 'rrt'


def parse(settings, env_path, robot_path):
	"""
	Reads the user submitted configuration data and creates an instance of the
	Problem class to store the data.
	"""

	problem = Problem()
	problem.name = settings['name']
	problem.location = "default_location"
	problem.date_modified = "2/15/2015"

	# start filename stuff #
	problem.env_path = env_path
	problem.robot_path = robot_path
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
	#	 print pathlist[0]

	return json.dumps(dictionary)


def solve(problem):
	"""
	Given an instance of the Problem class, containing the problem configuration
	data, solves the motion planning problem and returns either the solution
	path or a failure message.

	# Old code # NOTE: not very efficient, but I do not want any statefulness on server side
	"""

	# load problem information from Django.
	#problem = Problem.objects.get(pk=problem_id)
	# if problem == None:
		# print 'ompl.solve: problem failed to load'
		# return (False, '')

	# Initialize with the values in problem
	# Old code #  NOTE: just do super easy rigid body in 3D, SE(3) for now
	print 'OMPL:	Setting up problem'

	# space = ob.SE3StateSpace()
	# omplSetup = og.SimpleSetup(space)
	ompl_setup = oa.SE3RigidBodyPlanning()

	#TODO: set validity checker? how did OMPLApp handle this?

	# Set robot and environment meshes
	print "OMPL:	Setting robot mesh with: %s" % str(problem.robot_path)
	ompl_setup.setRobotMesh(str(problem.robot_path))
	print "OMPL:	Setting environment mesh with: %s" % str(problem.env_path)
	ompl_setup.setEnvironmentMesh(str(problem.env_path))

	# Set start pose
	start = ob.State(ompl_setup.getGeometricComponentStateSpace())
	start().setX(float(problem.start_x))
	start().setY(float(problem.start_y))
	start().setZ(float(problem.start_z))

	# Use Axis-Angle description to set axis and angle
	start().rotation().setAxisAngle(
			float(problem.start_axis_x), float(problem.start_axis_y),
			float(problem.start_axis_z), float(problem.start_theta))

	# Set goal pose
	goal = ob.State(ompl_setup.getGeometricComponentStateSpace())
	goal().setX(float(problem.goal_x))
	goal().setY(float(problem.goal_y))
	goal().setZ(float(problem.goal_z))

	# Use Axis-Angle description to set axis and angle
	goal().rotation().setAxisAngle(
			float(problem.goal_axis_x), float(problem.goal_axis_y),
			float(problem.goal_axis_z), float(problem.goal_theta))

	ompl_setup.setStartAndGoalStates(start, goal)

	# Set bounds
	bounds = ob.RealVectorBounds(3)
	bounds.low[0] = float(problem.bounds_min_x)
	bounds.low[1] = float(problem.bounds_min_y)
	bounds.low[2] = float(problem.bounds_min_z)
	bounds.high[0] = float(problem.bounds_max_x)
	bounds.high[1] = float(problem.bounds_max_y)
	bounds.high[2] = float(problem.bounds_max_z)
	ompl_setup.getGeometricComponentStateSpace().setBounds(bounds)


	# grab planner
	#TODO: grab this from the user's preference, but for now use the first planner in problem's list
	#TODO: um get a planner and use it instead of the default!


	# Solve
	did_solve = ompl_setup.solve(float(problem.time_limit))
	path_string = ''

	states_list = []
	if did_solve:
		path = ompl_setup.getSolutionPath()
		did_solve = path.check()
		if did_solve == False:
			print("OMPL: 	Path reported by planner seems to be invalid!")
			return "Solution path was invalid. Try again."
		else:
			print "OMPL: 	Solve seems successful"

			# convert to json string
			num_states = path.getStateCount()
			path_list = [ompl_setup.getGeometricComponentState(ob.State(
				ompl_setup.getGeometricComponentStateSpace(), path.getState(i)), 0)
				for i in range(num_states)]

			#BIG TODO: PROPERLY access states from ompl.
			# For states that don't let us access them like this WILL segfault
			for state in path_list:
				state_list = [
							str(state[0]),
							str(state[1]),
							str(state[2]),
							str(state[3]),
							str(state[4]),
							str(state[5]),
							str(state[6])
							]
				states_list.append(state_list)

			print 'OMPL:	Successful state to path (despite segfault risk here)'

			path_string = convert_path_to_json(states_list,did_solve)

	# return "DEBUGGING"
	return path_string


	# TODO: return path properly, right now just return it all and the status as a string


########## Flask Code ##########


@app.route("/omplapp", methods=['GET'])
def omplapp():
	"""
	Returns the problem configuration page.
	"""

	return flask.render_template("omplweb.html")


def allowed_file(filename):
	"""
	Checks that the parameter is a .dae file.
	"""

	if '.' in filename and filename.rsplit('.', 1)[1] == 'dae':
		# Extract the file extension and return true if dae
		return True

	return False


@app.route('/omplapp/upload', methods=['POST'])
def upload():
	"""
	This function is invoked when the client clicks 'Solve' and submits the
	problem configuration data.

	1.	The robot and environment files are checked for validity and downloaded to
		the server.
	2.	Configuration data is parsed and loaded via the parse function.
	3.	The problem is solved and the output of the solve function (either a
		solution path or an error) is returned.

	"""

	robotFile = request.files['robot']
	envFile = request.files['env']

	if robotFile and envFile:
		if allowed_file(robotFile.filename) and allowed_file(envFile.filename):

			# If valid files, save them to the server
			robot_filename = secure_filename(robotFile.filename)
			robotFile.save(os.path.join(app.config['UPLOAD_FOLDER'], robot_filename))
			robot_path = os.path.join(app.config['UPLOAD_FOLDER'], robot_filename)

			env_filename = secure_filename(envFile.filename)
			envFile.save(os.path.join(app.config['UPLOAD_FOLDER'], env_filename))
			env_path = os.path.join(app.config['UPLOAD_FOLDER'], env_filename)

			print "SERVER: Files saved as: " + robot_path + " and " + env_path

			print "SERVER: Will now parse configuration..."
			#TODO# Put some try/catches here
			problem = parse(request.form, env_path, robot_path)

			print "SERVER: Configuration has been parsed. Solving problem..."
			solution = solve(problem);
			# TODO # Delete the uploaded files?
			return solution;

		else:
			return "Error: Wrong file format. Robot and environment files must be .dae"
	else:
		return "Error: Didn't upload any files! Please choose both a robot and an environment file in the .dae format."

	return "Upload Successful."


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


if __name__ == "__main__":
	app.debug = True
	app.run()



