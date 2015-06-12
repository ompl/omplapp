import os
import json

import flask
from werkzeug import secure_filename

import ompl
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from ompl import app as oa

# Location of .dae files
UPLOAD_FOLDER = os.path.dirname(os.path.abspath(__file__)) + '/static/uploads'
PROBLEMS_FOLDER = os.path.dirname(os.path.abspath(__file__)) + '/static/problem_files'

app = flask.Flask(__name__)
app.config.from_object(__name__)



########## OMPL Code ##########


class LogOutputHandler(object):
	"""
	Object for handling various levels of logging.
	Logging levels:
		0 = Silent (probablly shouldn't use this)
		1 = Errors only
		2 = Level 0 and warnings
		3 = Levels 1, 2 and debugging
		4 = Levels 1, 2, 3 and info
	"""

	def __init__(self, log_level):
		# Specifies the level of logging should be printed to the console
		self.log_level = log_level

		self.messages = "Messages: \n"

	def error(self, text):
		if self.log_level > 0:
			print "# Error:    " + str(text)

	def warn(self, text):
		if self.log_level > 1:
			print "# Warning:    " + str(text)

	def debug(self, text):
		if self.log_level > 2:
			print "# Debug:    " + str(text)

	def info(self, text):
		# Store the message
		self.messages += str(text)

		if self.log_level > 3:
			print "# Info:    " + str(text)

	def getMessages(self):
		# Info messages are stored and can be retrieved via this function to
		# send to the client
		return self.messages

	def clearMessages(self):
		# Clear the stored messages, this should be called after sending to client
		self.messages = "Messages: \n"

log = LogOutputHandler(3)

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
		Initializes attributes to empty values.
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


def create_planners():
	"""
	Initializes the planner lists and formats each planner's parameters to be
	sent to the client.
	"""
	ompl.initializePlannerLists()
	# Create the geometric planners
	planners = ompl.PlanningAlgorithms(og)
	params_dict = planners.getPlanners()

	retval = "{'param name' : ('display name', 'range type', 'range \
		suggestion', 'default value')}\n"
	retval += "For KPIECE1: \n"
	retval += str(params_dict['ompl.geometric.KPIECE1'])

	return params_dict


def allowed_file(filename):
	"""
	Checks that the parameter is a .dae file.
	"""

	if '.' in filename and filename.rsplit('.', 1)[1] == 'dae':
		# Extract the file extension and return true if dae
		return True

	return False


def create_problem(settings, env_path, robot_path):
	"""
	Reads the configuration data and creates an instance of the
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
	problem.start_x = float(settings['start.x'])
	problem.start_y = float(settings['start.y'])
	problem.start_z = float(settings['start.z'])
	problem.start_theta = float(settings['start.theta'])
	problem.start_axis_x = float(settings['start.axis.x'])
	problem.start_axis_y = float(settings['start.axis.y'])
	problem.start_axis_z = float(settings['start.axis.z'])
	problem.goal_x = float(settings['goal.x'])
	problem.goal_y = float(settings['goal.y'])
	problem.goal_z = float(settings['goal.z'])
	problem.goal_theta = float(settings['goal.theta'])
	problem.goal_axis_x = float(settings['goal.axis.x'])
	problem.goal_axis_y = float(settings['goal.axis.y'])
	problem.goal_axis_z = float(settings['goal.axis.z'])
	problem.bounds_min_x = float(settings['bounds.min.x'])
	problem.bounds_min_y = float(settings['bounds.min.y'])
	problem.bounds_min_z = float(settings['bounds.min.z'])
	problem.bounds_max_x = float(settings['bounds.max.x'])
	problem.bounds_max_y = float(settings['bounds.max.y'])
	problem.bounds_max_z = float(settings['bounds.max.z'])

	# benchmark specific stuff #
	problem.time_limit = float(settings['time_limit'])
	problem.mem_limit = float(settings['mem_limit'])
	problem.run_count = float(settings['run_count'])

	# planners for benchmarks #
	problem.planners = settings['planners']

	return problem


def parse_cfg(cfg_path):
	"""
	Parses the configuration file for pre-defined problems and returns a
	settings dictionary that can be passed to create_problem
	"""
	settings = {}
	cfg = open(cfg_path, 'r')

	for line in cfg:
		if line[0] != '[':
			line = line.replace(" ", "") # Remove excess whitespace
			line = line.replace("\n", "") # Remove newline characters
			items = line.split("=", 1) # Split into [key, value] pairs
			if len(items) == 2:
				# 'bounds' is called 'volume' in the .cfg file, so change it
				if 'volume' in items[0]:
					items[0] = items[0].replace("volume", "bounds")

				# Store (key, value) pair
				settings[items[0]] = items[1]

	return settings


def format_solution(path, solved):
	"""
	Formats the either the solution, or a failure message for delivery to the
	client.
	"""

	solution = {}

	if solved:
		solution['solved'] = 'true'
	else:
		solution['solved'] = 'false'

	# Grab the messages to send to the user
	solution['messages'] = log.getMessages()
	# Clear messages in preparation for next request
	log.clearMessages()

	# Format the path
	path_matrix = path.printAsMatrix().strip().split('\n')

	# A list of n states, where path_list[0] is the start state and path_list[n]
	# is the goal state, and path_list[i] are the intermediary states.
	# Each state is also a list: [x, y, z,
	# path_list = []

	# for line in path_matrix:
		# print "\t " + line
		# path_list.append(line.split(" "))

	# print path_list
	# solution['path'] = path_list;

	solution['path'] = path.printAsMatrix().strip()
	return solution


def solve(problem):
	"""
	Given an instance of the Problem class, containing the problem configuration
	data, solves the motion planning problem and returns either the solution
	path or a failure message.
	"""

	## Configure the problem
	space = ob.SE3StateSpace()

	# Set the dimensions of the bounding box
	bounds = ob.RealVectorBounds(3)

	bounds.low[0] = problem.bounds_min_x
	bounds.low[1] = problem.bounds_min_y
	bounds.low[2] = problem.bounds_min_z

	bounds.high[0] = problem.bounds_max_x
	bounds.high[1] = problem.bounds_max_y
	bounds.high[2] = problem.bounds_max_z

	space.setBounds(bounds)

	ompl_setup = oa.SE3RigidBodyPlanning()

	ompl_setup.setEnvironmentMesh(str(problem.env_path))
	ompl_setup.setRobotMesh(str(problem.robot_path))

	# Set the start and goal states
	start = ob.State(space)
	start().setXYZ(problem.start_x, problem.start_y, problem.start_z)
	start().rotation().setAxisAngle(
			problem.start_axis_x, problem.start_axis_y,
			problem.start_axis_z, problem.start_theta
	)

	goal = ob.State(space)
	goal().setXYZ(problem.goal_x, problem.goal_y, problem.goal_z)
	goal().rotation().setAxisAngle(
			problem.goal_axis_x, problem.goal_axis_y,
			problem.goal_axis_z, problem.goal_theta
	)

	ompl_setup.setStartAndGoalStates(start, goal)


	## Load the planner
	space_info = ompl_setup.getSpaceInformation()

	planner = eval("%s(space_info)" % problem.planners)
	ompl_setup.setPlanner(planner)
	log.info("Using planner: %s\n" % ompl_setup.getPlanner().getName())

	# TODO: This is not that good...find a better way to get planner params
	params = str(planner.params()).split("\n")
	# For each parameter that this planner has
	for param in params:
		param = param.split(" ")[0]
		# See if a value for this param is provided by the client
		if flask.request.form.has_key(param):
			# If value exists, set the param to the value:w
			planner.params().setParam(param, str(flask.request.form[param]))

	## Solve the problem
	solution = {}
	solved = ompl_setup.solve(problem.time_limit)

	log.info(str(ompl_setup))
	log.info("\n\n")

	## Check for validity
	if solved:
		path = ompl_setup.getSolutionPath()
		initialValid = path.check()

		if initialValid:
			log.info("Initial path length: %d\n" % path.length())

			# If if initially valid, attempt to simplify
			ompl_setup.simplifySolution()
			# Get the simplified path
			simple_path = ompl_setup.getSolutionPath()
			simplifyValid = simple_path.check()
			if simplifyValid:
				log.info("Simplified path was found.\n")
				log.info("Simplified path length: %d\n" % path.length())
				solution = format_solution(simple_path, True)
			else:
				log.info("Simplified path was invalid. Returned \
					non-simplified path.\n")
				log.info("Path length: %d\n" % path.length())
				solution = format_solution(path, True)

			# TODO: Interpolation?

		else :
			log.info("Path reported by planner seems to be invalid.\n")
			solution = format_solution(path, False)
	else:
		log.info("No valid path was found with the provided \
			configuration.\n")
		solution = format_solution(None, False)

	solution['name'] = problem.name
	solution['planner'] = ompl_setup.getPlanner().getName()

	return solution


########## Flask Code ##########

@app.route("/")
def index():
	return flask.redirect(flask.url_for('omplapp'))


@app.route("/omplapp", methods=['GET'])
def omplapp():
	"""
	Returns the problem configuration page.
	"""

	return flask.render_template("omplweb.html")


@app.route('/omplapp/upload', methods=['POST'])
def upload():
	"""
	This function is invoked when the client clicks 'Solve' and submits the
	problem configuration data.

	1.	If custom problem, the robot and environment files are checked for
		validity and downloaded to the server.
	2.	Problem configuration data is parsed and loaded.
	3.	The problem is solved and the output of the solve function (either a
		solution path or an error) is returned.
	"""

	problem = None
	robot_location = ""
	env_location = ""

	# Uploaded custom problem
	if (flask.request.form['problems'] == 'custom'):
		robotFile = flask.request.files['robot']
		envFile = flask.request.files['env']

		# Check that the uploaded files are valid
		if robotFile and envFile:
			if allowed_file(robotFile.filename) and allowed_file(envFile.filename):

				# If valid files, save them to the server
				robot_filename = secure_filename(robotFile.filename)
				robotFile.save(os.path.join(app.config['UPLOAD_FOLDER'], \
					robot_filename))
				robot_path = os.path.join(app.config['UPLOAD_FOLDER'], \
					robot_filename)

				env_filename = secure_filename(envFile.filename)
				envFile.save(os.path.join(app.config['UPLOAD_FOLDER'], env_filename))
				env_path = os.path.join(app.config['UPLOAD_FOLDER'], env_filename)

				robot_location = "static/uploads/" + robot_filename
				env_location = "static/uploads/" + env_filename
				log.debug("Files saved as: " + robot_path + " and " + env_path)

				# TODO:Put some try/catches here
				problem = create_problem(flask.request.form, env_path, robot_path)


			else:
				return "Error: Wrong file format. Robot and environment files \
					must be .dae"
		else:
			return "Error: Didn't upload any files! Please choose both a robot \
				and environment file in the .dae format."

	# Selected pre-configured problem from server
	else:
		problem_name = flask.request.form['problems']
		robot_filename = problem_name + "_robot.dae"
		env_filename = problem_name + "_env.dae"
		cfg_filename = problem_name + ".cfg"

		robot_path = os.path.join(app.config['PROBLEMS_FOLDER'], robot_filename)
		env_path = os.path.join(app.config['PROBLEMS_FOLDER'], env_filename)
		cfg_path = os.path.join(app.config['PROBLEMS_FOLDER'], cfg_filename)

		robot_location = "static/problem_files/" + robot_filename
		env_location = "static/problem_files/" + env_filename
		log.debug("Problem location: " + robot_path + " and " + env_path)

		log.debug("Will now parse configuration...")
		# TODO:Put some try/catches here
		settings = parse_cfg(cfg_path)
		settings['name'] = problem_name
		settings['planners'] = flask.request.form['planners']

		log.debug("Configuration has been parsed, creating problem.")
		problem = create_problem(settings, env_path, robot_path)

	log.debug("Solving problem...")
	solution = solve(problem)
	solution['robot_location'] = robot_location
	solution['env_location'] = env_location

	log.debug("Problem solved")
	return json.dumps(solution)

@app.route('/omplapp/planners')
def planners():
	planners = create_planners()
	return json.dumps(planners)

@app.route('/omplapp/components/configuration')
def load_configuration():
	return flask.render_template("components/configuration.html")

@app.route('/omplapp/components/visualization')
def load_visualization():
	return flask.render_template("components/visualization.html")

@app.route('/omplapp/components/benchmarking')
def load_benchmarking():
	return flask.render_template("components/benchmarking.html")



if __name__ == "__main__":
	app.debug = True
	app.run()

