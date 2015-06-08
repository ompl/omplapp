import os
import json

import flask
from werkzeug import secure_filename

import ompl
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from ompl import app as oa

# Location of user uploaded .dae files
UPLOAD_FOLDER = '/Users/prudhvi/Dropbox/School/Research/KavrakiLab/OMPL/flask/omplweb/uploads'

app = flask.Flask(__name__)
app.config.from_object(__name__)



########## OMPL Code ##########


class LogOutputHandler(object):
	"""
	Object for handling various levels of logging.
	"""

	def __init__(self, log_level):
		# Specifies the level of logging should be printed to the console
		# 0 = Silent (probablly shouldn't use this)
		# 1 = Errors only
		# 2 = Level 0 and warnings
		# 3 = Levels 0, 1 and info
		# 4 = Levels 0, 1, 2 and debugging
		self.log_level = log_level

		self.messages = "Messages: \n"

	def error(self, text):
		if self.log_level > 0:
			print "# Error:    " + str(text)

	def warn(self, text):
		if self.log_level > 1:
			print "# Warning:    " + str(text)

	def info(self, text):
		"""
		Info messages are always stored and can be retrieved to send to the
		client
		"""
		# Store the message
		self.messages += str(text)

		if self.log_level > 2:
			print "# Info:    " + str(text)

	def debug(self, text):
		if self.log_level > 3:
			print "# Debug:    " + str(text)

	def getMessages(self):
		# 'Info' messages are stored and can be retrieved via this function to
		# send to the client
		return self.messages

log = LogOutputHandler(4)

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


def allowed_file(filename):
	"""
	Checks that the parameter is a .dae file.
	"""

	if '.' in filename and filename.rsplit('.', 1)[1] == 'dae':
		# Extract the file extension and return true if dae
		return True

	return False


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
	problem.start_x = float(settings['start_x'])
	problem.start_y = float(settings['start_y'])
	problem.start_z = float(settings['start_z'])
	problem.start_theta = float(settings['start_theta'])
	problem.start_axis_x = float(settings['start_axis_x'])
	problem.start_axis_y = float(settings['start_axis_y'])
	problem.start_axis_z = float(settings['start_axis_z'])
	problem.goal_x = float(settings['goal_x'])
	problem.goal_y = float(settings['goal_y'])
	problem.goal_z = float(settings['goal_z'])
	problem.goal_theta = float(settings['goal_theta'])
	problem.goal_axis_x = float(settings['goal_axis_x'])
	problem.goal_axis_y = float(settings['goal_axis_y'])
	problem.goal_axis_z = float(settings['goal_axis_z'])
	problem.bounds_min_x = float(settings['bounds_min_x'])
	problem.bounds_min_y = float(settings['bounds_min_y'])
	problem.bounds_min_z = float(settings['bounds_min_z'])
	problem.bounds_max_x = float(settings['bounds_max_x'])
	problem.bounds_max_y = float(settings['bounds_max_y'])
	problem.bounds_max_z = float(settings['bounds_max_z'])

	# benchmark specific stuff #
	problem.time_limit = float(settings['time_limit'])
	problem.mem_limit = float(settings['mem_limit'])
	problem.run_count = float(settings['run_count'])

	# planners for benchmarks #
	problem.planners = settings['planners']

	return problem


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

	# TODO: Better, neater formatting of path
	solution['path'] = str(path)

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


	# Load the planner
	space_info = ompl_setup.getSpaceInformation()

	if problem.planners != "":
		# If user selected a planner, load it
		planner = eval("ompl.%s(space_info)" % problem.planners)
		ompl_setup.setPlanner(planner)
		log.info("\tUsing planner: %s\n" % ompl_setup.getPlanner().getName())
	else:
		log.info("\tNo planner specified, using default\n")

	## Solve the problem
	solution = {}
	solved = ompl_setup.solve(problem.time_limit)

	## Check for validity
	if solved:
		path = ompl_setup.getSolutionPath()
		initialValid = path.check()

		if initialValid:
			log.info("\tInitial path length: %d\n" % path.length())

			# If if initially valid, attempt to simplify
			ompl_setup.simplifySolution()
			# Get the simplified path
			simple_path = ompl_setup.getSolutionPath()
			simplifyValid = simple_path.check()
			if simplifyValid:
				log.info("\tSimplified path was found.\n")
				log.info("\tSimplified path length: %d\n" % path.length())
				solution = format_solution(simple_path, True)
			else:
				log.info("\tSimplified path was invalid. Returned \
						non-simplified path.\n")
				log.info("\tPath length: %d\n" % path.length())
				solution = format_solution(path, True)

			# TODO: Interpolation?

		else :
			log.info("\tPath reported by planner seems to be invalid.\n")
			solution = format_solution(path, False)
	else:
		log.info("\tNo valid path was found with the provided \
				configuration.\n")
		solution = format_solution(None, False)

	solution['name'] = problem.name
	solution['planner'] = ompl_setup.getPlanner().getName()
	solution['messages'] = log.getMessages()

	return json.dumps(solution)


########## Flask Code ##########


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

	1.	The robot and environment files are checked for validity and downloaded to
		the server.
	2.	Configuration data is parsed and loaded via the parse function.
	3.	The problem is solved and the output of the solve function (either a
		solution path or an error) is returned.

	"""

	# The data received from the user
	data = flask.request.form

	log.debug(data)

	# Uploaded custom problem
	if (data['problems'] == 'custom'):
		robotFile = flask.request.files['robot']
		envFile = flask.request.files['env']

		# Check that the uploaded files are valid
		if robotFile and envFile:
			if allowed_file(robotFile.filename) and allowed_file(envFile.filename):

				# If valid files, save them to the server
				robot_filename = secure_filename(robotFile.filename)
				robotFile.save(os.path.join(app.config['UPLOAD_FOLDER'], robot_filename))
				robot_path = os.path.join(app.config['UPLOAD_FOLDER'], robot_filename)

				env_filename = secure_filename(envFile.filename)
				envFile.save(os.path.join(app.config['UPLOAD_FOLDER'], env_filename))
				env_path = os.path.join(app.config['UPLOAD_FOLDER'], env_filename)

				log.debug("Files saved as: " + robot_path + " and " + env_path)

				log.debug("Will now parse configuration...")
				# TODO:Put some try/catches here
				problem = parse(flask.request.form, env_path, robot_path)

				log.debug("Configuration has been parsed. Solving problem...")
				solution = solve(problem)
				# TODO: Delete the uploaded files?
				log.debug("Problem solved")
				return solution

			else:
				return "Error: Wrong file format. Robot and environment files must be .dae"
		else:
			return "Error: Didn't upload any files! Please choose both a robot and an environment file in the .dae format."

		return "Upload Successful."

	# Selected pre-configured problem from server


@app.route("/")
def index():
	return flask.redirect(flask.url_for('omplapp'))


if __name__ == "__main__":
	app.debug = True
	app.run()

