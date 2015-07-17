#!/usr/bin/env python

import os
import json
import sys
from os.path import dirname, abspath, join, basename

# The ConfigParser module has been renamed to configparser in Python 3.0
try:
	import ConfigParser
except:
	import configparser as ConfigParser

import tempfile

import flask
from flask import Flask
from werkzeug import secure_filename

from celery import Celery
from celery.result import AsyncResult

ompl_app_root = dirname(dirname(abspath(__file__)))
ompl_resources_dir = join(ompl_app_root, 'resources/3D')
try:
	import ompl
	from ompl import base as ob
	from ompl import geometric as og
	from ompl import control as oc
	from ompl import app as oa
	from ompl import tools as ot
except:
	sys.path.insert(0, join(ompl_app_root, 'ompl/py-bindings'))
	import ompl
	from ompl import base as ob
	from ompl import geometric as og
	from ompl import control as oc
	from ompl import app as oa
	from ompl import tools as ot

# Helper functions from helpers.py
import helpers

# Location of .dae files
PROBLEM_FILES = join(dirname(abspath(__file__)), 'static/problem_files')

# Configure Flask and Celery
app = flask.Flask(__name__)
app.config.from_object(__name__)
app.config['CELERY_BROKER_URL'] = 'redis://localhost:6379'
app.config['CELERY_RESULT_BACKEND'] = 'redis://localhost:6379'

celery = Celery(app.name, broker=app.config['CELERY_BROKER_URL'])
celery.conf.update(app.config)

# Create logger
log = helpers.LogOutputHandler(3)


########## OMPL ##########

def create_planners():
	"""
	Initializes the planner lists and formats each planner's parameters to be
	sent to the client.
	"""
	ompl.initializePlannerLists()
	# Create the geometric planners
	planners = ompl.PlanningAlgorithms(og)
	params_dict = planners.getPlanners()

	return params_dict


def allowed_file(filename):
	"""
	Checks that the parameter is a .dae file.
	"""

	if '.' in filename and filename.rsplit('.', 1)[1] == 'dae':
		# Extract the file extension and return true if dae
		return True

	return False


def parse_cfg(cfg_path):
	"""
	Parses the configuration file for pre-defined problems and returns a
	settings dictionary
	"""

	if (sys.version_info > (3, 0)):
		config = ConfigParser.ConfigParser(strict = False)
	else:
		config = ConfigParser.ConfigParser()

	config.readfp(open(cfg_path, 'r'))

	return config._sections['problem']


def save_cfg_file(name, session_id, text):
	"""
	Saves a .cfg file intended for benchmarking
	"""

	file_loc = join('static/sessions', session_id, name);
	f = open(file_loc + ".cfg", 'w')
	f.write(text)
	f.close()

	return file_loc


def format_solution(path, solved):
	"""
	Formats the either the solution, or a failure message for delivery to the
	client.
	"""

	solution = {}

	if solved:
		solution['solved'] = 'true'
		# Format the path
		path_matrix = path.printAsMatrix().strip().split('\n')

		# A list of n states, where path_list[0] is the start state and path_list[n]
		# is the goal state, and path_list[i] are the intermediary states.
		# Each state is also a list: [x, y, z,
		path_list = []

		for line in path_matrix:
			# print "\t " + line
			path_list.append(line.split(" "))

		# print path_list
		solution['path'] = path_list;

		solution['pathAsMatrix'] = path.printAsMatrix()
	else:
		solution['solved'] = 'false'

	# Grab the messages to send to the user
	solution['messages'] = log.getMessages()
	# Clear messages in preparation for next request
	log.clearMessages()

	return solution

@celery.task()
def solve(problem, flask_request_form):
	"""
	Given an instance of the Problem class, containing the problem configuration
	data, solves the motion planning problem and returns either the solution
	path or a failure message.
	"""


	## Configure the problem
	space = ob.SE3StateSpace()

	# Set the dimensions of the bounding box
	bounds = ob.RealVectorBounds(3)

	bounds.low[0] = float(problem['volume.min.x'])
	bounds.low[1] = float(problem['volume.min.y'])
	bounds.low[2] = float(problem['volume.min.z'])

	bounds.high[0] = float(problem['volume.max.x'])
	bounds.high[1] = float(problem['volume.max.y'])
	bounds.high[2] = float(problem['volume.max.z'])


	ompl_setup = oa.SE3RigidBodyPlanning()
	ompl_setup.getGeometricComponentStateSpace().setBounds(bounds)

	ompl_setup.setEnvironmentMesh(str(problem['env_loc']))
	ompl_setup.setRobotMesh(str(problem['robot_loc']))

	# Set the start state
	start = ob.State(space)
	start().setXYZ(float(problem['start.x']), float(problem['start.y']), float(problem['start.z']))

	# Set the start rotation
	start().rotation().x = float(problem['start.q.x'])
	start().rotation().y = float(problem['start.q.y'])
	start().rotation().z = float(problem['start.q.z'])
	start().rotation().w = float(problem['start.q.w'])

	# Set the goal state
	goal = ob.State(space)
	goal().setXYZ(float(problem['goal.x']), float(problem['goal.y']), float(problem['goal.z']))

	# Set the goal rotation
	goal().rotation().x = float(problem['goal.q.x'])
	goal().rotation().y = float(problem['goal.q.y'])
	goal().rotation().z = float(problem['goal.q.z'])
	goal().rotation().w = float(problem['goal.q.w'])

	ompl_setup.setStartAndGoalStates(start, goal)


	## Load the planner
	space_info = ompl_setup.getSpaceInformation()

	planner = eval("%s(space_info)" % problem['planner'])
	ompl_setup.setPlanner(planner)
	log.info("Using planner: %s\n" % ompl_setup.getPlanner().getName())

	# TODO: This is not that good...find a better way to get planner params
	params = str(planner.params()).split("\n")
	# For each parameter that this planner has
	for param in params:
		param = param.split(" ")[0]
		# See if a value for this param is provided by the client
		if param in flask_request_form:
			# If value exists, set the param to the value:w
			planner.params().setParam(param, str(problem['planner_params'][param]))

	## Solve the problem
	solution = {}
	solved = ompl_setup.solve(float(problem['solve_time']))

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
				path = simple_path;
			else:
				log.info("Simplified path was invalid. Returned non-simplified path.\n")
				log.info("Path length: %d\n" % path.length())

			# Interpolate path
			ns = int(100.0 * float(path.length()) / float(ompl_setup.getStateSpace().getMaximumExtent()))
			log.info("Interpolating solution path to " + str(ns) + " states")
			path.interpolate(ns)
			if len(path.getStates()) != ns:
				log.info("Interpolation produced " + str(len(path.getStates())) + " states instead of " + str(ns) + " states.")

			solution = format_solution(path, True)

		else :
			log.info("Path reported by planner seems to be invalid.\n")
			solution = format_solution(path, False)
	else:
		log.info("No valid path was found with the provided configuration.\n")
		solution = format_solution(None, False)

	solution['name'] = str(problem['name'])
	solution['planner'] = ompl_setup.getPlanner().getName()

	return solution

@celery.task()
def solve_multiple(runs, problem, flask_request_form):
	"""
	"""
	result = {}
	result['multiple'] = "true"
	solutions = []

	for i in range(0, runs):
		print("Solving run number: {}".format(i))
		solutions.append(solve(problem, flask_request_form))

	result['solutions'] = solutions

	return result

@celery.task()
def benchmark(name, session_id, cfg_loc, db_filename):
	"""
	Runs ompl_benchmark on cfg_loc and converts the resulting log file to a
	database with ompl_benchmark_statistics.

	cfg_loc - the location of the .cfg file to be benchmarked
	"""

	# Run the benchmark, produces .log file
	os.system("ompl_benchmark " + cfg_loc + ".cfg")

	# Convert .log into database
	os.system("python ../ompl/scripts/ompl_benchmark_statistics.py " + cfg_loc + ".log -d static/sessions/" + session_id + "/" + db_filename)


########## Flask ##########

# Page Loading
@app.route("/")
def index():
	return flask.redirect(flask.url_for('omplapp'))

@app.route("/omplapp", methods=['GET'])
def omplapp():
	"""
	Application starting point, loads everything.
	"""
	return flask.render_template("omplweb.html")

@app.route('/omplapp/components/configuration')
def load_configuration():
	return flask.render_template("components/configuration.html")

@app.route('/omplapp/components/benchmarking')
def load_benchmarking():
	return flask.render_template("components/benchmarking.html")

@app.route('/omplapp/components/about')
def load_about():
	return flask.render_template("components/about.html")

@app.route('/omplapp/session')
def create_session():
	"""
	"""

	session_path = tempfile.mkdtemp(prefix="", dir="static/sessions")
	session_name = basename(session_path)

	return session_name


# Problem Configuration
@app.route('/omplapp/planners')
def planners():
	planners = create_planners()
	return json.dumps(planners)

@app.route('/omplapp/upload', methods=['POST'])
def upload():
	"""
	This function is invoked when the client clicks 'Solve' and submits the
	problem configuration data. Problem configuration data is parsed and loaded
	and the problem is solved by an asynchronous celery task. The task ID of
	the task is returned.
	"""

	problem = flask.request.get_json(True, False, True)

	runs = int(problem['runs'])
	if runs > 1:
		solve_task = solve_multiple.delay(runs, problem, flask.request.form)
		log.debug("Started solving multiple runs with task id: " + solve_task.task_id)
		return str(solve_task.task_id)
	else:
		solve_task = solve.delay(problem, flask.request.form)
		log.debug("Started solving task with id: " + solve_task.task_id)
		return str(solve_task.task_id)

@app.route('/omplapp/poll/<task_id>', methods=['POST'])
def poll(task_id):
	"""
	Checks if the task corresponding to the input ID has completed. If the
	task is done solving, the solution is returned.
	"""

	log.info("Recieved poll for task: " + task_id)

	result = solve.AsyncResult(task_id)

	if result.ready():
		return json.dumps(result.get()), 200
	else :
		return "Result for task id: " + task_id + " isn't ready yet.", 202

@app.route("/omplapp/upload_models", methods=['POST'])
def upload_models():
	"""
	Uploads the user's robot and environment files and saves them to the
	server. The URL to these files are then returned for use by ColladaLoader
	to visualize.
	"""

	robot = flask.request.files['robot']
	env = flask.request.files['env']

	session_id = flask.request.form['session_id']
	session_dir = join("static/sessions", session_id)

	file_locs = {}

	# Check that the uploaded files are valid
	if robot and env:
		if allowed_file(robot.filename) and allowed_file(env.filename):

			# If valid files, save them to the server
			robot_file = join(session_dir, secure_filename(robot.filename))
			robot.save(robot_file)
			file_locs['robot_loc'] = robot_file

			env_file = join(session_dir, secure_filename(env.filename))
			env.save(env_file)
			file_locs['env_loc'] = env_file

		else:
			return "Error: Wrong file format. Robot and environment files must be .dae"
	else:
		return "Error: Didn't upload any files! Please choose both a robot and environment file in the .dae format."

	return json.dumps(file_locs)

@app.route("/omplapp/problem/<problem_name>", methods=['GET'])
def request_models(problem_name):
	"""
	Sends the user the user the location of the requested problem's model files
	and the problem configuration settings
	"""
	robot_filename = problem_name + "_robot.dae"
	env_filename = problem_name + "_env.dae"
	cfg_filename = problem_name + ".cfg"

	cfg_file = join(app.config['PROBLEM_FILES'], cfg_filename)
	cfg_data = parse_cfg(cfg_file)

	# Store the location of the robot and env models
	cfg_data['robot_loc'] = os.path.join("static/problem_files", robot_filename)
	cfg_data['env_loc'] = os.path.join("static/problem_files", env_filename)

	return json.dumps(cfg_data)


# Benchmarking
@app.route('/omplapp/benchmark', methods=['POST'])
def init_benchmark():
	print("Called benchmark.")

	session_id = flask.request.form['session_id']
	session_dir = join("static/sessions", session_id)
	cfg = flask.request.form['cfg']
	cfg_name = flask.request.form['filename']
	cfg_loc = save_cfg_file(cfg_name, session_id, cfg)

	db_file, db_filepath = tempfile.mkstemp(suffix=".db", prefix="", dir=session_dir)

	# Close the db_file, since we don't need it right now
	os.close(db_file)

	db_filename = basename(db_filepath)
	print("Created a placeholder db file: %s" % db_filename)

	result = benchmark.delay(cfg_name, session_id, cfg_loc, db_filename)

	return db_filename


if __name__ == "__main__":
	app.debug = True
	app.run()


