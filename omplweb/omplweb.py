import os
import json
import sys
import StringIO

from math import cos, sin, pi

from math import cos, sin, asin, acos, atan2, pi, pow, ceil, sqrt
# The ConfigParser module has been renamed to configparser in Python 3.0
try:
	import ConfigParser
except:
	import configparser as ConfigParser

import flask
from flask import Flask
from werkzeug import secure_filename

from celery import Celery
from celery.result import AsyncResult

import ompl
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from ompl import app as oa

import helpers
from helpers import LogOutputHandler as Logger

# Location of .dae files
UPLOAD_FOLDER = os.path.dirname(os.path.abspath(__file__)) + '/static/uploads'
PROBLEMS_FOLDER = os.path.dirname(os.path.abspath(__file__)) + '/static/problem_files'

app = flask.Flask(__name__)
app.config.from_object(__name__)
app.config['CELERY_BROKER_URL'] = 'redis://localhost:6379'
app.config['CELERY_RESULT_BACKEND'] = 'redis://localhost:6379'

celery = Celery(app.name, broker=app.config['CELERY_BROKER_URL'])
celery.conf.update(app.config)

########## OMPL ##########

global_vars = {}

log = Logger(3)


def create_planners():
	"""
	Initializes the planner lists and formats each planner's parameters to be
	sent to the client.
	"""
	ompl.initializePlannerLists()
	# Create the geometric planners
	planners = ompl.PlanningAlgorithms(og)
	params_dict = planners.getPlanners()

	# TODO: Por que? Is this accidentally left over from testing?
	retval = "{'param name' : ('display name', 'range type', 'range suggestion', 'default value')}\n"
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


def save_cfg_file(name, text):
	file_loc = os.path.join(app.config['PROBLEMS_FOLDER'], name);

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

	ompl_setup.setEnvironmentMesh(str(problem['env_path']))
	ompl_setup.setRobotMesh(str(problem['robot_path']))

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

	return json.dumps(solution)


@celery.task()
def benchmark(name, cfg_loc, user_email):
	"""
	Runs ompl_benchmark on cfg_loc and converts the resulting log file to a
	database with ompl_benchmark_statistics.

	cfg_loc - the location of the .cfg file to be benchmarked
	"""

	db_id = helpers.rand_num_as_str(10)

	os.system("ompl_benchmark " + cfg_loc + ".cfg")
	os.system("python ompl_benchmark_statistics.py " + cfg_loc + ".log -d static/problem_files/" + db_id + ".db")
	db = open("static/problem_files/" + name + ".db", 'r')
	send_email(name, db, db_id, user_email)


def send_email(name, db, db_id, user_email):
	import smtplib
	from os.path import basename
	from email.mime.application import MIMEApplication
	from email.mime.multipart import MIMEMultipart
	from email.mime.base import MIMEBase
	from email.mime.text import MIMEText
	from email.utils import COMMASPACE, formatdate

	passfile = open("pass", "r")
	username = "prrb02@gmail.com"
	password = passfile.read();
	db_name = name + ".db"
	subject = "OMPL Benchmarking Results"

	msg = MIMEMultipart()
	msg['Subject'] = subject
	msg['From'] = username
	msg['To'] = user_email

	msg_body = "OMPL Web\n\n"
	msg_body += "Benchmarking for problem '" + name + "' is complete.\n\n"
	msg_body += "View the results at: http://127.0.0.1:4290/?id=" + db_id + "\n\n"
	msg_body += "The results database has also been attached to this email."
	msg.attach(MIMEText(msg_body))

	attachment = MIMEApplication(db.read())
	attachment.add_header('Content-Disposition', 'attachment', filename=db_name)
	msg.attach(attachment)

	s = smtplib.SMTP('smtp.gmail.com:25')
	s.starttls()
	s.login(username, password)
	s.sendmail(username, [user_email], msg.as_string())
	s.quit()


########## Flask ##########

# This section loads html
@app.route("/")
def index():
	return flask.redirect(flask.url_for('omplapp'))

@app.route("/omplapp", methods=['GET'])
def omplapp():
	"""
	Returns the problem configuration page.
	"""

	return flask.render_template("omplweb.html")

@app.route('/omplapp/components/configuration')
def load_configuration():
	return flask.render_template("components/configuration.html")

@app.route('/omplapp/components/benchmarking')
def load_benchmarking():
	return flask.render_template("components/benchmarking.html")


# This section manages communication when solving a problem
@app.route('/omplapp/planners')
def planners():
	planners = create_planners()
	return json.dumps(planners)

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

	problem = flask.request.get_json(True, False, True)

	solve_task = solve.delay(problem, flask.request.form)
	log.debug("Started solving task with id: " + solve_task.task_id)

	return str(solve_task.task_id)

@app.route('/omplapp/poll/<task_id>', methods=['POST'])
def poll(task_id):
	"""docstring for poll"""

	log.info("Recieved poll for task: " + task_id)

	result = solve.AsyncResult(task_id)

	if result.ready():
		return str(result.get()), 200
	else :
		return "Result for task id: " + task_id + " isn't ready yet.", 202


@app.route("/omplapp/upload_models", methods=['POST'])
def upload_models():
	"""
	Uploads the user's robot and environment files and saves them to the
	server. The URL to these files are then returned for use by ColladaLoader
	to visualize.
	"""

	robotFile = flask.request.files['robot']
	envFile = flask.request.files['env']

	filepaths = {}

	# Check that the uploaded files are valid
	if robotFile and envFile:
		if allowed_file(robotFile.filename) and allowed_file(envFile.filename):

			# If valid files, save them to the server
			robot_filename = secure_filename(robotFile.filename)
			robotFile.save(os.path.join(app.config['UPLOAD_FOLDER'], robot_filename))

			filepaths['robot_path'] = os.path.join(app.config['UPLOAD_FOLDER'], robot_filename)
			filepaths['robot_loc'] = "static/uploads/" + robot_filename

			env_filename = secure_filename(envFile.filename)
			envFile.save(os.path.join(app.config['UPLOAD_FOLDER'], env_filename))

			filepaths['env_path'] = os.path.join(app.config['UPLOAD_FOLDER'], env_filename)
			filepaths['env_loc'] = "static/uploads/" + env_filename

		else:
			return "Error: Wrong file format. Robot and environment files must be .dae"
	else:
		return "Error: Didn't upload any files! Please choose both a robot and environment file in the .dae format."

	return json.dumps(filepaths)

@app.route("/omplapp/problem/<problem_name>", methods=['GET'])
def request_models(problem_name):
	"""
	Sends the user the user the location of the requested problem's model files
	and the problem configuration settings
	"""
	robot_filename = problem_name + "_robot.dae"
	env_filename = problem_name + "_env.dae"
	cfg_filename = problem_name + ".cfg"


	cfg_file = os.path.join(app.config['PROBLEMS_FOLDER'], cfg_filename)
	cfg_data = parse_cfg(cfg_file)
	cfg_data['robot_loc'] = os.path.join("static/problem_files", robot_filename)
	cfg_data['env_loc'] = os.path.join("static/problem_files", env_filename)

	cfg_data['robot_path'] = os.path.join(app.config['PROBLEMS_FOLDER'], robot_filename)
	cfg_data['env_path'] = os.path.join(app.config['PROBLEMS_FOLDER'], env_filename)

	return json.dumps(cfg_data)




# Benchmarking
@app.route('/omplapp/benchmark', methods=['POST'])
def init_benchmark():
	print("Called benchmark.")

	cfg_name = flask.request.form['filename']
	cfg = flask.request.form['cfg']
	user_email = flask.request.form['email']

	cfg_loc = save_cfg_file(cfg_name, cfg)

	result = benchmark.delay(cfg_name, cfg_loc, user_email)


	return "Saved file at: " + cfg_loc

if __name__ == "__main__":
	app.debug = True
	app.run()


