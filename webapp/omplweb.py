#!/usr/bin/env python

import os
from os.path import dirname, abspath, join, basename, exists
import json
import subprocess
import sys
import tempfile
import webbrowser
from inspect import isclass

# The ConfigParser module has been renamed to configparser in Python 3.0
try:
    import ConfigParser
except ImportError:
    import configparser as ConfigParser

# Constants
ompl_app_root = dirname(dirname(abspath(__file__)))
ompl_web_root = join(ompl_app_root, "webapp")
ompl_sessions_dir = join(ompl_web_root, 'static/sessions')
problem_files = join(ompl_web_root, 'static/problem_files')
prefix = dirname(dirname(ompl_app_root))

if exists(join(prefix, 'bin/ompl_benchmark_statistics.py')):
    sys.path.insert(0, join(prefix, 'bin'))

try:
    import ompl
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import control as oc
    from ompl import app as oa
    from ompl.util import OMPL_DEBUG, OMPL_INFORM, OMPL_WARN, OMPL_ERROR
    from ompl_benchmark_statistics import readBenchmarkLog
except ImportError:
    sys.path.insert(0, join(ompl_app_root, 'ompl/py-bindings'))
    import ompl
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import control as oc
    from ompl import app as oa
    from ompl.util import OMPL_DEBUG, OMPL_INFORM, OMPL_WARN, OMPL_ERROR
    sys.path.insert(0, join(ompl_app_root, 'ompl/scripts'))
    from ompl_benchmark_statistics import readBenchmarkLog

import flask
from werkzeug import secure_filename

from celery import Celery


def initialize():
    ompl.initializePlannerLists()

    if sys.version_info > (3, 0):
        config = ConfigParser.ConfigParser(strict=False)
    else:
        config = ConfigParser.ConfigParser()

    conf_file_loc = None
    for loc in [join(ompl_app_root, join("ompl", "ompl.conf")), \
        join(prefix, "share/ompl/ompl.conf")]:
        if exists(loc):
            conf_file_loc = loc
            break
    config.read([conf_file_loc])
    preferences = config._sections["webapp"]
    preferences["plannerarena_port"] = config.get("plannerarena", "plannerarena_port")
    return preferences

def make_celery():
    conf = {"rabbitmq" : {"broker" : "amqp://localhost", "backend" : "rpc://localhost"},
            "redis" : {"broker" : "redis://localhost", "backend" : "redis://localhost"}}

    broker_name = preferences["broker"]
    broker_url = conf[broker_name]["broker"]
    backend_url = conf[broker_name]["backend"]

    if "broker_port" in preferences:
        # If a port is specified, use it. Otherwise the default is automatically used
        broker_url += ":" + preferences["broker_port"]
        backend_url += ":" + preferences["broker_port"]

    celery = Celery(app.name, broker=broker_url, backend=backend_url)
    celery.conf.update({u'task_default_queue': u'omplapp'})
    return celery


# Load preferences
preferences = initialize()

# Configure Flask
app = flask.Flask(__name__)
app.config.from_object(__name__)

# Configure Celery
celery = make_celery()

########## OMPL ##########

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

    if sys.version_info > (3, 0):
        config = ConfigParser.ConfigParser(strict=False)
    else:
        config = ConfigParser.ConfigParser()

    config.read([cfg_path])
    return config._sections['problem']


def save_cfg_file(name, session_id, text):
    """
    Saves a .cfg file intended for benchmarking
    """
    file_loc = join(ompl_sessions_dir, session_id, name)
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
            path_list.append(line.strip().split(" "))

        solution['path'] = path_list

        solution['pathAsMatrix'] = path.printAsMatrix()
    else:
        solution['solved'] = 'false'

    return solution


def get_offset(env_mesh, robot_mesh):
    ompl_setup = oa.SE3RigidBodyPlanning()
    ompl_setup.setEnvironmentMesh(str(env_mesh))
    ompl_setup.setRobotMesh(str(robot_mesh))

    # full state
    start = ompl_setup.getDefaultStartState()
    # just the first geometric component
    start = ompl_setup.getGeometricComponentState(start, 0)
    # extract x,y,z coords
    # print start
    offset = {}
    offset['x'] = start[0]
    offset['y'] = start[1]
    offset['z'] = start[2]
    return offset


def setup(problem):
    OMPL_INFORM("Robot type is: %s" % str(problem["robot.type"]))

    ompl_setup = eval("oa.%s()" % problem["robot.type"])
    problem["is3D"] = isinstance(ompl_setup.getGeometricComponentStateSpace(), ob.SE3StateSpace)
    if str(ompl_setup.getAppType()) == "GEOMETRIC":
        problem["isGeometric"] = True
    else:
        problem["isGeometric"] = False

    ompl_setup.setEnvironmentMesh(str(problem['env_loc']))
    ompl_setup.setRobotMesh(str(problem['robot_loc']))

    if problem["is3D"]:
        # Set the dimensions of the bounding box
        bounds = ob.RealVectorBounds(3)

        bounds.low[0] = float(problem['volume.min.x'])
        bounds.low[1] = float(problem['volume.min.y'])
        bounds.low[2] = float(problem['volume.min.z'])

        bounds.high[0] = float(problem['volume.max.x'])
        bounds.high[1] = float(problem['volume.max.y'])
        bounds.high[2] = float(problem['volume.max.z'])

        ompl_setup.getGeometricComponentStateSpace().setBounds(bounds)

        space = ob.SE3StateSpace()
        # Set the start state
        start = ob.State(space)
        start().setXYZ(float(problem['start.x']), float(problem['start.y']), \
            float(problem['start.z']))

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

        start = ompl_setup.getFullStateFromGeometricComponent(start)
        goal = ompl_setup.getFullStateFromGeometricComponent(goal)
        ompl_setup.setStartAndGoalStates(start, goal)
    else:
        # Set the dimensions of the bounding box
        bounds = ob.RealVectorBounds(2)

        bounds.low[0] = float(problem['volume.min.x'])
        bounds.low[1] = float(problem['volume.min.y'])

        bounds.high[0] = float(problem['volume.max.x'])
        bounds.high[1] = float(problem['volume.max.y'])

        ompl_setup.getGeometricComponentStateSpace().setBounds(bounds)
        space = ob.SE2StateSpace()

        start = ob.State(space)
        start().setX(float(problem['start.x']))
        start().setY(float(problem['start.y']))
        start().setYaw(float(problem['start.yaw']))

        goal = ob.State(space)
        goal().setX(float(problem['goal.x']))
        goal().setY(float(problem['goal.y']))
        goal().setYaw(float(problem['goal.yaw']))

        start = ompl_setup.getFullStateFromGeometricComponent(start)
        goal = ompl_setup.getFullStateFromGeometricComponent(goal)
        ompl_setup.setStartAndGoalStates(start, goal)

    return ompl_setup


@celery.task()
def solve(problem):
    """
    Given an instance of the Problem class, containing the problem configuration
    data, solves the motion planning problem and returns either the solution
    path or a failure message.
    """

    # Sets up the robot type related information
    ompl_setup = setup(problem)

    # Load the planner
    space_info = ompl_setup.getSpaceInformation()
    if problem['planner'].startswith('ompl.control.Syclop'):
        decomposition = ompl_setup.allocDecomposition()
        planner = eval('%s(space_info, decomposition)' % problem['planner'])
        ompl_setup.setPlanner(planner)
    else:
        planner = eval("%s(space_info)" % problem['planner'])
        ompl_setup.setPlanner(planner)

    # Set the optimization objective
    objectives = {'length': 'PathLengthOptimizationObjective', \
        'max_min_clearance': 'MaximizeMinClearanceObjective', \
        'mechanical_work': 'MechanicalWorkOptimizationObjective'}
    objective = objectives[problem['objective']]
    obj = eval('ob.%s(space_info)' % objective)
    cost = ob.Cost(float(problem['objective.threshold']))
    obj.setCostThreshold(cost)
    ompl_setup.setOptimizationObjective(obj)

    # Set each parameter that was configured by the user
    for param in problem['planner_params']:
        planner.params().setParam(str(param), str(problem['planner_params'][param]))

    # Solve the problem
    solution = {}
    solved = ompl_setup.solve(float(problem['solve_time']))

    # Check for validity
    if solved:
        if problem['isGeometric']:
            path = ompl_setup.getSolutionPath()
            initialValid = path.check()
            if initialValid:
                # If initially valid, attempt to simplify
                ompl_setup.simplifySolution()
                # Get the simplified path
                simple_path = ompl_setup.getSolutionPath()
                simplifyValid = simple_path.check()
                if simplifyValid:
                    path = simple_path
                else:
                    OMPL_ERROR("Simplified path was invalid.")
            else:
                OMPL_ERROR("Invalid solution path.")
        else:
            path = ompl_setup.getSolutionPath().asGeometric()
            OMPL_INFORM("Path simplification skipped due to non rigid body.")

        # Interpolate path
        ns = int(100.0 * float(path.length()) / \
            float(ompl_setup.getStateSpace().getMaximumExtent()))
        if problem["isGeometric"] and len(path.getStates()) < ns:
            path.interpolate(ns)
            if len(path.getStates()) != ns:
                OMPL_WARN("Interpolation produced " + str(len(path.getStates())) + \
                    " states instead of " + str(ns) + " states.")

        solution = format_solution(path, True)
    else:
        solution = format_solution(None, False)

    solution['name'] = str(problem['name'])
    solution['planner'] = ompl_setup.getPlanner().getName()
    solution['status'] = str(solved)

    # Store the planner data
    pd = ob.PlannerData(ompl_setup.getSpaceInformation())
    ompl_setup.getPlannerData(pd)
    explored_states = []
    for i in range(0, pd.numVertices()):
        coords = []
        state = pd.getVertex(i).getState()

        if isinstance(state) == ob.CompoundStateInternal:
            state = state[0]

        coords.append(state.getX())
        coords.append(state.getY())

        if problem["is3D"]:
            coords.append(state.getZ())
        else:
            coords.append(0)

        explored_states.insert(i, coords)

    solution["explored_states"] = explored_states
    return solution

@celery.task()
def solve_multiple(runs, problem):
    """
    """
    result = {}
    result['multiple'] = "true"
    solutions = []

    for i in range(0, runs):
        OMPL_INFORM("Solving run number: {}".format(i))
        solutions.append(solve(problem))

    result['solutions'] = solutions

    return result

@celery.task()
def benchmark(name, session_id, cfg_loc, db_filename, problem_name, robot_loc, env_loc):
    """
    Runs ompl_benchmark on cfg_loc and converts the resulting log file to a
    database with ompl_benchmark_statistics.

    cfg_loc - the location of the .cfg file to be benchmarked
    """

    session_path = join(ompl_sessions_dir, session_id)
    db_filepath = join(ompl_sessions_dir, session_id, db_filename)
    # Adjust file permissions
    os.chmod(session_path, 0o2755)
    os.chmod(db_filepath, 0o0664)

    if problem_name != "custom":
        robot_file = join(ompl_sessions_dir, session_id, basename(robot_loc))
        env_file = join(ompl_sessions_dir, session_id, basename(env_loc))
        if not os.path.isfile(robot_file) and not os.path.isfile(env_file):
            # Copy over the needed mesh files to perform benchmarking, if they don't exist
            os.symlink(join(ompl_web_root, robot_loc), robot_file)
            os.symlink(join(ompl_web_root, env_loc), env_file)

    # Run the benchmark, produces .log file
    try:
        subprocess.check_output("ompl_benchmark " + cfg_loc + ".cfg", \
            shell=True, \
            stderr=subprocess.STDOUT, \
            env=dict(os.environ, PATH=preferences["ompl_benchmark_loc"] + ":" + os.environ["PATH"]))

        # Convert .log into database
        dbfile = join(ompl_sessions_dir, session_id, db_filename)
        logfile = []
        logfile.append(join(ompl_sessions_dir, session_id, name + ".log"))
        readBenchmarkLog(dbfile, logfile, "")

        # Open the planner arena page when benchmarking is done
        if preferences["show_results"] == "1":
            url = "http://127.0.0.1:" + preferences["plannerarena_port"] + "/?user=" + \
                session_id + "&job=" + db_filename
            webbrowser.open(url)
    except subprocess.CalledProcessError as cp:
        print('returncode=', cp.returncode)
        print('cmd=', cp.cmd)
    except:
        OMPL_ERROR("Unable to call 'ompl_benchmark'. Please ensure that it is in the PATH, " \
            "or add it with: 'export PATH=~/omplapp/build/Release/bin:${PATH}'")



########## Flask ##########

# Page Loading
@app.route("/")
def index():
    """
    Application starting point, loads everything.
    """
    return flask.render_template("omplweb.html")

@app.route('/components/configuration')
def load_configuration():
    return flask.render_template("components/configuration.html")

@app.route('/components/benchmarking')
def load_benchmarking():
    return flask.render_template("components/benchmarking.html")

@app.route('/components/about')
def load_about():
    return flask.render_template("components/about.html")


# Send client information
@app.route('/session')
def create_session():
    """
    Creates a session folder and returns its name
    """
    if not os.path.isdir("/tmp/omplweb_sessions") and not os.path.exists(ompl_sessions_dir):
        # Session folders reside in /tmp, but symlinked into static/sessions
        os.makedirs("/tmp/omplweb_sessions")
        os.symlink("/tmp/omplweb_sessions", ompl_sessions_dir)

    session_path = tempfile.mkdtemp(prefix="", dir=ompl_sessions_dir)
    session_name = basename(session_path)

    return session_name

@app.route('/preferences')
def load_preferences():
    """
    Sends the preferences read from the conf file to the client.
    """
    return json.dumps(preferences)

@app.route('/problems')
def send_problems():
    """
    Sends a list of the available pre-configured problems to the client.
    """

    two_d = []
    three_d = []

    files_2d = os.listdir(join(problem_files, "2D"))
    for filename in files_2d:
        if filename.endswith(".cfg"):
            two_d.append(filename)
    files_3d = os.listdir(join(problem_files, "3D"))
    for filename in files_3d:
        if filename.endswith(".cfg"):
            three_d.append(filename)

    return json.dumps({"2D" : sorted(two_d), "3D" : sorted(three_d)})


@app.route('/planners')
def planners():
    # Return the geometric and control based planners
    planners = {}
    planners['geometric'] = og.planners.plannerMap
    planners['control'] = oc.planners.plannerMap
    return json.dumps(planners, sort_keys=True)

@app.route('/offset', methods=["POST"])
def find_offset():
    env_mesh = flask.request.form['env_loc']
    robot_mesh = flask.request.form['robot_loc']

    offset = get_offset(join(ompl_web_root, env_mesh), join(ompl_web_root, robot_mesh))
    return json.dumps(offset)

@app.route('/robot_types')
def get_robot_types():
    """
    Finds the available robot types and returns as a list of tuples
    """

    robot_types = {}
    for c in dir(oa):
        if eval('isclass(oa.%s) and issubclass(oa.%s, (oa.AppBaseGeometric,oa.AppBaseControl)) ' \
            'and issubclass(oa.%s, oa.RenderGeometry)' % (c, c, c)):
            name = eval('oa.%s().getName()' % c)
            apptype = eval('oa.%s().getAppType()' % c)
            robot_types[str(c)] = {"name" : str(name), "apptype" : str(apptype)}
    return json.dumps(robot_types, sort_keys=True)

@app.route("/request_problem", methods=['POST'])
def request_problem():
    """
    Sends the user the user the location of the requested problem's model files
    and the problem configuration settings
    """
    problem_name = flask.request.form['problem_name']
    dimension = flask.request.form['dimension']
    cfg_filename = problem_name + ".cfg"

    cfg_file_loc = join(problem_files, dimension, cfg_filename)
    cfg_data = parse_cfg(cfg_file_loc)

    if sys.version_info > (3, 0):
        config = ConfigParser.ConfigParser(strict=False)
    else:
        config = ConfigParser.ConfigParser()

    config.read([cfg_file_loc])

    cfg_data['robot_loc'] = join("static/problem_files", dimension, config.get("problem", "robot"))
    cfg_data['env_loc'] = join("static/problem_files", dimension, config.get("problem", "world"))

    return json.dumps(cfg_data)


# Get client information
@app.route('/upload', methods=['POST'])
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
        solve_task = solve_multiple.delay(runs, problem)
        OMPL_DEBUG("Started solving multiple runs with task id: " + solve_task.task_id)
        return str(solve_task.task_id)
    solve_task = solve.delay(problem)
    OMPL_DEBUG("Started solving task with id: " + solve_task.task_id)
    return str(solve_task.task_id)

@app.route('/poll/<task_id>', methods=['POST'])
def poll(task_id):
    """
    Checks if the task corresponding to the input ID has completed. If the
    task is done solving, the solution is returned.
    """

    result = solve.AsyncResult(task_id)

    if result.ready():
        return json.dumps(result.get()), 200
    return "Result for task id: " + task_id + " isn't ready yet.", 202

@app.route("/upload_models", methods=['POST'])
def upload_models():
    """
    Uploads the user's robot and environment files and saves them to the
    server. The URL to these files are then returned for use by ColladaLoader
    to visualize.
    """

    robot = flask.request.files['robot']
    env = flask.request.files['env']

    session_id = flask.request.form['session_id']
    session_dir = join(ompl_sessions_dir, session_id)

    file_locs = {}

    # Check that the uploaded files are valid
    if robot and env:
        if allowed_file(robot.filename) and allowed_file(env.filename):

            # If valid files, save them to the server
            robot_file = join(session_dir, secure_filename(robot.filename))
            robot.save(robot_file)
            file_locs['robot_loc'] = join("static/sessions", session_id, basename(robot_file))

            env_file = join(session_dir, secure_filename(env.filename))
            env.save(env_file)
            file_locs['env_loc'] = join("static/sessions", session_id, basename(env_file))

        else:
            return "Error: Wrong file format. Robot and environment files must be .dae"
    else:
        return "Error: Didn't upload any files! Please choose both a robot and environment file" \
            " in the .dae format."

    return json.dumps(file_locs)


# Benchmarking
@app.route('/benchmark', methods=['POST'])
def init_benchmark():

    session_id = flask.request.form['session_id']
    session_dir = join(ompl_sessions_dir, session_id)
    cfg = flask.request.form['cfg']
    cfg_name = flask.request.form['filename']
    problem_name = flask.request.form['problem']
    env_loc = flask.request.form["env_loc"]
    robot_loc = flask.request.form["robot_loc"]
    cfg_loc = save_cfg_file(cfg_name, session_id, cfg)

    db_file, db_filepath = tempfile.mkstemp(suffix=".db", prefix="", dir=session_dir)

    # Close the db_file, since we don't need it right now
    os.close(db_file)

    db_filename = basename(db_filepath)

    benchmark.delay(cfg_name, session_id, cfg_loc, db_filename, problem_name, env_loc, robot_loc)

    return db_filename

@celery.task
def test_broker():
    print("Celery broker is running...")

if __name__ == "__main__":
    try:
        result = test_broker.delay()
    except:
        broker_url = celery.conf['BROKER_URL']
        broker = "Redis" if broker_url.startswith('redis') else "RabbitMQ"
        print("""The omplweb_app is configured to use a %s server for handling
background jobs, but this server is not running. Exiting...""" % broker)
        sys.exit(-1)
    app.run()
