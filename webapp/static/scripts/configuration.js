/* Primary objects */
var problem;
var solution;
var visualization;
var benchmark;

/* Global Variables */
var pollingInterval;
var robotAnimationInterval;

/* Constants, defaults will be overwritten by loadPreferences() with values specified in webapp.cfg */
var BENCHMARKING_LIMIT = 100000;
var MAX_UPLOAD_SIZE = 50000000;
var PLANNERARENA_PORT = 8888;
var POLL_INTERVAL = 5000;

// Define the Problem class
var Problem = function () {
    // Problem constructor
    this.config = {};

    // Initialize all problem values to 0 or empty
    this.config["name"] = "";
    this.config["robot"] = "";
    this.config["world"] = "";
    this.config["start.x"] = null;
    this.config["start.y"] = null;
    this.config["start.z"] = null;
    this.config["start.q.w"] = null;
    this.config["start.q.x"] = null;
    this.config["start.q.y"] = null;
    this.config["start.q.z"] = null;
    this.config["start.yaw"] = null;
    this.config["goal.x"] = null;
    this.config["goal.y"] = null;
    this.config["goal.z"] = null;
    this.config["goal.q.w"] = null;
    this.config["goal.q.x"] = null;
    this.config["goal.q.y"] = null;
    this.config["goal.q.z"] = null;
    this.config["goal.yaw"] = null;
    this.config["volume.min.x"] = null;
    this.config["volume.min.y"] = null;
    this.config["volume.min.z"] = null;
    this.config["volume.max.x"] = null;
    this.config["volume.max.y"] = null;
    this.config["volume.max.z"] = null;

    this.config["objective"] = "";
    this.config["objective.threshold"] = null;

    this.config["robot.type"] = "SE3RigidBodyPlanning";
    this.config["solve_time"] = 10;
    this.config["runs"] = 1;

    // The URI of the robot and env models on the server, for use by ColladaLoader
    this.config["robot_loc"] = "";
    this.config["env_loc"] = "";

    this.control = {};
    this.control["dynamic_car"] = "GDynamicCarPlanning";
    this.control["kinematic_car"] = "GKinematicCarPlanning";
    this.control["blimp"] = "GBlimpPlanning";
    this.control["quadrotor"] = "GQuadrotorPlanning";
    this.control["GDynamicCarPlanning"] = "dynamic_car";
    this.control["GKinematicCarPlanning"] = "kinematic_car";
    this.control["GBlimpPlanning"] = "blimp";
    this.control["GQuadrotorPlanning"] = "quadrotor";

    this.robots2D = ["GDynamicCarPlanning", "GKinematicCarPlanning", "GSE2RigidBodyPlanning"];

    this.is3D = true;

    this.dimensions = {};
}

Problem.prototype.update = function() {
    // Read all of the input files and update the internal values
    this.config["name"] = $("[name='name']").val();
    this.config["start.x"] = $("[name='start.x']").val();
    this.config["start.y"] = $("[name='start.y']").val();
    this.config["start.z"] = $("[name='start.z']").val();
    this.config["start.q.w"] = start_robot.quaternion.w;
    this.config["start.q.x"] = start_robot.quaternion.x;
    this.config["start.q.y"] = start_robot.quaternion.y;
    this.config["start.q.z"] = start_robot.quaternion.z;
    this.config["start.yaw"] = $("[name='start.yaw']").val() * DEG_TO_RAD;
    this.config["goal.x"] = $("[name='goal.x']").val();
    this.config["goal.y"] = $("[name='goal.y']").val();
    this.config["goal.z"] = $("[name='goal.z']").val();
    this.config["goal.q.w"] = goal_robot.quaternion.w;
    this.config["goal.q.x"] = goal_robot.quaternion.x;
    this.config["goal.q.y"] = goal_robot.quaternion.y;
    this.config["goal.q.z"] = goal_robot.quaternion.z;
    this.config["goal.yaw"] = $("[name='goal.yaw']").val() * DEG_TO_RAD;
    this.config["volume.min.x"] = $("[name='volume.min.x']").val();
    this.config["volume.min.y"] = $("[name='volume.min.y']").val();
    this.config["volume.min.z"] = $("[name='volume.min.z']").val();
    this.config["volume.max.x"] = $("[name='volume.max.x']").val();
    this.config["volume.max.y"] = $("[name='volume.max.y']").val();
    this.config["volume.max.z"] = $("[name='volume.max.z']").val();

    this.config["planner"]= $("[name='planners']").val();
    this.config["objective"] = $("[name='objective']").val();
    this.config["objective.threshold"] = $("[name='objective.threshold']").val();

    this.config["robot.type"] = $("[name='robot.type']").val();
    this.config["solve_time"] = $("[name='solve_time']").val();
    this.config["runs"] = $("[name='runs']").val();

    if ($("#problems").val() === "custom") {
        this.config["robot"]= $("input[name='robot']")[0].files[0].name;
        this.config["world"]= $("input[name='env']")[0].files[0].name;
    }

    // Get the params for the selected planner
    var paramData = {};
    $('.planner_param').each(function () {
        paramData[$(this).attr('name')] = $(this).val();
    });
    this.config["planner_params"] = paramData;
};

Problem.prototype.get = function(field) {
    return this.config[field]
};

Problem.prototype.asJSON = function() {
    return JSON.stringify(this.config);
};

Problem.prototype.isValid = function() {
    this.update();
    for (var item in this.config) {
        if (this.config[item] == null || this.config[item] === "") {
            if (problem.is3D == true) {
                if ($("[name='" + item + "']").hasClass("3D") == true) {
                    console.log(item);
                    return false;
                }
            } else {
                if ($("[name='" + item + "']").hasClass("3D") == false) {
                    console.log(item);
                    return false;
                }
            }
        }
    }
    return true;
};

/**
 * Gathers and formats problem data and submits the problem to the server for solving.
 * On successful solve, saves solution data and loads solution visualization.
 *
 * @param None
 * @return None
 */
Problem.prototype.solve = function() {
    // Ensure that we have the latest data from the user
    problem.update();

    if (problem.isValid()) {
        // Bring up the loading screen
        $.blockUI({
            css: {
                border: 'none',
                padding: '30px',
                backgroundColor: '#000',
                opacity: '0.7',
                color: '#fff',
            }
        });


        // Send the request
        $.ajax({
            url: "upload",
            type: "POST",
            data: problem.asJSON(),
            success: function(data){
                var taskID = String(data);
                console.log("Server successfully recieved solve request. Given task ID: " + taskID);
                solution.poll(taskID);
            },
            error: function(data) {
                $.unblockUI();
                showAlert("configuration", "danger", "Server responded with an error. Check the problem configuration and try again.");

                console.log('Solve failed, server responded with an error.', data);
            },
            cache: false,
            contentType: 'application/json',
            processData: false
        });
    } else {
        // Invalid fields have been highlighted by 'validateField()'.
        showAlert("configuration", "warning", "Please enter values for the indicated fields.");
    }
};

Problem.prototype.hasMultipleRuns = function() {
    if (self.config["runs"] > 1) {
        return true;
    }
    return false;
};

/**
 * Formats configuration fields into a .cfg text file
 *
 * @param None
 * @return {string} All the configuration inforamtion in text.
 */
Problem.prototype.getConfigText = function() {
    if (problem.isValid()) {
        var startQ = start_robot.quaternion;
        var goalQ = goal_robot.quaternion;

        var cfg = "";
        cfg += "[problem]\n";

        cfg += "name = " + problem.config["name"] + "\n";

        if ($('#problems').val() == "custom"){
            cfg += "robot = " + $("input[name='robot']")[0].files[0].name + "\n";
            cfg += "world = " + $("input[name='env']")[0].files[0].name + "\n";
        } else {
            cfg += "robot = " + problem.config["robot"] + "\n";
            cfg += "world = " + problem.config["world"] + "\n";
            // cfg += "robot = " + $("#problems").val() + "_robot.dae\n";
            // cfg += "world = " + $("#problems").val() + "_env.dae\n";
        }

        cfg += "objective = " + problem.config["objective"] + "\n";
        cfg += "objective.threshold = " + problem.config["objective.threshold"] + "\n";

        var control = problem.control[$("[name='robot.type']").val()];
        if (control != null) {
            cfg += "control = " + control + "\n";
        }

        cfg += "start.x = " + problem.config["start.x"] + "\n";
        cfg += "start.y = " + problem.config["start.y"] + "\n";
        cfg += "goal.x = " + problem.config["goal.x"] + "\n";
        cfg += "goal.y = " + problem.config["goal.y"] + "\n";

        cfg += "volume.min.x = " + problem.config["volume.min.x"] + "\n";
        cfg += "volume.min.y = " + problem.config["volume.min.y"] + "\n";
        cfg += "volume.max.x = " + problem.config["volume.max.x"] + "\n";
        cfg += "volume.max.y = " + problem.config["volume.max.y"] + "\n";

        if (problem.is3D == true) {
            cfg += "start.z = " + problem.config["start.z"] + "\n";
            cfg += "goal.z = " + problem.config["goal.z"] + "\n";

            cfg += "start.axis.x = " + startQ.x + "\n";
            cfg += "start.axis.y = " + startQ.y + "\n";
            cfg += "start.axis.z = " + startQ.z + "\n";
            cfg += "start.theta = " + startQ.w + "\n";

            cfg += "goal.axis.x = " + goalQ.x + "\n";
            cfg += "goal.axis.y = " + goalQ.y + "\n";
            cfg += "goal.axis.z = " + goalQ.z + "\n";
            cfg += "goal.theta = " + goalQ.w + "\n";

            cfg += "volume.min.z = " + problem.config["volume.min.z"] + "\n";
            cfg += "volume.max.z = " + problem.config["volume.max.z"] + "\n";
        } else {
            cfg += "start.theta = " + problem.config["start.yaw"] + "\n";
            cfg += "goal.theta = " + problem.config["goal.yaw"] + "\n";
        }



        cfg += "\n";
        cfg += "[benchmark]\n";
        cfg += "time_limit = " + $("[name='time_limit']").val() + "\n";
        // Set arbitrary, large mem limit
        cfg += "mem_limit = " + "10000\n"
        cfg += "run_count = " + $("[name='run_count']").val() + "\n";

        cfg += "\n";
        cfg += "[planner]\n";
        cfg += benchmark.getBenchmarkingPlanners();

        return cfg;
    } else {
        showAlert("configuration", "warning", "Please enter values for the indicated fields.");
        return null;
    }
}

Problem.prototype.parseConfigFile = function() {
    var cfgFile = $("#cfg-file")[0].files[0];

    if (cfgFile != null) {
        var reader = new FileReader();
        reader.readAsText(cfgFile);
        reader.onload = function () {
            var cfgData = {};
            // Separate into lines
            var cfgLines = reader.result.split("\n");
            for (var i=0; i < cfgLines.length; i++) {
                // Remove all extra spacing
                var line = cfgLines[i].replace(/\s+/g, '');
                if(line == ""){
                    continue;
                } else {
                    if(line[0] != "[") {
                        // Split into (key, value) pairs
                        var items = line.split("=");
                        if (items[1] == null){
                            throw "Invalid configuration on line containing: " + items[0];
                        } else {
                            cfgData[items[0]]= items[1];
                        }
                    } else {
                        // This config line isn't used
                        // console.log("Ignored: ", cfgLines[i]);
                    }
                }
            }
            problem.loadConfig(cfgData);
        }
    } else {
        showAlert("configuration", "danger", "Error parsing the configuration file, try again");
    }

}

Problem.prototype.loadConfig = function(data) {

    if (data['start.z'] != null) {
        this.is3D = true;
        show3DOptions();

        // Convert the rotation to degrees around each axis
        var startQ = axisAngleToQuaternion(data['start.axis.x'],
            data['start.axis.y'], data['start.axis.z'], data['start.theta']);
        var startRot = quaternionToAxisDegrees(startQ);

        var goalQ = axisAngleToQuaternion(data['goal.axis.x'],
            data['goal.axis.y'], data['goal.axis.z'], data['goal.theta']);
        var goalRot = quaternionToAxisDegrees(goalQ);

        // Set 3D only options
        $("[name='start.z']").val(data['start.z']);
        $("[name='start.deg.x']").val(startRot.x);
        $("[name='start.deg.y']").val(startRot.y);
        $("[name='start.deg.z']").val(startRot.z);
        $("[name='goal.z']").val(data['goal.z']);
        $("[name='goal.deg.x']").val(goalRot.x);
        $("[name='goal.deg.y']").val(goalRot.y);
        $("[name='goal.deg.z']").val(goalRot.z);
        $("[name='volume.min.z']").val(data['volume.min.z']);
        $("[name='volume.max.z']").val(data['volume.max.z']);
    } else {
        this.is3D = false;
        show2DOptions();

        var startRot = data['start.theta'] * RAD_TO_DEG;
        var goalRot = data['goal.theta'] * RAD_TO_DEG;

        $("[name='start.yaw']").val(startRot);
        $("[name='goal.yaw']").val(goalRot);
    }

    // Set common options for 3D and 2D problems
    $("[name='name']").val(data['name']);
    $("[name='start.x']").val(data['start.x']);
    $("[name='start.y']").val(data['start.y']);
    $("[name='goal.x']").val(data['goal.x']);
    $("[name='goal.y']").val(data['goal.y']);
    $("[name='volume.min.x']").val(data['volume.min.x']);
    $("[name='volume.min.y']").val(data['volume.min.y']);
    $("[name='volume.max.x']").val(data['volume.max.x']);
    $("[name='volume.max.y']").val(data['volume.max.y']);

    if (data['objective'] != null) {
        $("[name='objective']").val(data['objective']);
    }
    if (data['objective.threshold'] != null) {
        $("[name='objective.threshold']").val(data['objective.threshold']);
    }
    if (data['control'] != null) {
        var robotType = problem.control[data['control']];
        if (robotType != null) {
            $("[name='robot.type']").val(robotType);
        }
    } else if (problem.is3D == true) {
        $("[name='robot.type']").val("GSE3RigidBodyPlanning");
    } else {
        $("[name='robot.type']").val("GSE2RigidBodyPlanning");
    }

    // Benchmarking
    $("[name='time_limit']").val(data['time_limit']);
    $("[name='mem_limit']").val(data['mem_limit']);
    $("[name='run_count']").val(data['run_count']);

    problem.config["robot"] = data["robot"];
    problem.config["world"] = data["world"];

    setTimeout(function() {
        visualization.updatePose();
        visualization.updateBounds();
    }, 500);

    // Load the correct type of planners
    if (problem.control.hasOwnProperty($("#robot_type").val()) == true) {
        loadPlanners("control");
    } else {
        loadPlanners("geometric");
    }
}

Problem.prototype.showConfigData = function(data) {

}

/**
 * Gets the config data and prompts the user to download it.
 *
 * @param None
 * @return None
 */
Problem.prototype.downloadConfig = function(field) {
    var cfg = this.getConfigText();
    if (cfg != null) {
        var blob = new Blob([cfg], {type: "octet/stream"});
        var cfgName = $("[name='name']").val() + ".cfg";
        downloadFile(blob, cfgName);
    }
};


// Define the Solution class
var Solution = function() {
}

/**
 * Polls the server at an interval to check for problem solution. Continues
 * polling until a solution has been found or an error has been returned.
 *
 * @param {string} taskID The ID of the celery task which is solving the problem.
 * @return None
 */
Solution.prototype.poll = function(taskID) {
    var completed = false;
    var pollURL = '/poll/' + taskID;

    pollingInterval = window.setInterval(function() {

        $.ajax({
            url: pollURL,
            type: 'POST',
            data: taskID,
            success: function (data, textStatus, jqXHR) {
                if (jqXHR.status == 200) {
                    // Stop polling
                    clearInterval(pollingInterval);
                    solution.store(data);
                    solution.visualize();
                } else if (jqXHR.status == 202) {
                    // console.log("Polled, not ready yet.");
                } else {
                    console.info(data, textStatus, jqXHR);
                }
            },
            error: function (jqXHR, textStatus, errorThrown) {
                // Stop polling
                clearInterval(pollingInterval);
                $.unblockUI();
                showAlert("configuration", "danger", "Server responded with an error. Check the problem configuration and try again.");
                console.log('Solve failed, server responded with an error.', jqXHR, textStatus, errorThrown);
            }
        });

    }, POLL_INTERVAL);
};

Solution.prototype.store = function(solutionData) {
    this.data = JSON.parse(solutionData);
};

Solution.prototype.clear = function() {
    this.data = null;
    clearInterval(robotAnimationInterval);
    visualization.clearSolution();
    $('#pathButtons').addClass('hidden');
}

/**
 * Parses solution JSON from server and displays solution data.
 *
 * @param {string} data The solution data from the server as a JSON string
 * @return None
 */
Solution.prototype.visualize = function() {
    // Hide the bounding box
    $('#showBoundingBox').click();
    bbox.visible = false;

    // Clear the old solution visualization, if it existed
    visualization.clearSolution();

    if (this.data.multiple === "true") {
        var numSolved = 0;

        // Path animation options are not shown for multiple runs
        $('#pathButtons').addClass('hidden');

        $.each(this.data.solutions, function(index, run) {
            if (run.solved === "true") {
                visualization.drawSolutionPath(run.path);
                numSolved += 1;
            }
        });

        var msg = "Solutions found for " + numSolved + " of " + this.data.solutions.length + " runs.";
        showAlert("configuration", "success", msg);

    } else {
        if (this.data.solved == "true") {
            // Draw the solution path
            visualization.visualizeSolution(this.data);

            visualization.animationSpeed = 1000 - $('#animationSpeed').val();

            if ($('#pathButtons').hasClass('hidden')){
                $('#pathButtons').removeClass('hidden');
            } else {
                if ($('#animateToggleBtn').hasClass('active')) {
                    visualization.showAnimation();
                } else if ($('#toggleRobotPathBtn').hasClass('active')){
                    visualization.showRobotPath();
                }
            }

            var msg = this.data.status + " found."
            showAlert("configuration", "success", msg);
        } else {
            showAlert("configuration", "info", "No solution found. Try solving again.");
        }

        $("#showExplored").change(function() {
            if ($('#showExplored').prop('checked') == true) {
                visualization.showExploredStates();
            } else {
                visualization.hideExploredStates();
            }
        })
    }

    $.unblockUI();
};

/**
 * If a solution has been found, allows the user to download the path.
 *
 * @param None
 * @return None
 */
Solution.prototype.downloadSolutionPath = function() {
    if (this.data.pathAsMatrix != null) {
        var blob = new Blob([this.data.pathAsMatrix], {type: "octet/stream"});
        var pathName = this.data.name + "_path.txt";

        downloadFile(blob, pathName);
    } else {
        showAlert("configuration", "warning", "There is no valid solution path to download.");
    }
};


$(document).ready(function() {
    problem = new Problem();
    solution = new Solution();
    visualization = new Visualization();
    benchmark = new Benchmark();

    loadPreferences();

    initialize();
});


function loadPreferences() {
    $.get("/preferences", function(data) {
        var preferences = JSON.parse(data);

        BENCHMARKING_LIMIT = parseFloat(preferences["benchmarking_limit"]);
        MAX_UPLOAD_SIZE = parseFloat(preferences["max_upload_size"]);
        PLANNERARENA_PORT = parseFloat(preferences["plannerarena_port"])
        POLL_INTERVAL = parseFloat(preferences["poll_interval"])
    });
}


/**
 * Loads the components of the configuration page and sets up listeners
 * that make the page interactive.
 *
 * @param None
 * @return None
 */
function initialize() {
    // Load the configuration page by default
    $('#configuration-page').click();
    $("#configuration").load("components/configuration", function () {

        visualization.initialize();
        benchmark.initialize();

        // Retrieve the planners
        getPlannerData();

        // Retrieve robot types
        getRobotTypes();

        // Retrieve the preconfigured problems
        getProblems();

        // If this is a new session, get the session id
        if (!sessionStorage.getItem("session_id")){
            getSessionID();
        }

        // When user picks a planner, load the planner params
        $("#planners").change(function() {
            planner_name = $("#planners").val();
            loadPlannerParams(planner_name);
        });

        // Load config data when .cfg file is selected
        $("#cfg-file").change(function (){
            problem.parseConfigFile();
        });

        // Show upload buttons if user selects 'Custom' problem
        $("#problems").change(function() {
            visualization.clearScene();
            if ($("#problems").val() == 'custom'){
                $("#customProblem").collapse('show');

            } else {
                $("#customProblem").collapse('hide');

                // Retrieve config data for this problem
                loadRemoteProblem($("#problems").val());

                    // Load the correct type of planners
                    if (problem.control.hasOwnProperty($("#robot_type").val()) == true) {
                        loadPlanners("control");
                    } else {
                        loadPlanners("geometric");
                    }
            }
        });

        // Show 3D or 2D options, depending on the robot type
        $("#robot_type").change(function() {
            if (problem.robots2D.indexOf($("#robot_type").val()) > -1){
                problem.is3D = false;
                show2DOptions();
            } else {
                problem.is3D = true;
                show3DOptions();
            }

            // Load the correct type of planners
            if (problem.control.hasOwnProperty($("#robot_type").val()) == true) {
                loadPlanners("control");
            } else {
                loadPlanners("geometric");
            }
        });

        // Open the problem config tab
        $('#problem-tab').click()

        // Refresh the viz if pose fields are changed
        $('.pose').change(function () {
            visualization.updatePose();
        });

        // Refresh the viz if the bounds are changed
        $('.bounds').change(function () {
            visualization.updateBounds();
        });

        // Select Visualization Theme
        if (localStorage.getItem("clear_color") != null) {
            $('#vizTheme').val(localStorage.getItem("clear_color"));
        }
        $('#vizTheme').change(function() {
            var color = $('#vizTheme').val();
            if (color == "light") {
                renderer.setClearColor(LIGHT);
                localStorage.setItem("clear_color", "light");
            } else {
                renderer.setClearColor(DARK);
                localStorage.setItem("clear_color", "dark");
            }
        })

        // Toggle display of bounding box
        $('#showBoundingBox').change(function() {
            if ($('#showBoundingBox').prop('checked') == true) {
                bbox.visible = true;
            } else {
                bbox.visible = false;
            }
        });

        // Toggle display of axis helper
        if (localStorage.getItem("show_axis") != null) {
            if (localStorage.getItem("show_axis") == "true") {
               $('#showAxisHelper').prop("checked", true);
            } else {
               $('#showAxisHelper').prop("checked", false);
            }
        }
        $('#showAxisHelper').change(function() {
            if ($('#showAxisHelper').prop('checked') == true) {
                axisHelper.visible = true;
                localStorage.setItem("show_axis", true);
            } else {
                axisHelper.visible = false;
                localStorage.setItem("show_axis", false);
            }
        });

        // Adjust animation speed
        $('#animationSpeed').change(function() {
            visualization.animationSpeed = 1000 - $("#animationSpeed").val();
            $('#animateToggleBtn').click();
            $('#animateToggleBtn').click();
        });

        // Load the about page
        $("#about").load("components/about");

        // Activate popovers and tooltips
        $('[data-toggle="popover"]').popover()
        $('[data-toggle="tooltip"]').tooltip();
    });
}


/**
 * Retrieves planners from the server
 *
 * @param None
 * @return None
 */
function getPlannerData() {
    $.ajax({
        url: "planners",
        type: 'GET',
        success: function (data, textStatus, jqXHR) {
            problem.availablePlanners = JSON.parse(data);

        },
        error: function (jqXHR, textStatus, errorThrown) {
            console.log(jqXHR, textStatus, errorThrown);
        }
    });
}

/**
 * Retrieves robot types from the server and adds a selection dropdown to the
 * configuration page
 *
 * @param None
 * @return None
 */
function getRobotTypes() {
    $.get( "robot_types", function( data ) {
        problem.robotTypes = JSON.parse(data);

        // Populate the list of available robot types
        $.each(problem.robotTypes, function(type){
            $('#robot_type').append($("<option></option>").attr("value", type).text(problem.robotTypes[type]["name"]));
        });
        // Load the correct type of planners
        if (problem.control.hasOwnProperty($("#robot_type").val()) == true) {
            loadPlanners("control");
        } else {
            loadPlanners("geometric");
        }
    });
}

/**
 * Retrieves preconfigured problems from the server and adds them to the "Problem"
 * dropdown menu on the configuration page
 *
 * @param None
 * @return None
 */
function getProblems() {
    $.get( "/problems", function( data ) {
        var problems = JSON.parse(data);

        // Populate the list of available robot types
        $.each(problems, function(dimension, filenames){
            $('#problems').append($("<option></option>").attr("disabled", true).text(dimension));
            for (var i = 0; i < filenames.length; i++){
                name = filenames[i].split(".cfg")[0];
                $('#problems').append($("<option></option>").attr("value", name).text(name));

                problem.dimensions[name] = dimension;
            }
        });
        // Load up "cubicles" as the default problem
        $('#problems').val("cubicles");
        loadRemoteProblem("cubicles");
    });
}

/**
 * Gets the unique identifier for this session from the server and stores
 * it globally. This ID accompanies all future requests to the server
 * to ensure that the necessary files are available.
 *
 * @param None
 * @return None
 */
function getSessionID(){
    $.ajax({
        url: "session",
        type: 'GET',
        success: function (data, textStatus, jqXHR) {
            console.log("Got session id: " + data);
            sessionStorage.setItem("session_id", data);
        },
        error: function (jqXHR, textStatus, errorThrown) {
            showAlert("configuration", "danger", "Error getting session id. Please reload the page before continuing.");
            console.log(jqXHR, textStatus, errorThrown);
        },
    });
}

/**
 * Loads up the available planners on both the configure problem page and
 * benchmarking page.
 *
 * @param {String} kind Either 'geometric' or 'control', indicating which class of planners to load
 * @return None
 */
function loadPlanners(kind) {
    // Clear out the old planners
    $('#planners').html("");
    $('#addingPlanners').html("");

    // Add the planners to the page
    $.each(problem.availablePlanners[kind], function(fullName, data){
        var shortName = fullName.split(".")[2];

        // Configure problem page planners
        $('#planners').append($("<option></option>").attr("value", fullName).text(shortName));

        // Benchmarking page available planners
        $('#addingPlanners').append(
            $('<li></li>').append(
                $('<a></li>')
                    .attr("class", "dropdown-link")
                    .text(shortName)
                    .on("click", function() {
                        benchmark.addPlanner(fullName);
                    })
            )
        );
    });
    // Set KPIECE1 as the default planner
    loadPlannerParams("ompl." + kind + ".KPIECE1");
    $('#planners').val("ompl." + kind + ".KPIECE1");
}


/**
 * Given that the planners have been retrieved from the server, creates the
 * parameter fields for a specific planner and adds them to the page.
 *
 * @param {string} planner_name The planner to setup parameters for.
 * @return None
 */
function loadPlannerParams(plannerName) {
    var kind = plannerName.split(".")[1];
    var currentPlanners = problem.availablePlanners[kind]
    if (currentPlanners != null) {
        var plannerConfigHTML = "";
        plannerConfigHTML += "<form name='param_form'><table class='table'><caption>";
        plannerConfigHTML += plannerName.split(".")[2];
        plannerConfigHTML += " Options</caption><tbody>";
        params = currentPlanners[plannerName]
        for (var key in params) {
            if (params.hasOwnProperty(key)) {
                plannerConfigHTML += "<tr><td>";
                plannerConfigHTML += params[key][0];
                plannerConfigHTML += "</td><td><input type='text' name='" + key + "' class='planner_param form-control input-sm' value='" + params[key][3] + "'></td></tr>";
            }
        }
        plannerConfigHTML += "</tbody></table></form>"
        $("#plannerPane").html(plannerConfigHTML);
    } else {
        showAlert("configuration", "danger", "Planners are not loaded yet. Please refresh the page and try again.");
    }
}


/**
 * Loads a pre-defined problem from the server by drawing the models and
 * filling in configuration information.
 *
 * @param {string} problem_name The name of problem to load.
 * @return None
 */
function loadRemoteProblem(problemName) {
    var form = {};
    form["problem_name"] = problemName;
    form["dimension"] = problem.dimensions[problemName]

    // Retrieve problem configuration:
    $.ajax({
        url: "request_problem",
        type: 'POST',
        data: form,
        success: function (data, textStatus, jqXHR) {
            var data = JSON.parse(data);

            env_loc = data['env_loc'];
            robot_loc = data['robot_loc'];

            problem.config['env_loc']= data['env_loc'];
            problem.config['robot_loc'] = data['robot_loc'];

            // Load the robot and env models
            visualization.drawModels(data['env_loc'], data['robot_loc']);

            problem.loadConfig(data);
        },
        error: function (jqXHR, textStatus, errorThrown) {
            showAlert("configuration", "danger", "Error requesting problem. Please try again.");
            console.log(jqXHR, textStatus, errorThrown);
        }
    });
}


function show2DOptions() {
    var options3D = $(".3D");
    for (var i = 0; i < options3D.length; i++) {
        options3D[i].hidden = true;
    }
    var options2D = $(".2D");
    for (var i = 0; i < options2D.length; i++) {
        options2D[i].hidden = false;
    }
}

function show3DOptions() {
    var options3D = $(".3D");
    for (var i = 0; i < options3D.length; i++) {
        options3D[i].hidden = false;
    }
    var options2D = $(".2D");
    for (var i = 0; i < options2D.length; i++) {
        options2D[i].hidden = true;
    }
}


/**
 * Uploads the user's models to the server and then draws them to the scene.
 *
 * @param None
 * @return None
 */
function uploadModels() {
    // Read the input fields
    var formData = new FormData($('form')[0]); //TODO: Improve this, more robust selection
    formData.append('session_id', sessionStorage.getItem("session_id"));

    var valid = validateModels();

    if (valid) {
        // Send the request
        $.ajax({
            url: "upload_models",
            type: "POST",
            data: formData,
            success: function(data){
                data = JSON.parse(data);
                problem.config["env_loc"]= data['env_loc'];
                problem.config["robot_loc"] = data['robot_loc'];
                visualization.drawModels(data['env_loc'], data['robot_loc']);
            },
            error: function(data) {
                console.log(data);

                showAlert("configuration", "danger", "Unable to upload files.");
            },
            cache: false,
            contentType: false,
            processData: false
        });
    }
}


/**
 * Validates the user selected environment and robot files.
 *
 * @param None
 * @return {Boolean} A boolean indicating the validity of the files.
 */
function validateModels() {
    env_file = $('#env_path')[0].files[0];
    robot_file = $('#robot_path')[0].files[0];

    if (env_file != null && robot_file != null) {
        if (env_file.name.indexOf(".dae") > 0 && robot_file.name.indexOf(".dae") > 0) {
            if (env_file.size < MAX_UPLOAD_SIZE && robot_file.size < MAX_UPLOAD_SIZE) {
                return true;
            } else {
                var max_size = MAX_UPLOAD_SIZE / 1000000.0;
                var msg = "Robot and environment files must be smaller than " + max_size + " MB each."
                showAlert("configuration", "warning", msg);
            }
        } else {
            showAlert("configuration", "warning", "Robot and environment files must be in the .dae format.");
        }
    } else {
        showAlert("configuration", "warning", "Please select both robot and environment files in the .dae format.");
    }

    return false;
}


