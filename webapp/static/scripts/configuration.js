/* Global Variables */
var planners = null;
var results = "";
var intervalID;
var solutionData;
var animateRobot;
var animationSpeed;
var env_loc;
var robot_loc;
var MAX_UPLOAD_SIZE = 50000000; // In bytes, sets limit to 50MB

/* Load the configuration page */
$(document).ready(function() {
	$('#configuration-page').click();
	initialize();
});


/* Problem Configuration */

/**
 * Loads the components of the configuration page and sets up listeners
 * that make the page interactive.
 *
 * @param None
 * @return None
 */
function initialize() {
	$("#configuration").load("omplapp/components/configuration", function () {
		// Get the visualization ready
		initViz();

		initializeBenchmarking();

		// Retrieve the planners:
		loadPlanners();

		// When user picks planner, load the planner params
		$("#planners").change(function() {
			planner_name = $("#planners").val();
			load_planner_params(planner_name);
		});

		// Load config data when .cfg file is selected
		$("#config").change(function (){
			loadConfig();
		});

		// Show upload buttons if user selects 'Custom' problem
		$("#problems").change(function() {


			if ($("#problems").val() == 'custom'){
				clearAllFields();

				$("#customProblem").collapse('show');
				load_planner_params("ompl.geometric.KPIECE1");
			} else {
				$("#customProblem").collapse('hide');

				clearAllFields();

				// Retrieve config data for this problem
				loadRemoteProblem($("#problems").val());
				load_planner_params("ompl.geometric.KPIECE1");

			}
		});

		// Open the problem config tab
		$('#problem-tab').click()

		// Refresh the viz if pose fields are changed
		$('.pose').change(function () {
			updatePose();
		});

		// Refresh the viz if the bounds are changed
		$('.bounds').change(function () {
			updateBounds();
		});

		// Select Visualization Theme
		$('#vizTheme').change(function() {
			var color = $('#vizTheme').val();
			if (color == "light") {
				renderer.setClearColor(0xfafafa);
			} else {
				renderer.setClearColor(0x1a1a1a);
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
		$('#showAxisHelper').change(function() {
			if ($('#showAxisHelper').prop('checked') == true) {
				axisHelper.visible = true;
			} else {
				axisHelper.visible = false;
			}
		});

		// Adjust animation speed
		$('#animationSpeed').change(function() {
			animationSpeed = 1000 - $('#animationSpeed').val();
			$('#animateToggleBtn').click();
			$('#animateToggleBtn').click();
		});


		// Load the about page
		$("#about").load("omplapp/components/about");
	});
}


/* Set and Get Server Data */

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
		url: 'omplapp/session',
		type: 'GET',
		success: function (data, textStatus, jqXHR) {
			console.log("Got session id: " + data);
			sessionStorage.setItem("session_id", data);
		},
		error: function (jqXHR, textStatus, errorThrown) {
			console.log("Error getting session id.");
			console.log(jqXHR, textStatus, errorThrown);
		},
		async: false
	});
}


/**
 * Retrieves planners from the server and loads up the available planners on both
 * the configure problem page and benchmarking page
 *
 * @param None
 * @return None
 */
function loadPlanners() {
	$.get( "omplapp/planners", function( data ) {
		planners = JSON.parse(data);

		$.each(planners, function(fullName, data){
			var shortName = fullName.split(".")[2];

			// Configure problem page planners
			$('#planners').append($("<option></option>").attr("value", fullName).text(shortName));

			// Benchmarking page add planners
			$('#addingPlanners').append(
				$('<li></li>').append(
					$('<a></li>')
						.attr("class", "dropdown-link")
						.text(shortName)
						.on("click", function() {
							addPlanner(fullName);
						})
				)
			);

		});

		// Set the default planner
		load_planner_params("ompl.geometric.KPIECE1");
		$('#planners').val("ompl.geometric.KPIECE1")
	});
}


/**
 * Given that the planners have been retrieved from the server, creates the
 * parameter fields for a specific planner and adds them to the page.
 *
 * @param {string} planner_name The planner to setup parameters for.
 * @return None
 */
function load_planner_params(planner_name) {
	if (planners != null) {
		var plannerConfigHTML = "";
		plannerConfigHTML += "<form name='param_form'><table class='table'><caption>";
		plannerConfigHTML += planner_name.split(".")[2];
		plannerConfigHTML += " Options</caption><tbody>";
		params = planners[planner_name]
		for (var key in params) {
			if (params.hasOwnProperty(key)) {
				plannerConfigHTML += "<tr><td>";
				plannerConfigHTML += params[key][0];
				plannerConfigHTML += "</td><td><input type='text' name='" + key + "' class='planner_param form-field form-control input-sm' value='" + params[key][3] + "'></td></tr>";
			}
		}
		plannerConfigHTML += "</tbody></table></form>"
		$("#plannerPane").html(plannerConfigHTML);
	} else {
		showAlert("configuration", "danger", "Planners are not loaded yet. Please wait and try again.");
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
	var form = {'problem_name' : problemName};

	// Retrieve problem configuration:
	$.ajax({
		url: "omplapp/request_problem",
		type: 'POST',
		async: false,
		data: form,
		success: function (data, textStatus, jqXHR) {
			var data = JSON.parse(data);
			env_loc = data['env_loc'];
			robot_loc = data['robot_loc'];

			var startQ = axisAngleToQuaternion(data['start.axis.x'],
				data['start.axis.y'], data['start.axis.z'], data['start.theta']);
			var startRot = quaternionToAxisDegrees(startQ);

			var goalQ = axisAngleToQuaternion(data['goal.axis.x'],
				data['goal.axis.y'], data['goal.axis.z'], data['goal.theta']);
			var goalRot = quaternionToAxisDegrees(goalQ);

			// Load the data
			$("[name='name']").val(data['name']);
			$("[name='start.x']").val(data['start.x']);
			$("[name='start.y']").val(data['start.y']);
			$("[name='start.z']").val(data['start.z']);
			$("[name='start.axis.x']").val(startRot.x);
			$("[name='start.axis.y']").val(startRot.y);
			$("[name='start.axis.z']").val(startRot.z);
			$("[name='goal.x']").val(data['goal.x']);
			$("[name='goal.y']").val(data['goal.y']);
			$("[name='goal.z']").val(data['goal.z']);
			$("[name='goal.axis.x']").val(goalRot.x);
			$("[name='goal.axis.y']").val(goalRot.y);
			$("[name='goal.axis.z']").val(goalRot.z);
			$("[name='volume.min.x']").val(data['volume.min.x']);
			$("[name='volume.min.y']").val(data['volume.min.y']);
			$("[name='volume.min.z']").val(data['volume.min.z']);
			$("[name='volume.max.x']").val(data['volume.max.x']);
			$("[name='volume.max.y']").val(data['volume.max.y']);
			$("[name='volume.max.z']").val(data['volume.max.z']);

			$("[name='time_limit']").val(data['time_limit']);
			$("[name='mem_limit']").val(data['mem_limit']);
			$("[name='run_count']").val(data['run_count']);

			// Load the robot and env models
			drawModels(data['env_loc'], data['robot_loc']);

			// Give the models time to load, then update the positions and rotations
			setTimeout(function() {
				updatePose();
				updateBounds();
			}, 500);
		},
		error: function (jqXHR, textStatus, errorThrown) {
			console.log("Error requesting problem.");
			console.log(jqXHR, textStatus, errorThrown);
		}
	});
}


/**
 * Uploads the user's models to the server and then draws them to the scene.
 *
 * @param None
 * @return None
 */
function uploadModels() {
	// Read the input fields
	var formData = new FormData($('form')[0]);
	if (!sessionStorage.getItem("session_id")){
		getSessionID();
		formData.append('session_id', sessionStorage.getItem("session_id"));
	} else {
		formData.append('session_id', sessionStorage.getItem("session_id"));
	}

	var valid = validateFiles();

	if (valid) {
		// Send the request
		$.ajax({
			url: "omplapp/upload_models",
			type: "POST",
			data: formData,
			success: function(data){
				data = JSON.parse(data);
				env_loc = data['env_loc'];
				robot_loc = data['robot_loc'];
				drawModels(data['env_loc'], data['robot_loc']);

				$('#uploadModelsButton').addClass('disabled');
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


/* Configuration */

/**
 * Reads the user selected config file and loads values into config fields.
 *
 * @param None
 * @return None
 */
function loadConfig() {
	var cfgFile = $("#config")[0].files[0];

	if (cfgFile != null) {
		var reader = new FileReader();

		reader.readAsText(cfgFile);

		reader.onload = function () {
			try {
				var data = parseConfig(reader.result);

				var startQ = axisAngleToQuaternion(data['start.axis.x'],
					data['start.axis.y'], data['start.axis.z'], data['start.theta']);
				var startRot = quaternionToAxisDegrees(startQ);

				var goalQ = axisAngleToQuaternion(data['goal.axis.x'],
					data['goal.axis.y'], data['goal.axis.z'], data['goal.theta']);
				var goalRot = quaternionToAxisDegrees(goalQ);

				// Load the data
				$("[name='name']").val(data['name']);
				$("[name='start.x']").val(data['start.x']);
				$("[name='start.y']").val(data['start.y']);
				$("[name='start.z']").val(data['start.z']);
				$("[name='start.axis.x']").val(startRot.x);
				$("[name='start.axis.y']").val(startRot.y);
				$("[name='start.axis.z']").val(startRot.z);
				$("[name='goal.x']").val(data['goal.x']);
				$("[name='goal.y']").val(data['goal.y']);
				$("[name='goal.z']").val(data['goal.z']);
				$("[name='goal.axis.x']").val(goalRot.x);
				$("[name='goal.axis.y']").val(goalRot.y);
				$("[name='goal.axis.z']").val(goalRot.z);
				$("[name='volume.min.x']").val(data['volume.min.x']);
				$("[name='volume.min.y']").val(data['volume.min.y']);
				$("[name='volume.min.z']").val(data['volume.min.z']);
				$("[name='volume.max.x']").val(data['volume.max.x']);
				$("[name='volume.max.y']").val(data['volume.max.y']);
				$("[name='volume.max.z']").val(data['volume.max.z']);

				$("[name='time_limit']").val(data['time_limit']);
				$("[name='mem_limit']").val(data['mem_limit']);
				$("[name='run_count']").val(data['run_count']);

				setTimeout(function() {
					updatePose();
					updateBounds();
				}, 100);
			} catch (e) {
				showAlert("configuration", "danger", "There was a problem parsing the configuration file.")
				console.log(e);
			}
		}
	} else {
		showAlert("configuration", "warning" , "Please select a valid configuration file.");
	}
}


/**
 * Parses a configuration text file into a mapping.
 *
 * @param {string} cfgText A configuration file in string form.
 * @return {Object} An object mapping configuration fields to values.
 */
function parseConfig(cfgText) {
	var cfgData = {};

	// Separate into lines
	var cfgLines = cfgText.split("\n");

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

	return cfgData;
}


/**
 * Formats configuration fields into a .cfg text file
 *
 * @param None
 * @return {string} All the configuration inforamtion in text.
 */
function getConfigText() {

	if (validateFields()) {
		var startQ = start_robot.quaternion;
		var goalQ = goal_robot.quaternion;

		var cfg = "";
		cfg += "[problem]\n";

		cfg += "name = " + $("[name='name']").val() + "\n";
		cfg += "robot = " + $("[name='name']").val() + "_robot.dae\n";
		cfg += "world = " + $("[name='name']").val() + "_env.dae\n";

		cfg += "start.x = " + $("[name='start.x']").val() + "\n";
		cfg += "start.y = " + $("[name='start.y']").val() + "\n";
		cfg += "start.z = " + $("[name='start.z']").val() + "\n";

		cfg += "start.axis.x = " + startQ.x + "\n";
		cfg += "start.axis.y = " + startQ.y + "\n";
		cfg += "start.axis.z = " + startQ.z + "\n";
		cfg += "start.theta = " + startQ.w + "\n";

		cfg += "goal.x = " + $("[name='goal.x']").val() + "\n";
		cfg += "goal.y = " + $("[name='goal.y']").val() + "\n";
		cfg += "goal.z = " + $("[name='goal.z']").val() + "\n";

		cfg += "goal.axis.x = " + goalQ.x + "\n";
		cfg += "goal.axis.y = " + goalQ.y + "\n";
		cfg += "goal.axis.z = " + goalQ.z + "\n";
		cfg += "goal.theta = " + goalQ.w + "\n";

		cfg += "volume.min.x = " + $("[name='volume.min.x']").val() + "\n";
		cfg += "volume.min.y = " + $("[name='volume.min.y']").val() + "\n";
		cfg += "volume.min.z = " + $("[name='volume.min.z']").val() + "\n";
		cfg += "volume.max.x = " + $("[name='volume.max.x']").val() + "\n";
		cfg += "volume.max.y = " + $("[name='volume.max.y']").val() + "\n";
		cfg += "volume.max.z = " + $("[name='volume.max.z']").val() + "\n";

		cfg += "\n";
		cfg += "[benchmark]\n";
		cfg += "time_limit = " + $("[name='time_limit']").val() + "\n";
		// Set arbitrary, large mem limit
		cfg += "mem_limit = " + "10000\n"
		cfg += "run_count = " + $("[name='run_count']").val() + "\n";

		cfg += "\n";
		cfg += "[planner]\n";
		cfg += getBenchmarkingPlanners();

		return cfg;
	} else {
		showAlert("configuration", "warning", "Please enter values for the indicated fields.");
		return null;
	}
}


/**
 * Gets the config data and prompts the user to download it.
 *
 * @param None
 * @return None
 */
function downloadConfig() {

	var cfg = getConfigText();
	if (cfg != null) {
		var blob = new Blob([cfg], {type: "octet/stream"});
		var cfgName = $("[name='name']").val() + ".cfg";
		downloadFile(blob, cfgName);
	}
}


/* Data Validation */

/**
 * Clears all configuration fields and removes all objects from the scene by
 * calling 'clearScene()'.
 *
 * @param None
 * @return None
 */
function clearAllFields() {
	$('.form-field').each(function () {
		$(this).val('');
		$(this).css("background-color", "white");
	});
	$('#config').val('');
	$('#results').html('');
	$('#pathButtons').addClass('hidden');
	results = "";
	solutionData = null;
	clearScene();
}


/**
 * Validates all required fields.
 *
 * @param None
 * @return {Boolean} valid A boolean indicating form validity.
 */
function validateFields() {
	var valid = true;

	// Ensure that the all the needed fields for a problem have values
	$('.form-field').each(function () {
		if ($(this).val() === '') {
			valid = false;

			// Highlight the incorrect field
			$(this).css("background-color", "#fec7c1");
		} else {
			$(this).css("background-color", "white");
		}
	});
	return valid;
}


/**
 * Validates the user selected environment and robot files.
 *
 * @param None
 * @return {Boolean} A boolean indicating the validity of the files.
 */
function validateFiles() {
	env_file = $('#env_path')[0].files[0];
	robot_file = $('#robot_path')[0].files[0];

	if (env_file != null && robot_file != null) {
		if (env_file.name.indexOf(".dae") > 0 && robot_file.name.indexOf(".dae") > 0) {
			if (env_file.size < MAX_UPLOAD_SIZE && robot_file.size < MAX_UPLOAD_SIZE) {
				return true;
			} else {
				var max_size = MAX_UPLOAD_SIZE / 1000000;
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


/* Solve and Results */

/**
 * Gathers and formats problem data and submits the problem to the server for solving.
 * On successful solve, saves solution data and loads solution visualization.
 *
 * @param None
 * @return None
 */
function solve(){
	// Check that all fields are filled in
	var validConfig = validateFields();
	if (validConfig == true) {
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

		var startQ = start_robot.quaternion;
		var goalQ = goal_robot.quaternion;

		// Read the input fields
		{
			var problemData = {}
			problemData['name'] = $("[name='name']").val();
			problemData['start.x'] = $("[name='start.x']").val();
			problemData['start.y'] = $("[name='start.y']").val();
			problemData['start.z'] = $("[name='start.z']").val();
			problemData['start.q.x'] = startQ.x;
			problemData['start.q.y'] = startQ.y;
			problemData['start.q.z'] = startQ.z;
			problemData['start.q.w'] = startQ.w;
			problemData['goal.x'] = $("[name='goal.x']").val();
			problemData['goal.y'] = $("[name='goal.y']").val();
			problemData['goal.z'] = $("[name='goal.z']").val();
			problemData['goal.q.x'] = goalQ.x;
			problemData['goal.q.y'] = goalQ.y;
			problemData['goal.q.z'] = goalQ.z;
			problemData['goal.q.w'] = goalQ.w;
			problemData['volume.min.x'] = $("[name='volume.min.x']").val();
			problemData['volume.min.y'] = $("[name='volume.min.y']").val();
			problemData['volume.min.z'] = $("[name='volume.min.z']").val();
			problemData['volume.max.x'] = $("[name='volume.max.x']").val();
			problemData['volume.max.y'] = $("[name='volume.max.y']").val();
			problemData['volume.max.z'] = $("[name='volume.max.z']").val();
			problemData['solve_time'] = $("[name='solve_time']").val();
			problemData['planner'] = $("[name='planners']").val();
			problemData['env_loc'] = env_loc;
			problemData['robot_loc'] = robot_loc;


			if ($("[name='runs']").val() >= 1) {
				problemData['runs'] = $("[name='runs']").val();
			} else {
				problemData['runs'] = 1;
			}

		}

		// Get the params for the specific planner
		paramData = {};
		$('.planner_param').each(function () {
			paramData[$(this).attr('name')] = $(this).val();
		});

		problemData['planner_params'] = paramData;
		problemJSON = JSON.stringify(problemData);

		// Clear the old solution information, if there was one
		clearOldSolution();

		// Send the request
		$.ajax({
			url: "omplapp/upload",
			type: "POST",
			data: problemJSON,
			success: function(data){
				var taskID = String(data);
				console.log("Server successfully recieved solve request. Given task ID: " + taskID);
				waitForSolution(data);
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
}

/**
 * Polls the server at an interval to check for problem solution. Continues
 * polling until a solution has been found or an error has been returned.
 *
 * @param {string} taskID The ID of the celery task which is solving the problem.
 * @return None
 */
function waitForSolution(taskID) {
	var completed = false;
	var pollURL = '/omplapp/poll/' + taskID;

	intervalID = window.setInterval(function() {

		$.ajax({
			url: pollURL,
			type: 'POST',
			data: taskID,
			success: function (data, textStatus, jqXHR) {
				if (jqXHR.status == 200) {
					clearInterval(intervalID);
					displaySolution(data);
				} else {
					console.log(data, textStatus);
				}
			},
			error: function (jqXHR, textStatus, errorThrown) {
				$.unblockUI();
				html = "<pre>Server responded with an error. Check the problem configuration and try again.</pre>";
				$('#results').html(html);

				console.log('Solve failed, server responded with an error.', errorThrown);
			}
		});

	}, 2000);
}


/**
 * Parses solution JSON from server and displays solution data.
 *
 * @param {string} data The solution data from the server as a JSON string
 * @return None
 */
function displaySolution(data) {
	solutionData = JSON.parse(data);

	if (solutionData.multiple === "true") {
		var numSolved = 0;

		$.each(solutionData.solutions, function(index, solution) {
			if (solution.solved === "true") {
				drawSolutionPath(solution.path);
				numSolved += 1;
			}
		});

		var msg = "Solutions found for " + numSolved + " of " + solutionData.solutions.length + " runs.";
		showAlert("configuration", "success", msg);

	} else {
		if (solutionData.solved == "true") {
			// Draw the solution path
			visualizeSolution(solutionData);

			animationSpeed = 1000 - $('#animationSpeed').val();

			$('#pathButtons').removeClass('hidden');

			showAlert("configuration", "success", "Solution found!");
		} else {
			showAlert("configuration", "info", "No solution found. Try solving again.");
		}
	}

	$.unblockUI();

}


/**
 * Toggles the animation of robot along solution path.
 *
 * @param None
 * @return None
 */
function animateToggle() {
	if ($('#animateToggleBtn').hasClass('active')) {
		$('#animateToggleBtn').removeClass('active');

		clearInterval(animateRobot);
		path_robot.visible = false;
	} else {
		path_robot.visible = true;
		$('#animateToggleBtn').addClass('active');

		animateRobot = setInterval(function() {
			moveRobot(path);
		}, animationSpeed);
	}
}


/**
 * Toggles the display of static robots at each point along solution path.
 *
 * @param None
 * @return None
 */
function toggleRobotPath() {
	if ($('#toggleRobotPathBtn').hasClass('active')) {
		$('#toggleRobotPathBtn').removeClass('active');

		hideRobotPath();
	} else {
		$('#toggleRobotPathBtn').addClass('active');

		showRobotPath();
	}
}


/**
 * If a solution has been found, allows the user to download the path.
 *
 * @param None
 * @return None
 */
function downloadPath() {

	if (solutionData != null) {
		var blob = new Blob([solutionData.pathAsMatrix], {type: "octet/stream"});
		var pathName = $("[name='name']").val() + "_path.txt";

		downloadFile(blob, pathName);
	} else {
		showAlert("configuration", "warning", "There is no valid solution path to download.");
	}
}

