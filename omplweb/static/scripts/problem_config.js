/* Global Variables */
var planners = null;
var results = "";
var intervalID;
var solutionData;
var animateRobot;
var animationSpeed;
var env_path;
var robot_path;


// Load the configuration page by default
$(document).ready(function() {
	$('#configure-problem-page').click();
	initialize();
});


/* Problem Configuration */

/**
 * Loads the components of the configuration page and sets up listeners to make
 * that make the page interactive.
 *
 * @param 	None
 * @return 	None
 */
function initialize() {
	$("#configure").load("omplapp/components/configuration", function () {
		// Get the visualization ready
		initViz();

		// Retrieve the planners:
		$.get( "omplapp/planners", function( data ) {
			planners = JSON.parse(data);
			// Set the default planner
			load_planner_params("ompl.geometric.KPIECE1");
		});

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
				$("#customProblem").collapse('show');
				clearScene();
				clearAllFields();
			} else {
				$("#customProblem").collapse('hide');

				// Retrieve config data for this problem
				loadRemoteProblem($("#problems").val());

			}
		});

		// Open the problem config tab
		$('#problem-tab').click()

		// If user previously solved a problem, reload those results
		$('#results').html(results);

		// Refresh the viz if pose fields are changed
		$('.pose').change(function () {
			updatePose();
		});

		// Refresh the viz if the bounds are changed
		$('.bounds').change(function () {
			updateBounds();
		});

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
			console.log(animationSpeed);
		});

		initializeBenchmarking();
	});
}


/* Server Interaction */

/**
 * Given that the planners have been retrieved from the server, creates the
 * parameter fields for a specific planner and adds them to the page.
 *
 * @param 	{String} planner_name The planner to setup parameters for.
 * @return 	None
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
				plannerConfigHTML += "</td><td><input type='text' name='" + key +  "' class='planner_param form-field form-control input-sm' value='" + params[key][3] + "'></td></tr>";
			}
		}
		plannerConfigHTML += "</tbody></table></form>"
		$("#plannerPane").html(plannerConfigHTML);
	} else {
		showAlert("configure", "error", "Planners are not loaded yet. Please wait and try again.");
	}
}


/**
 * Loads a pre-defined problem from the server by drawing the models and
 * filling in configuration information.
 *
 * @param 	{String} problem_name The name of problem to load.
 * @return 	None
 */
function loadRemoteProblem(problem_name) {
	// Retrieve problem configuration:
	var url = "omplapp/problem/" + problem_name;
	$.get(url, function(data) {
		var data = JSON.parse(data);
		env_path = data['env_path'];
		robot_path = data['robot_path'];

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
		// $("[name='start.theta']").val(data['start.theta']);
		$("[name='goal.x']").val(data['goal.x']);
		$("[name='goal.y']").val(data['goal.y']);
		$("[name='goal.z']").val(data['goal.z']);
		$("[name='goal.axis.x']").val(goalRot.x);
		$("[name='goal.axis.y']").val(goalRot.y);
		$("[name='goal.axis.z']").val(goalRot.z);
		// $("[name='goal.theta']").val(data['goal.theta']);
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


	});
}


/**
 * Uploads the user's models to the server and then draws them to the scene.
 *
 * @param 	None
 * @return 	None
 */
function uploadModels() {
	// Read the input fields
	var formData = new FormData($('form')[0]);

	var valid = validateFiles();

	if (valid) {
		// Send the request
		$.ajax({
			url: "omplapp/upload_models",
			type: "POST",
			data: formData,
			success: function(data){
				data = JSON.parse(data);
				env_path = data['env_path'];
				robot_path = data['robot_path'];

				drawModels(data['env_loc'], data['robot_loc']);

				$('#uploadModelsButton').addClass('disabled');
			},
			error: function(data) {
				console.log(data);

				showAlert("configure", "error", "Unable to upload files.");
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
 * @param 	None
 * @return 	None
 */
function loadConfig() {
	var cfgFile = $("#config")[0].files[0];

	if (cfgFile != null) {
		var reader = new FileReader();

		reader.readAsText(cfgFile);

		reader.onload = function () {
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
			// $("[name='start.theta']").val(data['start.theta']);
			$("[name='goal.x']").val(data['goal.x']);
			$("[name='goal.y']").val(data['goal.y']);
			$("[name='goal.z']").val(data['goal.z']);
			$("[name='goal.axis.x']").val(goalRot.x);
			$("[name='goal.axis.y']").val(goalRot.y);
			$("[name='goal.axis.z']").val(goalRot.z);
			// $("[name='goal.theta']").val(data['goal.theta']);
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
		}
	} else {
		showAlert("configure", "warning" , "Please select a valid configuration file.");
	}
}


/**
 * Parses a configuration text file into a mapping.
 *
 * @param 	{String} cfgText A configuration file in string form.
 * @return  {Object} cfgData An object mapping configuration fields to values.
 */
function parseConfig(cfgText) {
	var cfgData = {};

	// Separate into lines
	var cfgLines = cfgText.split("\n");

	for (var i=0; i < cfgLines.length; i++) {
		if(cfgLines[i][0] != "[") {
			// Remove all extra spacing
			var line = cfgLines[i].replace(/\s+/g, '');

			// Split into (key, value) pairs
			var items = line.split("=");
			cfgData[items[0]]= items[1];
		} else {
			// This config line isn't used
			// console.log("Ignored: ", cfgLines[i]);
		}
	}

	return cfgData;
}

/**
 * Formats configuration fields into a .cfg text file
 *
 * @param 	None
 * @return 	{String} cfg All the configuration inforamtion in text.
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
		cfg += "mem_limit = " + $("[name='mem_limit']").val() + "\n";
		cfg += "run_count = " + $("[name='run_count']").val() + "\n";

		cfg += "\n";
		cfg += "[planner]\n";
		cfg += getBenchmarkingPlanners();

		return cfg;
	} else {
		showAlert("configure", "warning", "Please enter values for the indicated fields.");
		return null;
	}
}


/**
 * Gets the config data and prompts the user to download it.
 *
 * @param 	None
 * @return 	None
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
 * @param 	None
 * @return 	None
 */
function clearAllFields() {
	$('.form-field').each(function () {
		$(this).val('');
		$(this).css("background-color", "white");
	});
	$('#config').val('');
	$('#results').html('');
	$('#pathButtons').collapse('hide');
	results = "";
	solutionData = null;
	clearScene();
}


/**
 * Validates all required fields.
 *
 * @param 	None
 * @return 	{Boolean} valid A boolean indicating form validity.
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
 * @param 	None
 * @return 	{Boolean} valid A boolean indicating the validity of the files.
 */
function validateFiles() {
	env_file = $('#env_path')[0].files[0];
	robot_file = $('#robot_path')[0].files[0];

	if (env_file != null && robot_file != null) {
		if (env_file.name.indexOf(".dae") > 0 && robot_file.name.indexOf(".dae") > 0) {
			return true;
		} else {
			alert('error;');
			showAlert("configure", "warning", "Robot and environment files must be in the .dae format.");
		}
	} else {
		showAlert("configure", "warning", "Please select both robot and environment files in the .dae format.");
	}

	return false;
}


/* Solve and Results */

/**
 * Gathers and formats problem data and submits the problem to the server for solving.
 * On successful solve, saves solution data and loads solution visualization.
 *
 * @param 	None
 * @return 	None
 */
function solve(){
	// Check that all fields are filled in
	var validConfig = validateFields();
	if (validConfig == true) {
		var html = "";

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

		{
			// Read the input fields
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

			problemData['env_path'] = env_path;
			problemData['robot_path'] = robot_path;
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
				html += "<pre>Server responded with an error. Check the problem configuration and try again.</pre>";
				$('#results').html(html);

				console.log('Solve failed, server responded with an error.', data);
			},
			cache: false,
			contentType: 'application/json',
			processData: false
		});
	} else {
		// Invalid fields have been highlighted by 'validateField()'.
		showAlert("configure", "warning", "Please enter values for the indicated fields.");
	}
}

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

	}, 1000);
}


function displaySolution(data) {
	solutionData = JSON.parse(data);
	console.log(solutionData);

	var html = "";

	if (solutionData.solved == "true") {
		// Draw the solution path
		visualizePath(solutionData);
		animationSpeed = 1000 - $('#animationSpeed').val();
		$('#pathButtons').collapse('show');
		showAlert("configure", "success", "Solution found!");
		// html += "<br><h4><font color='#329B71'>Found solution.</font></h4>";
	} else {
		showAlert("configure", "info", "No solution found. Try solving again.");
		// html += "<font color='#cd535a'>No solution found. To try again, click the solve button.</font><br><br>";
	}

	html += "<br><br><pre>"
	html += solutionData.name;
	html += "<br>";
	html += solutionData.messages;
	html += "<br><br>"

	// Uncomment to display list of path states
	// html += solutionData.path;

	html += "</pre>";

	results = html;

	$('#results').html(html);

	$.unblockUI();

}


/**
 * Toggles the animation of robot along solution path.
 *
 * @param 	None
 * @return 	None
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
 * @param 	None
 * @return 	None
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
 * @param 	None
 * @return 	None
 */
function downloadPath() {

	if (solutionData != null) {
		var blob = new Blob([solutionData.pathAsMatrix], {type: "octet/stream"});
		var pathName = $("[name='name']").val() + "_path.txt";

		// TODO: Check that solution exists first
		downloadFile(blob, pathName);
	} else {
		showAlert("configure", "warning", "There is no valid solution path to download.");
	}
}


/* Helper Functions */

/**
 * Prompts the user to download a file.
 *
 * @param 	{Blog} blob The file blob to download
 * @param 	{String} name The filename
 * @return 	None
 */
function downloadFile(blob, name) {
	var url = window.URL.createObjectURL(blob);
	var a = document.createElement("a");
	document.body.appendChild(a);
	a.style = "display: none";
	a.href = url;
	a.download = name;
	a.click();
	window.URL.revokeObjectURL(url);
}


function getConfiguredPlanner() {
	var planner = {};
	planner['name'] = $("[name='planners']").val();

	// Get the params for the specific planner
	paramData = {};
	$('.planner_param').each(function () {
		paramData[$(this).attr('name')] = $(this).val();
	});

	planner['parameters'] = paramData;

	return planner;
}
