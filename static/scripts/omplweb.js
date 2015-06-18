
var planners = null;
var results = "";
var animateRobot;

$(document).ready(function() {
	load_configuration_page();
});

// Problem Configuration
function load_configuration_page () {
	// Unhighlight the old active tab
	$(".active_nav_item").removeClass('active_nav_item');

	// Load the HTML for the configuration settings
	$("#content").load("omplapp/components/configuration", function () {
		// Make config the active tab
		$('#nav_config').addClass('active_nav_item');

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
			updateViz();
		})
	});
}

// Server Interaction
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
		showPane('#plannerPane');
	} else {
		alert("Planners are not loaded yet. Please wait and try again.");
	}
}

function loadRemoteProblem(problem_name) {
	// Retrieve problem configuration:
	var url = "omplapp/problem/" + problem_name;
	$.get(url, function(data) {
		var data = JSON.parse(data);

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

		// Load the robot and env models
		drawModels(data['env_loc'], data['robot_loc']);

		// Give the models time to load, then update the positions and rotations
		setTimeout(function() {
			updateViz();
		}, 100);


	});
}


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
				drawModels(data['env_loc'], data['robot_loc']);
				loadDefaultPositions();
				$('#uploadModelsButton').addClass('disabled');
			},
			error: function(data) {
				console.log(data);
			},
			cache: false,
			contentType: false,
			processData: false
		});
	}
}

function loadDefaultPositions() {
	$('.pose').each(function () {
		$(this).val(0);
	});
	$("[name='start.x']").val(-50);
	$("[name='goal.x']").val(50);

	setTimeout(function() {
		updateViz();
	}, 100);
}



// Views Toggling
function togglePane(paneID) {
	// If this pane is visible
	if ($(paneID).hasClass('in')) {
		hidePane(paneID);
	} else {
		showPane(paneID)
	}
}

function hidePane(paneID) {
	// Hide the pane
	$(paneID).collapse('hide');
}

function showPane(paneID) {
	// Show the pane
	$(paneID).collapse('show');
}

// Configuration
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


			setTimeout(function() {
				updateViz();
			}, 100);
		}
	} else {
		alert("Please select a valid configuration file.")
	}
}

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

// Data Validation
function clearAllFields() {
	if (confirm("Are you sure you want to clear all fields?")) {
		$('.form-field').each(function () {
			$(this).val('');
			$(this).css("background-color", "white");
		});
		$('#config').val('');
		$('#results').html('');
		results = "";
	}
}

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

function validateFiles() {
	env_file = $('#env_path')[0].files[0];
	robot_file = $('#robot_path')[0].files[0];

	if (env_file != null && robot_file != null) {
		if (env_file.name.indexOf(".dae") > 0 && robot_file.name.indexOf(".dae") > 0) {
			return true;
		} else {
			alert("Robot and environment files must be in the .dae format.")
		}
	} else {
		alert("Please select both robot and environment files in the .dae format.")
	}

	return false;
}


// Solve and Results
function solve(){
	// Check that all fields are filled in
	var validConfig = validateFields();
	if (validConfig == true) {
		var html = "<br><br>";
		html += "<div class='pane-title'>Results</div>"

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
		var problemData = {}
		problemData['name'] = $("[name='name']").val();
		problemData['start.x'] = $("[name='start.x']").val();
		problemData['start.y'] = $("[name='start.y']").val();
		problemData['start.z'] = $("[name='start.z']").val();
		problemData['start.q.x'] = startQ.x;
		problemData['start.q.y'] = startQ.y;
		problemData['start.q.z'] = startQ.z;
		problemData['start.q.w'] = startQ.w;
		// problemData['start.axis.x'] = $("[name='start.axis.x']").val();
		// problemData['start.axis.y'] = $("[name='start.axis.y']").val();
		// problemData['start.axis.z'] = $("[name='start.axis.z']").val();
		// problemData['start.theta'] = $("[name='start.theta']").val();
		problemData['goal.x'] = $("[name='goal.x']").val();
		problemData['goal.y'] = $("[name='goal.y']").val();
		problemData['goal.z'] = $("[name='goal.z']").val();
		problemData['goal.q.x'] = goalQ.x;
		problemData['goal.q.y'] = goalQ.y;
		problemData['goal.q.z'] = goalQ.z;
		problemData['goal.q.w'] = goalQ.w;
		// problemData['goal.axis.x'] = $("[name='goal.axis.x']").val();
		// problemData['goal.axis.y'] = $("[name='goal.axis.y']").val();
		// problemData['goal.axis.z'] = $("[name='goal.axis.z']").val();
		// problemData['goal.theta'] = $("[name='goal.theta']").val();
		problemData['volume.min.x'] = $("[name='volume.min.x']").val();
		problemData['volume.min.y'] = $("[name='volume.min.y']").val();
		problemData['volume.min.z'] = $("[name='volume.min.z']").val();
		problemData['volume.max.x'] = $("[name='volume.max.x']").val();
		problemData['volume.max.y'] = $("[name='volume.max.y']").val();
		problemData['volume.max.z'] = $("[name='volume.max.z']").val();
		problemData['time_limit'] = $("[name='time_limit']").val();
		problemData['planner'] = $("[name='planners']").val();

		// Get the params for the specific planner
		paramData = {};
		$('.planner_param').each(function () {
			paramData[$(this).attr('name')] = $(this).val();
		})

		problemData['planner_params'] = paramData;
		problemJSON = JSON.stringify(problemData);

		// Send the request
		$.ajax({
			url: "omplapp/upload",
			type: "POST",
			data: problemJSON,
			success: function(data){
				solutionData = JSON.parse(data);


				if (solutionData.solved == "true") {
					// Draw the solution path
					visualizePath(solutionData);
					html += "<button type='button' onclick='animateToggle()' id='animateToggleBtn'>Animate</button>";
					html += "<button type='button' onclick='toggleRobotPath()' id='toggleRobotPathBtn'>Show Robot Path</button>";
					html += "<font color='#329B71'><br>Found solution.</font><br><br>";
				} else {
					html += "<font color='#cd535a'><br>No solution found. To try again, click the solve button.</font><br><br>";
				}

				html += "<pre>"
				html += solutionData.name;

				html += solutionData.messages;
				html += "<br><br>"

				// html += solutionData.path;

				html += "</pre>";

				hidePane('#plannerPane');

				results = html;


				$('#results').html(html);

				$.unblockUI();
			},
			error: function(data) {
				hidePane('#plannerPane');
				$.unblockUI();
				html += "<pre>Server responded with an error. Check the problem configuration and try again.</pre>";
				$('#results').html(html);

				console.log(data);
			},
			cache: false,
			contentType: 'application/json',
			processData: false
		});
	} else {
		// Invalid fields have been highlighted by 'validateField()'.
	}
}

function animateToggle() {
	if ($('#animateToggleBtn').hasClass('active')) {
		$('#animateToggleBtn').removeClass('active');
		
		clearInterval(animateRobot);
	} else {
		$('#animateToggleBtn').addClass('active');
		animateRobot = setInterval(function() {
			moveRobot(path);
		}, 100);
	}
}

function toggleRobotPath() {
	if ($('#toggleRobotPathBtn').hasClass('active')) {
		$('#toggleRobotPathBtn').removeClass('active');
		
		hideRobotPath();
	} else {
		$('#toggleRobotPathBtn').addClass('active');
		
		showRobotPath();	
	}
}


// Benchmarking
function load_benchmarking () {
	$(".active_nav_item").removeClass('active_nav_item')

	$("#content").load("omplapp/components/benchmarking", function () {
		$('#nav_bench').addClass('active_nav_item');
	});
}
