
var planners = null;

var results = "";

var solutionData;

$(document).ready(function() {
	load_configuration();

});


function load_configuration () {
	$("#content").load("omplapp/components/configuration", function () {
		$(".active_nav_item").removeClass('active_nav_item')
	});

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
			planner_params = planners[planner_name];
			load_planner_params(planner_name, planner_params);
		});

		// Load config data when .cfg file is selected
		$("#config").change(function (){
			loadConfig();
		});

		// Show upload buttons if user selects 'Custom' problem
		$("#problems").change(function() {
			if ($("#problems").val() == 'custom'){
				$("#customProblem").collapse('show');
				showPane('#robotPane');
			} else {
				$("#customProblem").collapse('hide');
				hidePane('#robotPane');
			}
		});
		
		// Open the problem config tab
		$('#problem-tab').click()
		
		// If user previously solved a problem, reload those results
		$('#results').html(results);

	});
}

function load_benchmarking () {
	$(".active_nav_item").removeClass('active_nav_item')

	$("#content").load("omplapp/components/benchmarking", function () {
		$('#nav_bench').addClass('active_nav_item');
	});
}

function load_planner_params(planner_name) {
	if (planners != null) {
		var plannerConfigHTML = "";
		plannerConfigHTML += "<table class='table'><caption>";
		plannerConfigHTML += planner_name.split(".")[2]; 
		plannerConfigHTML += " Options</caption><tbody>";
		params = planners[planner_name]
		for (var key in params) {
			if (params.hasOwnProperty(key)) {
				plannerConfigHTML += "<tr><td>";
				plannerConfigHTML += params[key][0];
				plannerConfigHTML += "</td><td><input type='text' name='" + key +  "' class='form-field form-control input-sm' value='" + params[key][3] + "'></td></tr>";
			}
		}
		plannerConfigHTML += "</tbody></table>"
		$("#plannerPane").html(plannerConfigHTML);
		hidePane('#robotPane');
		showPane('#plannerPane');
	} else {
		alert("Planners are not loaded yet. Please wait and try again.");
	}
}


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


function loadConfig() {
	var cfgFile = $("#config")[0].files[0];

	if (cfgFile != null) {
		var reader = new FileReader();

		reader.readAsText(cfgFile);

		reader.onload = function () {
			var cfgData = parseConfig(reader.result);

			$("[name='name']").val(cfgData['name']);
			$("[name='start.x']").val(cfgData['start.x']);
			$("[name='start.y']").val(cfgData['start.y']);
			$("[name='start.z']").val(cfgData['start.z']);
			$("[name='start.axis.x']").val(cfgData['start.axis.x']);
			$("[name='start.axis.y']").val(cfgData['start.axis.y']);
			$("[name='start.axis.z']").val(cfgData['start.axis.z']);
			$("[name='goal.x']").val(cfgData['goal.x']);
			$("[name='goal.y']").val(cfgData['goal.y']);
			$("[name='goal.z']").val(cfgData['goal.z']);
			$("[name='goal.axis.x']").val(cfgData['goal.axis.x']);
			$("[name='goal.axis.y']").val(cfgData['goal.axis.y']);
			$("[name='goal.axis.z']").val(cfgData['goal.axis.z']);
			$("[name='bounds.min.x']").val(cfgData['volume.min.x']);
			$("[name='bounds.min.y']").val(cfgData['volume.min.y']);
			$("[name='bounds.min.z']").val(cfgData['volume.min.z']);
			$("[name='bounds.max.x']").val(cfgData['volume.max.x']);
			$("[name='bounds.max.y']").val(cfgData['volume.max.y']);
			$("[name='bounds.max.z']").val(cfgData['volume.max.z']);
			$("[name='time_limit']").val(cfgData['time_limit']);
			
			// document.getElementsByName("start.theta")[0].value = cfgData['start.theta'];
			// document.getElementsByName("goal.theta")[0].value = cfgData['goal.theta'];

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

function clearAllFields() {
	if (confirm("Are you sure you want to clear all fields?")) {
		$('.form-field').each(function () {
			$(this).val('');
			$(this).css("background-color", "white");
		});
		$('#config').val('');
		$('#results').html('');
		results = "";
		showPane('#robotPane');
	}
	
}

function validateFields() {
	// Check that a problem has been selected
	if ($('#problems').val() != 'custom' && $('#planners').val() != null) {
		return true;
	}

	// Check that a planner has been selected
	if ($('#planners').val() == null) {
		alert("Please select a planner from the drop down menu.");
		return false;
	}

	// Check that a planner has been selected
	if ($('#problems').val() == null && $('#planners').val() == null) {
		alert("Please select a problem and planner from the drop down menus.");
		return false;
	}

	// Check if the user uploaded a custom problem
	if ($('#problems').val() == 'custom' && $('#planners').val() != null) {
		var valid = true;

		// Ensure that the all the needed fields for a problem have values
		$('.form-field').each(function () {
			if ($(this).val() === '') {
				valid = false;

				// Highlight the incorrect field
				$(this).css("background-color", "#fda9a0");
			} else {
				$(this).css("background-color", "white");
			}
		});
		return valid;
	}

}


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

		// Read the input fields
		var formData = new FormData($('form')[0]);
		
		// Send the request
		$.ajax({
			url: "omplapp/upload",
			type: "POST",
			data: formData,
			success: function(data){
				solutionData = JSON.parse(data);
				
				html += "<pre>";
				html += solutionData.name;

				if (solutionData.solved == "true") {
					html += "<font color='green'>: Found solution.</font><br><br>";
				} else {
					html += "<font color='red'>: No solution found.</font><br><br>";
				}

				html += solutionData.messages;
				html += "<br><br>"

				// html += solutionData.path;

				html += "</pre>";
				
				hidePane('#robotPane');
				hidePane('#plannerPane');
				
				results = html;
				
				
				$('#results').html(html);
				
				$.unblockUI();
			},
			error: function(data) {
				hidePane('#robotPane');
				hidePane('#plannerPane');
				$.unblockUI();
				html += "<pre>Server responded with an error. Check the problem configuration and try again.</pre>";
				$('#results').html(html);

				console.log(data);
			},
			cache: false,
			contentType: false,
			processData: false
		});
	} else {
		// Invalid fields have been highlighted by 'validateField()'.
	}
}