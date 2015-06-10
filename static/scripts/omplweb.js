
var planners = null;

var results = "";

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

		// If user previously solved a problem, reload those results
		$('#results').html(results);

	});
}

function load_visualization () {
	$(".active_nav_item").removeClass('active_nav_item')

	$("#content").load("omplapp/components/visualization", function () {
		$('#nav_viz').addClass('active_nav_item');
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
		plannerConfigHTML += "<div class='pane-title'>" + planner_name.split(".")[2] + " Parameters</div>";
		plannerConfigHTML += "<div class='container-fluid'>";
		plannerConfigHTML += "<div class='col-md-12'>";

		params = planners[planner_name]
		for (var key in params) {
			if (params.hasOwnProperty(key)) {
				plannerConfigHTML += params[key][0];
				plannerConfigHTML += "<input type='text' name='" + key +  "' class='form-field form-control' value='" + params[key][3] + "'><br><br>";
			}
		}

		plannerConfigHTML += "</div>";
		plannerConfigHTML += "</div>";

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
	linkID = paneID + "Link";
	
	// Hide the pane
	$(paneID).collapse('hide');
	
	// Unhighlight sidebar item
	$(linkID).removeClass('sidebar-active');
}

function showPane(paneID) {
	linkID = paneID + "Link";
	
	// Show the pane
	$(paneID).collapse('show');
	
	// Highlight sidebar item
	$(linkID).addClass('sidebar-active');
}


function loadConfig() {
	var cfgFile = $("#config")[0].files[0];

	if (cfgFile != null) {
		var reader = new FileReader();

		reader.readAsText(cfgFile);

		reader.onload = function () {
			var cfgData = parseConfig(reader.result);

			//TODO: Make this neater
			document.getElementsByName("name")[0].value = cfgData['name'];
			// document.getElementsByName("planners")[0].value = "";
			document.getElementsByName("start.x")[0].value = cfgData['start.x'];
			document.getElementsByName("start.y")[0].value = cfgData['start.y'];
			document.getElementsByName("start.z")[0].value = cfgData['start.z'];
			document.getElementsByName("start.theta")[0].value = cfgData['start.theta'];
			document.getElementsByName("start.axis.x")[0].value = cfgData['start.axis.x'];
			document.getElementsByName("start.axis.y")[0].value = cfgData['start.axis.y'];
			document.getElementsByName("start.axis.z")[0].value = cfgData['start.axis.z'];
			document.getElementsByName("goal.x")[0].value = cfgData['goal.x'];
			document.getElementsByName("goal.y")[0].value = cfgData['goal.y'];
			document.getElementsByName("goal.z")[0].value = cfgData['goal.z'];
			document.getElementsByName("goal.theta")[0].value = cfgData['goal.theta'];
			document.getElementsByName("goal.axis.x")[0].value = cfgData['goal.axis.x'];
			document.getElementsByName("goal.axis.y")[0].value = cfgData['goal.axis.y'];
			document.getElementsByName("goal.axis.z")[0].value = cfgData['goal.axis.z'];
			document.getElementsByName("bounds.min.x")[0].value = cfgData['volume.min.x'];
			document.getElementsByName("bounds.min.y")[0].value = cfgData['volume.min.y'];
			document.getElementsByName("bounds.min.z")[0].value = cfgData['volume.min.z'];
			document.getElementsByName("bounds.max.x")[0].value = cfgData['volume.max.x'];
			document.getElementsByName("bounds.max.y")[0].value = cfgData['volume.max.y'];
			document.getElementsByName("bounds.max.z")[0].value = cfgData['volume.max.z'];
			document.getElementsByName("time_limit")[0].value = cfgData['time_limit'];
			document.getElementsByName("mem_limit")[0].value = cfgData['mem_limit'];
			document.getElementsByName("run_count")[0].value = cfgData['run_count'];

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
		console.log("Sending: ", formData);
		// Send the request
		$.ajax({
			url: "omplapp/upload",
			type: "POST",
			data: formData,
			success: function(data){
				data = JSON.parse(data);

				html += "<pre>";
				html += data.name;

				if (data.solved == "true") {
					html += "<font color='green'>: Found solution.</font><br><br>";
				} else {
					html += "<font color='red'>: No solution found.</font><br><br>";
				}

				html += data.messages;
				html += "<br><br>"

				html += data.path;

				html += "</pre>";
				
				hidePane('#robotPane');
				hidePane('#plannerPane');
				$.unblockUI();
				results = html;
				$('#results').html(html);
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