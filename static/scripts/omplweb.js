
var asdf_planners= {
	"BKPIECE1" : {
		"Border fraction" : 0.90,
		"Range" : 0.0
	},
	"EST" : {
		"Goal bias" : 0.05,
		"Range" : 0.0
	},
	"FMT" : {
		"Free space volume" : 1.0,
		"Num samples" : 1000.0,
		"Radius multiplier" : 1.10
	},
	"KPIECE1" : {
		"Border fraction" : 0.90,
		"Goal bias" : 0.05,
		"Range" : 0.0
	},
	"LBKPIECE1" : {
		"Border fraction" : 0.90,
		"Range" : 0.0
	},
	"LBTRRT" : {
		"Epsilon" : 0.40,
		"Goal bias" : 0.05,
		"Range" : 0.0
	},
	"LazyPRM" : {
		"Max nearest neighbors" : 8,
		"Range" : 0.0
	},
	"LazyPRMstar" : {
		"Max nearest neighbors" : 8,
		"Range" : 0
	},
	"LazyRRT" : {
		"Goal bias" : 0.05,
		"Range" : 0.0
	},
	"PDST" : {
		"Goal bias" : 0.05,
	},
	"PRM" : {
		"Max nearest neighbors" : 8,
	},
	"PRMstar" : {
		"Max nearest neighbors" : 8,
	},
	"RRT" : {
		"Goal bias" : 0.05,
		"Range" : 0.0
	},
	"RRTConnect" : {
		"Range" : 0.0
	},
	"RRTstar" : {
		"Delay collision checking" : "False",
		"Goal bias" : 0.05,
		"Prune" : "False",
		"Range" : 0.0
	},
	"SBL" : {
		"Range" : 0.0
	},
	"SPARS" : {
		"Dense delta fraction" : 0.0010,
		"Max failures" : 1000.0,
		"Sparse delta fraction" : 0.25,
		"Stretch factor" : 3.0
	},
	"SPARStwo" : {
		"Dense delta fraction" : 0.0010,
		"Max failures" : 3000.0,
		"Sparse delta fraction" : 0.25,
		"Stretch factor" : 3.0

	},
	"STRIDE" : {
		"Degree" : 16.0,
		"Estimated dimension" : 1.0,
		"Goal bias" : 0.05,
		"Max degree" : 18.0,
		"Max pts per leaf" : 6.0,
		"Min degree" : 12.0,
		"Min valid path fraction" : 0.20,
		"Range" : 0.0,
		"Use projected distance" : "False"
	},
	"TRRT" : {
		"Goal bias" : 0.05,
		"Max states failed" : 10.0,
		"Range" : 0.0,
		"Temp change factor" : 2.00
	}
}

var planners = null;

$(document).ready(function() {

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
				$("#generalConfig").collapse('show');
			} else {
				$("#customProblem").collapse('hide');
				$("#generalConfig").collapse('hide');
			}
		});
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
		plannerConfigHTML += "<div class='col-md-8'>";
		plannerConfigHTML += "<h4>" + planner_name.split(".")[2] + " Parameters</h4>";
		plannerConfigHTML += "<table class='table'>";

		params = planners[planner_name]
		for (var key in params) {
			if (params.hasOwnProperty(key)) {
				plannerConfigHTML += "<tr><td>" + params[key][0]  + "</td>";
				plannerConfigHTML += "<td><input type='text' name='" + key +  "' class='form-field' value='" + params[key][3] + "'></td></tr>";
			}
		}

		plannerConfigHTML += "</table>";
		plannerConfigHTML += "</div>";

		$("#plannerConfig").html(plannerConfigHTML);
		$("#generalConfig").collapse("hide");
		$("#plannerConfig").collapse("show");
	} else {
		alert("Planners are not loaded yet. Please wait and try again.");
	}
}


function solve(){
	// Check that all fields are filled in
	var validConfig = validateFields();
	if (validConfig == true) {
		var html = "";
		html += "<h2><center>Results</h2>"

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

				$('#generalConfig').collapse('hide');
				$('#plannerConfig').collapse('hide');
				$.unblockUI();
				$('#results').html(html);
			},
			error: function(data) {
				$('#generalConfig').collapse('hide');
				$('#plannerConfig').collapse('hide');
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
	$('.form-field').each(function () {
		$(this).val('');
		$(this).css("background-color", "white");
	});
	$('#config').val('');
	$('#results').html('');
	$('#generalConfig').collapse('show');
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
