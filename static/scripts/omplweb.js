
var planners= {
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



$(document).ready(function() {
	// Load config data when .cfg file is selected
	$("#config").change(function (){
		loadConfig();
	});


	// Default planner
	var planner_name = "KPIECE1";
	var planner_params = planners["KPIECE1"]

	// When user picks planner, update our info
	$("#planners").change(function() {
		planner_name = $("#planners").val().split(".")[1];
		planner_params = planners[planner_name];

		

		$("#generalConfig").collapse("hide");
		$("#plannerConfig").collapse("show");
	});
})


function load_planner_params() {
}


function solve(){
	// Check that all fields are filled in
	var validConfig = validateFields();

	if (validConfig == true) {
		var html = "";
		html += "<h2><center>Results</h2>"
		$('#results').html(html);
		
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

				html += "Path Length: " + data.length + "<br>";

				html += data.path;

				html += "</pre>";

				$('#generalConfig').collapse('hide');
				$('#plannerConfig').collapse('hide');
				$.unblockUI();
				$('#results').html(html);
			},
			error: function(data) {
				$.unblockUI();
				alert("Server responded with an error.");
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
			document.getElementsByName("start_x")[0].value = cfgData['start.x'];
			document.getElementsByName("start_y")[0].value = cfgData['start.y'];
			document.getElementsByName("start_z")[0].value = cfgData['start.z'];
			document.getElementsByName("start_theta")[0].value = cfgData['start.theta'];
			document.getElementsByName("start_axis_x")[0].value = cfgData['start.axis.x'];
			document.getElementsByName("start_axis_y")[0].value = cfgData['start.axis.y'];
			document.getElementsByName("start_axis_z")[0].value = cfgData['start.axis.z'];
			document.getElementsByName("goal_x")[0].value = cfgData['goal.x'];
			document.getElementsByName("goal_y")[0].value = cfgData['goal.y'];
			document.getElementsByName("goal_z")[0].value = cfgData['goal.z'];
			document.getElementsByName("goal_theta")[0].value = cfgData['goal.theta'];
			document.getElementsByName("goal_axis_x")[0].value = cfgData['goal.axis.x'];
			document.getElementsByName("goal_axis_y")[0].value = cfgData['goal.axis.y'];
			document.getElementsByName("goal_axis_z")[0].value = cfgData['goal.axis.z'];
			document.getElementsByName("bounds_min_x")[0].value = cfgData['volume.min.x'];
			document.getElementsByName("bounds_min_y")[0].value = cfgData['volume.min.y'];
			document.getElementsByName("bounds_min_z")[0].value = cfgData['volume.min.z'];
			document.getElementsByName("bounds_max_x")[0].value = cfgData['volume.max.x'];
			document.getElementsByName("bounds_max_y")[0].value = cfgData['volume.max.y'];
			document.getElementsByName("bounds_max_z")[0].value = cfgData['volume.max.z'];
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
	var valid = true;

	// Check that all the fields have values
	$('.form-field').each(function () {
		if ($(this).val() === '' || $(this).val() == null) {
			valid = false;

			// Highlight the incorrect field
			$(this).css("background-color", "#fda9a0");
		} else {
			$(this).css("background-color", "white");
		}
	});


	return valid
}





	// inputData = {}
	// inputData.name = $("#name").val();
	//		   inputData.start_x = $("#start_x").val();
	//		   inputData.start_y = $("#start_y").val();
	//		   inputData.start_z = $("#start_z").val();
	//		   inputData.start_theta = $("#start_theta").val();
	//		   inputData.start_axis_x = $("#start_axis_x").val();
	//		   inputData.start_axis_y = $("#start_axis_y").val();
	//		   inputData.start_axis_z = $("#start_axis_z").val();
	//		   inputData.goal_x = $("#goal_x").val();
	//		   inputData.goal_y = $("#goal_y").val();
	//		   inputData.goal_z = $("#goal_z").val();
	//		   inputData.goal_theta = $("#goal_theta").val();
	//		   inputData.goal_axis_x = $("#goal_axis_x").val();
	//		   inputData.goal_axis_y = $("#goal_axis_y").val();
	//		   inputData.goal_axis_z = $("#goal_axis_z").val();
	//		   inputData.bounds_min_x = $("#bounds_min_x").val();
	//		   inputData.bounds_min_y = $("#bounds_min_y").val();
	//		   inputData.bounds_min_z = $("#bounds_min_z").val();
	//		   inputData.bounds_max_x = $("#bounds_max_x").val();
	//		   inputData.bounds_max_y = $("#bounds_max_y").val();
	//		   inputData.bounds_max_z = $("#bounds_max_z").val();
	//		   inputData.time_limit = $("#time_limit").val();
	//		   inputData.mem_limit = $("#mem_limit").val();
	//		   inputData.run_count = $("#run_count").val();
	//		   inputData.planners = $("#planners").val();
	//
	//		   inputData.location = "default_location";
	//		   inputData.date_modified = "2/15/2015";
	//		   inputData.env_path = "../../../robots/3D/Apartment_env.dae";
	//		   inputData.robot_path = "../../../robots/3D/Apartment_robot.dae";
	//
	// console.log(inputData);

	// var promise = postData(JSON.stringify(inputData));
	//
//	promise.success(function (data){
//		console.log(data);
//		$('#results').html(
//			'<pre>' +
//			JSON.stringify(data, ['solved', 'path'], ' ') +
//			'</pre>'
//		);
//	})
// }

// function postData(sendData) {
//	return $.ajax({
//		url: "omplapp/solve",
//		type: "POST",
//		dataType: "json",
//		data: {settings : sendData}
//	});


