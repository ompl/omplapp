
/* Global Variables */
var benchmarkPlanners = {};
var firstTime = true;
var plannerCounter = 0;


/**
 * Loads benchmarking components.
 *
 * @param None
 * @return None
 */
function initializeBenchmarking() {
	// Load the HTML for the configuration settings
	$("#benchmarking").load("omplapp/components/benchmarking", function () {

		$("#benchmarking-page").click(function() {
			if (firstTime) {
				createDefaultPlannerEntry();
				firstTime = false;
			}
		});

	});
}


/**
 * Retrieves information about the user configured planner from the configuration page.
 *
 * @param None
 * @return {Object} A mapping of the planner info and parameters to their values
 */
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


/**
 * Creates an default planner entry for benchmarking by retrieving the user
 * configured planner from the configuration page.
 *
 * @param None
 * @return None
 */
function createDefaultPlannerEntry () {
	var defaultPlanner = getConfiguredPlanner();

	var name = defaultPlanner['name'];
	var params = defaultPlanner['parameters'];
	var numParams = Object.keys(params).length + 1;

	var planner = "<table class='table planner-table table-condensed' id='" + plannerCounter + "'>";

	planner += "<tr><th>" + name.split(".")[2] + "</th>";
	planner += "<th><a title='Remove planner' data-toggle='tooltip' data-placement='right' class='remove' onclick='removePlanner(" + plannerCounter  + ")'><span class='glyphicon glyphicon-remove'></a></th></tr>";
	plannerCounter += 1;

	for (var param in params) {
		planner += "<tr><td class='planner-param'>" + planners[name][param][0] + "</td>";
		planner += "<td class='param-value'><input type='text' name='" + name.split(".")[2] + "' class='form-control input-sm' value='" + params[param] + "' id='" + param + "'></td></tr>";

	}

	planner += "</table>";
	$('#planner-tables').append(planner);
	$('[data-toggle="tooltip"]').tooltip();
}


/**
 * Adds a new editable planner to be benchmarked and updates the cfg file.
 *
 * @param {String} name The name of the planner to add
 * @return None
 */
function addPlanner (name) {

	var params = planners[name]
	var numParams = Object.keys(params).length + 1;
	var name = name.split(".")[2];

	var planner = "<table class='table planner-table table-condensed' id='" + plannerCounter + "'>";

	planner += "<tr><th>" + name + "</th>";
	planner += "<th><a title='Remove planner' data-toggle='tooltip' data-placement='right' class='remove' onclick='removePlanner(" + plannerCounter  + ")'><span class='glyphicon glyphicon-remove'></a></th></tr>";
	plannerCounter += 1;

	for (var key in params) {
		if (params.hasOwnProperty(key)) {
			planner	+= "<tr>";
			planner	+= "<td class='planner-param'>" + params[key][0] + "</td>";
			planner	+= "<td class='param-value'><input type='text' name='" + name + "' class='form-control input-sm' value='" + params[key][3] + "' id='" + key + "'></td></tr>";
		}
	}

	planner += "</table>";
	$('#planner-tables').append(planner);
	$('[data-toggle="tooltip"]').tooltip();

}

/**
 * Removes a planner entry from the table
 *
 * @param {string} planner The ID of the planner entry to remove
 * @return None
 */
function removePlanner (planner) {
	$('table').remove('#' + planner);
}


/**
 * Updates the internal cfg file to reflect the planners the user has configured.
 *
 * @param  None
 * @return {String} A string containing the planners to use for benchmarking.
 */
function getBenchmarkingPlanners() {
	var benchPlanners = "";
	var currentPlanner = "";
	var modifiedPlannerName = "";

	var inputs = $('#planner-tables input');
	for (var i = 0; i < inputs.length; i++) {
		modifiedPlannerName = inputs[i].name.toLowerCase().replace("1", "").replace("two", "2");

		if (currentPlanner != inputs[i].name) {
			benchPlanners +=  modifiedPlannerName + "=\n";
		}
		currentPlanner = inputs[i].name;

		benchPlanners += modifiedPlannerName + "." + inputs[i].id + "=" + inputs[i].value + "\n";
	}

	return benchPlanners;
}


/**
 * Checks that the benchmark settings are complete and within limits.
 *
 * @param None
 * @return {Boolean} Whether the settings are valid or not
 */
function validateBenchmarkSettings() {
	var time = $("[name='time_limit']").val();
	var mem = $("[name='mem_limit']").val();
	var runs = $("[name='run_count']").val();

	if (time != null && mem != null && runs != null) {
		if (time > 0 && mem > 0 && runs > 0) {
			if (time * mem * runs < 1000000) {
				return true;
			}
		}
	}
	return false;
}


/**
 * Sends configuration data to the server for benchmarking.
 *
 * @param None
 * @return None
 */
function startBenchmarking() {

	if (validateBenchmarkSettings() == true) {
		var cfgText = getConfigText();

		if (getBenchmarkingPlanners() == "") {
			showAlert("benchmark", "warning", "Please add one or more planners to benchmark.");
		} else if (cfgText == null){
			showAlert("benchmark", "danger", "Problem was not properly configured. Ensure all fields are completed and try again.");
		} else {
			var form = new FormData();
			form.append('cfg', cfgText);
			form.append('filename', $("[name='name']").val());
			// form.append('email', $("#notificationEmail").val());

			$.ajax({
				url: "/omplapp/benchmark",
				type: "POST",
				data: form,
				success: function(data){
					console.log(data);
					var url = "http://127.0.0.1:4407/?job=" + data
					var msg = "The benchmark job was submitted successfully. ";
					msg += "The results will be available at: <a target='none' href='" + url + "'>" + url +  "</a>";
					showAlert("benchmark", "success", msg);
				},
				error: function(data) {
					console.log(data);

					showAlert("benchmark", "danger", "There was a problem submitting the benchmark job. Try again.");
				},
				cache: false,
				contentType: false,
				processData: false
			});
		}
	} else {
		showAlert("benchmark", "warning", "Please check the benchmarking settings and try again. Time Limit * Memory Limit * Run Count cannot exceed 1,000,000.");
	}
}


