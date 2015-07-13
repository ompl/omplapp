/* Benchmarking */

var benchmarkPlanners = {};
var firstTime = true;
var plannerCounter = 0;


/**
 * Loads benchmarking components.
 * TODO: Should probably move all benchmarking related code to another file.
 *
 * @param 	None
 * @return 	None
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
 * @param 	{String} name The name of the planner to add
 * @return 	None
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
 *
 */
function removePlanner (planner) {
	$('table').remove('#' + planner);
}


/**
 * Updates the internal cfg file to reflect the planners the user has configured.
 *
 * @param  None
 * @return {String} benchPlanners A string containing the planners to use for benchmarking.
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


function startBenchmarking() {

	if (getBenchmarkingPlanners() == "") {
		showAlert("benchmark", "warning", "Please add one or more planners to benchmark.");
	} else {
		var form = new FormData();
		form.append('cfg', getConfigText());
		form.append('filename', $("[name='name']").val());
		form.append('email', $("#notificationEmail").val());

		$.ajax({
			url: "/omplapp/benchmark",
			type: "POST",
			data: form,
			success: function(data){
				showAlert("benchmark", "success", "The benchmark job was submitted successfully. The results will be sent to the provided email address once the job is complete.");
			},
			error: function(data) {
				console.log(data);

				showAlert("benchmark", "error", "There was a problem submitting the benchmark job. Try again.");
			},
			cache: false,
			contentType: false,
			processData: false
		});
	}

}


