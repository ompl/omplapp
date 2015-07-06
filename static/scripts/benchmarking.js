/* Benchmarking */

var benchmarkPlanners = {};
var firstTime = true;

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

	var planner = "<tr><th colspan='3' rowspan='" + numParams + "' class='planner-name'>" + name.split(".")[2] + "</th></tr>";

	for (var param in params) {
		planner += "<tr><td class='planner-param'>" + planners[name][param][0] + "</td>";
		planner += "<td class='param-value'><input type='text' name='" + name.split(".")[2] + "' class='form-control input-sm' value='" + params[param] + "' id='" + param + "'></td></tr>";

	}

	$('#planner-entries').append(planner);
}



/**
 * Adds a new editable planner to be benchmarked and updates the cfg file.
 *
 * @param 	{String} name The name of the planner to add
 * @return 	None
 */
function addPlanner (name) {

	createPlannerEntry(name);

}


function createPlannerEntry(name) {
	var params = planners[name]
	var numParams = Object.keys(params).length + 1;
	var planner = "";
	var name = name.split(".")[2];

	planner += "<tr><th colspan='3' rowspan='" + numParams + "' class='planner-name'>";
	planner += name;
	planner += "</th></tr>";


	for (var key in params) {
		if (params.hasOwnProperty(key)) {
			planner	+= "<tr>";
			planner	+= "<td class='planner-param'>" + params[key][0] + "</td>";
			planner	+= "<td class='param-value'><input type='text' name='" + name + "' class='form-control input-sm' value='" + params[key][3] + "' id='" + key + "'></td></tr>";
		}
	}

	$('#planner-entries').append(planner);

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

	var inputs = $('#planner-entries input');
	for (var i = 0; i < inputs.length; i++) {
		if (currentPlanner != inputs[i].name) {
			benchPlanners += inputs[i].name.toLowerCase() + "=\n";
		}
		currentPlanner = inputs[i].name;

		benchPlanners += inputs[i].name.toLowerCase() + "." + inputs[i].id + "=" + inputs[i].value + "\n";
	}
	return benchPlanners;
}


function startBenchmarking() {
	$.ajax({
		url: "omplapp/benchmark",
		type: "GET",
		success: function(data){
			console.log(data);
		},
		error: function(data) {
			console.log(data);
		},
		cache: false,
		contentType: false,
		processData: false
	});
}


