/* Benchmarking */

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

		$('#benchmarking-page').click(function () {
				var defaultPlanner = getConfiguredPlanner();
				var name = defaultPlanner['name'];
				var params = defaultPlanner['parameters'];

				$('#defaultPlannerName').html(name.split(".")[2]);

				$('#defaultPlannerParams').html("");

				for (var param in params) {
					$('#defaultPlannerParams').append("<tr><td>" + planners[name][param][0] + "</td><td>" + params[param] + "</td></tr>");
				}

		});

	});
}



function createPlannerEntry(name) {
	var planner = "";
	planner += "<tr>";
	planner += "<th>" + name.split(".")[2] + "</th><td>";

	planner += "<table class='table-condensed'>";

	params = planners[name]
	for (var key in params) {
		if (params.hasOwnProperty(key)) {
		planner	+= "<tr><td>";
		planner	+= params[key][0];
		planner	+= "</td><td><input type='text' name='" + key +  "' class='form-control input-sm' value='" + params[key][3] + "'></td></tr>";
		}
	}
	planner += "</tbody></table>"

	$('#planner-entries').append(planner);

}


function load_planner_entry(name) {

	if (planners != null) {
		var plannerConfigHTML = "";


	} else {
		alert("Planners are not loaded yet. Please wait and try again.");
	}

}

