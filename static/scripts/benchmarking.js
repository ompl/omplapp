/* Benchmarking */

/**
 * Loads benchmarking components.
 * TODO: Should probably move all benchmarking related code to another file.
 *
 * @param 	None
 * @return 	None
 */
function initializeBenchmarking() {

	var defaultPlanner = getConfiguredPlanner();
	
	// Load the HTML for the configuration settings
	$("#benchmarking").load("omplapp/components/benchmarking", function () {
		// Make config the active tab
		$('#nav_bench').addClass('active_nav_item');

		addPlanner(defaultPlanner['name'], defaultPlanner['parameters']);
	});
}

function addPlanner(name, params) {
	
	var planner = "";
	planner += "<tr>";
	planner += "<th>" + name + "</th>";
	planner += "<td><table class='table-condensed' id='defaultPlannerParams'>"
			
	for (var param in params) {
		planner += "<tr><td>" + param + "</td><td>" + params[param] + "</td></tr>";
	}

	planner+= "</table></td></tr>";
	$('#planners').append(planner);
}
