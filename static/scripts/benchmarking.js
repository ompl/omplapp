var config;



/* Benchmarking */

/**
 * Loads benchmarking components.
 * TODO: Should probably move all benchmarking related code to another file.
 *
 * @param 	None
 * @return 	None
 */
function load_benchmarking_page() {
	// TODO: Save all of the config info
	if (solutionData == null) {
		alert("Configure and solve a problem before benchmarking.");
	} else {
		// config = getProblemConfig();
		var defaultPlanner = getConfiguredPlanner();

		console.log(defaultPlanner);

		// Unhighlight the old active tab
		$(".active_nav_item").removeClass('active_nav_item');

		// Load the HTML for the configuration settings
		$("#content").load("omplapp/components/benchmarking", function () {
			// Make config the active tab
			$('#nav_bench').addClass('active_nav_item');

			addPlanner(defaultPlanner['name'], defaultPlanner['parameters']);
		});
	}
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
