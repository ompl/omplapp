



/* Benchmarking */

/**
 * Loads benchmarking components.
 * TODO: Should probably move all benchmarking related code to another file.
 *
 * @param 	None
 * @return 	None
 */
function load_benchmarking_page () {
	// TODO: Save all of the config info


	// Unhighlight the old active tab
	$(".active_nav_item").removeClass('active_nav_item');

	// Load the HTML for the configuration settings
	$("#content").load("omplapp/components/benchmarking", function () {
		// Make config the active tab
		$('#nav_bench').addClass('active_nav_item');

	});
}
