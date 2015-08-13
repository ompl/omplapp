
/* Global Variables */


// Define the benchmark class
var Benchmark = function() {
    this.benchmarkPlanners = {};
    this.firstTime = true;
    this.plannerCounter = 0;
}

/**
 * Loads benchmarking components.
 *
 * @param None
 * @return None
 */
Benchmark.prototype.initialize = function() {
    // Load the HTML for the configuration settings
    $("#benchmarking").load("/components/benchmarking", function () {
        $("#benchmarking-page").click(function() {
            if (benchmark.firstTime) {
                benchmark.createDefaultPlannerEntry();
                benchmark.firstTime = false;
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
Benchmark.prototype.getConfiguredPlanner = function() {
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
Benchmark.prototype.createDefaultPlannerEntry = function() {
    var defaultPlanner = this.getConfiguredPlanner();

    var name = defaultPlanner['name'];
    var params = defaultPlanner['parameters'];
    var numParams = Object.keys(params).length + 1;
    var kind = name.split(".")[1];

    var planner = "<table class='table planner-table table-condensed' id='" + this.plannerCounter + "'>";

    planner += "<tr><th>" + name.split(".")[2] + "</th>";
    planner += "<th><a title='Remove planner' data-toggle='tooltip' data-placement='right' class='remove' onclick='benchmark.removePlanner(" + this.plannerCounter  + ")'><span class='glyphicon glyphicon-remove'></a></th></tr>";
    this.plannerCounter += 1;

    for (var param in params) {
        planner += "<tr><td class='planner-param'>" + problem.availablePlanners[kind][name][param][0] + "</td>";
        planner += "<td class='param-value'><input type='text' name='" + name.split(".")[2] + "' class='form-control input-sm' value='" + params[param] + "' id='" + param + "'></td></tr>";

    }

    planner += "</table>";
    $('#planner-tables').append(planner);
}


/**
 * Adds a new editable planner to be benchmarked and updates the cfg file.
 *
 * @param {String} name The name of the planner to add
 * @return None
 */
Benchmark.prototype.addPlanner = function(name) {
    var kind = name.split(".")[1];
    var params = problem.availablePlanners[kind][name]
    var numParams = Object.keys(params).length + 1;
    var name = name.split(".")[2];

    var planner = "<table class='table planner-table table-condensed' id='" + this.plannerCounter + "'>";

    planner += "<tr><th>" + name + "</th>";
    planner += "<th><a title='Remove planner' data-toggle='tooltip' data-placement='right' class='remove' onclick='benchmark.removePlanner(" + this.plannerCounter  + ")'><span class='glyphicon glyphicon-remove'></a></th></tr>";
    this.plannerCounter += 1;

    for (var key in params) {
        if (params.hasOwnProperty(key)) {
            planner += "<tr>";
            planner += "<td class='planner-param'>" + params[key][0] + "</td>";
            planner += "<td class='param-value'><input type='text' name='" + name + "' class='form-control input-sm' value='" + params[key][3] + "' id='" + key + "'></td></tr>";
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
Benchmark.prototype.removePlanner = function(planner) {
    $('table').remove('#' + planner);
}


/**
 * Updates the internal cfg file to reflect the planners the user has configured.
 *
 * @param  None
 * @return {String} A string containing the planners to use for benchmarking.
 */
Benchmark.prototype.getBenchmarkingPlanners = function() {
    var benchPlanners = "";
    var modifiedPlannerName = "";
    var currentPlanner = "";

    var plannerTables = $('.planner-table');
    // Iterates for each planner
    $.each(plannerTables, function(idx, table) {
        currentPlanner = table.id;
        var params = $(this).find('input');

        // Set the planner and name
        modifiedPlannerName = params[0].name.toLowerCase().replace("1", "").replace("two", "2");
        benchPlanners += modifiedPlannerName + "=\n";
        benchPlanners += modifiedPlannerName + ".name=" + currentPlanner + "_" + modifiedPlannerName +  "\n";

        // Iterates for each planner parameter
        $.each(params, function(index, param) {
            benchPlanners += modifiedPlannerName + "." + params[index].id + "=" + params[index].value + "\n";
        });

    });

    return benchPlanners;

}


/**
 * Checks that the benchmark settings are complete and within limits.
 *
 * @param None
 * @return {Boolean} Whether the settings are valid or not
 */
Benchmark.prototype.validateBenchmarkSettings = function() {
    var time = $("[name='time_limit']").val();
    var runs = $("[name='run_count']").val();
    var numPlanners = $(".planner-table").length;

    if (time != null && runs != null && numPlanners != null) {
        if (time > 0 && runs > 0 && numPlanners > 0) {
            if (time * runs * numPlanners < BENCHMARKING_LIMIT) {
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
Benchmark.prototype.startBenchmarking = function(){

    if (this.validateBenchmarkSettings() == true) {
        var cfgText = problem.getConfigText();

        if (this.getBenchmarkingPlanners() == "") {
            showAlert("benchmark", "warning", "Please add one or more planners to benchmark.");
        } else if (cfgText == null){
            showAlert("benchmark", "warning", "Problem was not properly configured. Ensure all fields are completed and try again.");
        } else {
            var form = new FormData();
            form.append('cfg', cfgText);
            form.append('filename', $("[name='name']").val());
            form.append('problem', $("#problems").val());
            form.append('env_loc', problem.config["env_loc"]);
            form.append('robot_loc', problem.config["robot_loc"]);

            form.append('session_id', sessionStorage.getItem("session_id"));

            $.ajax({
                url: "/benchmark",
                type: "POST",
                data: form,
                success: function(data){
                    var base_url = "";
                    if (window.location.href.indexOf("kavrakilab.org") > 0){
                        base_url = "http://plannerarena.org";
                    } else {
                        var href = window.location.href.split(":");
                        base_url = href[0] + ":" + href[1] + ":" + PLANNERARENA_PORT;
                    }
                    var url =  base_url + "/?" + "user=" + sessionStorage.getItem("session_id") + "&job=" + data;
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
        showAlert("benchmark", "warning", "Please check the benchmarking settings and try again. Time &times; Runs &times; Number of Planners cannot exceed " + BENCHMARKING_LIMIT);
    }
}


