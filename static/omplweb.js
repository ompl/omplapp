

function solve(){
	
	//TODO: Get user"s config / data and then post to the server for solving
	//
	//
	//TODO: Wait for server"s response and display back to user.
	
	// sendData = '{"name" : "apartment_piano","location" : "default_location","date_modified" : "2/15/2015","env_path" : "../../../robots/3D/Apartment_env.dae","robot_path" : "../../../robots/3D/Apartment_robot.dae","start_x" : 241.81,"start_y" : 106.15,"start_z" : 36.46,"start_theta" : 3.12413936107,"start_axis_x" : 0.0,"start_axis_y" : 0.0,"start_axis_z" : -1.0,"goal_x" : -31.19,"goal_y" : -99.85,"goal_z" : 36.46,"goal_theta" : 3.12413936107,"goal_axis_x" : 0.0,"goal_axis_y" : 0.0,"goal_axis_z" : -1.0,"bounds_min_x" : -73.76,"bounds_min_y" : -179.59,"bounds_min_z" : -0.03,"bounds_max_x" : 295.77,"bounds_max_y" : 168.26,"bounds_max_z" : 90.39,"time_limit" : 5.0,"mem_limit" : 10000.0,"run_count" : 1,"planners" : "rrt"}';
	
	inputData = {}
	inputData.name = "Test Name"
        inputData.start_x = $("#start_x").val();
        inputData.start_y = $("#start_y").val();
        inputData.start_z = $("#start_z").val();
        inputData.start_theta = $("#start_theta").val();
        inputData.start_axis_x = $("#start_axis_x").val();
        inputData.start_axis_y = $("#start_axis_y").val();
        inputData.start_axis_z = $("#start_axis_z").val();
        inputData.goal_x = $("#goal_x").val();
        inputData.goal_y = $("#goal_y").val();
        inputData.goal_z = $("#goal_z").val();
        inputData.goal_theta = $("#goal_theta").val();
        inputData.goal_axis_x = $("#goal_axis_x").val();
        inputData.goal_axis_y = $("#goal_axis_y").val();
        inputData.goal_axis_z = $("#goal_axis_z").val();
        inputData.bounds_min_x = $("#bounds_min_x").val();
        inputData.bounds_min_y = $("#bounds_min_y").val();
        inputData.bounds_min_z = $("#bounds_min_z").val();
        inputData.bounds_max_x = $("#bounds_max_x").val();
        inputData.bounds_max_y = $("#bounds_max_y").val();
        inputData.bounds_max_z = $("#bounds_max_z").val();
        inputData.time_limit = $("#time_limit").val();
        inputData.mem_limit = $("#mem_limit").val();
        inputData.run_count = $("#run_count").val();
        inputData.planners = $("#planners").val();
	
        inputData.location = "default_location";
        inputData.date_modified = "2/15/2015";
        inputData.env_path = "../../../robots/3D/Apartment_env.dae";
        inputData.robot_path = "../../../robots/3D/Apartment_robot.dae";
	
	console.log(inputData);
	
	var promise = postData(JSON.stringify(inputData));
	
	promise.success(function (data){
		console.log(data);
		$('#results').html(
			'<pre>' +
			JSON.stringify(data, ['solved', 'path'], ' ') +
			'</pre>'
		);
	})
}

function postData(sendData) {
	return $.ajax({
		url: "omplapp/solve",
		type: "POST",
		dataType: "json",
		data: {settings : sendData}
	});
}