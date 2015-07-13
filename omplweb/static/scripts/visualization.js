/* Mathematic Constants */
var DEG_TO_RAD = Math.PI/180;
var RAD_TO_DEG = 180/Math.PI;

/* Globally Needed Information */
var env_loc;
var robot_loc;

/* Basic Scene Objects */
var scene;
var camera;
var renderer;
var controls;
var axisHelper;

/* Problem Objects */
var env;
var start_robot;
var goal_robot;
var bbox;
var path_robot;
var path_line;

/* Animation Information */
var path;
var step = 0;
var staticPathRobots = [];


/**
 * Sets up the initial scene and loads the lights, camera, and axis helper.
 *
 * @param 	None
 * @return 	None
 */
function initViz() {

	// Create a new scene
	scene = new THREE.Scene();

	// Set width and height
	var WIDTH = $('#viewer').css('width').replace("px", "");
	var HEIGHT = $('#viewer').css('height').replace("px", "");

	// Create a renderer
	renderer = new THREE.WebGLRenderer({alpha:true, antialias:true});
	renderer.setSize(WIDTH, HEIGHT);
	renderer.setClearColor(0xfafafa);
	$('#viewer').append(renderer.domElement);

	// Create a camera
	camera = new THREE.PerspectiveCamera(45, WIDTH / HEIGHT, 1, 10000);
	camera.position.set(0, 0, 1000);

	// Attach the camera to the scene
	scene.add(camera);

	// Resize the viewer if the window is resized by the user
	window.addEventListener('resize', function(){
		var WIDTH = $('#viewer').css('width').replace("px", "");
		var HEIGHT = $('#viewer').css('height').replace("px", "");
		renderer.setSize(WIDTH, HEIGHT);
		camera.aspect = WIDTH / HEIGHT;
		camera.updateProjectionMatrix();
	});

	// Create the controls
	controls = new THREE.TrackballControls(camera, renderer.domElement);
	controls.zoomSpeed = 0.5;

	// Create the axis helper
	axisHelper = new THREE.AxisHelper( 500 );
	scene.add( axisHelper );

	// Create a light and attach it to the camera
	var point_light = new THREE.PointLight(0xffffff, 1.0, 0);
	camera.add(point_light);


	// Render everything to the screen
	render();

}


/**
 * Draws the scene. Only needs to be called once (by 'initViz()').
 *
 * @param 	None
 * @return 	None
 */
function render() {

	requestAnimationFrame(render);
	renderer.render(scene, camera);
	controls.update();

}


/**
 * Creates environment, bounding box, and start/goal robot objects and draws
 * them to the screen.
 *
 * @param 	{String} e_loc The URL of the environment file
 * @param 	{String} r_loc The URL of the robot file
 * @return 	None
 */
function drawModels(e_loc, r_loc) {

	// Store locations globally
	env_loc = e_loc;
	robot_loc = r_loc;

	// Clear the scene in case there are robots/env left from a previous draw
	clearScene()

	// Load the environment file
	var env_loader = new THREE.ColladaLoader();
	env_loader.options.convertUpAxis = true;
	env_loader.load(env_loc, function(collada){

		// Add the env to the scene
		env = collada.scene.children[0];
		env.name = "env";
		env.position.set(0,0,0);
		env.scale.set(1,1,1);
		scene.add(env);
	});

	// Load the robot file
	var start_robot_loader = new THREE.ColladaLoader();
	start_robot_loader.options.convertUpAxis = true;
	start_robot_loader.load(robot_loc, function(collada){

		// Add the start robot to the scene
		start_robot = collada.scene.children[0];
		start_robot.name = "start_robot";
		start_robot.position.set(0,0,0);
		start_robot.scale.set(1,1,1);
		scene.add(start_robot);
	});

	// Load the robot file, again
	var goal_robot_loader = new THREE.ColladaLoader();
	goal_robot_loader.options.convertUpAxis = true;
	goal_robot_loader.load(robot_loc, function(collada){

		// Add the goal robot to the scene
		goal_robot = collada.scene.children[0];
		goal_robot.name = "goal_robot"
		goal_robot.position.set(0,0,0);
		goal_robot.scale.set(1,1,1);
		scene.add(goal_robot);
	});

	// Create the bounding box
	var geometry = new THREE.BoxGeometry(1,1,1);
	var material = new THREE.MeshBasicMaterial({
		color: 0xf15c40,
		wireframe: true
	});
	bbox = new THREE.Mesh(geometry, material);
	scene.add(bbox);

}



/**
 * Clears everything from the scene.
 *
 * @param 	None
 * @return 	None
 */
function clearScene() {

	scene.remove(env);
	scene.remove(start_robot);
	scene.remove(goal_robot);
	scene.remove(path_robot);
	scene.remove(bbox);
	scene.remove(path_line);

	staticPathRobots.forEach(function (element, index) {
		scene.remove(element);
	});
	staticPathRobots = [];

}


/**
 * Clears solution information such as path and static path robots.
 *
 * @param 	None
 * @return 	None
 */
function clearOldSolution() {

	scene.remove(path_robot);
	staticPathRobots.forEach(function (element, index) {
		scene.remove(element);
	});
	staticPathRobots = [];
	scene.remove(path_line);

}


/**
 * Refreshes the visualization to reflect changes in position and rotation.
 *
 * @param 	None
 * @return 	None
 */
function updatePose() {

	// Update the start position
	var start = {};
	start.x = $("[name='start.x']").val()
	start.y = $("[name='start.y']").val()
	start.z = $("[name='start.z']").val()
	// TODO: Try catch here in case not loaded yet, to avoid error msg
	start_robot.position.set(start.x, start.y, start.z)

	// Update start rotation
	var startRot = {};
	startRot.x = $("[name='start.axis.x']").val()
	startRot.y = $("[name='start.axis.y']").val()
	startRot.z = $("[name='start.axis.z']").val()
	start_robot.rotation.set(DEG_TO_RAD*startRot.x, DEG_TO_RAD*startRot.y, DEG_TO_RAD*startRot.z);

	// Update the goal position
	var goal = {};
	goal.x = $("[name='goal.x']").val()
	goal.y = $("[name='goal.y']").val()
	goal.z = $("[name='goal.z']").val()
	// TODO: Try catch here in case not loaded yet, to avoid error msg
	goal_robot.position.set(goal.x, goal.y, goal.z)

	// Update start rotation
	var goalRot = {};
	goalRot.x = $("[name='goal.axis.x']").val()
	goalRot.y = $("[name='goal.axis.y']").val()
	goalRot.z = $("[name='goal.axis.z']").val()
	goal_robot.rotation.set(DEG_TO_RAD*goalRot.x, DEG_TO_RAD*goalRot.y, DEG_TO_RAD*goalRot.z);

}


/**
 * Refreshes the visualization to reflect changes to the bounding box.
 *
 * @param 	None
 * @return 	None
 */
function updateBounds() {

	// Update bounds
	var min = {};
	min.x = $("[name='volume.min.x']").val();
	min.y = $("[name='volume.min.y']").val();
	min.z = $("[name='volume.min.z']").val();


	var max = {};
	max.x = $("[name='volume.max.x']").val();
	max.y = $("[name='volume.max.y']").val();
	max.z = $("[name='volume.max.z']").val();

	// From the lower and upper bounds, get the 8 points of the box (order matters).
	var vertices = [];
	vertices.push(new THREE.Vector3(max.x, max.y, max.z));
	vertices.push(new THREE.Vector3(max.x, max.y, min.z));
	vertices.push(new THREE.Vector3(max.x, min.y, max.z));
	vertices.push(new THREE.Vector3(max.x, min.y, min.z));
	vertices.push(new THREE.Vector3(min.x, max.y, min.z));
	vertices.push(new THREE.Vector3(min.x, max.y, max.z));
	vertices.push(new THREE.Vector3(min.x, min.y, min.z));
	vertices.push(new THREE.Vector3(min.x, min.y, max.z));

	// Update the bounding box with the new vertices
	bbox.geometry.vertices = vertices;
	bbox.geometry.verticesNeedUpdate = true;

}


/**
 * Estimates upper and lower bounds from the environment and updates the bounding box.
 *
 * @param 	None
 * @return 	None
 */
function estimateBounds() {
	var estimated = new THREE.BoundingBoxHelper(env, 0x000000);
	estimated.update();

	$("[name='volume.min.x']").val(estimated.box.min.x);
	$("[name='volume.min.y']").val(estimated.box.min.y);
	$("[name='volume.min.z']").val(estimated.box.min.z);

	$("[name='volume.max.x']").val(estimated.box.max.x);
	$("[name='volume.max.y']").val(estimated.box.max.y);
	$("[name='volume.max.z']").val(estimated.box.max.z);

	updateBounds();
}


/**
 * Extracts the path from the data and draws a spline to show the path.
 * Also creates a path robot which can later be animated by the user.
 *
 * @param 	{Object} solutionData An object containing information from the
 * 			server about the solution and path.
 * @return 	None
 */
function visualizePath(solutionData) {

	// Store the path globally
	path = solutionData.path;
	pathVectorsArray = [];

	// Parse the path string into an array of position vectors
	for (var i = 0; i < path.length; i++) {
		pathVectorsArray.push(new THREE.Vector3(
			parseFloat(path[i][0]), parseFloat(path[i][1]), parseFloat(path[i][2])
		));
	}

	// Create the spline
	var spline = new THREE.SplineCurve3(pathVectorsArray);
	var material = new THREE.LineBasicMaterial({
		color: 0x329B71,
	});
	var geometry = new THREE.Geometry();
	var splinePoints = spline.getPoints(pathVectorsArray.length);
	for (var i = 0; i < splinePoints.length; i++) {
		geometry.vertices.push(splinePoints[i]);
	}

	path_line = new THREE.Line(geometry, material);
	path_line.parent = env;
	scene.add(path_line);

	// Create the path robot
	var path_robot_loader = new THREE.ColladaLoader();
	path_robot_loader.options.convertUpAxis = true;
	path_robot_loader.load(robot_loc, function(collada){
		path_robot = collada.scene.children[0];
		path_robot.scale.set(1,1,1);

		// Initially place the path robot at the start state
		path_robot.position.x = start_robot.position.x;
		path_robot.position.y = start_robot.position.y;
		path_robot.position.z = start_robot.position.z;

		path_robot.rotation.x = start_robot.rotation.x;
		path_robot.rotation.y = start_robot.rotation.y;
		path_robot.rotation.z = start_robot.rotation.z;

		scene.add(path_robot);
	});

}


/**
 * Moves the path robot along the path by one step. This function is called on
 * an interval to create the animation of the robot traveling along the path.
 *
 * @param 	None
 * @return 	None
 */
function moveRobot() {

	// If the robot is not at the end of the path, step forward
	if (step < path.length) {
		// Set the new position
		path_robot.position.set(path[step][0], path[step][1], path[step][2]);

		// Set the new rotation
		path_robot.quaternion.set(parseFloat(path[step][3]), parseFloat(path[step][4]),
			parseFloat(path[step][5]), parseFloat(path[step][6]));

		step += 1;
	} else {
		// If the robot is at the end of the path, restart from the beginning
		step = 0;
	}

}


/**
 * Statically displays a robot at each point along the entire path.
 *
 * @param 	None
 * @return  None
 */
function showRobotPath() {

	// If the path robots do not already exist, create them
	if (staticPathRobots.length == 0) {

		// Load the robot file
		var path_robot_loader = new THREE.ColladaLoader();
		path_robot_loader.options.convertUpAxis = true;
		path_robot_loader.load(robot_loc, function(collada){
			var path_robot = collada.scene;
			var skin = collada.skins[0];

			// Clone the path robot and place it along each point in the path
			for (var i = 0; i < path.length; i++){
				var temp_robot = path_robot.clone();
				temp_robot.scale.set(1,1,1);
				temp_robot.position.set(path[i][0], path[i][1], path[i][2]);
				temp_robot.quaternion.set(parseFloat(path[i][3]), parseFloat(path[i][4]),
					parseFloat(path[i][5]), parseFloat(path[i][6]));
				staticPathRobots.push(temp_robot);
				scene.add(temp_robot);
			}
			console.log(staticPathRobots);

		});

	} else {
		// Since the static robots have already been created, just reload them
		staticPathRobots.forEach(function (element, index) {
			element.visible = true;
		});
	}

}


/**
 * Takes the static robots out of the environment and stores them
 * for easy reloading later.
 *
 * @param 	None
 * @return 	None
 */
function hideRobotPath() {

	// Hide each static path robot from the scene
	staticPathRobots.forEach(function (element, index) {
		element.visible = false;
	});

}


/**
 * Converts a rotation from axis-angle representation into a quaternion.
 *
 * @param {float} xyz The unit vector to rotate around.
 * @param {float} theta The number of radians to rotate.
 */
function axisAngleToQuaternion(x, y, z, theta) {

	var q = new THREE.Quaternion();
	q.setFromAxisAngle(new THREE.Vector3(x, y, z), theta);

	return q;

}


/**
 * Translates a quaternion into degrees around each axis.
 *
 * @param 	{THREE.Quaternion} q A quaternion to be converted
 * @return 	{Object} rot An object describing the number of degrees of rotation
 * 			around each axis
 */
function quaternionToAxisDegrees(q) {

	var rot = {};
	rot.x = RAD_TO_DEG * Math.atan2(2.*(q.w*q.x+q.y*q.z), 1.-2.*(q.x*q.x+q.y*q.y));
	rot.y = RAD_TO_DEG * Math.asin(Math.max(Math.min(2.*(q.w*q.y-q.z*q.x),1.),-1.));
	rot.z = RAD_TO_DEG * Math.atan2(2.*(q.w*q.z+q.x*q.y), 1.-2.*(q.y*q.y+q.z*q.z));

	return rot;
}

