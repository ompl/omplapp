var scene;
var camera;
var renderer;
var controls;
var env;
var start_robot;
var goal_robot;
var env_loc;
var robot_loc;


/**
 * Creates the initial, static visualization. Loads the robot and environment
 * files and set position and rotation. Also creates lights. The animate()
 * function must be called to actually draw the scene to the screen.
 *
 * @param 	{object} data A mapping of all the problem configuration fields
 * 			(start and goal positions and rotations, bounding box, etc.) to their
 * 			values.
 * @return 	None
 */
function initViz(data) {
	clearAnimation();

	env_loc = data.env_loc;
	robot_loc = data.robot_loc;

	// Create a new scene
	scene = new THREE.Scene();

	// Set width and height
	var WIDTH = $('#viewer').css('width').replace("px", "")-4;
	var HEIGHT = $('#viewer').css('height').replace("px", "")-4;

	// Create a renderer
	renderer = new THREE.WebGLRenderer({alpha:true, antialias:true});
	renderer.setSize(WIDTH, HEIGHT);
	renderer.setClearColor(0xffffff);
	$('#viewer').append(renderer.domElement);

	// Create a camera
	camera = new THREE.PerspectiveCamera(45, WIDTH / HEIGHT, 0.1, 10000);
	camera.position.z = 800;

	// Attach the camera to the scene
	scene.add(camera);

	// Resize the viewer if the window is resized by the user
	window.addEventListener('resize', function(){
		var WIDTH = $('#viewer').css('width').replace("px", "")-4;
		var HEIGHT = $('#viewer').css('height').replace("px", "")-4;
		renderer.setSize(WIDTH, HEIGHT);
		camera.aspect = WIDTH / HEIGHT;
		camera.updateProjectionMatrix();
	});

	var light_top = new THREE.PointLight(0xffffff, 0.5, 0);
	light_top.position.y = 1000;
	scene.add(light_top);

	var light_bottom= new THREE.PointLight(0xffffff, 1, 0);
	light_bottom.position.y = -1000;
	scene.add(light_bottom);

	var light_left = new THREE.PointLight(0xffffff, 1, 0);
	light_left.position.x = -1000;
	scene.add(light_left);

	var light_right = new THREE.PointLight(0xffffff, 1, 0);
	light_right.position.x = 1000;
	scene.add(light_right);

	var env_loader = new THREE.ColladaLoader();
	env_loader.options.converUpAxis = true;
	env_loader.load(env_loc, function(collada){
		env = collada.scene.children[0];
		var skin = collada.skins[0];

		// Initially set position at origin
		env.position.set(0, 0, 0); // x,z,-y in blender dimensions
		env.rotation.set(0, 0, 0);
		env.scale.set(1,1,1);

		scene.add(env);
	});

	var start_robot_loader = new THREE.ColladaLoader();
	start_robot_loader.load(robot_loc, function(collada){
		start_robot = collada.scene;
		var skin = collada.skins[0];

		// Initially set position at origin
		start_robot.position.set(-50, 0, 0);
		start_robot.rotation.set(0, 0, 0);
		start_robot.scale.set(1,1,1);

		scene.add(start_robot);
	});

	var goal_robot_loader = new THREE.ColladaLoader();
	goal_robot_loader.load(data.robot_loc, function(collada){
		goal_robot = collada.scene;
		var skin = collada.skins[0];

		// Initially set position at origin
		goal_robot.position.set(50, 0, 0);
		goal_robot.rotation.set(0, 0, 0);
		goal_robot.scale.set(1,1,1);

		scene.add(goal_robot);
	});
	// Create the controls
	controls = new THREE.OrbitControls(camera, renderer.domElement);

	animate();
}

/**
 * Draws the scene created by initViz() to the screen.
 *
 * @param 	None
 * @return 	None
 */
function animate() {
	requestAnimationFrame(animate);
	renderer.render(scene, camera);
	controls.update();

	updateViz();
}

/**
 * Clears the canvas so that a new robot and envrionment can be loaded.
 *
 * @param 	None
 * @return 	None
 */
function clearAnimation() {
	// Prep the canvas for loading a new env and robot
}

/**
 * Refreshes the visualization to reflect changes in position, rotation, etc.
 * from user input.
 *
 * @param 	None
 * @return 	None
 */
function updateViz() {
	// Update the start position
	start = {};
	start.x = $("[name='start.x']").val()
	start.y = $("[name='start.y']").val()
	start.z = $("[name='start.z']").val()
	// TODO: Try catch here in case not loaded yet, to avoid error msg
	start_robot.position.set(start.x, start.y, start.z)

	// Update the goal position
	goal = {};
	goal.x = $("[name='goal.x']").val()
	goal.y = $("[name='goal.y']").val()
	goal.z = $("[name='goal.z']").val()
	// TODO: Try catch here in case not loaded yet, to avoid error msg
	goal_robot.position.set(goal.x, goal.y, goal.z)
}

function visualizePath(solutionData) {
	var path = solutionData.path;
	for (var i = 0; i < path.length; i++) {
		console.log("I: ", i);
		console.log(path[i]);

		// Initially set position at origin
		// start_robot.position.set(path[i][0], path[i][1], path[i][2]);
	}
}

function parsePath() {
	// console.log(solutionData.path);
	var path_list = []
	var pathStrings = solutionData.path.split("\n");


	for (var i = 0; i < pathStrings.length; i++) {
		var state = pathStrings[i].trim();
		path_list.push(state.split(" "));
	}

	return path_list;
}


