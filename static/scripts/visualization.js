var scene;
var camera;
var renderer;
var controls;
var env;
var start_robot;
var goal_robot;
var env_loc;
var robot_loc;
var bbox;
var baseRotation;
var spline;
var counter = 0;
var tangent = new THREE.Vector3();
var axis = new THREE.Vector3();
var up = new THREE.Vector3(0, 1, 0);
var step = 0;

initViz();

/**
 * Creates the initial, static visualization. Loads the robot and environment
 * files and set position and rotation. Also creates lights. The render()
 * function must be called to actually draw the scene to the screen.
 *
 * @param 	{object} data A mapping of all the problem configuration fields
 * 			(start and goal positions and rotations, bounding box, etc.) to their
 * 			values.
 * @return 	None
 */
function initViz() {
	// Create a new scene
	scene = new THREE.Scene();

	// Set width and height
	var WIDTH = $('#viewer').css('width').replace("px", "")-4;
	var HEIGHT = $('#viewer').css('height').replace("px", "")-4;

	// Create a renderer
	renderer = new THREE.WebGLRenderer({alpha:true, antialias:true});
	renderer.setSize(WIDTH, HEIGHT);
	renderer.setClearColor(0x022037);
	$('#viewer').append(renderer.domElement);

	// Create a camera
	camera = new THREE.PerspectiveCamera(45, WIDTH / HEIGHT, 0.1, 10000);
	camera.position.z = 1000;

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
	
	// Create the controls
	controls = new THREE.TrackballControls(camera, renderer.domElement);
	controls.target.set(0,0,0);

	var axisHelper = new THREE.AxisHelper( 500 );
	scene.add( axisHelper );

	var light_top = new THREE.PointLight(0xffffff, 0.7, 0);
	light_top.position.y = 1000;
	scene.add(light_top);

	var light_bottom= new THREE.PointLight(0xffffff, 0.7, 0);
	light_bottom.position.y = -1000;
	scene.add(light_bottom);

	var light_left = new THREE.PointLight(0xffffff, 0.7, 0);
	light_left.position.x = -1000;
	scene.add(light_left);

	var light_right = new THREE.PointLight(0xffffff, 0.7, 0);
	light_right.position.x = 1000;
	scene.add(light_right);

	var light_front = new THREE.PointLight(0xffffff, 0.7, 0);
	light_front.position.z = 1000;
	scene.add(light_front);

	var light_back = new THREE.PointLight(0xffffff, 0.7, 0);
	light_back.position.z = -1000;
	scene.add(light_back);

}

function drawModels(data) {
	env_loc = data.env_loc;
	robot_loc = data.robot_loc;

	// Needed to get the inital orientation right. Add this to every rotation.
	baseRotation = new THREE.Vector3(-1.57079632679, 0, 0);

	var env_loader = new THREE.ColladaLoader();
	// env_loader.options.
	env_loader.options.convertUpAxis = true;
	env_loader.load(env_loc, function(collada){
		env = collada.scene.children[0];
		var skin = collada.skins[0];

		// Initially set position at origin
		env.position.set(0, 0, 0); // x,z,-y in blender dimensions
		// Necessary to get the right initial orientation, try to fix
		env.scale.set(1,1,1);

		scene.add(env);

		bbox = new THREE.BoundingBoxHelper(env, 0xffffff);
		bbox.update();
		scene.add(bbox);
	});

	var start_robot_loader = new THREE.ColladaLoader();
	start_robot_loader.options.convertUpAxis = true;
	start_robot_loader.load(robot_loc, function(collada){
		start_robot = collada.scene;
		var skin = collada.skins[0];

		// Initially set position at origin
		start_robot.position.set(data['start.x'], data['start.y'], data['start.z']);
		start_robot.scale.set(1,1,1);

		// Get rotation data into quaternion form
		var q = new THREE.Quaternion();
		q.setFromAxisAngle(new THREE.Vector3(data['start.axis.x'], data['start.axis.y'],
				data['start.axis.z']), data['start.theta']);

		// Set the goal robot's rotation with quaternion
		start_robot.rotation.setFromQuaternion(q);

		scene.add(start_robot);
	});

	var goal_robot_loader = new THREE.ColladaLoader();
	goal_robot_loader.options.convertUpAxis = true;
	goal_robot_loader.load(data.robot_loc, function(collada){
		goal_robot = collada.scene;
		var skin = collada.skins[0];

		// Initially set position at origin
		goal_robot.position.set(data['goal.x'], data['goal.y'], data['goal.z']);
		goal_robot.scale.set(1,1,1);

		// Get rotation data into quaternion form
		var q = new THREE.Quaternion();
		q.setFromAxisAngle(new THREE.Vector3(data['goal.axis.x'], data['goal.axis.y'],
				data['goal.axis.z']), data['goal.theta']);

		// Set the goal robot's rotation with quaternion
		goal_robot.rotation.setFromQuaternion(q);

		scene.add(goal_robot);
	});

	render();
}

/**
 * Draws the scene created by initViz() to the screen.
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
 * Clears the canvas so that a new robot and envrionment can be loaded.
 *
 * @param 	None
 * @return 	None
 */
function clearAnimation() {
	cancelAnimationFrame(render);
	scene = null;
	camera = null;
	controls = null;
	initViz();
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

	degToRad = Math.PI/180;
	// Update start rotation
	var startRot = {};
	startRot.x = $("[name='start.axis.x']").val()
	startRot.y = $("[name='start.axis.y']").val()
	startRot.z = $("[name='start.axis.z']").val()
	start_robot.rotation.set(degToRad*startRot.x, degToRad*startRot.y, degToRad*startRot.z);

	// Update the goal position
	goal = {};
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
	goal_robot.rotation.set(degToRad*goalRot.x, degToRad*goalRot.y, degToRad*goalRot.z);
}

function visualizePath(solutionData) {
	var path = solutionData.path;
	pathVectorsArray = [];
	console.log(path);
	for (var i = 0; i < path.length; i++) {
		pathVectorsArray.push(new THREE.Vector3(
			parseFloat(path[i][0]), parseFloat(path[i][1]), parseFloat(path[i][2])
		));
	}
	
	console.log(pathVectorsArray);

	spline = new THREE.SplineCurve3(pathVectorsArray);
	
	console.log("spline: ", spline);
	
	setInterval(function() {
		moveRobot(path);
	}, 100);
}

function moveRobot(path) {
	if (counter <= 1) {
		start_robot.position.copy(spline.getPointAt(counter));
		// start_robot.quaternion = new THREE.Quaternion(
			// parseFloat(path[step][3]), parseFloat(path[step][4]),
			// parseFloat(path[step][5]), parseFloat(path[step][6])		
		// );
		
		// tangent = spline.getTangentAt(counter).normalize();
		// axis.crossVectors(up, tangent).normalize();
		// var radians = Math.acos(up.dot(tangent));
		// start_robot.quaternion.setFromAxisAngle(axis, radians);

		counter += 0.005;
		step += 1;
	} else {
		counter = 0;
		step = 0;
	}
}


function axisAngleToQuaternion(x, y, z, theta) {
	var q = new THREE.Quaternion();
	q.setFromAxisAngle(new THREE.Vector3(x, y, z), theta);

	return q;
}

function quaternionToAxisDegrees(q) {
    console.log("q: ", q);
    rad2deg = 180/Math.PI;

    var rot = {};
    rot.x = rad2deg * Math.atan2(2.*(q.w*q.x+q.y*q.z), 1.-2.*(q.x*q.x+q.y*q.y));
    rot.y = rad2deg * Math.asin(Math.max(Math.min(2.*(q.w*q.y-q.z*q.x),1.),-1.));
    rot.z = rad2deg * Math.atan2(2.*(q.w*q.z+q.x*q.y), 1.-2.*(q.y*q.y+q.z*q.z));

    return rot;
}

