var scene;
var camera;
var renderer;
var controls;
var env;
var robot;

// var path = parsePath();
var path=[[0,0,0,0,0,0]]
// var env_location = "static/uploads/Abstract_env.dae";
// var robot_location = "static/uploads/Abstract_robot.dae";
// init();
// animate();


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

function clearAnimation() {
	// Prep the canvas for loading a new env and robot
}

function updateViz() {
}

function initViz(env_path, robot_path) {
	clearAnimation();

	console.log(robot_path, env_path);
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

	var loader = new THREE.ColladaLoader();
	loader.options.converUpAxis = true;
	loader.load(env_path, function(collada){
		env = collada.scene.children[0];
		var skin = collada.skins[0];

		// Initially set position at origin
		env.position.set(0, 0, 0); // x,z,-y in blender dimensions
		env.rotation.set(0, 0, 0);
		env.scale.set(1,1,1);

		scene.add(env);
	});


	var robot_loader = new THREE.ColladaLoader();
	robot_loader.load(robot_path, function(collada){
		robot = collada.scene;
		var skin = collada.skins[0];

		// Initially set position at origin
		robot.position.set(0, 0, 0);
		robot.rotation.set(0, 0, 0);
		robot.scale.set(1,1,1);

		scene.add(robot);
	});

	// Create the controls
	controls = new THREE.OrbitControls(camera, renderer.domElement);
}

function animate() {
	requestAnimationFrame(animate);
	renderer.render(scene, camera);
	controls.update();
}

