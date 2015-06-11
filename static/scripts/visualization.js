var scene;
var camera;
var renderer;
var controls;
var env;
var robot;

var path = parsePath();
init();
animate();

console.log(solutionData);

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

function init() {
	// Create a new scene
	scene = new THREE.Scene();

	// Set width and height
	var WIDTH = window.innerWidth/1.75;
	var HEIGHT = window.innerHeight/1.5;

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
		var WIDTH = window.innerWidth/1.75;
		var HEIGHT = window.innerHeight/1.5;
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
	
        // light1 = new THREE.DirectionalLight( 0xffffff, 1 );
 //        light1.position.set( 100, 100, 100 );
 //        scene.add( light1 );
 //
 //        light2 = new THREE.DirectionalLight( 0xffffff, 1 );
 //        light2.position.set( -100, -100, -100 );
 //        scene.add( light2 );
	
	// scene.add( new THREE.AmbientLight( 0xffffff, 0.1 ) );

	// var light1 = new THREE.DirectionalLight(0xffffff, 0.8);
	// light1.position.set(0,300,0);
	// scene.add(light1);
	//
	// var light2 = new THREE.DirectionalLight(0xffffff, 0.8);
	// light2.position.set(0,0,300);
	// scene.add(light2);

	var loader = new THREE.ColladaLoader();
	loader.options.converUpAxis = true;
	loader.load(solutionData.env_location, function(collada){
		env = collada.scene.children[0];
		var skin = collada.skins[0];

		env.position.set(0,0,0); // x,z,-y in blender dimensions
		env.rotation.set(210,0,0);
		env.scale.set(1,1,1);
		scene.add(env);

		// var axes = new THREE.AxisHelper(50);
		// axes.position = env.position;
		// scene.add(axes);

		// var gridXZ = new THREE.GridHelper(100, 10);
		// gridXZ.setColors( new THREE.Color(0x8f8f8f), new THREE.Color(0x8f8f8f) );
		// gridXZ.position.set(0,0,0);
		// scene.add(gridXZ);
	});
	
	var robot_loader = new THREE.ColladaLoader();
	robot_loader.load(solutionData.robot_location, function(collada){
		robot = collada.scene;
		var skin = collada.skins[0];

		// robot.position.set(path[0][0], path[0][1], path[0][2]);
		// robot.rotation.set(path[0][3], path[0][4], path[0][5]);
		robot.scale.set(1,1,1);
		
		robot.position.x = path[0][0];
		robot.position.y = path[0][1]; 
		robot.position.z = path[0][2];
		robot.rotation.x = path[0][3];
		robot.rotation.y = path[0][4]; 
		robot.rotation.z = path[0][5];

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

