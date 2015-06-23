var scene;
var camera;
var renderer;
var controls;
var env;
var robot;

init();
animate();

function init() {
	// Create a new scene
	scene = new THREE.Scene();

	// Set width and height
	var WIDTH = window.innerWidth;
	var HEIGHT = window.innerHeight;

	// Create a renderer
	renderer = new THREE.WebGLRenderer({alpha:true, antialias:true});
	renderer.setSize(WIDTH, HEIGHT);
	renderer.setClearColor(0xffffff);
	document.body.appendChild(renderer.domElement);

	// Create a camera
	camera = new THREE.PerspectiveCamera(45, WIDTH / HEIGHT, 0.1, 10000);
	// camera.position.set(0,-500,-100);
	camera.position.z = 300;

	// Attach the camera to the scene
	scene.add(camera);

	// Resize the viewer if the window is resized by the user
	window.addEventListener('resize', function(){
		var WIDTH = window.innerWidth;
		var HEIGHT = window.innerHeight;
		renderer.setSize(WIDTH, HEIGHT);
		camera.aspect = WIDTH / HEIGHT;
		camera.updateProjectionMatrix();
	});

	var light = new THREE.DirectionalLight(0xffffff,0.8);
	light.position.set(300,0,0);
	scene.add(light);
	
	// var sphereSize = 1;
	// var pointLightHelper = new THREE.PointLightHelper( light, sphereSize );
	// scene.add( pointLightHelper );

	var light2 = new THREE.DirectionalLight(0xffffff, 0.8);
	light2.position.set(0,0,300);
	scene.add(light2);

	// var sphereSize = 1;
	// var pointLightHelper2 = new THREE.PointLightHelper( light2, sphereSize );
	// scene.add( pointLightHelper2 );

	// var light3 = new THREE.PointLight(0xFFFFFF, 0.8);
	// light3.position.set(0,0,0);
	// scene.add(light3);

	// var sphereSize = 1;
	// var pointLightHelper3 = new THREE.PointLightHelper( light3, sphereSize );
	// scene.add( pointLightHelper3 );

	var loader = new THREE.ColladaLoader();
	loader.options.converUpAxis = true;
	loader.load('static/resources/Abstract_env.dae', function(collada){
		env = collada.scene.children[0];
		var skin = collada.skins[0];

		env.position.set(0,100,0); // x,z,-y in blender dimensions
		env.scale.set(1,1,1);
		scene.add(env);

		var axes = new THREE.AxisHelper(50);
		axes.position = env.position;
		scene.add(axes);

		var gridXZ = new THREE.GridHelper(100, 10);
		gridXZ.setColors( new THREE.Color(0x8f8f8f), new THREE.Color(0x8f8f8f) );
		gridXZ.position.set(0,0,0);
		scene.add(gridXZ);
	});
	
	// var geometry = new THREE.BoxGeometry( 5, 5, 5 );
	// var material = new THREE.MeshLambertMaterial( { color: 0xFF0000 } );
	// var mesh = new THREE.Mesh( geometry, material );
     // console.log("Mesh: ", mesh);
	// scene.add( mesh );
	// console.log("matrix: ", new THREE.Matrix4());
	// mesh.geometry.applyMatrix( new THREE.Matrix4().setTranslation( -10,-22,-30 ) );
	
	var robot_loader = new THREE.ColladaLoader();
	robot_loader.load('static/resources/Abstract_robot.dae', function(collada){
		robot = collada.scene;
		var skin = collada.skins[0];

		robot.position.set(0,0,0);
		robot.scale.set(1,1,1);

		scene.add(robot);
	});
	// Create the controls
	controls = new THREE.OrbitControls(camera, renderer.domElement);
}

function animate() {
	requestAnimationFrame(animate);
	// robot.position.set(84.98,-60,180.16);
	renderer.render(scene, camera);
	controls.update();
	alert("talsd;function");
}

