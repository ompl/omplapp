/* Globally Needed Information */
var env_loc;
var robot_loc;
var LIGHT = "rgb(250, 250, 250)";
var DARK = "rgb(25, 25, 25)";


/* Basic Scene Objects */
var scene;
var camera;
var renderer;
var controls;
var axisHelper;

/* Problem Objects */
var env;
var robot_model;
var start_robot;
var goal_robot;
var bbox;
var path_robot;
var pathLines = [];

/* Animation Information */
var path;
var step = 0;
var staticPathRobots = [];
var exploredStates = [];


/**
 * Draws the scene. Only needs to be called once, by 'initViz()'.
 *
 * @param None
 * @return None
 */
render = function() {
    requestAnimationFrame(render);
    renderer.render(scene, camera);
    controls.update();
}


// Define the Visualization class
var Visualization = function() {
    this.animationSpeed = 1000 - $('#animationSpeed').val();
}


/**
 * Sets up the initial scene and loads the lights, camera, and axis helper.
 *
 * @param None
 * @return None
 */
Visualization.prototype.initialize = function() {
    // Create a new scene
    scene = new THREE.Scene();

    // Set width and height
    var WIDTH = $('#viewer').css('width').replace("px", "");
    var HEIGHT = $('#viewer').css('height').replace("px", "");

    // Create a renderer
    renderer = new THREE.WebGLRenderer({alpha:true, antialias:true});
    renderer.setSize(WIDTH, HEIGHT);
    if (!localStorage.getItem("clear_color")) {
        // Default to light theme
        renderer.setClearColor(LIGHT);
    } else {
        // Set to user preference
        if (localStorage.getItem("clear_color") == "light") {
            renderer.setClearColor(LIGHT);
        } else {
            renderer.setClearColor(DARK);
        }
    }
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

    // Create the axis helper, hidden by default, can be turned on
    axisHelper = new THREE.AxisHelper( 500 );
    if (!localStorage.getItem("show_axis")) {
        // Hide by default
        axisHelper.visible = false;
    } else {
        // Load user preference
        if (localStorage.getItem("show_axis") == "true") {
            axisHelper.visible = true;
        } else {
            axisHelper.visible = false;
        }
    }
    scene.add( axisHelper );

    // Create a light and attach it to the camera
    var point_light = new THREE.PointLight(0xffffff, 1.0, 0);
    camera.add(point_light);


    // Render everything to the screen
    render();
};


/**
 * Creates environment, bounding box, and start/goal robot objects and draws
 * them to the screen.
 *
 * @param {String} e_loc The URL of the environment file
 * @param {String} r_loc The URL of the robot file
 * @return None
 */
Visualization.prototype.drawModels = function(e_loc, r_loc) {
    // Store locations globally
    env_loc = e_loc;
    robot_loc = r_loc;

    // Clear the scene in case there are robots/env left from a previous draw
    this.clearScene()

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

    // Create the bounding box
    var geometry = new THREE.BoxGeometry(1,1,1);
    var material = new THREE.MeshBasicMaterial({
        color: 0xf15c40,
        wireframe: true
    });
    bbox = new THREE.Mesh(geometry, material);
    scene.add(bbox);

    // Load the robot file
    var robot_loader = new THREE.ColladaLoader();
    robot_loader.options.convertUpAxis = true;
    robot_loader.load(robot_loc, function(collada){

        // Add the start robot to the scene
        robot_model = collada.scene.children[0];
        robot_model.position.set(0,0,0);
        robot_model.scale.set(1,1,1);

        start_robot = robot_model.clone();
        start_robot.name = "start_robot";
        scene.add(start_robot);

        goal_robot = robot_model.clone();
        goal_robot.name = "goal_robot";
        scene.add(goal_robot);

        visualization.updatePose();
        visualization.updateBounds();
        visualization.applyOffset(env_loc, robot_loc);
    });


}


/**
 * description
 *
 */
Visualization.prototype.applyOffset = function(env, robot) {
    var form = {}
    form['env_loc'] = env;
    form['robot_loc'] = robot;
    $.ajax({
        url: '/offset',
        type: 'POST',
        data: form,
        success: function (data, textStatus, jqXHR) {
            // success callback
            offset = JSON.parse(data);

            robot_model.children[0].position.x -= offset.x;
            robot_model.children[0].position.y -= offset.y;
            robot_model.children[0].position.z -= offset.z;

            start_robot.children[0].position.x -= offset.x;
            start_robot.children[0].position.y -= offset.y;
            start_robot.children[0].position.z -= offset.z;

            goal_robot.children[0].position.x -= offset.x;
            goal_robot.children[0].position.y -= offset.y;
            goal_robot.children[0].position.z -= offset.z;
        },
        error: function (jqXHR, textStatus, errorThrown) {
            // error callback
            console.log(jqXHR);
            showAlert("configuration", "danger", "Error loading robot model. Refresh the page and try again.");
        }
    });

}


Visualization.prototype.updatePose = function() {
    // Update the start position
    var start = {};
    start.x = $("[name='start.x']").val();
    start.y = $("[name='start.y']").val();


    // Update the goal position
    var goal = {};
    goal.x = $("[name='goal.x']").val();
    goal.y = $("[name='goal.y']").val();


    if (problem.is3D == true) {
        // Update start pose
        start.z = $("[name='start.z']").val();
        var startRot = {};
        startRot.x = $("[name='start.deg.x']").val();
        startRot.y = $("[name='start.deg.y']").val();
        startRot.z = $("[name='start.deg.z']").val();
        start_robot.rotation.set(DEG_TO_RAD*startRot.x, DEG_TO_RAD*startRot.y, DEG_TO_RAD*startRot.z);

        // Update goal pose
        goal.z = $("[name='goal.z']").val();
        var goalRot = {};
        goalRot.x = $("[name='goal.deg.x']").val();
        goalRot.y = $("[name='goal.deg.y']").val();
        goalRot.z = $("[name='goal.deg.z']").val();
        goal_robot.rotation.set(DEG_TO_RAD*goalRot.x, DEG_TO_RAD*goalRot.y, DEG_TO_RAD*goalRot.z);
    } else {
        start.z = 0;
        goal.z = 0;

        var startRot = DEG_TO_RAD * $("[name='start.yaw']").val();
        start_robot.rotation.z = startRot;
        var goalRot = DEG_TO_RAD * $("[name='goal.yaw']").val();
        goal_robot.rotation.z = goalRot;
    }
    start_robot.position.set(start.x, start.y,  start.z);
    goal_robot.position.set(goal.x, goal.y, goal.z);
};


Visualization.prototype.updateBounds = function() {
    if (problem.is3D == false) {
        $("[name='volume.min.z']").val("0");
        $("[name='volume.max.z']").val("0");
    }
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
};


/**
 * Toggles the animation of robot along solution path.
 *
 * @param None
 * @return None
 */
Visualization.prototype.toggleAnimation = function() {
    if ($('#animateToggleBtn').hasClass('active')) {
        $('#animateToggleBtn').removeClass('active');
        this.hideAnimation();
    } else {
        if ($('#toggleRobotPathBtn').hasClass('active')) {
            $('#toggleRobotPathBtn').click();
        }
        $('#animateToggleBtn').addClass('active');
        this.showAnimation();
    }
};

Visualization.prototype.showAnimation = function() {
        path_robot.visible = true;
        robotAnimationInterval = setInterval(function() {
            visualization.moveRobot(path);
        }, this.animationSpeed);
}

Visualization.prototype.hideAnimation = function() {
        clearInterval(robotAnimationInterval);
        path_robot.visible = false;
}

/**
 * Toggles the display of static robots at each point along solution path.
 *
 * @param None
 * @return None
 */
Visualization.prototype.toggleRobotPath = function() {
    if ($('#toggleRobotPathBtn').hasClass('active')) {
        $('#toggleRobotPathBtn').removeClass('active');

        this.hideRobotPath();
    } else {
        $('#toggleRobotPathBtn').addClass('active');
        this.showRobotPath();

        if ($('#animateToggleBtn').hasClass('active')) {
            $('#animateToggleBtn').click();
        }
    }
};


/**
 * Clears everything from the scene.
 *
 * @param None
 * @return None
 */
Visualization.prototype.clearScene = function() {

    scene.remove(env);
    scene.remove(start_robot);
    scene.remove(goal_robot);
    scene.remove(path_robot);
    scene.remove(bbox);

    pathLines.forEach(function (element, index) {
        scene.remove(element);
    });
    staticPathRobots.forEach(function (element, index) {
        scene.remove(element);
    });
    staticPathRobots = [];

    $("#showExplored").prop("checked", false);
    exploredStates.forEach(function (element, index) {
        scene.remove(element);
    });
    exploredStates = []

    step = 0;
}


/**
 * Clears solution information such as path and static path robots.
 *
 * @param None
 * @return None
 */
Visualization.prototype.clearSolution = function() {

    scene.remove(path_robot);
    pathLines.forEach(function (element, index) {
        scene.remove(element);
    });
    pathLines = [];
    staticPathRobots.forEach(function (element, index) {
        scene.remove(element);
    });
    staticPathRobots = [];

    $("#showExplored").prop("checked", false);
    exploredStates.forEach(function (element, index) {
        scene.remove(element);
    });
    exploredStates = []
}


/**
 * Estimates upper and lower bounds from the environment and updates the bounding box.
 *
 * @param None
 * @return None
 */
Visualization.prototype.calculateBounds = function() {
    var calculated = new THREE.BoundingBoxHelper(env, 0x000000);
    calculated.update();

    $("[name='volume.min.x']").val(calculated.box.min.x);
    $("[name='volume.min.y']").val(calculated.box.min.y);
    $("[name='volume.min.z']").val(calculated.box.min.z);

    $("[name='volume.max.x']").val(calculated.box.max.x);
    $("[name='volume.max.y']").val(calculated.box.max.y);
    $("[name='volume.max.z']").val(calculated.box.max.z);

    this.updateBounds();
}


/**
 * Extracts the path from the data and draws a spline to show the path.
 * Also creates a path robot which can be animated by the user.
 *
 * @param {Object} solutionData An object containing information from the
 * server about the solution and path.
 * @return None
 */
Visualization.prototype.visualizeSolution = function(solutionData) {

    // Store the path globally
    path = solutionData.path;
    this.drawSolutionPath(path);

    // Create the path robot
    path_robot = robot_model.clone();
    path_robot.position.x = start_robot.position.x;
    path_robot.position.y = start_robot.position.y;
    if (problem.is3D == true) {
        path_robot.position.z = start_robot.position.z;
    }
    path_robot.visible = false;
    scene.add(path_robot);
}


/**
 * description
 *
 */
Visualization.prototype.drawSolutionPath = function(path) {
    pathVectorsArray = [];

    // Parse the path string into an array of position vectors
    if (problem.is3D == true) {
        for (var i = 0; i < path.length; i++) {
            pathVectorsArray.push(new THREE.Vector3(
                parseFloat(path[i][0]), parseFloat(path[i][1]), parseFloat(path[i][2])
            ));
        }
    } else {
        for (var i = 0; i < path.length; i++) {
            pathVectorsArray.push(new THREE.Vector3(
                parseFloat(path[i][0]), parseFloat(path[i][1]), 0
            ));
        }
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

    var path_line = new THREE.Line(geometry, material);
    path_line.parent = env;
    pathLines.push(path_line);
    scene.add(path_line);
}


Visualization.prototype.drawExploredStates = function() {
    var states = solution.data.explored_states;

    // Set the radius of the explored states as a function of the robot size
    var box = new THREE.Box3().setFromObject(start_robot);
    var radius = 0.01 * ((box.max.x - box.min.x)+(box.max.y - box.min.y)+(box.max.z - box.min.z));
    var geometry = new THREE.SphereGeometry(radius, 16, 16);
    var material = new THREE.MeshBasicMaterial( {color: 0xff0000} );
    var sphere = new THREE.Mesh( geometry, material );
    exploredStates = [];

    if (states.length > 10000) {
        showAlert("configuration", "warning", "There are a large number of explored states and visualization performance may be affected. Hide the explored states to improve performance.");
    }

    for (var i = 0; i < states.length; i++) {
        var point = sphere.clone();
        point.position.set(states[i][0], states[i][1], states[i][2]);
        exploredStates.push(point);
        scene.add(point);
    }
}

Visualization.prototype.showExploredStates = function() {
    if (exploredStates.length == 0) {
        this.drawExploredStates();
    } else {
        exploredStates.forEach(function (element, index) {
            element.visible = true;
        });
    }
}

Visualization.prototype.hideExploredStates = function() {
    exploredStates.forEach(function (element, index) {
        element.visible = false;
    });
}


/**
 * Moves the path robot along the path by one step. This function is called on
 * an interval to create the animation of the robot traveling along the path.
 *
 * @param None
 * @return None
 */
Visualization.prototype.moveRobot = function() {

    // If the robot is not at the end of the path, step forward
    if (step < path.length) {
        // Set the new position and rotation
        if (problem.is3D == true) {
            path_robot.position.set(parseFloat(path[step][0]), parseFloat(path[step][1]),
                parseFloat(path[step][2]));
            path_robot.quaternion.set(parseFloat(path[step][3]), parseFloat(path[step][4]),
                parseFloat(path[step][5]), parseFloat(path[step][6]));
        } else {
            path_robot.position.set(parseFloat(path[step][0]), parseFloat(path[step][1]), 0);
            path_robot.rotation.z = parseFloat(path[step][2]);
        }

        step += 1;
    } else {
        // If the robot is at the end of the path, restart from the beginning
        step = 0;
    }

}


/**
 * Statically displays a robot at each point along the entire path.
 *
 * @param None
 * @return None
 */
Visualization.prototype.showRobotPath = function() {

    // If the path robots do not already exist, create them
    if (staticPathRobots.length == 0) {

        // Clone the path robot and place it along each point in the path
        for (var i = 0; i < path.length; i++){
            var temp_robot = robot_model.clone();
            temp_robot.scale.set(1,1,1);
            if (problem.is3D == true){
                temp_robot.position.set(parseFloat(path[i][0]), parseFloat(path[i][1]),
                    parseFloat(path[i][2]));
                temp_robot.quaternion.set(parseFloat(path[i][3]), parseFloat(path[i][4]),
                    parseFloat(path[i][5]), parseFloat(path[i][6]));
            } else {
                temp_robot.position.set(parseFloat(path[i][0]), parseFloat(path[i][1]), 0);
                temp_robot.rotation.z = parseFloat(path[i][2]);
            }
            staticPathRobots.push(temp_robot);
            scene.add(temp_robot);
        }

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
 * @param None
 * @return None
 */
Visualization.prototype.hideRobotPath = function() {

    // Hide each static path robot from the scene
    staticPathRobots.forEach(function (element, index) {
        element.visible = false;
    });

}

