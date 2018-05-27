# OMPL Web App {#webapp}

__Contents__:

- \ref webapp_usage
- \ref webapp_deploy
- \ref webapp_licenses

# Usage {#webapp_usage}

The [OMPL web app](http://omplapp.kavrakilab.org) is a web front end for motion planning and benchmarking using OMPL. Basic usage is very similar to the standalone [GUI](gui.html). Since you only need a web browser, this may be the easiest way to try out OMPL without having to install or compile anything. Detailed instructions on how to use the site are available [on the site itself](http://omplapp.kavrakilab.org/) (click on “About” in the top right).

One of the cool features of the [OMPL web app](http://omplapp.kavrakilab.org) is that you can configure and run motion planning benchmarks on our server. Once you have configured a motion planning problem, you can specify that this problem should be solved by various different planning algorithm. The results will be computed in the background and passed on to [Planner Arena](http://plannerarena.org), a server that is designed for letting you explore interactively the results of motion planning benchmarks.

# Deploying the OMPL Web App Locally {#webapp_deploy}

It is possible to run the [OMPL web app](http://omplapp.kavrakilab.org) locally, i.e., run the web server on your own machine. The instructions below have been tested on Ubuntu Linux.

1. [Compile and install](installation.html) OMPL.app (including its Python bindings).
2. Install the Python Flask and Celery packages:

        sudo apt-get install python-pip
        pip install flask
        pip install celery

3. Install RabbitMQ (the daemon is automatically launched upon install):

        sudo apt-get install rabbitmq-server

4. Run the OMPL web app:

        ompl_webapp

On macOS, the following steps get the Web App can be installed and launched using [Macports](http://macPorts.org) with these commands:

    sudo port install ompl +app
    sudo port load redis
    ompl_webapp

# JavaScript Libraries Used by the OMPL Web App {#webapp_licenses}

| Library                 | License                                                         | Distributable   |
| ----------------------- | :-------------------------------------------------------------- | --------------- |
| [jQuery](https://jquery.com/) | [MIT](https://github.com/jquery/jquery/blob/master/LICENSE.txt) | Yes             |
| [blockUI](http://malsup.com/jquery/block/) (jQuery plugin) | [MIT and GPL Dual License](https://github.com/malsup/blockui/) | Yes (under MIT) |
| [Bootstrap](http://getbootstrap.com) | [MIT](https://github.com/twbs/bootstrap/blob/master/LICENSE) | Yes             |
| [THREE.js](threejs.org) | [MIT](https://github.com/mrdoob/three.js/blob/master/LICENSE)   | Yes             |
| TrackballControls       | MIT (part of THREE.js)                                          | Yes             |
| ColladaLoader           | MIT (part of THREE.js)                                          | Yes             |
