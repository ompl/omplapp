# OMPL Web

A web front end for sample based motion planning and benchmarking using the Open Motion Planning Library.

[ompl.kavrakilab.org](http://ompl.kavrakilab.org)

## Installation

### Mac OSX


- Acquire OMPL.app and OMPL source code
  ```
  git clone https://github.com/ompl/omplapp.git
  cd omplapp
  git clone https://github.com/ompl/ompl.git
  ```

- Acquire Flask and Celery
  ```
  pip install flask
  pip install celery
  ```

- Acquire and run RabbitMQ
    - MacPorts
      ```
      sudo port install rabbitmq-server
      ```
    - Standalone:
      [follow instuctions found here.](http://www.rabbitmq.com/install-standalone-mac.html)
  ```
  // Add rabbitmq-server to path
  ```

- Build OMPL and generate Python bindings
  ```
  mkdir -p build/Release
  cd build/Release
  cmake ../..
  make installpyplusplus && cmake .
  make update_bindings
  make -j 4
  ```

- Run webapp startup script
  ```
  cd webapp
  ./webapp
  ```


## Used Libraries

| Library                 | License                                                      | Distributable   |
| ----------------------- | :----------------------------------------------------------- | --------------- |
| jQuery                  | MIT https://github.com/jquery/jquery/blob/master/LICENSE.txt | Yes             |
| blockUI (jQuery plugin) | MIT and GPL Dual License https://github.com/malsup/blockui/  | Yes (under MIT) |
| Bootstrap               | MIT https://github.com/twbs/bootstrap/blob/master/LICENSE    | Yes             |
| THREE.js                | MIT https://github.com/mrdoob/three.js/blob/master/LICENSE   | Yes             |
| TrackballControls       | MIT (part of THREE.js)                                       | Yes             |
| ColladaLoader           | MIT (part of THREE.js)                                       | Yes             |
