

function webGLStart() {
	var canvas = document.getElementById("lesson01-canvas");
	initGL(canvas);
	initShaders();
	initBuffers();

	gl.clearColor(0.0, 0.0, 0.0, 1.0);
	gl.enable(gl.DEPTH_TEST);

	drawScene();
}

console.log("Hello from JS");
