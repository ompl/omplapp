/**
 * Displays dismissable alerts to the page.
 *
 * @param {String} page The page on which to display the alert, 'configuration' or 'benchmarking'
 * @param {String} type The style (color) of the alert: 'info', 'success', 'warning', 'danger'
 * @param {String} msg The body text of the alert
 * @return None
 */
function showAlert(page, type, msg) {
	var alert = "<div class='alert alert-dismissible";
	alert += " alert-" + type;
	alert += "' role='alert'>";

	alert += "<button type='button' class='close' data-dismiss='alert' aria-label='Close'><span aria=hidden='true'>&times;</span></button>";

	alert += msg

	alert += "</div>";

	var area = "#" + page + "-alerts";
	$(area).html(alert);
}

/**
 * Prompts the user to download a file.
 *
 * @param {Blob} blob The file blob to download
 * @param {String} name The filename
 * @return None
 */
function downloadFile(blob, name) {
	var url = window.URL.createObjectURL(blob);
	var a = document.createElement("a");
	document.body.appendChild(a);
	a.style = "display: none";
	a.href = url;
	a.download = name;
	a.click();
	window.URL.revokeObjectURL(url);
}



/* Keyboard Shortcuts */

Mousetrap.bind('ctrl+c', function(e) {
	$("#configuration-page").click();
});

Mousetrap.bind('ctrl+b', function(e) {
	$("#benchmarking-page").click();
});

Mousetrap.bind('ctrl+s', function(e) {
	$("#configuration-page").click();
	$("#solve-tab").click();
	$("#solve-button").click();
});

Mousetrap.bind('ctrl+e', function(e) {
	if ($("#problems").val() == "custom") {
		$("#configuration-page").click();
		$("#problem-tab").click();
		$("#env_path").click();
	}
});

Mousetrap.bind('ctrl+r', function(e) {
	if ($("#problems").val() == "custom") {
		$("#configuration-page").click();
		$("#problem-tab").click();
		$("#robot_path").click();
	}
});

Mousetrap.bind('ctrl+o', function(e) {
	if ($("#problems").val() == "custom") {
		$("#configuration-page").click();
		$("#problem-tab").click();
		$("#config").click();
	}
});

Mousetrap.bind('ctrl+u', function(e) {
	if ($("#problems").val() == "custom") {
		$("#configuration-page").click();
		$("#problem-tab").click();
		$("#uploadModelsButton").click();
	}
});
