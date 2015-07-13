
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


/* Keyboard Shortcuts */

Mousetrap.bind('ctrl+c', function(e) {
	$("#configure-problem-page").click();
});

Mousetrap.bind('ctrl+b', function(e) {
	$("#benchmarking-page").click();
});

Mousetrap.bind('ctrl+s', function(e) {
	$("#configure-problem-page").click();
	$("#solve-tab").click();
	$("#solve-button").click();
});

Mousetrap.bind('ctrl+e', function(e) {
	if ($("#problems").val() == "custom") {
		$("#configure-problem-page").click();
		$("#problem-tab").click();
		$("#env_path").click();
	}
});

Mousetrap.bind('ctrl+r', function(e) {
	if ($("#problems").val() == "custom") {
		$("#configure-problem-page").click();
		$("#problem-tab").click();
		$("#robot_path").click();
	}
});

Mousetrap.bind('ctrl+o', function(e) {
	if ($("#problems").val() == "custom") {
		$("#configure-problem-page").click();
		$("#problem-tab").click();
		$("#config").click();
	}
});
