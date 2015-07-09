
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
