/* Mathematic Constants */
var DEG_TO_RAD = Math.PI/180;
var RAD_TO_DEG = 180/Math.PI;


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


/**
 * Converts a rotation from axis-angle representation into a quaternion.
 *
 * @param {float} xyz The unit vector to rotate around.
 * @param {float} theta The number of radians to rotate.
 * @return {THREE.Quaternion} The resulting quaternion
 */
function axisAngleToQuaternion(x, y, z, theta) {

    var q = new THREE.Quaternion();
    q.setFromAxisAngle(new THREE.Vector3(x, y, z), theta);

    return q;

}


/**
 * Translates a quaternion into degrees around each axis.
 *
 * @param {THREE.Quaternion} q A quaternion to be converted
 * @return {Object} An object describing the number of degrees of rotation
 * around each axis
 */
function quaternionToAxisDegrees(q) {

    var rot = {};
    rot.x = RAD_TO_DEG * Math.atan2(2.*(q.w*q.x+q.y*q.z), 1.-2.*(q.x*q.x+q.y*q.y));
    rot.y = RAD_TO_DEG * Math.asin(Math.max(Math.min(2.*(q.w*q.y-q.z*q.x),1.),-1.));
    rot.z = RAD_TO_DEG * Math.atan2(2.*(q.w*q.z+q.x*q.y), 1.-2.*(q.y*q.y+q.z*q.z));

    return rot;
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
