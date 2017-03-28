//ParameterCamera
var ParameterCamera = new ROSLIB.Param({
    ros: ros,
    name: '/interface/parametercamera',
});
/*function ParameterCameraTransfer() {
	var ParameterCameraBox = [];
	$("[name=CameraElement]").each(function() {
	    ParameterCameraBox.push(parseInt($(this).val()));
	});
	console.log(ParameterCameraBox);
	ParameterCamera.set(ParameterCameraBox);
}*/
function ParameterCameraValue(){
	var value = parseInt(document.getElementsByName('CameraElement')[0].value);
	console.log(value);
    ParameterCamera.set(value);
}
//ParameterCenter
var ParameterCenter = new ROSLIB.Param({
    ros: ros,
    name: '/interface/parametercenter',
});
function ParameterCenterTransfer(){
	var box = [];
	$("[name=CenterElement]").each(function() {
	    box.push(parseInt($(this).val()));
	});
	console.log(box);
	ParameterCenter.set(box);
}
/*ParameterCenter.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("CenterElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});*/
//ParameterScan
var ParameterScan = new ROSLIB.Param({
    ros: ros,
    name: '/interface/parameterscan',
});
function ParameterScanTransfer(){
	var box = [];
	$("[name=ScanElement]").each(function() {
	    box.push(parseInt($(this).val()));
	});
	console.log(box);
	ParameterScan.set(box);
}
//ParameterHSV
var ParameterHSV = new ROSLIB.Param({
    ros: ros,
    name: '/interface/parameterhsv',
});
//ParameterWhite
var ParameterWhite = new ROSLIB.Param({
    ros: ros,
    name: '/interface/parameterwhite',
});
function ParameterWhiteTransfer(){
	var box = [];
	$("[name=WhiteElement]").each(function() {
	    box.push(parseInt($(this).val()));
	});
	console.log(box);
	ParameterWhite.set(box);
}
//ParameterBlack
var ParameterBlack = new ROSLIB.Param({
    ros: ros,
    name: '/interface/parameterblack',
});
function ParameterBlackTransfer(){
	var box = [];
	$("[name=BlackElement]").each(function() {
	    box.push(parseInt($(this).val()));
	});
	console.log(box);
	ParameterBlack.set(box);
}
