function recv_motor_eunms(message){
    $("#device_list").empty();
    for(var c = 0; c < message.devices.length; c++){
        var item = message.devices[c];
        $("#device_list").append("<tr><td>" + item.name + "</td><td>" + item.port + "</td></tr>");
    }
}

function call_motor_enums(){
    var motor_eunums = new ROSLIB.Service({
        ros : ros,
        name : '/motor_controller/getMotorEnums',
        messageType : '/peripherals/get_motor_enums'
    });

    var request = new ROSLIB.ServiceRequest({});

    motor_eunums.callService(request, function(message){
        recv_motor_eunms(message);
    });
}

$("#get_motor_enums").click(function(){
    call_motor_enums();
});
