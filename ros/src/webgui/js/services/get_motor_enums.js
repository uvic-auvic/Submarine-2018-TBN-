function recv_motor_eunms(message){
    $("#rpm-table-body").text("");
    

    for(key in message.motors)
    {
        var motor_name = key;
        if(motor_name.slice(-3) == "idx")
        {
            motor_name = motor_name.substr(0, (motor_name.length - 3));
        }
        motor_name = motor_name.replace(/_/g, ' ');
        
        $("#rpm-table-body").append("<tr><td>" + motor_name + "</td><td><span id='rpm-"+message.motors[key]+"'></span> RPM</td></tr>");  
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

call_motor_enums();

var rpms_in = new ROSLIB.Topic({
    ros : ros,
    name : '/motor_controller/MotorsRPMs/',
    messageType : '/peripherals/rpms/'
});

rpms_in.subscribe(function(message) {
  //console.log('Received message on ' + depth_node.name + ': ' + message.temperature);
  for(var k=0; k<message.rpms.length;k++)
  {
    $("#rpm-"+k).text(message.rpms[k]);
  }
});