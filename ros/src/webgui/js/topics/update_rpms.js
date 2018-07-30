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