var rpms_act = new ROSLIB.Topic({
    ros : ros,
    name : '/motor_controller/MotorsRPMs/',
    messageType : '/peripherals/rpms/'
});

rpms_act.subscribe(function(message) {
  //console.log('Received message on ' + depth_node.name + ': ' + message.temperature);
  for(var k=0; k<message.rpms.length;k++)
  {
    $("#rpm-act-"+k).text(message.rpms[k]);
  }
});

var rpms_set = new ROSLIB.Topic({
    ros : ros,
    name : '/nav/rpms',
    messageType : '/peripherals/rpms/'
});

rpms_set.subscribe(function(message) {
  //console.log('Received message on ' + depth_node.name + ': ' + message.temperature);
  for(var k=0; k<message.rpms.length;k++)
  {
    $("#rpm-set-"+k).text(message.rpms[k]);
  }
});