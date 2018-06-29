var depth_node = new ROSLIB.Topic({
    ros : ros,
    name : '/powerboad/power_board_data',
    messageType : '/peripherals/powerboard'
  });

  depth_node.subscribe(function(message) {
    //console.log('Received message on ' + depth_node.name + ': ' + message.temperature);
    $('#pb_batt_1_volt').text(message.voltage_battery_1);
    $('#pb_batt_2_volt').text(message.voltage_battery_2);
    $('#pb_motor_curr').text(message.current_motors);
    $('#pb_system_curr').text(message.current_system);
    $('#pb_temperature').text(message.temperature);
    $('#pb_rh').text(message.humidity);
    $('#pb_internal_pressure').text(message.internal_pressure);
    $('#pb_external_pressure').text(message.external_pressure);
  });