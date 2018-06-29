var depth_node = new ROSLIB.Service({
    ros : ros,
    name : '/powerboad/power_board_data',
    messageType : '/peripherals/powerboard'
  });

  Ros.prototype.getTopicsForType = function(topicType, callback, failedCallback) {
    var topicsForTypeClient = new Service({
      ros : this,
      name : '/rosapi/topics_for_type',
      serviceType : 'rosapi/TopicsForType'
    });
    var request = new ServiceRequest({
      type: topicType
    });
    if (typeof failedCallback === 'function'){
      topicsForTypeClient.callService(request,
        function(result) {
          callback(result.topics);
        },
        function(message){
          failedCallback(message);
        }
      );
    }else{
      topicsForTypeClient.callService(request, function(result) {
        callback(result.topics);
      });
    }
  };