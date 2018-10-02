/* Extension demonstrating a blocking command block */
/* Sayamindu Dasgupta <sayamindu@media.mit.edu>, May 2014 */

var ros;

new (function() {
    var ext = this;

		$.getScript('http://static.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js', getEventLid);
		$.getScript('http://static.robotwebtools.org/roslibjs/current/roslib.min.js', getRosLid);

		function getEventLid() {
			console.log('Event library loaded');
		}

		function getRosLid() {
			console.log('Ros library loaded');

			ros = new ROSLIB.Ros({
    		url : 'ws://localhost:9090'
	  	});

			ros.on('connection', function() {
    		console.log('Connected to websocket server.');
  		});

  		ros.on('error', function(error) {
    		console.log('Error connecting to websocket server: ', error);
  		});

  		ros.on('close', function() {
    		console.log('Connection to websocket server closed.');
  		});

		}


    // Cleanup function when the extension is unloaded
    ext._shutdown = function() {};

    // Status reporting code
    // Use this to report missing hardware, plugin or unsupported browser
    ext._getStatus = function() {
        return {status: 2, msg: 'Ready'};
    };

    // Functions for block with type 'w' will get a callback function as the 
    // final argument. This should be called to indicate that the block can
    // stop waiting.
    ext.move = function(callback) {
        console.log('Move');

  			var cmdVel = new ROSLIB.Topic({
    			ros : ros,
    			name : '/car/cmd_vel',
    			messageType : 'geometry_msgs/Twist'
  			});

		  	var twist = new ROSLIB.Message({
    			linear : {
      			x : 2.0,
      			y : 0.0,
      			z : 0.0
    			},
    			angular : {
      			x : 0.0,
     				y : 0.0,
      			z : 0.0
    			}	
  			});
				cmdVel.publish(twist);
    };

    ext.stop = function(callback) {
        console.log('Stop');

  			var cmdVel = new ROSLIB.Topic({
    			ros : ros,
    			name : '/car/cmd_vel',
    			messageType : 'geometry_msgs/Twist'
  			});

		  	var twist = new ROSLIB.Message({
    			linear : {
      			x : 0.0,
      			y : 0.0,
      			z : 0.0
    			},
    			angular : {
      			x : 0.0,
     				y : 0.0,
      			z : 0.0
    			}	
  			});
				cmdVel.publish(twist);
    };

		ext.get_postion_x = function(callback) {
				var listener = new ROSLIB.Topic({
					ros : ros,
  				name : '/car/odom',
       		messageType : 'nav_msgs/Odometry'
     		});
   
     		listener.subscribe(function(message) {
       		console.log('Received message on ' + listener.name + ': ' + message.pose.pose.position.x );
       		listener.unsubscribe();
          callback(1.0);
     		});
    };


    // Block and block menu descriptions
    var descriptor = {
        blocks: [
            [' ', 'move', 'move'],
            [' ', 'stop', 'stop'],
					  ['R', 'current position x', 'get_postion_x'],
        ]
    };

    // Register the extension
    ScratchExtensions.register('Random wait extension', descriptor, ext);
})();
