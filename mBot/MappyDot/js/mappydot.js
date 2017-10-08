// mappydot.js

(function(ext) {
    var device = null;
    
    var ports = {
        Port1: 1,
        Port2: 2,
        Port3: 3,
        Port4: 4
    };
    
	ext.resetAll = function(){};
	
	ext.runArduino = function(){
		
	};
		ext.getRGBComponentScratch = function(port,RGBColorComponent) {
        device.send([port, RGBColorComponent])
    };
	ext.getMappyDotDistance = function(port,Distance){
		device.send([port, Distance])
	};

    function processData(bytes) {
        trace(bytes);
    }

    // Extension API interactions
    var potentialDevices = [];
    ext._deviceConnected = function(dev) {
        potentialDevices.push(dev);

        if (!device) {
            tryNextDevice();
        }
    }

    function tryNextDevice() {
        // If potentialDevices is empty, device will be undefined.
        // That will get us back here next time a device is connected.
        device = potentialDevices.shift();
        if (device) {
            device.open({ stopBits: 0, bitRate: 115200, ctsFlowControl: 0 }, deviceOpened);
        }
    }

    function deviceOpened(dev) {
        if (!dev) {
            // Opening the port failed.
            tryNextDevice();
            return;
        }
        device.set_receive_handler('mappydot',function(data) {
            processData(data);
        });
    };

    ext._deviceRemoved = function(dev) {
        if(device != dev) return;
        device = null;
    };

    ext._shutdown = function() {
        if(device) device.close();
        device = null;
    };

    ext._getStatus = function() {
        if(!device) return {status: 1, msg: 'mapptdot disconnected'};
        return {status: 2, msg: 'mapptdot connected'};
    }

    var descriptor = {};
	ScratchExtensions.register('mappydot', descriptor, ext, {type: 'serial'});
})({});
