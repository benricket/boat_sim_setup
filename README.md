Boat sim setup

### Using the existing code

Run the simulator, from the gazebo_maritime_ws directory:

```
git@github.com:ArduPilot/ardupilot.git
```

Spawn obstacles:
```
python obs.py test_obs.yaml fixed_obs.yaml 
```

Activate the right environment varibles
You'll need to set these yourself (the path is currently specific to my computer)
```
source tutorial.env
```

Run the Ardupilot simulator:
```
sim_vehicle.py -v Rover -f gazebo-rover --model JSON
```

In the Ardupilot simulator, we need to add a few outputs or links in order for the simulator to communicate with other things we need.
```
sim_vehicle.py -v Rover --model JSON --mavproxy-args="--out=127.0.0.1:14552 --out=127.0.0.1:14553 --master=udp:127.0.0.1:14560"
```

mention these are heartbeat to the vision script, output to log obstacle distances, and link to communicate obstacle distances, 

Run QGroundControl (should be an executable file)

Start streaming the depth image and image
```
gz topic -t /depth_camera/depth_image/enable_streaming -m gz.msgs.Boolean -p "data: 1"
gz topic -t /depth_camera/image/enable_streaming -m gz.msgs.Boolean -p "data: 1"
```

Stream the camera image
```
gst-launch-1.0 -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

Process the depth via Python
```
python process_sim_depth.py
```

#### Setting up a mission

In QGC, with the sim_vehicle simulation running, the vehicle should appear as a connection in QGC, typically showing 'Not Ready' in the corner. By default, the GPS position of the world file is somewhere in Australia, though this can be moved.

Click on the upper left Q icon, and select the 'Analyze Tools' menu. In this menu, the MAVLink inspector allows us to see MAVLink messages coming into the vehicle, as well as their frequencies. In our case, we want to ensure the OBSTACLE_DISTANCES are being logged. For other projects, where other MAVLink messages may be desired, something else may be desirable here.

We need to upload waypoints for the boat to move to. This can be done directly in QGC using the `Plan Flight` option, accessed via the button in the upper left corner. However, it may be desirable for us to automate this process somewhat via a script. To do this, the `upload_waypoints.py` script takes in a list of (lat, lon, alt) points (where altitude doens't matter for our use case) and uploads them to the simulated robot. 

Upon running this script, the boat will immediately enter AUTO state and begin following the mission. 

# Changing this to work with other code

