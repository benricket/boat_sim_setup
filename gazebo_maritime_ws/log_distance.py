"""
For testing, make sure we can log the OBSTACLE_DISTANCE parameter in MAVLINK to make sure we're sending it
and the distances are being received properly
"""
import os
from pymavlink import mavutil
import time

os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'

# connect to mavlink
conn = mavutil.mavlink_connection('udpin:127.0.0.1:14553',baud=5760)
conn.wait_heartbeat()
print("Heartbeat received. Logging obstacle distances...")

# Open a log file
logfile = open("obstacle_distance_log.csv", "w")
logfile.write("timestamp,time_usec,frame,min,max,increment,angle_offset,distances\n")

while True:
    # Wait for the next obstacle distance message (non-blocking)
    msg = conn.recv_match(type="OBSTACLE_DISTANCE", blocking=False)
    #msg = conn.recv_match(blocking=False)

    if msg:
        # Convert to dict for easy handling
        d = msg.to_dict()

        # Log or print
        print("\nReceived OBSTACLE_DISTANCE:")
        print(f"  time_usec:     {d['time_usec']}")
        print(f"  frame:         {d['frame']}")
        print(f"  min_distance:  {d['min_distance']}")
        print(f"  max_distance:  {d['max_distance']}")
        print(f"  increment:     {d['increment']}")
        print(f"  angle_offset:  {d['angle_offset']}")
        print(f"  distances:     {d['distances'][:]} ...")  # print first 10

        # Save as CSV
        logfile.write(
            f"{time.time()},{d['time_usec']},{d['frame']},"
            f"{d['min_distance']},{d['max_distance']},"
            f"{d['increment']},{d['angle_offset']},"
            f"{d['distances']}\n"
        )
        logfile.flush()

    time.sleep(0.01)
