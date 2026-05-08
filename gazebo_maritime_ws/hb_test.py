from pymavlink import mavutil
import time

tx = mavutil.mavlink_connection("udpout:127.0.0.1:14560")

while True:
    tx.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )
    print("sent heartbeat")
    time.sleep(1)