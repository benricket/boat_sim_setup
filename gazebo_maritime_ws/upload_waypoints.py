from pymavlink import mavutil
import time

CONNECTION = "udp:127.0.0.1:14552"

waypoints = [
    (-35.360863, 149.160016, 10.0),
    (-35.3612151, 149.1592322, 10.0),
    #(-35.3614084, 149.1542881, 10.0),
    #(0.0, 0.0, 10.0)
]

conn = mavutil.mavlink_connection(CONNECTION)

print("Waiting for heartbeat...")
conn.wait_heartbeat()
print(f"Connected: system={conn.target_system}, component={conn.target_component}")
print("Mode mapping:", conn.mode_mapping())

# Clear existing mission
print("Clearing existing mission...")
conn.mav.mission_clear_all_send(
    conn.target_system,
    conn.target_component,
    mavutil.mavlink.MAV_MISSION_TYPE_MISSION
)

time.sleep(1)

# Drain stale messages
while conn.recv_match(blocking=False):
    pass

# Send waypoint count
print(f"Sending waypoint count: {len(waypoints)}")
conn.mav.mission_count_send(
    conn.target_system,
    conn.target_component,
    len(waypoints),
    mavutil.mavlink.MAV_MISSION_TYPE_MISSION
)

# Upload mission items
for i, (lat, lon, alt) in enumerate(waypoints):
    print(f"Waiting for mission request for waypoint {i}...")

    req = conn.recv_match(
        type=["MISSION_REQUEST", "MISSION_REQUEST_INT"],
        blocking=True,
        timeout=10
    )

    if req is None:
        raise RuntimeError(f"Timed out waiting for mission request {i}")

    print(f"Got {req.get_type()} for seq={req.seq}")

    conn.mav.mission_item_int_send(
        conn.target_system,
        conn.target_component,
        req.seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,
        1,
        0, 2, 0, 0,
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    )
    
# Wait for mission ACK
ack = conn.recv_match(type="MISSION_ACK", blocking=True, timeout=10)
print("Mission ACK:", ack)

if ack is None:
    raise RuntimeError("No MISSION_ACK received")

if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
    raise RuntimeError(f"Mission rejected: {ack}")

print("Mission accepted")

# Drain stale messages before requesting mission list
while conn.recv_match(blocking=False):
    pass

# Request mission count back from vehicle
print("Requesting mission count from vehicle...")

conn.mav.mission_request_list_send(
    conn.target_system,
    conn.target_component,
    mavutil.mavlink.MAV_MISSION_TYPE_MISSION
)

mission_count = None

while True:
    msg = conn.recv_match(blocking=True, timeout=5)

    if msg is None:
        print("No MAVLink response while waiting for MISSION_COUNT")
        break

    print("Got:", msg.get_type(), msg)

    if msg.get_type() == "MISSION_COUNT":
        mission_count = msg.count
        print("Mission count on vehicle:", mission_count)
        break

if mission_count is None or mission_count == 0:
    raise RuntimeError("Vehicle does not appear to have a valid mission")

# Set current mission item to first waypoint
print("Setting current mission item to 0...")
conn.mav.mission_set_current_send(
    conn.target_system,
    conn.target_component,
    0
)

time.sleep(1)

# Arm vehicle
print("Arming...")
conn.arducopter_arm()
conn.motors_armed_wait()
print("Armed")

# Set AUTO mode
print("Setting AUTO mode...")

mode_mapping = conn.mode_mapping()

if "AUTO" not in mode_mapping:
    raise RuntimeError(f"AUTO mode not available. Mode mapping: {mode_mapping}")

auto_mode = mode_mapping["AUTO"]

conn.mav.command_long_send(
    conn.target_system,
    conn.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    auto_mode,
    0, 0, 0, 0, 0
)

mode_ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
print("Mode change ACK:", mode_ack)

print("Done")