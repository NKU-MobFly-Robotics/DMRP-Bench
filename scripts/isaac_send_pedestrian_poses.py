import socket
import time
import omni.timeline
from omni.kit.app import get_app
from omni.isaac.core.prims import XFormPrim

timeline = omni.timeline.get_timeline_interface()
timeline.play()

# Map pedestrian names to their prim paths in the scene
target_prims = {
    "F_Business_02": "/World/F_Business_02",
    "M_Medical_01": "/World/M_Medical_01",
    "Police_Male_04": "/World/male_adult_police_04",
    "F_Medical_01": "/World/F_Medical_01",
    "Police_Female_01": "/World/female_adult_police_01_new",
    "Police_Female_02": "/World/female_adult_police_02",
    "Police_Female_03": "/World/female_adult_police_03_new"
}

# Initialize XFormPrim for each pedestrian
xform_dict = {}
for name, path in target_prims.items():
    xform = XFormPrim(path)
    if xform.is_valid():
        xform_dict[name] = xform
        print(f"Found {name}")
    else:
        print(f"Could not find {name}")

# Connect to ROS socket server
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(("127.0.0.1", 5566))
    print("✅ Connected to ROS receiver")
except Exception as e:
    print(f"Connection failed: {e}")
    sock = None

last_time = time.time()

def on_update(dt):
    global last_time
    if not timeline.is_playing() or sock is None:
        return

    now = time.time()
    if now - last_time >= 1.0:  # Send every second
        for name, xform in xform_dict.items():
            pos, ori = xform.get_world_pose()
            msg = f"{name} {pos[0]} {pos[1]} {pos[2]} {ori[0]} {ori[1]} {ori[2]} {ori[3]}\n"
            try:
                sock.sendall(msg.encode("utf-8"))
                print(f"📤 Sent: {msg.strip()}")
            except Exception as e:
                print(f"Failed to send {name}: {e}")
        last_time = now

update_sub = get_app().get_update_event_stream().create_subscription_to_push(on_update)