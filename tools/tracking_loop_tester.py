import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

rclpy.init()
node = Node('sim_tracker_injector')
pub = node.create_publisher(String, '/trash_detections', 10)

print("Injecting tracking sweep...")
# Loop to simulate approaching the trash object
for i in range(1, 40):
    # Gradually increase area to simulate getting closer
    sim_area = 5000 + (i * 800) 
    
    payload = {
        'cx': 320.0, # Target perfectly centered
        'cy': 240.0,
        'area': sim_area,
        'conf': 0.85,
        'tracking': True,
        'bbox': [290, 210, 350, 270]
    }
    
    msg = String()
    msg.data = json.dumps(payload)
    pub.publish(msg)
    time.sleep(0.1) # 10 Hz frequency matches MissionFSM tick rate

node.destroy_node()
rclpy.shutdown()