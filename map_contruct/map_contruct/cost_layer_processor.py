# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from visualization_msgs.msg import MarkerArray
# import numpy as np

# class CostLayerProcessor(Node):
#     def __init__(self):
#         super().__init__('cost_layer_processor')
        
#         # Subscribe to the cost layer topic
#         self.cost_sub = self.create_subscription(
#             MarkerArray,
#             '/cost_layer',
#             self.cost_callback,
#             10)
        
#         self.get_logger().info('Cost Layer Processor initialized')

#     def cost_callback(self, msg):
#         # Process the cost layer markers
#         self.get_logger().info(f'Received cost layer with {len(msg.markers)} markers')
        
#         # Example processing: Count markers by color
#         red_count = 0
#         green_count = 0
        
#         for marker in msg.markers:
#             if marker.color.r > 0.5:  # Red markers
#                 red_count += 1
#             elif marker.color.g > 0.5:  # Green markers
#                 green_count += 1
        
#         # self.get_logger().info(f'High cost areas (red): {red_count}, Low cost areas (green): {green_count}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = CostLayerProcessor()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()





#just for testing
# import rclpy
# from rclpy.node import Node
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point

# class MapMarkerPublisher(Node):
#     def __init__(self):
#         super().__init__('map_marker_publisher')
#         self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
#         self.timer = self.create_timer(0.1, self.publish_marker)  # Publish every second

#     def publish_marker(self):
#         marker = Marker()
#         marker.header.frame_id = "map"  # Make sure this matches your map frame
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = "my_markers"
#         marker.id = 0
#         marker.type = Marker.SPHERE
#         marker.action = Marker.ADD
#         marker.pose.position.x = 0.0  # Change as needed
#         marker.pose.position.y = 0.0
#         marker.pose.position.z = 0.0
#         marker.pose.orientation.x = 0.0
#         marker.pose.orientation.y = 0.0
#         marker.pose.orientation.z = 0.0
#         marker.pose.orientation.w = 1.0
#         marker.scale.x = 0.5
#         marker.scale.y = 0.5
#         marker.scale.z = 0.5
#         marker.color.r = 1.0
#         marker.color.g = 0.0
#         marker.color.b = 0.0
#         marker.color.a = 1.0
#         marker.lifetime.sec = 0  # 0 means forever

#         self.marker_pub.publish(marker)
#         self.get_logger().info('Published marker at the origin in map frame')

# def main(args=None):
#     rclpy.init(args=args)
#     node = MapMarkerPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




#version2 for testing with the model outputs 

# import rclpy
# from rclpy.node import Node
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point, Quaternion

# class PathStatusMarkerPublisher(Node):
#     def __init__(self):
#         super().__init__('path_status_marker_publisher')
#         self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
#         self.timer = self.create_timer(0.1, self.timer_callback)

#     def timer_callback(self):
#         self.publish_status_marker(178.57425688821206, 242.46553534578473, 0)
#         self.publish_status_marker(178.63493242417223, 242.38462782020656, 0)
#         self.publish_status_marker(179.1022361002315, 241.6514129759289, 0)
#         self.publish_status_marker(179.49896604798366, 240.88101154655078, 0)
#         self.publish_status_marker(180.25333221213998, 240.4074740601949, 0)
#         self.publish_status_marker( 180.7946842471102, 240.1756296300597, 0)
#         self.publish_status_marker(181.57825996883346, 240.154137141226, 1)
#         self.publish_status_marker(181.86372700928558, 240.15611587755865, 1)
#         # self.publish_status_marker(6.0, 2.0, 1)
#         # self.publish_status_marker(3.0, 0.0, 0)
#         # self.publish_status_marker(1.0, 2.0, 1)
#         # self.publish_status_marker(2.0, 3.0, 0)
#         # self.publish_status_marker(1.0, 2.0, 1)
#         # self.publish_status_marker(2.0, 3.0, 0)

#     def publish_status_marker(self, x, y, status):
#         marker = Marker()
#         marker.header.frame_id = "body"
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = "path_status"
#         marker.id = int(x*1000 + y)  # Unique ID for each location
#         marker.pose.position.x = x
#         marker.pose.position.y = y
#         marker.pose.position.z = 0.1
#         marker.pose.orientation.x = 0.0
#         marker.pose.orientation.y = 0.0
#         marker.pose.orientation.z = 0.0
#         marker.pose.orientation.w = 1.0
#         marker.lifetime.sec = 0  # Forever

#         if status == 1:  # Dead end
#             marker.type = Marker.TEXT_VIEW_FACING
#             marker.text = "X"
#             marker.scale.z = 0.5  # Height of the text
#             marker.color.r = 1.0
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.color.a = 1.0
#         else:  # Open path
#             marker.type = Marker.ARROW
#             marker.scale.x = 1.0  # Arrow length
#             marker.scale.y = 0.2  # Arrow width
#             marker.scale.z = 0.2  # Arrow height
#             marker.color.r = 0.0
#             marker.color.g = 1.0
#             marker.color.b = 0.0
#             marker.color.a = 1.0
#             # Optionally, set orientation for arrow direction

#         self.marker_pub.publish(marker)
#         self.get_logger().info(f'Published marker at ({x}, {y}) for status {status}')

#     def scale_value(self, val, data_min, data_max, map_min, map_max):
#         # Linearly scale val from data range to map range
#         return map_min + (val - data_min) * (map_max - map_min) / (data_max - data_min)

#     data_x_min, data_x_max = 178, 182
#     data_y_min, data_y_max = 240, 245
#     map_x_min, map_x_max = 0, 10
#     map_y_min, map_y_max = 0, 10

# def main(args=None):
#     rclpy.init(args=args)
#     node = PathStatusMarkerPublisher()
#     # Example usage:
#     node.publish_status_marker(1.0, 2.0, 1)  # Dead end at (1,2)
#     node.publish_status_marker(2.0, 3.0, 0)  # Open path at (2,3)
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#version 3 for testing with the model outputs 

# import rclpy
# from rclpy.node import Node
# from visualization_msgs.msg import Marker
# import json
# from geometry_msgs.msg import Point
# import math
# from std_msgs.msg import ColorRGBA

# class PathStatusMarkerPublisher(Node):
#     def __init__(self, json_path):
#         super().__init__('path_status_marker_publisher')
#         self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
#         self.json_path = json_path
#         self.markers = self.load_json()
#         self.tile_state = {}  # (x, y) -> log-odds
#         self.p_hit = 0.7  # Model confidence for correct detection
#         self.timer = self.create_timer(0.1, self.publish_all_markers)

#     def load_json(self):
#         with open(self.json_path, 'r') as f:
#             data = json.load(f)
#         return data

#     def bayesian_update(self, logodds, is_dead_end):
#         # log-odds update
#         l_occ = math.log(self.p_hit / (1 - self.p_hit))
#         l_free = math.log((1 - self.p_hit) / self.p_hit)
#         if is_dead_end:
#             return logodds + l_occ
#         else:
#             return logodds + l_free

#     def publish_all_markers(self):
#         marker = Marker()
#         marker.header.frame_id = "map"
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = "path_status"
#         marker.id = 0
#         marker.type = Marker.CUBE_LIST
#         marker.scale.x = 0.5
#         marker.scale.y = 0.5
#         marker.scale.z = 0.01  # Thin plate
#         marker.lifetime.sec = 0

#         marker.points = []
#         marker.colors = []

#         # For each new inference, update the state
#         for sample_id, entry in self.markers.items():
#             x = entry['x']
#             y = entry['y']
#             z = entry.get('z', 0.1)
#             is_dead_end = int(entry['is_dead_end'])

#             # Use a grid key (rounded to nearest 0.5 for example)
#             tile_size = 0.5  # meters
#             key = (round(x / tile_size) * tile_size, round(y / tile_size) * tile_size)
#             prev_logodds = self.tile_state.get(key, 0.0)
#             new_logodds = self.bayesian_update(prev_logodds, is_dead_end)
#             self.tile_state[key] = new_logodds

#         # Now publish one marker per tile, colored by probability
#         for (x, y), logodds in self.tile_state.items():
#             # Clamp logodds to avoid overflow
#             logodds = max(min(logodds, 10), -10)
#             p = 1 / (1 + math.exp(-logodds))
#             pt = Point()
#             pt.x = x
#             pt.y = y
#             pt.z = 0.1
#             marker.points.append(pt)

#             # Interpolate color: red (dead end) to green (open)
#             r = p
#             g = 1 - p
#             b = 0.0
#             a = 1.0
            
#             color = ColorRGBA()
#             color.r = r
#             color.g = g
#             color.b = b
#             color.a = a
#             marker.colors.append(color)

#         self.marker_pub.publish(marker)

# def main(args=None):
#     import sys
#     if len(sys.argv) < 2:
#         print("Usage: python3 cost_layer_processor.py path_to_json_file")
#         return
#     json_path = sys.argv[1]
#     rclpy.init(args=args)
#     node = PathStatusMarkerPublisher(json_path)
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#version 4 for testing with model outputs directly live node 
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math

class BayesianCostLayerProcessor(Node):
    def __init__(self):
        super().__init__('bayesian_cost_layer_processor')

        # Subscribe to the model's path status output
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/dead_end_detection/path_status',
            self.path_status_callback,
            10
        )

        # Publisher for the cost layer visualization
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/cost_layer',
            10
        )

        # Bayesian parameters
        self.tile_logodds = {}  # (x, y) -> log-odds

        self.grid_size = 10  # Set this to match your model output (e.g., 10 for 10x10 grid)
        self.tile_size = 1.0  # Size of each grid cell in meters

        self.get_logger().info('Bayesian Cost Layer Processor initialized')

    def bayesian_update(self, logodds, p):
        # p: model's probability for this cell (e.g., dead-end probability)
        # log-odds update: l_new = l_old + log(p/(1-p))
        if p <= 0.0:
            p = 1e-6
        if p >= 1.0:
            p = 1.0 - 1e-6
        l_obs = math.log(p / (1 - p))
        return logodds + l_obs

    def path_status_callback(self, msg):
        data = np.array(msg.data)
        num_cells = len(data)
        grid_size = self.grid_size

        marker_array = MarkerArray()
        marker_id = 0

        for idx, p in enumerate(data):
            x_idx = idx % grid_size
            y_idx = idx // grid_size

            key = (x_idx, y_idx)
            prev_logodds = self.tile_logodds.get(key, 0.0)
            new_logodds = self.bayesian_update(prev_logodds, p)
            # Clamp logodds to avoid overflow
            new_logodds = max(min(new_logodds, 10), -10)
            self.tile_logodds[key] = new_logodds

            # Convert log-odds to probability
            prob = 1 / (1 + math.exp(-new_logodds))

            # Visualization marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "cost_layer"
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = x_idx * self.tile_size
            marker.pose.position.y = y_idx * self.tile_size
            marker.pose.position.z = 0.05
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.tile_size
            marker.scale.y = self.tile_size
            marker.scale.z = 0.1

            # Color: Red (high cost) to Green (low cost)
            marker.color.r = float(prob)
            marker.color.g = 1.0 - float(prob)
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)
            marker_id += 1

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {marker_id} Bayesian cost markers')

def main(args=None):
    rclpy.init(args=args)
    node = BayesianCostLayerProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()