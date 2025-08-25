#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, Float32MultiArray
from geometry_msgs.msg import Point
import numpy as np
import torch
from torchvision import transforms
from PIL import Image as PILImage
import cv2
import time
import os
import json
from typing import Dict, List, Optional

# Visualization imports - similar to data_loader.py
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
from matplotlib import pyplot as plt

# Import your model and recovery points
from model_CA import DeadEndDetectionModel
from recovery_points import RecoveryPointManager

class DeadEndDetectionNodeWithVisualization(Node):
    def __init__(self):
        super().__init__('dead_end_detection_visual_node')
        
        # Visualization and saving setup
        self.output_dir = '/home/mrvik/dram_ws/inference_results'
        self.create_output_directory()
        
        # Frame counting and performance tracking
        self.frame_count = 0
        self.total_processed = 0
        self.processing_times = []
        self.results_history = []
        
        # Initialize model
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = DeadEndDetectionModel()
        
        # Load model weights
        model_path = '/media/mrvik/ROGZ/Ubuntu_files/DRaM/RAL2025/saved_models_latest/model_best.pth'
        try:
            checkpoint = torch.load(model_path, map_location=self.device)
            self.model.load_state_dict(checkpoint, strict=False)
            self.model.to(self.device)
            self.model.eval()
            self.get_logger().info(f'✅ Model loaded successfully from {model_path}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load model: {e}')
            self.model = None
            self.get_logger().warn('⚠️ Using dummy model for testing purposes')
        
        # Initialize recovery point manager
        self.recovery_manager = RecoveryPointManager(confidence_threshold=0.5)
        
        # Initialize transforms - same as training
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        
        # Store raw images for visualization (before transform)
        self.front_img_raw = None
        self.left_img_raw = None
        self.right_img_raw = None
        
        # Initialize subscribers
        self.front_cam_sub = self.create_subscription(
            Image, '/argus/ar0234_front_left/image_raw', self.front_cam_callback, 10)
        self.left_cam_sub = self.create_subscription(
            Image, '/argus/ar0234_side_left/image_raw', self.left_cam_callback, 10)
        self.right_cam_sub = self.create_subscription(
            Image, '/argus/ar0234_side_right/image_raw', self.right_cam_callback, 10)
        self.front_lidar_sub = self.create_subscription(
            PointCloud2, '/lidar/front/points', self.front_lidar_callback, 10)
        self.left_lidar_sub = self.create_subscription(
            PointCloud2, '/lidar/left/points', self.left_lidar_callback, 10)
        self.right_lidar_sub = self.create_subscription(
            PointCloud2, '/lidar/right/points', self.right_lidar_callback, 10)
        
        # Initialize publishers
        self.dead_end_pub = self.create_publisher(Bool, '/dead_end_detection/is_dead_end', 10)
        self.path_status_pub = self.create_publisher(Float32MultiArray, '/dead_end_detection/path_status', 10)
        self.recovery_point_pub = self.create_publisher(Float32MultiArray, '/dead_end_detection/recovery_points', 10)
        
        # Initialize message storage
        self.front_img = None
        self.left_img = None
        self.right_img = None
        self.front_lidar = None
        self.left_lidar = None
        self.right_lidar = None
        
        # Initialize synchronization
        self.last_processed_time = 0
        self.processing_interval = 0.2  # Process every 200ms for better visualization
        
        self.get_logger().info('🚀 Dead End Detection Node with Visualization initialized')
        self.get_logger().info(f'📁 Output directory: {self.output_dir}')

    def create_output_directory(self):
        """Create output directory structure"""
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.output_dir = os.path.join(self.output_dir, f'inference_{timestamp}')
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f'📁 Created output directory: {self.output_dir}')

    def front_cam_callback(self, msg: Image):
        """Process front camera image"""
        try:
            img_pil = self.ros_image_to_pil(msg)
            self.front_img_raw = np.array(img_pil)  # Store raw for visualization
            self.front_img = self.transform(img_pil)
        except Exception as e:
            self.get_logger().error(f'❌ Error processing front camera: {e}')

    def left_cam_callback(self, msg: Image):
        """Process left camera image"""
        try:
            img_pil = self.ros_image_to_pil(msg)
            self.left_img_raw = np.array(img_pil)
            self.left_img = self.transform(img_pil)
        except Exception as e:
            self.get_logger().error(f'❌ Error processing left camera: {e}')

    def right_cam_callback(self, msg: Image):
        """Process right camera image"""
        try:
            img_pil = self.ros_image_to_pil(msg)
            self.right_img_raw = np.array(img_pil)
            self.right_img = self.transform(img_pil)
        except Exception as e:
            self.get_logger().error(f'❌ Error processing right camera: {e}')

    def front_lidar_callback(self, msg: PointCloud2):
        """Process front LiDAR data"""
        try:
            points = self.ros_pointcloud_to_numpy(msg)
            self.front_lidar = torch.from_numpy(points).float()
        except Exception as e:
            self.get_logger().error(f'❌ Error processing front LiDAR: {e}')

    def left_lidar_callback(self, msg: PointCloud2):
        """Process left LiDAR data"""
        try:
            points = self.ros_pointcloud_to_numpy(msg)
            self.left_lidar = torch.from_numpy(points).float()
        except Exception as e:
            self.get_logger().error(f'❌ Error processing left LiDAR: {e}')

    def right_lidar_callback(self, msg: PointCloud2):
        """Process right LiDAR data"""
        try:
            points = self.ros_pointcloud_to_numpy(msg)
            self.right_lidar = torch.from_numpy(points).float()
        except Exception as e:
            self.get_logger().error(f'❌ Error processing right LiDAR: {e}')

    def ros_image_to_pil(self, msg: Image) -> PILImage.Image:
        """Convert ROS Image message to PIL Image"""
        cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        return PILImage.fromarray(cv_image)

    def ros_pointcloud_to_numpy(self, msg: PointCloud2) -> np.ndarray:
        """Convert ROS PointCloud2 message to numpy array - match training format"""
        import sensor_msgs_py.point_cloud2 as pc2
        
        try:
            points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            
            if not points_list:
                return np.zeros((3, 1), dtype=np.float32)
            
            # Handle both structured and unstructured arrays
            if isinstance(points_list[0], (list, tuple)):
                points = np.array(points_list, dtype=np.float32)
            else:
                structured_array = np.array(points_list)
                points = np.column_stack([
                    structured_array['x'], structured_array['y'], structured_array['z']
                ]).astype(np.float32)
            
            # Sample/pad to exactly 4096 points (same as training)
            num_points = 4096
            if points.shape[0] > num_points:
                indices = np.random.choice(points.shape[0], num_points, replace=False)
                points = points[indices, :]
            elif points.shape[0] < num_points:
                repeat_times = (num_points + points.shape[0] - 1) // points.shape[0]
                points = np.tile(points, (repeat_times, 1))[:num_points, :]
            
            return points.T  # Shape: (3, N) for PointNet
            
        except Exception as e:
            self.get_logger().error(f'❌ Error in point cloud conversion: {e}')
            return np.zeros((3, 1), dtype=np.float32)

    def plot_direction_vectors(self, ax, img, direction_vectors, path_probs, view_type='front', scale=50):
        """Plot direction vectors as arrows on the image - same as data_loader.py"""
        h, w = img.shape[:2]
        center = np.array([w/2, h/2])
        view_map = {
            'front': (0, 'blue'),
            'left': (1, 'green'), 
            'right': (2, 'red')
        }

        if view_type not in view_map:
            raise ValueError(f'Invalid view_type: {view_type}')

        idx, color = view_map[view_type]
        
        if torch.is_tensor(direction_vectors):
            direction_vectors = direction_vectors.detach().cpu().numpy()
            
        vec = direction_vectors[idx]
        prob = float(path_probs[idx])
        
        # Use correct threshold (0.56) for arrow plotting
        if prob > 0.56:  # Only plot arrows for open paths
            direction = np.array([vec[0], vec[2]])  # Use x,z components
            
            if np.linalg.norm(direction) > 0:
                direction = direction / np.linalg.norm(direction) * scale
                
                ax.arrow(center[0], center[1], 
                        direction[0], direction[1],
                        head_width=5, head_length=10, 
                        fc=color, ec=color, 
                        label=f"{view_type.capitalize()}")
        
        ax.legend()

    def process_data(self):
        """Process all sensor data and run model inference with visualization"""
        current_time = time.time()
        
        if current_time - self.last_processed_time < self.processing_interval:
            return
        
        # Check if we have all required LiDAR data
        if (self.front_lidar is None or self.front_lidar.numel() == 0 or
            self.left_lidar is None or self.left_lidar.numel() == 0 or 
            self.right_lidar is None or self.right_lidar.numel() == 0):
            return
        
        try:
            inference_start = time.time()
            
            # Prepare input tensors
            if self.front_img is not None:
                front_img = self.front_img.unsqueeze(0).to(self.device)
                left_img = self.left_img.unsqueeze(0).to(self.device) if self.left_img is not None else front_img
                right_img = self.right_img.unsqueeze(0).to(self.device) if self.right_img is not None else front_img
            else:
                dummy_img = torch.zeros(1, 3, 224, 224).to(self.device)
                front_img = left_img = right_img = dummy_img
            
            # Prepare LiDAR tensors
            front_lidar = self.front_lidar.unsqueeze(0).to(self.device)
            left_lidar = self.left_lidar.unsqueeze(0).to(self.device)
            right_lidar = self.right_lidar.unsqueeze(0).to(self.device)
            
            # Run model inference
            if self.model is None:
                outputs = {
                    'path_status': torch.tensor([[0.6, 0.4, 0.8]]),  # Varied dummy outputs
                    'is_dead_end': torch.tensor([[0.2]]),
                    'direction_vectors': torch.randn(1, 3, 3) * 0.5
                }
            else:
                with torch.no_grad():
                    outputs = self.model(front_img, right_img, left_img,
                                       front_lidar, right_lidar, left_lidar)
            
            inference_time = time.time() - inference_start
            self.processing_times.append(inference_time)
            
            # Process results
            path_probs = torch.sigmoid(outputs['path_status']).cpu().numpy().flatten()
            is_dead_end = torch.sigmoid(outputs['is_dead_end']).cpu().numpy().flatten()[0]
            direction_vectors = outputs['direction_vectors'][0]  # Remove batch dimension
            
            # Process recovery points
            strategy = self.recovery_manager.get_recovery_strategy(outputs)
            
            # Print status for each processed frame
            self.print_frame_status(path_probs, is_dead_end, inference_time)
            
            # Save visualization
            self.save_visualization(path_probs, is_dead_end, direction_vectors, inference_time)
            
            # Publish results
            self.publish_results(outputs, strategy)
            
            # Update counters
            self.frame_count += 1
            self.total_processed += 1
            
            # Apply correct dead-end logic for results storage
            threshold = 0.56
            front_open = path_probs[0] > threshold
            left_open = path_probs[1] > threshold  
            right_open = path_probs[2] > threshold
            is_dead_end_correct = not (front_open or left_open or right_open)
            
            # Store results for performance analysis
            self.results_history.append({
                'frame': self.frame_count,
                'front_open_prob': float(path_probs[0]),
                'left_open_prob': float(path_probs[1]), 
                'right_open_prob': float(path_probs[2]),
                'front_open': bool(front_open),
                'left_open': bool(left_open),
                'right_open': bool(right_open),
                'is_dead_end': bool(is_dead_end_correct),
                'open_paths_count': int(sum([front_open, left_open, right_open])),
                'threshold_used': threshold,
                'inference_time': inference_time,
                'timestamp': current_time
            })
            
            # Clear processed data
            self.clear_data()
            self.last_processed_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'❌ Error in process_data: {e}')

    def print_frame_status(self, path_probs, is_dead_end, inference_time):
        """Print comprehensive status for each processed frame"""
        # Correct dead-end logic: if NO path has >0.56 probability, it's a dead end
        threshold = 0.56
        front_open = path_probs[0] > threshold
        left_open = path_probs[1] > threshold  
        right_open = path_probs[2] > threshold
        
        # Dead end = NO paths are open (all below threshold)
        is_dead_end_correct = not (front_open or left_open or right_open)
        
        # Create status strings
        front_status = "🟢 OPEN" if front_open else "🔴 BLOCKED"
        left_status = "🟢 OPEN" if left_open else "🔴 BLOCKED"
        right_status = "🟢 OPEN" if right_open else "🔴 BLOCKED"
        dead_end_status = "⚠️ DEAD END" if is_dead_end_correct else "✅ PATH AVAILABLE"
        
        # Calculate performance metrics
        avg_inference_time = np.mean(self.processing_times[-10:]) if self.processing_times else 0
        fps = 1.0 / avg_inference_time if avg_inference_time > 0 else 0
        
        # Count open paths
        open_paths_count = sum([front_open, left_open, right_open])
        
        # Print comprehensive status
        self.get_logger().info(f"""
╔══════════════════ FRAME {self.frame_count:04d} ══════════════════╗
║ 🎯 PATHS:   Front: {front_status} ({path_probs[0]:.3f})
║            Left:  {left_status} ({path_probs[1]:.3f})  
║            Right: {right_status} ({path_probs[2]:.3f})
║ 📊 SUMMARY: {open_paths_count}/3 paths open (threshold: {threshold})
║ 🛑 STATUS:  {dead_end_status}
║ ⚡ PERF:    {inference_time*1000:.1f}ms | {fps:.1f} FPS
╚══════════════════════════════════════════════════════════════╝""")

    def save_visualization(self, path_probs, is_dead_end, direction_vectors, inference_time):
        """Save visualization similar to data_loader.py"""
        if (self.front_img_raw is None or self.left_img_raw is None or self.right_img_raw is None):
            return
            
        try:
            # Apply correct dead-end logic (same as print_frame_status)
            threshold = 0.56
            front_open = path_probs[0] > threshold
            left_open = path_probs[1] > threshold  
            right_open = path_probs[2] > threshold
            is_dead_end_correct = not (front_open or left_open or right_open)
            
            # Create figure with 3 subplots (like data_loader.py)
            fig = plt.figure(figsize=(15, 5))
            
            # Front camera
            ax1 = plt.subplot(1, 3, 1)
            ax1.imshow(self.front_img_raw)
            self.plot_direction_vectors(ax1, self.front_img_raw, direction_vectors, path_probs, 'front')
            front_status = "OPEN" if front_open else "BLOCKED"
            ax1.set_title(f"Front - {front_status}: {path_probs[0]:.3f}")
            ax1.axis('off')

            # Left camera  
            ax2 = plt.subplot(1, 3, 2)
            ax2.imshow(self.left_img_raw)
            self.plot_direction_vectors(ax2, self.left_img_raw, direction_vectors, path_probs, 'left')
            left_status = "OPEN" if left_open else "BLOCKED"
            ax2.set_title(f"Left - {left_status}: {path_probs[1]:.3f}")
            ax2.axis('off')

            # Right camera
            ax3 = plt.subplot(1, 3, 3)
            ax3.imshow(self.right_img_raw)
            self.plot_direction_vectors(ax3, self.right_img_raw, direction_vectors, path_probs, 'right')
            right_status = "OPEN" if right_open else "BLOCKED"
            ax3.set_title(f"Right - {right_status}: {path_probs[2]:.3f}")
            ax3.axis('off')

            # Add overall title with correct logic
            open_count = sum([front_open, left_open, right_open])
            dead_end_text = "DEAD END" if is_dead_end_correct else f"PATH AVAILABLE ({open_count}/3 open)"
            plt.suptitle(f"Frame {self.frame_count:04d} - {dead_end_text} | {inference_time*1000:.1f}ms", 
                        fontsize=14, y=0.95)

            # Save individual frame
            output_path = os.path.join(self.output_dir, f'frame_{self.frame_count:04d}.png')
            plt.savefig(output_path, bbox_inches='tight', dpi=150)
            plt.close()  # Important: close to prevent memory leaks
            
            # Save results JSON every 10 frames
            if self.frame_count % 10 == 0:
                self.save_results_json()
                
        except Exception as e:
            self.get_logger().error(f'❌ Error saving visualization: {e}')

    def save_results_json(self):
        """Save results to JSON file for analysis"""
        try:
            results_file = os.path.join(self.output_dir, 'inference_results.json')
            
            # Calculate summary statistics
            summary = {
                'total_frames': self.total_processed,
                'avg_inference_time': float(np.mean(self.processing_times)) if self.processing_times else 0,
                'avg_fps': float(1.0 / np.mean(self.processing_times)) if self.processing_times else 0,
                'threshold_used': 0.56,
                'dead_end_rate': float(np.mean([r['is_dead_end'] for r in self.results_history])) if self.results_history else 0,
                'avg_open_paths_per_frame': float(np.mean([r['open_paths_count'] for r in self.results_history])) if self.results_history else 0,
                'open_path_rates': {
                    'front': float(np.mean([r['front_open'] for r in self.results_history])) if self.results_history else 0,
                    'left': float(np.mean([r['left_open'] for r in self.results_history])) if self.results_history else 0,
                    'right': float(np.mean([r['right_open'] for r in self.results_history])) if self.results_history else 0,
                },
                'avg_path_probabilities': {
                    'front': float(np.mean([r['front_open_prob'] for r in self.results_history])) if self.results_history else 0,
                    'left': float(np.mean([r['left_open_prob'] for r in self.results_history])) if self.results_history else 0,
                    'right': float(np.mean([r['right_open_prob'] for r in self.results_history])) if self.results_history else 0,
                }
            }
            
            data = {
                'summary': summary,
                'frames': self.results_history
            }
            
            with open(results_file, 'w') as f:
                json.dump(data, f, indent=2)
                
            self.get_logger().info(f'💾 Saved results to {results_file}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Error saving results JSON: {e}')

    def publish_results(self, outputs: Dict[str, torch.Tensor], strategy: dict):
        """Publish model outputs and recovery points"""
        # Publish dead end status
        dead_end_msg = Bool()
        dead_end_msg.data = strategy['is_dead_end']
        self.dead_end_pub.publish(dead_end_msg)
        
        # Publish path status
        path_status = torch.sigmoid(outputs['path_status']).cpu().numpy()
        path_msg = Float32MultiArray()
        path_msg.data = path_status.flatten().tolist()
        self.path_status_pub.publish(path_msg)
        
        # Publish recovery points
        if strategy['recovery_points']:
            recovery_msg = Float32MultiArray()
            recovery_data = []
            
            if strategy['recovery_points']['last']:
                last_point = strategy['recovery_points']['last']
                recovery_data.extend([
                    last_point.index, last_point.rank,
                    *last_point.open_directions, *last_point.confidence
                ])
            
            if strategy['recovery_points']['best']:
                best_point = strategy['recovery_points']['best']
                recovery_data.extend([
                    best_point.index, best_point.rank,
                    *best_point.open_directions, *best_point.confidence
                ])
            
            recovery_msg.data = recovery_data
            self.recovery_point_pub.publish(recovery_msg)

    def clear_data(self):
        """Clear processed sensor data"""
        self.front_img = None
        self.left_img = None
        self.right_img = None
        self.front_img_raw = None
        self.left_img_raw = None
        self.right_img_raw = None
        self.front_lidar = None
        self.left_lidar = None
        self.right_lidar = None

def main(args=None):
    rclpy.init(args=args)
    node = DeadEndDetectionNodeWithVisualization()
    
    try:
        node.create_timer(0.1, node.process_data)  # 10Hz processing
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down...')
        # Save final results
        node.save_results_json()
        node.get_logger().info(f'📊 Final stats: {node.total_processed} frames processed')
        if node.processing_times:
            avg_time = np.mean(node.processing_times)
            node.get_logger().info(f'⚡ Average inference: {avg_time*1000:.1f}ms ({1.0/avg_time:.1f} FPS)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()