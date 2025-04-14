#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
import numpy as np

class CoordinateValidator:
    def __init__(self):
        rospy.init_node('coordinate_validator', anonymous=True)
        
        # Subscribe to the 3D centroid topics
        self.orange_sub = rospy.Subscriber('/color_tracker/orange_centroid_3d', 
                                          PointStamped, 
                                          self.orange_callback)
        
        self.brown_sub = rospy.Subscriber('/color_tracker/brown_centroid_3d', 
                                         PointStamped, 
                                         self.brown_callback)
        
        # Store measurements for statistics
        self.orange_measurements = []
        self.brown_measurements = []
        self.max_measurements = 100  # Keep last 100 measurements
        
        # Print statistics every 5 seconds
        rospy.Timer(rospy.Duration(5), self.print_statistics)
        
        rospy.loginfo("Coordinate validator started")
    
    def orange_callback(self, msg):
        # Extract the 3D point
        point = (msg.point.x, msg.point.y, msg.point.z)
        
        # Add to measurements
        self.orange_measurements.append(point)
        if len(self.orange_measurements) > self.max_measurements:
            self.orange_measurements.pop(0)
    
    def brown_callback(self, msg):
        # Extract the 3D point
        point = (msg.point.x, msg.point.y, msg.point.z)
        
        # Add to measurements
        self.brown_measurements.append(point)
        if len(self.brown_measurements) > self.max_measurements:
            self.brown_measurements.pop(0)
    
    def calculate_stats(self, measurements):
        if not measurements:
            return None
        
        # Convert to numpy array for easier statistics
        points = np.array(measurements)
        
        # Calculate statistics
        mean = np.mean(points, axis=0)
        std_dev = np.std(points, axis=0)
        min_vals = np.min(points, axis=0)
        max_vals = np.max(points, axis=0)
        
        return {
            'count': len(measurements),
            'mean': mean,
            'std_dev': std_dev,
            'min': min_vals,
            'max': max_vals
        }
    
    def print_statistics(self, event=None):
        # Print orange statistics
        orange_stats = self.calculate_stats(self.orange_measurements)
        if orange_stats:
            rospy.loginfo("\n--- ORANGE OBJECT STATISTICS ---")
            rospy.loginfo(f"Sample count: {orange_stats['count']}")
            rospy.loginfo(f"Mean position (x,y,z): ({orange_stats['mean'][0]:.3f}, {orange_stats['mean'][1]:.3f}, {orange_stats['mean'][2]:.3f})")
            rospy.loginfo(f"Std deviation: ({orange_stats['std_dev'][0]:.3f}, {orange_stats['std_dev'][1]:.3f}, {orange_stats['std_dev'][2]:.3f})")
            rospy.loginfo(f"Min values: ({orange_stats['min'][0]:.3f}, {orange_stats['min'][1]:.3f}, {orange_stats['min'][2]:.3f})")
            rospy.loginfo(f"Max values: ({orange_stats['max'][0]:.3f}, {orange_stats['max'][1]:.3f}, {orange_stats['max'][2]:.3f})")
            
            # Estimate stability (lower is better)
            stability = np.mean(orange_stats['std_dev'])
            rospy.loginfo(f"Stability score: {stability:.5f} (lower is better)")
            
            # Assess plausibility
            is_plausible = all(abs(val) < 10 for val in orange_stats['mean']) and orange_stats['mean'][2] > 0
            rospy.loginfo(f"Plausibility check: {'PASS' if is_plausible else 'SUSPICIOUS'}")
        else:
            rospy.loginfo("No orange measurements received yet")
        
        # Print brown statistics
        brown_stats = self.calculate_stats(self.brown_measurements)
        if brown_stats:
            rospy.loginfo("\n--- BROWN OBJECT STATISTICS ---")
            rospy.loginfo(f"Sample count: {brown_stats['count']}")
            rospy.loginfo(f"Mean position (x,y,z): ({brown_stats['mean'][0]:.3f}, {brown_stats['mean'][1]:.3f}, {brown_stats['mean'][2]:.3f})")
            rospy.loginfo(f"Std deviation: ({brown_stats['std_dev'][0]:.3f}, {brown_stats['std_dev'][1]:.3f}, {brown_stats['std_dev'][2]:.3f})")
            rospy.loginfo(f"Min values: ({brown_stats['min'][0]:.3f}, {brown_stats['min'][1]:.3f}, {brown_stats['min'][2]:.3f})")
            rospy.loginfo(f"Max values: ({brown_stats['max'][0]:.3f}, {brown_stats['max'][1]:.3f}, {brown_stats['max'][2]:.3f})")
            
            # Estimate stability (lower is better)
            stability = np.mean(brown_stats['std_dev'])
            rospy.loginfo(f"Stability score: {stability:.5f} (lower is better)")
            
            # Assess plausibility
            is_plausible = all(abs(val) < 10 for val in brown_stats['mean']) and brown_stats['mean'][2] > 0
            rospy.loginfo(f"Plausibility check: {'PASS' if is_plausible else 'SUSPICIOUS'}")
        else:
            rospy.loginfo("No brown measurements received yet")

def main():
    validator = CoordinateValidator()
    rospy.spin()

if __name__ == '__main__':
    main()