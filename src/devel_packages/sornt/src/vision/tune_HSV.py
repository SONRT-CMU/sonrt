#!/usr/bin/env python3

import cv2
import numpy as np
import argparse
import os
import glob
import subprocess

def nothing(x):
    """Dummy callback function for trackbar"""
    pass

def find_available_cameras():
    """Find available camera devices in the system"""
    # Method 1: Check /dev/video* devices
    video_devices = sorted(glob.glob('/dev/video*'))
    
    # Method 2: Use v4l2-ctl to get more detailed info if available
    camera_info = []
    try:
        for device in video_devices:
            try:
                # Get camera name/info
                result = subprocess.run(['v4l2-ctl', '--device', device, '--info'], 
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, 
                                       text=True, timeout=1)
                if result.returncode == 0:
                    # Extract camera name from info
                    for line in result.stdout.split('\n'):
                        if 'Card type' in line:
                            camera_name = line.split(':')[1].strip()
                            camera_info.append((device, camera_name))
                            break
                    else:
                        camera_info.append((device, "Unknown camera"))
            except (subprocess.SubprocessError, subprocess.TimeoutExpired):
                camera_info.append((device, "Unknown camera"))
    except FileNotFoundError:
        # v4l2-ctl not available, just use device paths
        camera_info = [(device, "Camera device") for device in video_devices]
    
    return camera_info

def create_hsv_tuner():
    # Find available cameras
    available_cameras = find_available_cameras()
    
    if not available_cameras:
        print("No camera devices found in /dev/video*")
        print("Trying to create a test image instead...")
        use_test_image = True
    else:
        print("Available cameras:")
        for i, (device, name) in enumerate(available_cameras):
            print(f"{i}: {device} - {name}")
        
        use_test_image = False
    
    # Create a window for the trackbars
    cv2.namedWindow('HSV Tuner')
    cv2.resizeWindow('HSV Tuner', 700, 350)
    
    # Create trackbars for HSV lower and upper bounds
    # H (Hue) range in OpenCV is [0, 179], S and V are [0, 255]
    cv2.createTrackbar('H Lower', 'HSV Tuner', 35, 179, nothing)  # Initial 35 (green starts around here)
    cv2.createTrackbar('S Lower', 'HSV Tuner', 50, 255, nothing)  # Initial 50
    cv2.createTrackbar('V Lower', 'HSV Tuner', 50, 255, nothing)  # Initial 50
    
    cv2.createTrackbar('H Upper', 'HSV Tuner', 85, 179, nothing)  # Initial 85 (green ends around here)
    cv2.createTrackbar('S Upper', 'HSV Tuner', 255, 255, nothing)  # Initial 255
    cv2.createTrackbar('V Upper', 'HSV Tuner', 255, 255, nothing)  # Initial 255
    
    # Add a trackbar for minimum contour area
    cv2.createTrackbar('Min Area', 'HSV Tuner', 500, 10000, nothing)  # Initial 500 px²
    
    # Create a window for the video feed with mask and result
    cv2.namedWindow('Camera Feed')
    cv2.namedWindow('Mask')
    cv2.namedWindow('Result')
    
    # Parse arguments
    parser = argparse.ArgumentParser(description='HSV Tuner for Green Detection')
    parser.add_argument('--video_path', type=str, default=None, 
                       help='Path to video file or camera index')
    parser.add_argument('--image_path', type=str, default=None,
                       help='Path to a test image')
    parser.add_argument('--camera', type=int, default=None,
                       help='Camera index from the available cameras list')
    parser.add_argument('--ros_topic', type=str, default=None,
                       help='ROS topic for image (requires ROS environment)')
    args = parser.parse_args()
    
    # Initialize video source
    cap = None
    test_image = None
    use_ros = False
    
    # Try to use ROS topic if specified
    if args.ros_topic:
        try:
            import rospy
            from sensor_msgs.msg import Image
            from cv_bridge import CvBridge
            
            bridge = CvBridge()
            latest_image = None
            
            def image_callback(msg):
                nonlocal latest_image
                try:
                    latest_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                except Exception as e:
                    print(f"Error converting ROS image: {e}")
            
            rospy.init_node('hsv_tuner', anonymous=True)
            sub = rospy.Subscriber(args.ros_topic, Image, image_callback)
            use_ros = True
            print(f"Subscribed to ROS topic: {args.ros_topic}")
        except ImportError:
            print("ROS packages not found. Make sure your ROS environment is properly set up.")
            use_ros = False
    
    # If not using ROS, try other video sources
    if not use_ros:
        # Priority: 1. Specified camera index from available list
        if args.camera is not None and 0 <= args.camera < len(available_cameras):
            device_path = available_cameras[args.camera][0]
            print(f"Using camera at {device_path}")
            cap = cv2.VideoCapture(device_path)
        
        # 2. Specified video path
        elif args.video_path:
            # If it's a number, interpret as camera index
            if args.video_path.isdigit():
                args.video_path = int(args.video_path)
            print(f"Using video source: {args.video_path}")
            cap = cv2.VideoCapture(args.video_path)
        
        # 3. Image path
        elif args.image_path and os.path.exists(args.image_path):
            print(f"Using image file: {args.image_path}")
            test_image = cv2.imread(args.image_path)
            if test_image is None:
                print(f"Error: Could not read image file {args.image_path}")
        
        # 4. First available camera
        elif available_cameras:
            device_path = available_cameras[0][0]
            print(f"Using first available camera: {device_path}")
            cap = cv2.VideoCapture(device_path)
        
        # 5. Create a test image as a last resort
        else:
            print("Creating a test image with color squares...")
            test_image = np.zeros((480, 640, 3), dtype=np.uint8)
            
            # Add color squares including a green one
            # Red square
            test_image[100:200, 100:200] = [0, 0, 255]
            # Green square
            test_image[100:200, 250:350] = [0, 255, 0]
            # Blue square
            test_image[100:200, 400:500] = [255, 0, 0]
            # Yellow square
            test_image[250:350, 100:200] = [0, 255, 255]
            # Cyan square
            test_image[250:350, 250:350] = [255, 255, 0]
            # Magenta square
            test_image[250:350, 400:500] = [255, 0, 255]
    
    # Check if we have a valid camera
    if cap is not None and not cap.isOpened() and test_image is None:
        print("Error: Could not open camera or video file")
        
        # Create a test image instead
        print("Creating a test image with color squares...")
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Add color squares including a green one
        # Red square
        test_image[100:200, 100:200] = [0, 0, 255]
        # Green square
        test_image[100:200, 250:350] = [0, 255, 0]
        # Blue square
        test_image[100:200, 400:500] = [255, 0, 0]
        # Yellow square
        test_image[250:350, 100:200] = [0, 255, 255]
        # Cyan square
        test_image[250:350, 250:350] = [255, 255, 0]
        # Magenta square
        test_image[250:350, 400:500] = [255, 0, 255]
    
    # Main loop
    while True:
        # Get frame (different sources)
        frame = None
        
        if use_ros:
            # Get latest image from ROS
            if latest_image is not None:
                frame = latest_image.copy()
            else:
                # Wait for first image
                print("Waiting for ROS image...")
                cv2.waitKey(100)
                continue
        elif test_image is not None:
            # Use static test image
            frame = test_image.copy()
        elif cap is not None:
            # Read from camera or video
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                # For video files, we break; for cameras, we might retry
                if args.video_path and not isinstance(args.video_path, int):
                    break
                continue
        else:
            print("No valid video source")
            break
        
        # Get current positions of all trackbars
        h_lower = cv2.getTrackbarPos('H Lower', 'HSV Tuner')
        s_lower = cv2.getTrackbarPos('S Lower', 'HSV Tuner')
        v_lower = cv2.getTrackbarPos('V Lower', 'HSV Tuner')
        
        h_upper = cv2.getTrackbarPos('H Upper', 'HSV Tuner')
        s_upper = cv2.getTrackbarPos('S Upper', 'HSV Tuner')
        v_upper = cv2.getTrackbarPos('V Upper', 'HSV Tuner')
        
        min_area = cv2.getTrackbarPos('Min Area', 'HSV Tuner')
        
        # Define range for green color in HSV
        lower_green = np.array([h_lower, s_lower, v_lower])
        upper_green = np.array([h_upper, s_upper, v_upper])
        
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Apply the mask to get the result
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Display contour outlines on the original frame
        frame_with_contours = frame.copy()
        
        # Process contours
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area > min_area:
                # Draw contour outline
                cv2.drawContours(frame_with_contours, [contour], -1, (0, 255, 0), 2)
                
                # Calculate centroid
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # Draw centroid
                    cv2.circle(frame_with_contours, (cx, cy), 5, (0, 255, 0), -1)
                    
                    # Add text with coordinates
                    text = f"green: ({cx}, {cy})"
                    cv2.putText(frame_with_contours, text, (cx + 10, cy), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Display HSV values on the feed
        hsv_text = f"Green HSV: [{h_lower}, {s_lower}, {v_lower}] to [{h_upper}, {s_upper}, {v_upper}]"
        cv2.putText(frame_with_contours, hsv_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Display instructions
        instructions_text = "Press 's' to save values, 'q' to quit"
        cv2.putText(frame_with_contours, instructions_text, (10, frame.shape[0] - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Show images
        cv2.imshow('Camera Feed', frame_with_contours)
        cv2.imshow('Mask', mask)
        cv2.imshow('Result', result)
        
        # Break loop on 'q' key press
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print(f"\nFinal HSV values for green detection:")
            print(f"lower_green = np.array([{h_lower}, {s_lower}, {v_lower}])")
            print(f"upper_green = np.array([{h_upper}, {s_upper}, {v_upper}])")
            print(f"Min contour area: {min_area}")
            
            # Print ready-to-use code for the ROS color tracker
            print("\nAdd to your ColorTrackerNode class:")
            print("# Green")
            print("'green': (")
            print(f"    np.array([{h_lower}, {s_lower}, {v_lower}]),  # lower_green")
            print(f"    np.array([{h_upper}, {s_upper}, {v_upper}]),  # upper_green")
            print("    (0, 255, 0)  # Display color (BGR): Green")
            print(")")
            break
        
        # Save values on 's' key press
        if key == ord('s'):
            print(f"\nSaved HSV values for green detection:")
            print(f"lower_green = np.array([{h_lower}, {s_lower}, {v_lower}])")
            print(f"upper_green = np.array([{h_upper}, {s_upper}, {v_upper}])")
            print(f"Min contour area: {min_area}")
    
    # Release resources
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()
    
    # Print help if we had to use a test image
    if test_image is not None:
        print("\nNOTE: You used a test image. For better results:")
        print("1. Try specifying a camera device directly:")
        for i, (device, name) in enumerate(available_cameras):
            print(f"   python3 tune_HSV.py --camera {i}  # {device} - {name}")
        print("2. Or use a ROS topic:")
        print("   python3 tune_HSV.py --ros_topic /camera/color/image_raw")
        print("3. Or use a video file:")
        print("   python3 tune_HSV.py --video_path my_video.mp4")
        print("4. Or use a test image:")
        print("   python3 tune_HSV.py --image_path test_image.jpg")

if __name__ == "__main__":
    create_hsv_tuner()