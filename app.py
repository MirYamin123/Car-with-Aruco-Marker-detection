from flask import Flask, render_template, Response, jsonify
from flask_socketio import SocketIO, emit
import serial
import time
import subprocess
import cv2
import numpy as np
from adafruit_servokit import ServoKit
import threading

app = Flask(__name__)
socketio = SocketIO(app)

# Initialize PCA9685 with 16 channels for servo control
kit = ServoKit(channels=16)

# Initial angles for the servos
angle1 = 82
angle2 = 90
angle3 = 134
angle4 = 90
kit.servo[0].angle = angle1
kit.servo[1].angle = angle2
kit.servo[2].angle = angle3
kit.servo[3].angle = angle4

# Lock for thread synchronization
servo_lock = threading.Lock()
detection_lock = threading.Lock()

# Global variables
detected_markers = {}
marker_miss_count = {}  # Track how many frames a marker hasn't been detected
max_miss_frames = 4  # Number of frames before marking as "Not Detected"
latest_frame = None  # Store the latest frame for streaming
scan_active = False  # Flag to control the scanning process
abort_scan_flag = False  # Flag to control aborting the scan
motor_pressed = False  # Track whether a motor movement button is pressed

# Adjust servos function
def adjust_servo(servo_channel, delta):
    global angle1, angle2, angle3, angle4

    with servo_lock:
        if servo_channel == 0:
            angle1 = max(0, min(180, angle1 + delta))
            kit.servo[servo_channel].angle = angle1
            socketio.emit('update_angle', {'servo': 1, 'angle': angle1})
        elif servo_channel == 1:
            angle2 = max(0, min(178, angle2 + delta))  # Limit to 178 degrees
            kit.servo[servo_channel].angle = angle2
            socketio.emit('update_angle', {'servo': 2, 'angle': angle2})
        elif servo_channel == 2:
            angle3 = max(0, min(180, angle3 + delta))
            kit.servo[servo_channel].angle = angle3
            socketio.emit('update_angle', {'servo': 3, 'angle': angle3})
        elif servo_channel == 3:
            angle4 = max(0, min(180, angle4 + delta))
            kit.servo[servo_channel].angle = angle4
            socketio.emit('update_angle', {'servo': 4, 'angle': angle4})

# Setup the serial connection for motor control
try:
    arduino = serial.Serial('/dev/ttyACM0', 9600)
    time.sleep(2)
except serial.SerialException as e:
    print(f"Error opening serial connection: {e}")
    exit(1)

def send_command(command):
    """Send a command to the Arduino."""
    arduino.write(command.encode())
    arduino.flush()
    print(f"Command sent: {command}")

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return render_template('index.html', angle1=angle1, angle2=angle2, angle3=angle3, angle4=angle4)

@app.route('/marker_info')
def marker_info():
    """Return the detected marker information as JSON."""
    return jsonify(detected_markers)

def reset_movement_state():
    """Resets the movement state to allow smooth continuous control."""
    send_command('S')  # Ensure the car is stopped
    time.sleep(0.2)  # Ensure a small delay for the stop command to take effect
    global motor_pressed
    motor_pressed = False  # Reset motor_pressed flag to allow smooth movement on button press
    
# Thread for running ArUco detection
def detect_markers():
    global detected_markers, marker_miss_count, latest_frame, scan_active

    # Load the ArUco dictionary and parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    aruco_params = cv2.aruco.DetectorParameters()

    marker_real_size = 0.05  # 5 cm
    focal_length = 640
    frame_width = 1280  # Adjust according to your camera resolution

    while True:
        with detection_lock:
            if latest_frame is None:
                continue

            # Detect ArUco markers in the frame
            gray_frame = cv2.cvtColor(latest_frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=aruco_params)

            current_ids = set()  # Track markers detected in this frame

            if ids is not None:
                for i, corner in enumerate(corners):
                    marker_id = int(ids[i][0])
                    current_ids.add(marker_id)

                    # Reset the miss count when marker is detected
                    marker_miss_count[marker_id] = 0

                    # Calculate the center of the marker
                    center_x = int(corner[0][:, 0].mean())

                    # Calculate the marker's apparent width in the image
                    marker_width_px = np.linalg.norm(corner[0][0] - corner[0][1])

                    # Estimate the distance from the camera
                    if marker_width_px > 0:
                        distance_to_marker = (marker_real_size * focal_length) / marker_width_px

                        # Calculate how far the marker is from the midline
                        offset_from_center = center_x - (frame_width // 2)
                        offset_percentage = (offset_from_center / (frame_width // 2)) * 100  # In percentage
                        detected_markers[marker_id] = {
                            'distance': f'{distance_to_marker:.2f}m',
                            'offset_from_center': f'{offset_percentage:.2f}%',  # Show percentage
                            'status': 'Detected'
                        }
                    else:
                        detected_markers[marker_id] = {'distance': 'Size error', 'status': 'Detected'}

            # Increment the miss count for markers not detected in this frame
            for marker_id in list(detected_markers.keys()):
                if marker_id not in current_ids:
                    if marker_id not in marker_miss_count:
                        marker_miss_count[marker_id] = 0
                    marker_miss_count[marker_id] += 1

                    # Mark marker as "Not in Range" if it hasn't been detected in max_miss_frames
                    if marker_miss_count[marker_id] > max_miss_frames:
                        detected_markers[marker_id]['status'] = 'Not in Range'
                        send_command('S')  # Automatically stop the car if marker is not detected
                        socketio.emit('marker_lost', {'message': f'Marker {marker_id} lost. Car stopped.'})

# Thread for video streaming
def generate_frames():
    global latest_frame

    # Increase the resolution and test performance
    command = "rpicam-vid -t 0 --output - --width 1280 --height 720 --framerate 15 --codec yuv420"
    process = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True, bufsize=4096)

    frame_width = 1280
    frame_height = 720
    frame_size = frame_width * frame_height * 3 // 2

    while True:
        raw_image = process.stdout.read(frame_size)
        if len(raw_image) != frame_size:
            continue

        yuv_image = np.frombuffer(raw_image, dtype=np.uint8).reshape((frame_height * 3 // 2, frame_width))
        latest_frame = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_I420)  # Update the latest frame

        # Draw midline
        cv2.line(latest_frame, (frame_width // 2, 0), (frame_width // 2, frame_height), (0, 255, 0), 2)

        ret, buffer = cv2.imencode('.jpg', latest_frame)
        if not ret:
            continue

        frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# Additional global variable to manage the stop signal
stop_near_marker = False

@socketio.on('stop_near_marker')
def stop_near_marker_event():
    global stop_near_marker
    stop_near_marker = True
    send_command('S')  # Stop movement immediately
    socketio.emit('stop_signal_received', {'message': 'Stopped going near the marker.'})

# Modify the existing align_with_marker and move_to_distance to check stop_near_marker
@socketio.on('align_with_marker')
def align_with_marker(data):
    global stop_near_marker
    marker_id = data['marker_id']
    stop_near_marker = False  # Reset stop flag
    
    if marker_id in detected_markers:
        offset_str = detected_markers[marker_id]['offset_from_center']
        offset = float(offset_str.replace('%', ''))

        while abs(offset) > 13:
            if stop_near_marker:
                send_command('S')
                break
            if marker_id not in detected_markers:
                send_command('S')
                break
            if offset > 0:
                send_command('X')
            else:
                send_command('Z')
            time.sleep(0.1)  # Delay to check marker detection
            send_command('S')  # Stop after each adjustment
            time.sleep(3)  # Wait for detection updates
            offset_str = detected_markers.get(marker_id, {}).get('offset_from_center', '0%')
            offset = float(offset_str.replace('%', ''))

        socketio.emit('alignment_complete', {'message': f'Marker {marker_id} aligned.'})
        reset_movement_state()  # Reset the movement behavior to smooth after aligning

@socketio.on('move_to_distance')
def move_to_distance(data):
    global stop_near_marker
    marker_id = data['marker_id']
    target_distance = data['distance']
    stop_near_marker = False  # Reset stop flag

    if marker_id in detected_markers:
        distance_str = detected_markers[marker_id]['distance']
        distance = float(distance_str.replace('m', ''))

        while distance > target_distance:
            if stop_near_marker:
                send_command('S')
                break
            if marker_id not in detected_markers:
                send_command('S')
                break
            send_command('F')  # Move forward
            time.sleep(1)  # Delay for smooth movement
            send_command('S')
            time.sleep(3)  # Wait for detection updates
            distance_str = detected_markers.get(marker_id, {}).get('distance', '0m')
            distance = float(distance_str.replace('m', ''))

        socketio.emit('movement_complete', {'message': f'Marker {marker_id} is now {target_distance}m away.'})
        reset_movement_state()  # Reset the movement behavior to smooth after moving

### Updated handle_command to ensure continuous movement
@socketio.on('command')
def handle_command(data):
    global motor_pressed
    cmd = data['command']
    event = data['event']
    
    motor_commands = ['F', 'B', 'G', 'H', 'I', 'J', 'K', 'M', 'Z', 'X']

    if cmd in motor_commands:
        if event == 'press':
            send_command(cmd)  # Start movement (keep moving until button release)
        elif event == 'release':
            send_command('S')  # Stop movement when button is released
    else:
        # Servo command handling
        if cmd == 'A':
            adjust_servo(0, 1)
        elif cmd == 'a':
            adjust_servo(0, -1)
        elif cmd == 'E':
            adjust_servo(1, 1)
        elif cmd == 'e':
            adjust_servo(1, -1)
        elif cmd == 'C':
            adjust_servo(2, 1)
        elif cmd == 'c':
            adjust_servo(2, -1)
        elif cmd == 'L':
            adjust_servo(3, 1)
        elif cmd == 'l':
            adjust_servo(3, -1)
        else:
            send_command(cmd)  # Handles other commands

if __name__ == '__main__':
    # Start detection thread
    detection_thread = threading.Thread(target=detect_markers)
    detection_thread.daemon = True
    detection_thread.start()

    # Run Flask app
    socketio.run(app, host='0.0.0.0', port=5000)
