# Car-with-Aruco-Marker-detection
This project demonstrates real-time object detection using ArUco markers with a Flask-based web interface for controlling motors and aligning objects. The project utilizes a Raspberry Pi with a camera for image capture and processes the feed using Python, OpenCV, and Flask. Servo motors and DC motors are controlled via an Arduino over serial communication to facilitate movement and alignment with detected markers.

Features:

1.ArUco Marker Detection: Detects ArUco markers in real-time from a camera feed.

2.Distance Estimation: Estimates the distance of ArUco markers from the camera.

3.Motor Control: Controls motors to move a robot towards or align with detected markers.

4.Real-time Video Feed: Streams live video from the Raspberry Pi camera to a web interface.

5.Flask Web Interface: Provides an interactive interface to control motor movements and trigger functions like marker alignment or stopping the robot.

6.Failsafe Mechanism: Automatically stops the robot if the marker is lost from view.

Requirements:
1. Hardware
   a. Raspberry Pi
   
   b. Arduino UNO (for motor control)
   
   c. Servo 995 and DC motors 60RPM 12V
   
   d. PCA9685 servo controller
   
   e. L298d Motor Driver (2x)
   
   f. Camera (Pi Camera)
   
3. Software Dependencies
   
    a. Python 3

    b. Flask
  
    c. Flask SocketIO
  
    d. OpenCV
  
    e. NumPy
  
    f. PySerial
  
    g. Adafruit_PCA9685


   

  
