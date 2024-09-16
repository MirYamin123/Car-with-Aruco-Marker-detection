# Car-with-Aruco-Marker-detection
This project demonstrates real-time object detection using ArUco markers with a Flask-based web interface for controlling motors, a robotic arm, and aligning objects. The project utilizes a Raspberry Pi with a camera for image capture and processes the feed using Python, OpenCV, and Flask. DC motors are controlled via an Arduino over serial communication to facilitate movement, alignment with detected marker. Rasberry Pi with PCA Servo controller control the robotic arm to pick and place the objects.

## Features:

1. ArUco Marker Detection: Detects ArUco markers in real-time from a camera feed.

2. Distance Estimation: Estimates the distance of ArUco markers from the camera.

3. Motor Control: Controls motors to move a robot towards or align with detected markers.

4. Real-time Video Feed: Streams live video from the Raspberry Pi camera to a web interface.

5. Flask Web Interface: Provides an interactive interface to control motor movements and trigger functions like marker alignment or stopping the robot.

6. Failsafe Mechanism: Automatically stops the robot if the marker is lost from view.

7. Robotic Arm: A mounted robotic arm controlled via servos that can be manipulated to pick up objects.

## Requirements:
1. Hardware
   
   a. Raspberry Pi 3B+
   
   b. Arduino UNO R3(for motor control)
   
   c. Servo 995 and DC motors 60RPM 12V
   
   d. PCA9685 servo controller
   
   e. L298d Motor Driver (2x)
   
   f. Camera (Pi Camera)

   g. Robotic Arm mounted on the car, controlled by 4 servo motors (connected to the PCA9685)
   
3. Software Dependencies
   
    a. Python 3

    b. Flask
  
    c. Flask SocketIO
  
    d. OpenCV
  
    e. NumPy
  
    f. PySerial
  
    g. Adafruit_PCA9685

## SolidWorks Car Model
The project includes SolidWorks files for the car design used in this project. The model showcases the design and component placement used for motor control. Design doesnt include tyres used. The prototype used 80mm Mecanum wheels.

## Installation:

1. Set up the Raspberry Pi and Arduino
   
2. Ensure the Raspberry Pi is properly connected to the Pi Camera.

3. Connect the Arduino UNO to control the motors and servos.

4. Wire the PCA9685 Servo Controller and L298D Motor Drivers correctly.

5. To install all required libraries, run:

`pip install -r requirements.txt`

6. Upload Arduino Code.

7. Connect the Arduino UNO to your PC.
   
8. Open the Arduino IDE and upload the corresponding motor control code.
    
9. Set the baud rate to 9600 for serial communication with the Raspberry Pi.

10.
    File Structure:

      a. Create a folder for your project.

      b. Place app.py in this folder.

      c. Inside the same folder, create a subfolder named templates.

      d. Inside the templates folder, create a file named index.html.

12. Run the Flask App
    
    `python app.py`
    
     The app will be accessible at `http://<your_pi_ip>:5000`.

    To find your Raspberry Pi's IP address, run the following command in the terminal:

    ```bash
    hostname -I
    ```

    Replace `<your_pi_ip>` with the IP address returned by the command.


## How it works:

The Pi camera captures a real-time video stream, which is processed using OpenCV to detect ArUco markers.

Based on the marker's position, the robot can move forward, rotate, or align itself using the motor control code.

The Flask web interface allows remote control of the robot and provides a real-time video feed from the camera.

The robotic arm can be manually controlled through the web interface. Using buttons on the interface, you can control the arm's servos to move it and pick up objects.










   

  
