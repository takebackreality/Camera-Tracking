#!/usr/bin/env python3

import cv2
import numpy as np
import serial
import time
import argparse
from datetime import datetime

class TurretController:
    def __init__(self, serial_port, baud_rate=9600):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None
        self.connect()
        
    def connect(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)
            print(f"Connected to Arduino on {self.serial_port}")
        except serial.SerialException as e:
            print(f"Error connecting to Arduino: {e}")
            self.ser = None
            
    def send_command(self, pan, tilt):
        if self.ser and self.ser.is_open:
            command = f"{pan}:{tilt}\n"
            self.ser.write(command.encode())
            response = self.ser.readline().decode().strip()
            if response:
                print(f"Arduino response: {response}")
        else:
            print("Not connected to Arduino")
            
    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed")

class MotionDetector:
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self.previous_frame = None
        self.motion_threshold = 1000
        self.sensitivity = 25
        
    def detect_motion(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        
        if self.previous_frame is None:
            self.previous_frame = gray
            return None
            
        frame_delta = cv2.absdiff(self.previous_frame, gray)
        thresh = cv2.threshold(frame_delta, self.sensitivity, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)
        
        contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        centroid = None
        for contour in contours:
            if cv2.contourArea(contour) < self.motion_threshold:
                continue
            (x, y, w, h) = cv2.boundingRect(contour)
            centroid = (x + w//2, y + h//2)
            break
        
        self.previous_frame = gray
        return centroid

def main():
    parser = argparse.ArgumentParser(description='Arduino Turret Camera Tracking')
    parser.add_argument('--serial', '-s', type=str, default='/dev/ttyUSB0',
                       help='Serial port for Arduino (default: /dev/ttyUSB0)')
    parser.add_argument('--camera', '-c', type=int, default=0,
                       help='Camera index (default: 0)')
    parser.add_argument('--no-arduino', action='store_true',
                       help='Run without Arduino connection (for testing)')
    args = parser.parse_args()
    
    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    motion_detector = MotionDetector()
    
    turret_controller = None
    if not args.no_arduino:
        turret_controller = TurretController(args.serial)
    else:
        print("Running in test mode (no Arduino connection)")
    
    pan, tilt = 90, 90
    
    print("Starting camera tracking. Press 'q' to quit.")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
                
            frame = cv2.flip(frame, 1)
            centroid = motion_detector.detect_motion(frame)
            
            if centroid is not None:
                cv2.circle(frame, centroid, 10, (0, 0, 255), -1)
                pan = int(centroid[0] * 180 / frame.shape[1])
                tilt = int(centroid[1] * 180 / frame.shape[0])
                
                if turret_controller:
                    turret_controller.send_command(pan, tilt)
            
            cv2.putText(frame, f"Pan: {pan}, Tilt: {tilt}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow("Turret Camera Tracking", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if turret_controller:
            turret_controller.close()
        print("Program ended")

if __name__ == "__main__":
    main()
