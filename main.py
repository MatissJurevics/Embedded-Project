#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile, Font
import json
import http



# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

ev3 = EV3Brick()

# Create your objects here.
font = Font('Lucida', 10)
ev3.screen.set_font(font)


sensorL = ColorSensor(Port.S1)
sensorR = ColorSensor(Port.S4)
ultrasonic = UltrasonicSensor(Port.S2)
motorL = Motor(Port.D )
motorL.dc(100)

motorR = Motor(Port.A)
# button = TouchSensor(Port.S2)
robot = DriveBase(motorL, motorR, wheel_diameter=55.5, axle_track=104)



class Vehicle:
    def __init__(self, sensorL, sensorR, robot, motorL = None, motorR = None, hub = None, ultrasonic = None):
        self.sensorL = sensorL;
        self.sensorR = sensorR;
        self.robot = robot;
        self.hub = hub;
        self.ultraSonic = ultrasonic;
        self.screen = hub.screen;
        self.motorL = motorL;
        self.motorR = motorR;
        self.lows = {"left": [100,100,100], "right": [100,100,100]}
        self.highs = {"left": [0,0,0], "right": [0,0,0]}
        self.mappedColors = {}
    
    def _waitForClick(self):
        """
        `waitForClick` checks if the touch sensor is attached and if it is, 
        waits until it is pressed. 
        """
        pressed = False
        if self.hub == None:
            raise Exception("Button is not attached to the vehicle (check if you passed it on init)")
        else:
            while not pressed:
                if self.hub.buttons.pressed():
                    pressed = True

    def _map(self, cur, low, high, min = 0, max = 100):
        if cur < low:
            cur = low
        elif cur > high:
            cur = high
        ratio = (cur - low) / (high - low)
        return min + (max - min) * ratio
    
    def _getMappedColors(self):
        leftVal = self.sensorL.rgb()
        rightVal = self.sensorR.rgb()
        mappedRight = []
        mappedLeft = []
        for i in range(3):
            mappedRight.append(self._map(rightVal[i], self.lows["right"][i], self.highs["right"][i])) 
            mappedLeft.append(self._map(leftVal[i], self.lows["left"][i], self.highs["left"][i]))
            if i == 1:
                mappedRight[i] *= 1.3
                mappedLeft[i] *= 1.3
        return [tuple(mappedLeft), tuple(mappedRight)]

    def loadCalibratedData(self):
        with open("calibrationData.json", "r") as f:
            data = json.load(f)
            self.lows = data["lows"]
            self.highs = data["highs"]
            self.mappedColors = data["mappedColors"]
            print(data)
            print("Calibration data loaded...")
    
    def calibrate(self):
        
        colors = ["red", "blue", "yellow", "green", "black", "white"]
        values = {}

        for color in colors:
            print("place on " + color)
            self.screen.clear()
            self.screen.draw_text(0, 20, "Place on " + color)
            self._waitForClick()
            valL = self.sensorL.rgb()
            valR = self.sensorR.rgb()
            print(valL, valR)
            self.screen.clear()
            self.screen.draw_text(0, 20, str(valL))
            self.screen.draw_text(0, 40, str(valR))
            values[color] = {"left": valL, "right": valR}
            wait(300)
        
        maxVals = {
            "left": [0,0,0],
            "right": [0,0,0]
        }
        
        minVals = {
            "left": [100,100,100],
            "right": [100,100,100]
        }

        for value in values:
            for side in values[value]:
                for idx, val in enumerate(values[value][side]):
                    if val < self.lows[side][idx]:
                        self.lows[side][idx] = val
                    elif val > self.highs[side][idx]:
                        self.highs[side][idx] = val
        
        mappedColors = {}
        for color in colors:
            mappedColors[color] = {"left": [0,0,0], "right": [0,0,0]}
            for side in values[color]:
                for i in range(3):
                    mappedVal = self._map(values[color][side][i], self.lows[side][i], self.highs[side][i])
                    mappedColors[color][side][i] = mappedVal
        self.mappedColors = mappedColors
        calData = {
            "lows": self.lows,
            "highs": self.highs,
            "mappedColors": self.mappedColors
        }
        with open("calibrationData.json", "w") as f:
            json.dump(calData, f)

    def _getClosestColor(self):
        mappedColors = self._getMappedColors()
        # print(mappedColors)
        closestColor = ["",""]
        closestDistance = [1000,1000]
        for color in self.mappedColors:
            for idx, side in enumerate(mappedColors):
                sides = ["left", "right"]
                distance = 0
                for i in range(3):
                    distance += abs(side[i] - self.mappedColors[color][sides[idx]][i])
                if distance < closestDistance[idx]:
                    closestDistance[idx] = distance
                    closestColor[idx] = color
        for i in range(2):
            closestDistance[i] = round(closestDistance[i], 2)
        return [closestColor[0], closestColor[1], closestDistance]  

    def driveStraight(self):
        while True:
            self.robot.drive(100,0)
            
    def detectObstacle(self):
        dist = self.ultraSonic.distance()
        return dist
        
    def switchLane(self):
        self.robot.turn(-90)
        self.robot.drive(200,0)
        wait(1000)
        self.robot.turn(90)
        self.robot.drive(100,0)
        wait(750)
        
        
    def drive(self):
        timer = 0
        max_speed = 200        # Maximum speed
        min_speed = 100        # Minimum speed (for turning)
        accel = 5    # Acceleration rate
        speed = min_speed     # Initial speed
        turning_angle = 0     # Initial turning angle
        frame = 0
        blue_frames = 0
        side_weight = [0,0]
        soft_turn = 30
        sharp_turn = 85
        left_turns = 0
        drift = -4
        
        stopped_at_blue = False
        at_crossing = False   
        changed_lanes = False    
        self.screen.clear()
        while timer < 1500:
            self.screen.draw_text(0, 20, "press to calibrate")
            if self.hub.buttons.pressed():
                wait(300)
                self.calibrate()
            wait(100)
            timer += 100
        
        self.hub.speaker.beep()
        while True:
            objectClose = (self.detectObstacle() < 500)
            frame += 1
            # Get the color from the sensors
            color = self._getClosestColor()
            if objectClose:
                self.switchLane()
            
            # Check if either side is white or green (road edges)
            left_white = (color[0] == "white" or color[0] == "green")
            right_white = (color[1] == "white" or color[1] == "green")
            
            # Check if both sides are red (stop condition)
            left_red = (color[0] == "red")
            right_red = (color[1] == "red")
            
            if left_red and right_red:
                if changed_lanes:
                    self.robot.stop()
                    self.hub.speaker.beep()
                    break
                else:
                    left_lane = (side_weight[0] > side_weight[1])
                    changed_lanes = True
                    if left_lane:
                        self.robot.turn(-90)
                        self.robot.drive(200,0)
                        wait(1000)
                        self.robot.turn(90)
                        self.robot.drive(100,0)
                        wait(750)
                        speed = min_speed
                        side_weight = [0,1000000]
                    else:
                        self.robot.turn(90)
                        self.robot.drive(200,0)
                        wait(850)
                        self.robot.turn(-70)
                        self.robot.drive(100,0)
                        wait(750)
                        speed = min_speed
                        side_weight = [1000000,0]
                    
            
            
            
            # If road edge is detected on the left, turn right and slow down
            if left_white and not at_crossing:
                if color[0] == "green":
                    side_weight[0] += 1
                    left_turns += 1
                else:
                    side_weight[0] -= 1
                turning_angle = -soft_turn if side_weight[0] > side_weight[1] else -sharp_turn
                turning_angle += 20 if left_turns > 5 else 0
                speed -= accel * 10
                
                if speed < min_speed:
                    speed = min_speed  # Slow down for turning
                    
            # If road edge is detected on the right, turn left and slow down
            elif right_white and not at_crossing:
                if color[1] == "green":
                    side_weight[1] += 1
                else:
                    side_weight[1] -= 1
                left_turns = 0
                turning_angle = soft_turn
                speed -= accel * 10
                if speed < min_speed:
                    speed = min_speed  # Slow down for turning
            # If no road edge detected, drive straight and accelerate
            else:
                turning_angle = drift if side_weight[0] < side_weight[1] else 0
                if speed < max_speed:
                    speed += accel  # Gradually increase speed
            # print(color, speed, turning_angle)
            if frame % 10 == 0:
                lane = "left lane" if side_weight[0] > side_weight[1] else "right lane"
                blue = "blue" if color[0] == "blue" or color[1] == "blue" else "not blue"
                self.screen.clear()
                self.screen.draw_text(0, 20, str("L:{}  R:{} CL:{}  CR:{}".format(color[0], color[1], color[2][0], color[2][1])))
                self.screen.draw_text(0, 40, str(speed))
                self.screen.draw_text(0, 60, str(turning_angle))
                self.screen.draw_text(0, 80, "{lane}: {side_weight}".format(lane=lane, side_weight=side_weight))
                self.screen.draw_text(0, 100, str(blue))
            # Drive the robot with the calculated speed and turning angle
            if (color[0] == "yellow" or color[1] == "yellow") and not at_crossing:
                at_crossing = True
                self.robot.drive(75,0)
                wait(2300)
                speed = min_speed
                at_crossing = False
                
            if (color[0] == "blue" and color[1] == "blue"):
                print("blue found", blue_frames)
                blue_frames += 1
                if blue_frames > 1:
                    self.robot.stop()
                    wait(3000)
                    self.robot.drive(100,0)
                    wait(300)
                    speed = min_speed
                    blue_frames = 0
            else:
                blue_frames = 0
            self.robot.drive(speed, turning_angle)
           

        



        


car = Vehicle(sensorL, sensorR, robot,motorL, motorR, ev3, ultrasonic)
# car.calibrate()
car.loadCalibratedData()
car.drive()
# car.driveStraight()
