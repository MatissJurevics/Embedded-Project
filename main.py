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
        # Values used in the driving stage
        # -------------------------------
        # Add values here to adjust the driving behavior between functions
        self.min_speed = 100
        self.max_speed = 200
        self.accel = 5
        self.turning_angle = 0
        self.speed = 0
        self.frame = 0
        self.speed = self.min_speed
        self.blue_frames = 0
        self.side_weight = [0,0]
        self.soft_turn = 30
        self.sharp_turn = 85
        self.left_turns = 0
        self.drift = -4
        self.objectDistance = 0
        self.color = None
        self.changed_lanes = False 
        self.at_crossing = False
        
        
    
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
        returnVal = [closestColor[0], closestColor[1], closestDistance]
        self.color = returnVal

    def driveStraight(self): # Redundant function, used for testing
        while True:
            self.robot.drive(100,0)
            
    def detectObstacle(self):
        dist = self.ultraSonic.distance()
        self.objectDistance = dist
        
    def switchLane(self): # TODO: Make this more fluid for the obstacle avoidance
        self.robot.turn(-90)
        self.robot.drive(200,0)
        wait(1000)
        self.robot.turn(90)
        self.robot.drive(100,0)
        wait(750)
        
    def _print_data(self):
        lane = "left lane" if  self.side_weight[0] > self.side_weight[1] else "right lane"
        blue = "blue" if self.color[0] == "blue" or self.color[1] == "blue" else "not blue"
        self.screen.clear()
        self.screen.draw_text(0, 20, str("L:{}  R:{} CL:{}  CR:{}".format(self.color[0], self.color[1], self.color[2][0], self.color[2][1])))
        self.screen.draw_text(0, 40, str(self.speed))
        self.screen.draw_text(0, 60, str(self.turning_angle))
        self.screen.draw_text(0, 80, "{lane}: {side_weight}".format(lane=lane, side_weight=self.side_weight))
        self.screen.draw_text(0, 100, str(self.objectDistance))
    
    def _print_menu(self, timer):
        self.screen.draw_text(0, 20, "press to calibrate")
        if self.hub.buttons.pressed():
            wait(300)
            self.calibrate()
        wait(100)
        return timer + 100
    
    def _handle_blue(self):
        self.robot.stop()
        wait(3000)
        self.robot.drive(100,0)
        wait(300)
        self.speed = self.min_speed
        self.blue_frames = 0
    
    def _handle_yellow(self):
        self.at_crossing = True
        self.robot.drive(75,0)
        wait(2300)
        self.speed = self.min_speed
        self.at_crossing = False
        
    def _handle_red(self):
        if self.changed_lanes:
            self.robot.stop()
            self.hub.speaker.beep()
            exit()
        else:
            left_lane = (self.side_weight[0] > self.side_weight[1])
            self.changed_lanes = True
            if left_lane:
                self.robot.turn(-90)
                self.robot.drive(200,0)
                wait(1000)
                self.robot.turn(90)
                self.robot.drive(100,0)
                wait(750)
                self.speed = self.min_speed
                self.side_weight = [0,1000000]
            else:
                self.robot.turn(90)
                self.robot.drive(200,0)
                wait(850)
                self.robot.turn(-70)
                self.robot.drive(100,0)
                wait(750)
                self.speed = self.min_speed
                self.side_weight = [1000000,0]
    
    def _handle_light(self):
        if not self.at_crossing:
            if self.color[0] == "green" or self.color[0] == "white":
                if self.color[0] == "green":
                    self.side_weight[0] += 1
                    self.left_turns += 1
                else:
                    self.side_weight[0] -= 1
                self.turning_angle = -self.soft_turn if self.side_weight[0] > self.side_weight[1] else -self.sharp_turn
                self.turning_angle += 20 if self.left_turns > 5 else 0
                self.speed -= self.accel * 10
                if self.speed < self.min_speed:
                    self.speed = self.min_speed
            elif self.color[1] == "green" or self.color[1] == "white":
                if self.color[1] == "green":
                    self.side_weight[1] += 1
                else:
                    self.side_weight[1] -= 1
                self.left_turns = 0
                self.turning_angle = self.soft_turn
                self.speed -= self.accel * 10
                if self.speed < self.min_speed:
                    self.speed = self.min_speed
    
    def _process_color(self):
        light = (self.color[0] == "white" or self.color[0] == "green" or self.color[1] == "white" or self.color[1] == "green")
        red = (self.color[0] == "red" and self.color[1] == "red")
        yellow = (self.color[0] == "yellow" or self.color[1] == "yellow")
        blue = (self.color[0] == "blue" and self.color[1] == "blue")
        if not blue:
            self.blue_frames = 0
        if red:
            self._handle_red()
        elif yellow:
            self._handle_yellow()
        elif blue:
            self._handle_blue()
        elif light:
            self._handle_light()        
           
    
    def drive(self):
        timer = 0        
        while timer < 1500:
            self.screen.clear()
            timer = self._print_menu(timer)
        self.hub.speaker.beep()
        
        while True:
            self.detectObstacle()
            self._getClosestColor()
            self._process_color()
            self.frame += 1
            if self.frame % 10 == 0:
                self._print_data()
            self.robot.drive(self.speed, self.turning_angle)
           

        



        


car = Vehicle(sensorL, sensorR, robot,motorL, motorR, ev3, ultrasonic)
# car.calibrate()
car.loadCalibratedData()
car.drive()
# car.driveStraight()
