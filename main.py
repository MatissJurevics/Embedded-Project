#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile, Font
from pybricks.messaging import BluetoothMailboxServer, TextMailbox, BluetoothMailboxClient
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
ir = InfraredSensor(Port.S3)
motorL = Motor(Port.D )
motorL.dc(100)

motorR = Motor(Port.A)
# button = TouchSensor(Port.S2)
robot = DriveBase(motorL, motorR, wheel_diameter=55.5, axle_track=104)



class Vehicle:
    def __init__(self, sensorL, sensorR, robot, motorL = None, motorR = None, hub = None, ultrasonic = None, infrared = None):
        self.sensorL = sensorL;
        self.sensorR = sensorR;
        self.robot = robot;
        self.hub = hub;
        self.ultrasonic = ultrasonic;
        self.infrared = infrared;
        self.screen = hub.screen;
        self.motorL = motorL;
        self.motorR = motorR;
        self.lows = {"left": [100,100,100], "right": [100,100,100]}
        self.highs = {"left": [0,0,0], "right": [0,0,0]}
        self.mappedColors = {}
        # Values used in the driving stage
        # -------------------------------
        # Add values here to adjust the driving behavior between functions
        self.continue_driving = True
        self.min_speed = 100
        self.max_speed = 200
        self.accel = 3
        self.turning_angle = 0
        self.speed = 0
        self.frame = 0
        self.speed = self.min_speed
        self.blue_frames = 0
        self.side_weight = [0,0]
        self.soft_turn = 50
        self.sharp_turn = 70
        self.left_turns = 0
        self.drift = -4
        self.objectDistance = 0
        self.color = None
        self.changed_lanes = False 
        self.at_crossing = False
        self.obstacle_distance = 0
        self.obstacle_count = 0
        self.obstacle_higher_threshold = 50.8
        self.obstacle_lower_threshold = 10.0
        self.objectAvoidThreashold = 250
        self.skip_turn_logic = False
        
        
    
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

            
    def detectObstacle(self):
        dist = self.ultrasonic.distance()
        self.objectDistance = dist
        
        
    def switchLane(self): # DEPRECATED: No longer relevant for the assignment
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
        
    def _print_data_console(self):
        return
        # print("L:{}  R:{} CL:{}  CR:{}".format(self.color[0], self.color[1], self.color[2][0], self.color[2][1]), end="\r", flush=True)
        # print("Speed: " ,self.speed, end="\r", flush=True)
        # print("Turning angle: ", self.turning_angle, end="\r", flush=True)
        # print("{lane}: {side_weight}".format(lane=lane, side_weight=self.side_weight), end="\r", flush=True)
        # print(self.objectDistance)
    
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
            print("changed lanes")
            self.hub.speaker.beep()
            # self.robot.stop()
            # self.hub.speaker.beep()
            # exit()
        else:
            self.changed_lanes = True # called changed lanes but refers to crossing red
            # self._switch_lane()          
    
    def _switch_lane(self):
        left_lane = (self.side_weight[0] > self.side_weight[1])
        if left_lane:
            # self.robot.turn(-90)
            self.robot.drive(150,-80)
            wait(700)
            self.robot.stop()
            wait(100)
            # self.robot.turn(90)
            self.robot.drive(150,80)
            wait(700)
            self.robot.stop()
            wait(100)
            self._getClosestColor()
            if self.color[0] == "green" and self.color[1] == "green":
                self.robot.turn(30)
            self.speed = self.min_speed
            self.side_weight = [0,1000000]
        else:
            # self.robot.turn(90)
            self.robot.drive(150,80)
            wait(500)
            # self.robot.turn(90)
            self.robot.drive(150,-45)
            wait(400)
            self.robot.stop()
            wait(100)
            if self.color[0] == "green" and self.color[1] == "green":
                self.robot.turn(-30)
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
                self.turning_angle -= 40 if self.left_turns > 5 else 0
               
                self.speed -= self.accel * 5
                if self.speed < self.min_speed:
                    self.speed = self.min_speed
            elif self.color[1] == "green" or self.color[1] == "white":
                if self.color[1] == "green":
                    self.side_weight[1] += 1
                else:
                    self.side_weight[1] -= 1
                self.left_turns = 0
                self.turning_angle = self.soft_turn
                self.speed -= self.accel * 5
                if self.speed < self.min_speed:
                    self.speed = self.min_speed
    
    def _obstacle_check(self):
        self.detectObstacle()
        if self.objectDistance < self.objectAvoidThreashold:
            self._handle_obstacle()
        return
    
    def _handle_obstacle(self):
        left_lane = -1 if self.side_weight[0] > self.side_weight[1] else 1
        detecting_sensor = 1 if left_lane == 1 else 0
        switching = True
        angle = 70
        white_detected = False
        startframe = self.frame
        endframe = 0
        crossed_line = False
        time_diff = 0
        # self.skip_turn_logic = True
        self._switch_lane()
        # while switching:
        #     print("Switching lanes ")
        #     self.robot.drive(self.speed,(angle * left_lane))
        #     self._getClosestColor()
        #     self.frame += 1
        #     if self.frame % 10 == 0:
        #         self._print_data()
            
        #     if self.color[detecting_sensor-1] == "white"and not crossed_line: # sensor - 1 is the other sensor. at 0, 0 -1 = -1, which is the right sensor  at 1, 1 - 1 = 0, which is the left sensor
        #         self.robot.drive(self.speed,(angle * left_lane))
        #         wait((20/angle)*1000)
        #         white_detected = True
                
            
        #     if self.color[detecting_sensor] == "white"and not crossed_line:
        #         white_detected = True
                
        #     if white_detected and not crossed_line:
        #         time_diff = self.frame - startframe
        #         startframe = self.frame
        #         left_lane = -left_lane
        #         crossed_line = True
        #         endframe = self.frame + time_diff
        #         self.side_weight = self.side_weight[::-1]
        #         self.robot.drive(self.speed,0)
        #         wait(600)
        #         print("startframe: ", startframe, "endframe: ", endframe, "time_diff: ", time_diff, "frame: ", self.frame)
        #         white_detected = False
            
        #     if crossed_line and self.frame > endframe:
        #         self.robot.stop()
        #         switching = False
        #         self.skip_turn_logic = False
        #         wait(3000)
                
            
                
                
        print("Switched Lanes")
        self.robot.drive(100, (angle+30 * -left_lane))
        self.turning_angle = 0
        
        
        return
    
    
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
        elif self.skip_turn_logic:
            return
        elif light:
            self._handle_light() 
        else:
            self.speed += self.accel
    
    def detectObstacleForParking(self):
        distance = self.infrared.distance()

        
        if distance > self.obstacle_distance:
            self.besides_obstacle = False
            self.obstacle_count += 1
            print("Obstacle passed")
        
        self.obstacle_distance = distance    

        if distance <= self.obstacle_higher_threshold and distance > self.obstacle_lower_threshold:
            print("Obstacle detected")
            self.besides_obstacle = True
            wait(1000)  

            if self.obstacle_count == 2:
                # parking begins
                self.parallel_park()
                self.continue_driving = False  # Exit the loop after parking

    def _checkForParking(self):
        if self.infrared.distance() < 50:
            self.robot.drive(100,0)
            wait(1000)
            self.robot.turn(-90)
            parked = False
            while self.ultrasonic.distance() > 100:
                self.robot.drive(60,0)
                wait(100)
            self.hub.speaker.beep()
            self.robot.stop()
            wait(10000000)
    
            
    
    def parallel_park(self):
        #changed
        forward_distance = self.infrared.distance() / 2  
        self.motorL.run_angle(10, forward_distance, Stop.BRAKE, False)
        self.motorR.run_angle(10, forward_distance, Stop.BRAKE, True)
        #backwards
        self.motorL.run_angle(-20, 230, Stop.BRAKE, False) 
        self.motorR.run_angle(Stop.Break)  
        # 
        self.motorL.run_angle(-20, 360, Stop.BRAKE, False)
        self.motorR.run_angle(-20, 360, Stop.BRAKE, True)
        # Straighten the robot
        self.motorL.run_angle(20, 115, Stop.BRAKE, False)
        self.motorR.run_angle(-20, 115, Stop.BRAKE, True)
    
    def server(self):
        self.server = BluetoothMailboxServer()
        mbox = TextMailbox('mbox', self.server)
        self.server.wait_for_connection()
        mbox.wait()
        print(mbox.read())
        mbox.send("Karlos")
    
    def client(self):
        self.client = BluetoothMailboxClient()
        mbox = TextMailbox('mbox', client)
        SERVER = "ev3dev"
        print("setting up connection")
        client.connect(SERVER)
        print("connected")
        
        mbox.send("hello to you")
        mbox.wait()
        print(mbox.read())
    
    def drive(self):
        
        
        timer = 0        
        while timer < 1500:
            self.screen.clear()
            timer = self._print_menu(timer)
        self.hub.speaker.beep()
        
        while True:
            self._getClosestColor()
            self._process_color()
            # self.detectObstacleForParking()
            if self.changed_lanes:
                self._checkForParking()
            else:
                print(self.objectDistance)
                self._obstacle_check()
            self.frame += 1
            if self.frame % 10 == 0:
                self._print_data()
                self._print_data_console()
            self.speed = self.max_speed if self.speed > self.max_speed else self.speed
            if self.side_weight[0] < self.side_weight[1]:
                self.turning_angle -= 10 if not self.changed_lanes else 10
            
            self.robot.drive(self.speed, self.turning_angle)
            self.turning_angle = 0
 


car = Vehicle(sensorL, sensorR, robot,motorL, motorR, ev3, ultrasonic, ir)
# car.calibrate()
car.loadCalibratedData()
# car.drive()
car.server()
