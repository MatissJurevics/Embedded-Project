#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile, Font
from pybricks.messaging import BluetoothMailboxServer, TextMailbox, NumericMailbox, BluetoothMailboxClient
import json
import http
from utils.buffer import Buffer
import os




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
        self.buffer = Buffer(30)        # Values used in the driving stage
        # -------------------------------
        # Add values here to adjust the driving behavior between functions
        self.continue_driving = True
        self.min_speed = 100 # Minimum speed of the robot (right its just the speed)
        self.turning_angle = 0 # The turning angle of the robot on any given frame
        self.frame = 0 # The frame number
        self.speed = self.min_speed # The speed of the robot
        self.blue_frames = 0 # The number of frames the robot has been on blue
        self.side_weight = [0,0] # The weight of the sides of the robot (left, right) 
        self.soft_turn = 50 # The turning angle for a soft turn 
        self.sharp_turn = 70 # The turning angle for a sharp turn
        self.left_turns = 0
        self.drift = -4 # The drift of the robot (done to prevent the robot from going between the white lines
        self.objectDistance = 0 # The distance of the object in front of the robot
        self.color = None
        self.changed_lanes = False 
        self.at_crossing = False
        self.obstacle_distance = 0
        self.obstacle_count = 0
        self.obstacle_higher_threshold = 50.8
        self.obstacle_lower_threshold = 10.0
        self.objectAvoidThreashold = 250
        self.skip_turn_logic = False
        self.onBlue = False
        self.onYellow = False
        self.onLaneSwitch = False
        self.eventBus = []
        self.latestBufLen = 0
        self.lanes = 0
        self.blue = 0
        self.yellow = 0
        self.park = 0
        self.follower_ignore_white = None
        
        
    
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
        wait(500)
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
        self.buffer.append(dist)
        # print(self.buffer.buffer)
        self.objectDistance = dist
        
    def _print_data(self):
        lane = "left lane" if  self.side_weight[0] > self.side_weight[1] else "right lane"
        blue = "blue" if self.color[0] == "blue" or self.color[1] == "blue" else "not blue"
        self.screen.clear()
        self.screen.draw_text(85, 5, "Convoy Mode" if self.convoy else "Independent Mode", text_color=Color.WHITE, background_color=Color.BLACK)
        if self.convoy:
            self.screen.draw_text(85, 20, "Follow Mode" if self.follow else "Leader Mode", text_color=Color.WHITE, background_color=Color.BLACK)
        self.screen.draw_text(0, 30, str("L:{}  R:{} CL:{}  CR:{}".format(self.color[0], self.color[1], self.color[2][0], self.color[2][1])))
        self.screen.draw_text(0, 50, str(self.speed))
        self.screen.draw_text(0, 70, str(self.turning_angle))
        self.screen.draw_text(0, 90, "{lane}: {side_weight}".format(lane=lane, side_weight=self.side_weight))
        self.screen.draw_text(0, 110, str(self.objectDistance))
        
    def _print_data_console(self):
        return
        # print("L:{}  R:{} CL:{}  CR:{}".format(self.color[0], self.color[1], self.color[2][0], self.color[2][1]), end="\r", flush=True)
        # print("Speed: " ,self.speed, end="\r", flush=True)
        # print("Turning angle: ", self.turning_angle, end="\r", flush=True)
        # print("{lane}: {side_weight}".format(lane=lane, side_weight=self.side_weight), end="\r", flush=True)
        # print(self.objectDistance)
    
    def _print_menu(self, timer):
        """
        `_print_menu` is a function that prints the menu to the screen and
        allows the user to select an option. 
        """
        selected = 0
        while True:
            self.screen.clear()
            buttons = self.hub.buttons.pressed()
            vals = ["Calibrate", "Drive (independant)", "Drive (leader)", "Drive (follower)", "Doom"]
            for idx, val in enumerate(vals): 
                if idx == selected:
                    self.screen.draw_text(0, 10 + (idx * 20), "> " + val)
                else:
                    self.screen.draw_text(0, 10 + (idx * 20), val)
            if buttons:
                if buttons[0] == Button.UP:
                    selected -= 1
                    print("up")
                elif Button.DOWN in buttons:
                    selected += 1
                if selected < 0:
                    selected = len(vals) - 1
                elif selected >= len(vals):
                    selected = 0
            
            if Button.CENTER in buttons:
                if selected == 0:
                    self.calibrate()
                elif selected == 1:
                    self.convoy = False
                    self.follow = False
                    self.drive()
                elif selected == 2:
                    self.convoy = True
                    self.follow = False
                    self.drive()
                elif selected == 3:
                    self.convoy = True
                    self.follow = True
                    self.drive()
                elif selected == 4:
                    try:
                        os.system("/home/robot/testing/ev3doom/ev3doom -iwad /home/robot/testing/ev3doom/doom1.wad")
                    except:
                        print("Cannot open Doom")
            wait(200)
        return timer
    
    def _send_data(self, msg):
        """
        Send data to the client
        """
        if self.convoy:
            self.mbox.send(msg)
            acked = False
            while not acked:
                # regex for numbers
                recData = self.mbox.read()
                print("recData", recData)
                
                # test if the data is a number
                if recData.isdigit():
                    newval = int(recData)
                    if newval > self.latestBufLen:
                        acked = True
                        print("Acked")
                        break
                    else:
                        self.mbox.send(msg)
                self.mbox.send(msg)
                
                
            self.msg_sent = True
            
    def _receive_data(self):
        """
        Receive data from the server
        """
        if self.convoy:
            
            data = self.mbox.read()
            print("data", data)
            try:
                if data == self.eventBus[-1]:
                    return "none"
                else:
                    self.eventBus.append(data)
                    print(self.eventBus)
                    for i in range(5):
                        self.mbox.send(len(self.eventBus))
                        wait(10)
                    print(len(self.eventBus))
                    
                    return data
            except:
                self.eventBus.append(data)
                return data
        return
        
    
    def _handle_blue(self):
        """
        stop the robot for 3 seconds and then drive forward
        """
        self.blue += 1
        self.mbox_blue.send(self.blue)
        self.robot.stop()
        wait(3000)
        self.robot.drive(100,0)
        wait(300)
        self.blue_frames = 0
    
    def _handle_yellow(self):
        """
        move the robot slowly for 2.3 seconds
        """
        self.yellow += 1
        self.mbox_yellow.send(self.yellow)
        self.at_crossing = True
        self.robot.drive(75,0)
        wait(2300)
        self.at_crossing = False
        
    def _handle_red(self):
        """
        handle logic for when the robot drives over red
        """
        if self.changed_lanes:
            print("changed lanes")
            self.hub.speaker.beep()
            self.mbox_park.send('1')
            # self.robot.stop()
            # self.hub.speaker.beep()
            # exit()
        else:
            self.changed_lanes = True # called changed lanes but refers to crossing red
            self.park += 10
            # self._switch_lane()          
    
    def _switch_lane(self):
        """
        Function for lane switching
        """
        
        if not self.follow and self.convoy:
            self.hub.speaker.beep()
            self.lanes += 1
            self.mbox_lanes.send(self.lanes)
            
        left_lane = (self.side_weight[0] > self.side_weight[1]) # True if left lane is heavier
        rotate = -90 if left_lane else 90
        print(left_lane, rotate)
        self.robot.turn(rotate)
        wait(100)
        both_green = self.color[0] == "green" and self.color[1] == "green"
        while not both_green:
            self._getClosestColor()
            both_green = self.color[0] == "green" and self.color[1] == "green"
            self.robot.drive(100,0)
            wait(100)
        self.robot.turn(-rotate)
        if self.follow:
            wait(1500)
        self.side_weight.reverse()
        
    
    def _handle_light(self):
        """
        handle the logic for when the robot drives over a
        light color (white or green)
        """
        if self.convoy and not self.follow:
            self._send_data("none")
        if not self.at_crossing:
            if self.color[0] == "green" or self.color[0] == "white":
                if self.color[0] == "green":
                    self.side_weight[0] += 1
                    self.left_turns += 1
                else:
                    self.side_weight[0] -= 1
                self.turning_angle = -self.soft_turn if self.side_weight[0] > self.side_weight[1] else -self.sharp_turn
                self.turning_angle -= 40 if self.left_turns > 5 else 0

            elif self.color[1] == "green" or self.color[1] == "white":
                if self.color[1] == "green":
                    self.side_weight[1] += 1
                else:
                    self.side_weight[1] -= 1
                self.left_turns = 0
                self.turning_angle = self.soft_turn
    
    def _obstacle_check(self):
        self.detectObstacle()
        if self.objectDistance < self.objectAvoidThreashold:
            self._handle_obstacle()
        return
    
    def _handle_obstacle(self):
        """
        Handle the obstacle avoidance
        """
        left_lane = -1 if self.side_weight[0] > self.side_weight[1] else 1
        detecting_sensor = 1 if left_lane == 1 else 0
        switching = True
        angle = 70
        white_detected = False
        startframe = self.frame
        endframe = 0
        crossed_line = False
        time_diff = 0
        self._switch_lane()   
        print("Switched Lanes")
        self.robot.drive(100, (angle+30 * -left_lane))
        self.turning_angle = 0
        return
    
    
    def _process_color(self):
        """
        Process the color of the robot
        """
        light = (self.color[0] == "white" or self.color[0] == "green" or self.color[1] == "white" or self.color[1] == "green")
        red = (self.color[0] == "red" and self.color[1] == "red")
        yellow = (self.color[0] == "yellow" or self.color[1] == "yellow")
        blue = (self.color[0] == "blue" and self.color[1] == "blue")
        
        if yellow and self.follow:
            if not self.follower_ignore_white:
                self.follower_ignore_white = self.frame
            if self.frame - self.follower_ignore_white < 100:
                if self.color[1] == "white":
                    self.color[1] = "black"
                if self.color[0] == "white":
                    self.color[0] = "black"
                    
        if not blue:
            self.blue_frames = 0
        if red and not self.follow:
            
            self._handle_red()
        elif yellow and not self.follow:
            self._handle_yellow()
        
        elif blue and not self.follow:
            self._handle_blue()
        # elif self.skip_turn_logic:
        #     return
        elif light:
            self._handle_light()
        
            
        
    
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
            self.mbox_lanes.send(self.lanes)
            self.robot.drive(100,0)
            wait(1000)
            self.robot.turn(-90) #backwards parking
            parked = False
            while self.ultrasonic.distance() > 100:
                self.robot.drive(-60,0)
                wait(100)
            self.hub.speaker.beep()
            self.robot.stop()
            wait(10000000)
    
   
    
    def leader(self):
        """
        Set up the server for the robot
        """
        self.server = BluetoothMailboxServer()
        self.mbox = TextMailbox('mbox', self.server)
        self.mbox_lanes = TextMailbox("lanes", self.server)
        self.mbox_blue = TextMailbox("blue", self.server)
        self.mbox_yellow = TextMailbox("yellow", self.server)
        self.mbox_park = TextMailbox("park", self.server)
        self.server.wait_for_connection()
        
        self.mbox.wait()
        self.mbox_lanes.wait()
        self.mbox_blue.wait()
        self.mbox_yellow.wait()
        self.mbox_park.wait()
        print(self.mbox.read(), self.mbox_lanes.read(), self.mbox_blue.read(), self.mbox_yellow.read(), self.mbox_park.read())
        self.mbox.send("First message sent")
        self.mbox_lanes.send('0')
        self.mbox_blue.send('0')
        self.mbox_yellow.send('0')
        self.mbox_park.send('0')
    
    def follower(self):
        """
        Set up the client for the robot and connect it to the server
        """
        self.client = BluetoothMailboxClient()
        self.mbox = TextMailbox('mbox', self.client)
        self.mbox_lanes = TextMailbox("lanes", self.client)
        self.mbox_blue = TextMailbox("blue", self.client)
        self.mbox_yellow = TextMailbox("yellow", self.client)
        self.mbox_park = TextMailbox("park", self.client)
        SERVER = "ev3dev"
        print("setting up connection")
        self.client.connect(SERVER)
        print("connected")
        self.mbox.send("First message sent")
        self.mbox_lanes.send('0')
        self.mbox_blue.send('0')
        self.mbox_yellow.send('0')
        self.mbox_park.send('0')
        print("first message sent, waiting...")
        self.mbox.wait()
        self.mbox_lanes.wait()
        self.mbox_blue.wait()
        self.mbox_yellow.wait()
        self.mbox_park.wait()
        print(self.mbox.read(), self.mbox_lanes.read(), self.mbox_blue.read(), self.mbox_yellow.read(), self.mbox_park.read())
        print("first message received")
    
    def _handle_sync_data(self):
        if self.follow:
            data = self._receive_data()
            
            if int(self.mbox_blue.read()) > 0 and self.blue < 1000:
                self.blue += 100000
                print("Stopping: Doing Blue Logic")
                if not self.onBlue:
                    self.onBlue = True
                    self._handle_blue()

                    
            elif int(self.mbox_yellow.read()) > 0 and self.yellow < 1000:
                self.yellow += 100000
                print("Doing Yellow Logic")
                if not self.onYellow:
                    self.onYellow = True
                    self._handle_yellow()
            # print("lanes", self.lanes, self.mbox_lanes.read())
            
            elif int(self.mbox_lanes.read()) > self.lanes:
                self.hub.speaker.beep()
                print("Lane Switching ")
                if not self.onLaneSwitch:
                    self.onLaneSwitch = True
                    self._switch_lane()
                    self.onLaneSwitch = False
                    print('lanes', self.lanes, self.mbox_lanes.read())
                    self.lanes = int(self.mbox_lanes.read())
            
            elif int(self.mbox_park.read()) > 0 and self.park < 1000:
                self.park += 100000
                print("Parking")
            
                

        return
    
    def _start_parking(self):
        print("Starting reverse parking...")
        self.detectObstacleforParking()
    
    
    def run(self):
        """
        The main function that opens the menu
        """
        timer = 0        
        while timer < 1500:
            self.screen.clear()
            timer = self._print_menu(timer)        

    def drive(self):
        """
        The main driving function
        """
        self.loadCalibratedData()
        self.hub.speaker.beep()
        if self.convoy:
            if self.follow:
                self.follower()
            else:
                self.leader()
        # ---------------------------------------------
        

        print("blue", self.blue, "yellow", self.yellow, "lanes", self.lanes)
        
        while True:
            self.frame += 1
            print("before sync")
            if self.convoy:        
                self._handle_sync_data()
            self._getClosestColor() # Get the closest color
            # self.detectObstacleForParking()
            self._process_color() # Process the color
            
            try:
                print(int(self.park))
                if int(self.park) > 0:
                    print("parking")
                    if self.follow:
                        self._checkForParking() # Check for parking if the robot passes red
                    elif self.changed_lanes:
                        self.robot.drive(100,0)
                        wait(1000)
                        self.robot.turn(-90) #backwards parking
                        parked = False
                        while self.ultrasonic.distance() > 100:
                            self.robot.drive(-60,0)
                            wait(100)
                        self.hub.speaker.beep()
                        self.robot.stop()
                        wait(10000000)
            except:
                pass
                        
            if not self.follow:    
                self._obstacle_check()
                
            if self.frame % 10 == 0: # Print data every 10 frames
                self._print_data()
                self._print_data_console()
            if self.follow:
                self.detectObstacle()
                
                avgDist = 0
                for val in self.buffer.buffer:
                    if val < 2000:
                        avgDist += val/len(self.buffer.buffer)
                # print("buffer", self.buffer.buffer)
                # print(avgDist)
                diff = 200 - avgDist
                # print("dif", diff)
                if (diff/20)**2 > 400:
                    print("diff", diff/20)
                self.speed = self.min_speed - (diff/20)
                # print("speed", self.speed)
            self.robot.drive(self.speed, self.turning_angle)
            self.turning_angle = 0
    
  
    


 



car = Vehicle(sensorL, sensorR, robot,motorL, motorR, ev3, ultrasonic, ir)
# car.calibrate()
car.run()
# car.server()
