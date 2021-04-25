import random
import easygopigo3 as easy
import threading
import time
from time import sleep
import math
from math import pi
import easysensors

        
class GoPiGo3WithKeyboard(object):

    KEY_DESCRIPTION = 0
    KEY_FUNC_SUFFIX = 1

    servo1_position = 0
    servo2_position = 0
    EYES_ON = False
    Color_Drive = (0, 100, 0)
    Color_Pause = (100, 60, 0)
    Color_Stuck = (100, 0, 0)
    Color_Manuever = (0, 0, 100)
    Color_Manuever_Stuck = (130, 0, 100)
    Color_Done = (0, 40, 40)
    
    ###------- MAIN MENU -------###
    def __init__(self):
        self.gopigo3 = easy.EasyGoPiGo3()
        self.servo1 = self.gopigo3.init_servo("SERVO1")
        
        self.keybindings = {
            "w" : ["Drive the GoPiGo along the delivery route", "delivery"],
            "1" : ["BRONZE Tier Challenge Function", "bronze"],
            "3" : ["GOLD Tier Challenge Function", "gold"],
            "5" : ["Manuever: test sensor functionality", "manuever_test"],
            "<SPACE>" : ["Stop the GoPiGo3 from moving", "stop"],

            "<LEFT>" : ["Turn servo completely to 0 degrees", "servo_total_left"],
            "<UP>" : ["Turn servo to 90 degrees (centered)", "servo_total_center"],
            "<RIGHT>" : ["Turn servo completely to 180 degrees", "servo_total_right"],
            "<DOWN>" : ["Take a distance sensor reading", "test_sensor"],
            

            "<ESC>" : ["Exit", "exit"],
        }
        self.order_of_keys = ["w", "1", "3", "5", "<SPACE>", "<LEFT>", "<UP>", "<RIGHT>", "<DOWN>", "<ESC>"]
  

    ###------- BUILT-IN FUNCTIONS (don't change any of these) -------###
    def executeKeyboardJob(self, argument):
        method_prefix = "_gopigo3_command_"
        try:
            method_suffix = str(self.keybindings[argument][self.KEY_FUNC_SUFFIX])
        except KeyError:
            method_suffix = ""
        method_name = method_prefix + method_suffix

        method = getattr(self, method_name, lambda : "nothing")

        return method()

    def drawLogo(self):
        """
        Draws the name of the GoPiGo3.
        """
        print("__________                     __________                             ")
        print("\______   \_____ _______   ____\______   \ ____   ____   ____   ______")
        print(" |    |  _/\__  \\_  __ \_/ __ \|    |  _//  _ \ /    \_/ __ \ /  ___/")
        print(" |    |   \ / __ \|  | \/\  ___/|    |   (  <_> )   |  \  ___/ \___ \ ")
        print(" |______  /(____  /__|    \___  >______  /\____/|___|  /\___  >____  >")
        print("        \/      \/            \/       \/            \/     \/     \/ ")

    def drawDescription(self):
        """
        Prints details related on how to operate the GoPiGo3.
        """
        print("\nPress the following keys to run the features of the GoPiGo3.")
        print("To move the motors, make sure you have a fresh set of batteries powering the GoPiGo3.\n")

    def drawMenu(self):
        """
        Prints all the key-bindings between the keys and the GoPiGo3's commands on the screen.
        """
        try:
            for key in self.order_of_keys:
                print("\r[key {:8}] :  {}".format(key, self.keybindings[key][self.KEY_DESCRIPTION]))
        except KeyError:
            print("Error: Keys found GoPiGo3WithKeyboard.order_of_keys don't match with those in GoPiGo3WithKeyboard.keybindings.")


    ###------- DISTANCE SENSOR FUNCTIONS (exactly the same as in the simplified file you saw previously) -------###
    # modified to return the sensor reading (DRY!)
    def _gopigo3_command_test_sensor(self):
        # initialize the sensor then print the current reading
        my_distance_sensor = self.gopigo3.init_distance_sensor()
        distanceDetected = my_distance_sensor.read_mm()
        #print("Distance Sensor Reading: {} mm ".format(distanceDetected))
        return distanceDetected

    def _gopigo3_command_read_respond_sensor(self):
	# initialize the sensor then print the current reading
        my_distance_sensor = self.gopigo3.init_distance_sensor()
        print("Distance Sensor Reading: {} mm ".format(my_distance_sensor.read_mm()))
        
        if (my_distance_sensor.read_mm() < 150):
            self.gopigo3.set_speed(1) #NOTE: Setting speed to '0' causes the robot to move at max speed backward then forward ???
            print("obstacle detected!")
        elif (my_distance_sensor.read_mm() < 750):
            self.gopigo3.set_speed(150)
            print("obstacle approaching...")
        else:
            self.gopigo3.set_speed(300)
            print("coast is clear!")
    
    # modified to also call stop_motors
    def _gopigo3_command_stop(self):
        self.gopigo3.stop()
        self.stop_motors()
        self.lightOff()
        
    
    ###------- SERVO FUNCTIONS (copied over from ServoControl example) -------###
    def _gopigo3_command_servo_total_left(self):
        self.servo1_position = 0
        self.servo1.rotate_servo(self.servo1_position)
        return "complete_turn_servo1"

    def _gopigo3_command_servo_total_center(self):
        self.servo1_position = 90
        self.servo1.rotate_servo(self.servo1_position)
        return "complete_turn_servo1"
    
    def _gopigo3_command_servo_total_right(self):
        self.servo1_position = 180
        self.servo1.rotate_servo(self.servo1_position)
        return "complete_turn_servo1"
   

    def _gopigo3_command_exit(self):
        return "exit"

    ###------- CUSTOMIZED FUNCTIONS (modified from the drive_in function in easygopigo3.py) -------###
    # convert inches to mm
    def dvc_in_to_encoder(self, inches):
        wheel_circumference = math.pi * 66.5
        dist_cm = inches * 2.54
        dist_mm = dist_cm * 10
        # the number of degrees each wheel needs to turn
        return ((dist_mm / wheel_circumference) * 360)
    
    #converts mm encoder values into exact inches
    def dvc_encoder_to_in(self, encoderUnits):
        wheel_circumference = math.pi * 66.5
        distMM = (encoderUnits / 360) * wheel_circumference
        distCM = distMM / 10
        distIN = distCM / 2.54
        return distIN
    
    #DRY principle! used for keeping track of driving state through color
    def lightOn(self):
        self.gopigo3.open_eyes()
        self.EYES_ON = True
    def lightOff(self):
        self.gopigo3.close_eyes()
        self.EYES_ON = False
    def lightFlash(self):
        if (self.EYES_ON):
            self.gopigo3.close_eyes()
            self.EYES_ON = False
        else:
            self.gopigo3.open_eyes()
            self.EYES_ON = True
    def lightColor(self, color):
        self.gopigo3.set_eye_color(color)
        self.lightOn()
        
    #equivalent to writing gopigo3.stop() but with preserving motor positions
    def stop_motors(self):
        CurrentPositionLeft = self.gopigo3.get_motor_encoder(self.gopigo3.MOTOR_LEFT)
        CurrentPositionRight = self.gopigo3.get_motor_encoder(self.gopigo3.MOTOR_RIGHT)

        self.gopigo3.set_motor_position(self.gopigo3.MOTOR_LEFT, CurrentPositionLeft)
        self.gopigo3.set_motor_position(self.gopigo3.MOTOR_RIGHT, CurrentPositionRight)
        
    def start_motors(self, TargetPosL, TargetPosR):
        self.gopigo3.set_motor_position(self.gopigo3.MOTOR_LEFT, TargetPosL)
        self.gopigo3.set_motor_position(self.gopigo3.MOTOR_RIGHT, TargetPosR)
    
    #calculates the different between the current encoding and target encoding in MM
    def encoder_distance_remaining(self, leftTarget, rightTarget):
        distanceToTravelL = leftTarget - self.gopigo3.get_motor_encoder(self.gopigo3.MOTOR_LEFT)
        distanceToTravelR = rightTarget - self.gopigo3.get_motor_encoder(self.gopigo3.MOTOR_RIGHT)
        distanceToTravel = (distanceToTravelL + distanceToTravelR) / 2
        return distanceToTravel
    
    #will drive backwards with no sensor. simply uses motor tracking
    def dvc_drive_in_backwards(self, dist):    
        self.lightColor(self.Color_Drive)
        dist = -dist
        # convert inches to mm
        WheelTurnDegrees = self.dvc_in_to_encoder(dist)

        # get the starting position of each motor
        StartPositionLeft = self.gopigo3.get_motor_encoder(self.gopigo3.MOTOR_LEFT)
        StartPositionRight = self.gopigo3.get_motor_encoder(self.gopigo3.MOTOR_RIGHT)
        TargetPosL = (StartPositionLeft + WheelTurnDegrees)
        TargetPosR = (StartPositionRight + WheelTurnDegrees)
        
        # add the degrees it must turn to the starting position
        self.start_motors(TargetPosL, TargetPosR)
        
        distanceToTravel = self.encoder_distance_remaining(TargetPosL, TargetPosR)
        # move backward, checking every 0.1 seconds to see if the target has been reached
        while (abs(distanceToTravel) > 10):
            distanceToTravel = self.encoder_distance_remaining(TargetPosL, TargetPosR)
            print("backwards: {} mm left to travel ".format(round(distanceToTravel)))
            sleep(0.2)

    #used to drive forward with basic stop+start functionality. will continue until finished
    def dvc_drive_in(self, dist, objectDetecting = True):      
        # convert inches to mm
        WheelTurnDegrees = self.dvc_in_to_encoder(dist)
        self.lightColor(self.Color_Manuever)

        # get the starting position of each motor
        StartPositionLeft = self.gopigo3.get_motor_encoder(self.gopigo3.MOTOR_LEFT)
        StartPositionRight = self.gopigo3.get_motor_encoder(self.gopigo3.MOTOR_RIGHT)
        TargetPosL = (StartPositionLeft + WheelTurnDegrees)
        TargetPosR = (StartPositionRight + WheelTurnDegrees)
        
        # add the degrees it must turn to the starting position
        self.start_motors(TargetPosL, TargetPosR)
        
        distanceToTravel = self.encoder_distance_remaining(TargetPosL, TargetPosR)
        # move forward, checking every 0.1 seconds to see if the target has been reached
        while (abs(distanceToTravel) > 10):

            #each time we travel 0.1 inches we check the area in front of us
            distanceAhead = self._gopigo3_command_test_sensor()
            distanceToTravel = self.encoder_distance_remaining(TargetPosL, TargetPosR)
            print("normal: {} mm left to travel ".format(round(distanceToTravel)))
            
            print(abs(distanceToTravel) > 10)
            
            if (distanceAhead < 80):
                self.stop_motors()
                self.lightColor(self.Color_Manuever_Stuck)
            else:
                self.start_motors(TargetPosL, TargetPosR)
                self.lightColor(self.Color_Manuever)

            time.sleep(0.05)
    
    #will drive forward and return a value when hitting a wall. Returns any distance not covered
    def dvc_drive_in_breakable(self, dist):       
        # convert inches to mm
        WheelTurnDegrees = self.dvc_in_to_encoder(dist)

        # get the starting position of each motor
        StartPositionLeft = self.gopigo3.get_motor_encoder(self.gopigo3.MOTOR_LEFT)
        StartPositionRight = self.gopigo3.get_motor_encoder(self.gopigo3.MOTOR_RIGHT)
        TargetPosL = (StartPositionLeft + WheelTurnDegrees)
        TargetPosR = (StartPositionRight + WheelTurnDegrees)
        
        # set the motor target to start driving towards
        self.start_motors(TargetPosL, TargetPosR)
        
        distanceToTravel = self.encoder_distance_remaining(TargetPosL, TargetPosR)
        # move forward, checking every 0.05 seconds to see if the target has been reached
        while (abs(distanceToTravel) > 10):

            #each time we travel 0.1 inches we check the area in front of us
            distanceAhead = self._gopigo3_command_test_sensor()
            distanceToTravel = self.encoder_distance_remaining(TargetPosL, TargetPosR)
            
            print("breakable: {} mm left to travel ".format(round(distanceToTravel)))

            if (distanceAhead < 180):
                print("Pausing current path, obstruction at {} mm".format(self._gopigo3_command_test_sensor()))
                self.stop_motors()
                
                #tell the rerouter that we still have inches left to travel
                return self.dvc_encoder_to_in(distanceToTravel)
            
            time.sleep(0.05)
        #tell the rerouter that we have reached our target
        return 0
    
    def dvc_drive_rerouting(self, inches, turn90):
        distance = inches
        #how long the robot will wait for the obstruction to dissapear
        maxWait = 5
        manueverLength = 15
        
        #while we arent at our destination
        while (abs(distance) > 0.05):
            self.lightColor(self.Color_Drive)
            
            # drive_in_breakable will return 0 if the target is reached, and a larger number if 
            distance = self.dvc_drive_in_breakable(distance)
            
            self.lightColor(self.Color_Pause)
            
            curWait = 0
            
            #as long as we have distance to cover and are blocked, we gotta start waiting
            while (abs(distance) > 0.05 and self._gopigo3_command_test_sensor() < 200):
                
                #it is possible for us to manuever, so lets start the countdown
                if (abs(distance) > manueverLength):
                    if (maxWait - curWait >= 0):
                        self.lightColor(self.Color_Pause)
                        print("waiting...")

                    sleep(0.1)
                    #update the countdown
                    curWait += 1

                    #only do the meanuever if there is free space between our current Pos and the target waypoint (10 in)
                    if (curWait >= maxWait):
                        # make sure to subtract the new 10 inches from the current Leg
                        groundCovered = self.doManuever(turn90)
                        distance -= groundCovered
                        #if 0 ground is covered, that means every path is blocked :(
                        if (groundCovered == 0):
                            self.lightColor(self.Color_Stuck)
                #there is no chance we will ever do a manuever here
                else:
                    self.lightColor(self.Color_Stuck)
        
    #search the area and either 1. go right, 2. go left, 3. return if either there is NO paths or straight path
    def doManuever(self, turn90):
        
        length = 15
        
        #look to the right and see if there is roughly enough space to do a manuever
        self.servo1_position = 15
        self.servo1.rotate_servo(self.servo1_position)
        print("right side is {} mm away".format(self._gopigo3_command_test_sensor()))
        openR = self._gopigo3_command_test_sensor() > 400
        
        #look to the left and see if there is roughly enough space to do a manuever
        self.servo1_position = 165
        self.servo1.rotate_servo(self.servo1_position)
        openL = self._gopigo3_command_test_sensor() > 400
        
        self.servo1_position = 90
        self.servo1.rotate_servo(self.servo1_position)
        # look forward, and if the path has opened in the last couple seconds then quit the manuever attempt
        if (self._gopigo3_command_test_sensor() > 500):
            return 0
        
        print ("can go left: {}".format(openL))
        print ("can go right: {}".format(openR))
        
        #go around the obstruction in a rectangular path. 
        if (openR):
            self.gopigo3.turn_degrees(turn90)  
            self.dvc_drive_in(9)        
            self.gopigo3.turn_degrees(-turn90)  
            self.dvc_drive_in(length-1)        
            self.gopigo3.turn_degrees(-turn90)  
            self.dvc_drive_in(9)
            self.gopigo3.turn_degrees(turn90)
            return 10
        elif (openL):
            self.gopigo3.turn_degrees(-turn90)  
            self.dvc_drive_in(9)        
            self.gopigo3.turn_degrees(turn90)  
            self.dvc_drive_in(length-1)        
            self.gopigo3.turn_degrees(turn90)  
            self.dvc_drive_in(9)
            self.gopigo3.turn_degrees(-turn90)
            # we covered 10 inches of ground!
            return length
        else:
            #if we didnt do anything, return 0 since 0 is the total ground we covered
            return 0
    
    ###------- FINAL ROUTE FUNCTIONS -------###
    def _gopigo3_command_manuever_test(self):
        
        my_distance_sensor = self.gopigo3.init_distance_sensor()
        servo = self.gopigo3.init_servo()
        
        self.gopigo3.set_speed(300)
        turn90 = 84

        self.dvc_drive_rerouting(30, turn90)
        print("~~~~~~~~~ completed Leg")
        
        self.dvc_drive_in_backwards(30)
        print("~~~~~~~~~ completed Leg")
        
        self.lightOff()
            
        return "moving"

    def _gopigo3_command_bronze(self):
        print("~~running bronze tier object stopping")
        
        servo = self.gopigo3.init_servo()
        self.gopigo3.set_speed(300)
        turn90 = 84
        
        #waypoint 1 -> 2
        self.gopigo3.drive_inches(49)
        self.gopigo3.turn_degrees(turn90)  
        
        #waypoint 2 -> 3
        self.gopigo3.drive_inches(64)
        self.gopigo3.turn_degrees(turn90-2)
        
        #waypoint 3 -> 4
        self.gopigo3.drive_inches(40)
        
        self.gopigo3.turn_degrees(turn90*2)
        self.gopigo3.drive_inches(-36)
        
        #waypoint 4 -> 5
        self.gopigo3.set_speed(100)
        self.gopigo3.drive_inches(-48)
        self.gopigo3.set_speed(300)
        self.gopigo3.turn_degrees(-turn90+2)
                
        #waypoint 5 -> 6
        self.gopigo3.drive_inches(80)
        self.gopigo3.turn_degrees(-turn90)
        
        self.gopigo3.stop()
        
        return "moving"
    
    def _gopigo3_command_gold(self):
        print("~~running gold tier object avoidance")
        
        my_distance_sensor = self.gopigo3.init_distance_sensor()
        servo = self.gopigo3.init_servo()
        
        self.gopigo3.set_speed(300)
        turn90 = 84

        #self.dvc_drive_rerouting(10, turn90)
        #print("~~~~~~~~~ completed Leg")
        
        #self.dvc_drive_in_backwards(10)
        #print("~~~~~~~~~ completed Leg")
            
        #return "moving"
        
        
        #waypoint 1 -> 2
        self.dvc_drive_rerouting(49, turn90)
        self.gopigo3.turn_degrees(turn90)  
        print("~~~~~~~~~ reached Waypoint 2")
        
        #waypoint 2 -> 3
        self.dvc_drive_rerouting(64, turn90)
        self.gopigo3.turn_degrees(turn90-2)
        print("~~~~~~~~~ reached Waypoint 3")
        
        #waypoint 3 -> 4
        self.dvc_drive_rerouting(40, turn90)
        print("~~ halfway to waypoint 4...")
        
        self.gopigo3.turn_degrees(turn90*2)
        self.dvc_drive_in_backwards(36)
        print("~~~~~~~~~ reached Waypoint 4")
        
        #waypoint 4 -> 5
        self.gopigo3.set_speed(100)
        self.dvc_drive_in_backwards(54)
        self.gopigo3.set_speed(300)
        self.gopigo3.turn_degrees(-turn90+2)
        print("~~~~~~~~~ reached Waypoint 5")
                
        #waypoint 5 -> 6
        self.dvc_drive_rerouting(80, turn90)
        self.gopigo3.turn_degrees(-turn90)
        print("~~~~~~~~~ reached Waypoint 6")
        
        
        self.stop_motors()
        
        counter = 0
        self.lightColor(self.Color_Done)
        while (counter < 5):
            self.lightFlash()
            sleep(0.4)
            counter += 1
        self.lightOff()
        
        return "moving"
