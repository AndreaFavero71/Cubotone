#!/usr/bin/env python
# coding: utf-8


""" 
#############################################################################################################
#  Andrea Favero   rev02    10 September 2023
#
#  While dealing with Covid 19 pandemic on 2021, and turning 50 years old, I thought a Rubik's cube
#  solver robot to be a nice challenge, to learn computer vision and to improve my coding skills.
#
#  The below code works on a PC (presenting the cube faces manually to the webcam) and/or on a raspberry pi.
#  I've used the Kociemba solver (from: https://github.com/hkociemba/RubiksCube-TwophaseSolver)
#
#  The Rpi part integrates the controls for a robot I've develloped.
#  It shouldn't be too difficult to comment out the robot related parts, and just use the Rpi + camera;
#  This will allow to read the cube status via the Vision part, and to retrieve the cube solution.
#
#
#  Developped on:
#  --> W10 PC with Python ver: 3.8.12 [MSC v.1916 64 bit (AMD64)] and cv2 ver: 4.5.1
#  --> Raspberry pi 4B 2Gb (Linux-5.10.52-v7l+-armv7l-with-debian-10.10) with Python version: 3.7.3
#      and cv2 ver: 4.1.0; piCamera v1.3
#
#  Verified on:
#  --> W10 PC with Python ver: 3.9.7 |packaged by conda-forge| [MSC v.1916 64 bit (AMD64)] and cv2 ver: 4.5.1
#
#############################################################################################################
"""



################  setting argparser for robot remote usage, and other settings  #################
import argparse

# argument parser object creation
parser = argparse.ArgumentParser(description='Arguments for Cubotone')

# --debug argument is added to the parser
parser.add_argument("-d", "--debug", action='store_true',
                    help="Activates printout of settings, variables and info for debug purpose")

# --debug argument is added to the parser
parser.add_argument("-l", "--led", action='store_true',
                    help="Activates the top_cover led usage")

# --cv_wow argument is added to the parser
parser.add_argument("--cv_wow", action='store_true',
                    help="Activates the cv_wow (image analysis steps) on screen")

# --delay argument is added to the parser
parser.add_argument("--delay", type=int,
                    help="Delay after a cube face change before start detecting the facelets (PC use case)")

# --scramble argument is added to the parser
parser.add_argument("--scramble", action='store_true',
                    help="Ramdom scrambling of the cube before solving it")

# --picamera_test argument is added to the parser
parser.add_argument("--picamera_test", action='store_true',
                    help="Visualizes PiCamera images for test purpose")

# --no_motors argument is added to the parser
parser.add_argument("--no_motors", action='store_true',
                    help="Excludes servos and motor for test purpose")

# --no_motors argument is added to the parser
parser.add_argument("--size", type=int,
                    help="Cube size deviation, in tents of mm (+/-), vs reference cube used for settings")

args = parser.parse_args()   # argument parsed assignement
# ###############################################################################################




import cv2
import numpy as np
from copy import deepcopy
import math
import statistics
import time
import datetime as dt
import sys
import pathlib



def import_parameters():
    """ Function to import parameters from a json file, to make easier to list/document/change the variables
        that are expected to vary on each robot."""
    
    global sv, cubie, camera_width_res, camera_hight_res
    global zoom_x, zoom_y, zoom_w, zoom_h, kl, scale_perc, square_ratio, rhombus_ratio
    global delta_area_limit, sv_max_moves, sv_max_time, collage_w, marg_coef, cam_led_bright
    global detect_timeout, show_time, gap_w, gap_h

    
    # convenient choice for Andrea Favero, to upload the settings fitting my robot, via mac address check
    import os.path, pathlib, json                                 # libraries needed for the json, and parameter import
    from getmac import get_mac_address                            # library to get the device MAC ddress
        
    folder = pathlib.Path().resolve()                             # active folder (should be home/pi/cube)  
    eth_mac = get_mac_address()                                   # mac address is retrieved
    if eth_mac == 'e4:5f:01:0a:31:ce':                            # case the script is running on AF (Andrea Favero) robot
        fname = os.path.join(folder,'Cubotone_settings_AF.txt')   # AF robot settings (do not use these at the start)
    else:                                                         # case the script is not running on AF (Andrea Favero) robot
        fname = os.path.join(folder,'Cubotone_settings.txt')      # folder and file name for the settings, to be tuned
    
    if os.path.exists(fname):                                     # case the settings file exists
        with open(fname, "r") as f:                               # settings file is opened in reading mode
            settings = json.load(f)                               # json file is parsed to a local dict variable
            # NOTE: in case of git pull, the settings file will be overwritten, the backup file not
            
        if debug:                                                 # case debug variable is set true on __main_
            print('\nimporting settings from the text file:', fname)    # feedback is printed to the termina
            print('\nimported parameters: ')                      # feedback is printed to the terminal
            for parameter, setting in settings.items():           # iteration over the settings dict
                print(parameter,': ', setting)                    # feedback is printed to the terminal
            print()
        
        os.umask(0) # The default umask is 0o22 which turns off write permission of group and others
        backup_fname = os.path.join(folder,'Cubotone_settings_backup.txt')            # folder and file name for the settings backup
        with open(os.open(backup_fname, os.O_CREAT | os.O_WRONLY, 0o777), 'w') as f:  # settings_backup file is opened in writing mode
            if debug:                                             # case debug variable is set true on __main_
                print('\ncopy of settings parameter is saved as backup at: ', backup_fname, '\n')   # feedback is printed to the terminal
            f.write(json.dumps(settings, indent=0))               # content of the setting file is saved in another file, as backup
            # NOTE: in case of git pull, the settings file will be overwritten, the backup file not

        try:
            camera_width_res = int(settings['camera_width_res']) # Picamera resolution on width 
            camera_hight_res = int(settings['camera_hight_res']) # Picamera resolution on heigh
            zoom_x = float(settings['zoom_x'])                   # image crop on left, at PiCamera sensor level
            zoom_y = float(settings['zoom_y'])                   # image crop on top, at PiCamera sensor level
            zoom_w = float(settings['zoom_w'])                   # image crop width from x, at PiCamera sensor level
            zoom_h = float(settings['zoom_h'])                   # image crop height from y, at PiCamera sensor level
            kl = float(settings['kl'])                           # coff. for PiCamera stabilization acceptance
            scale_perc = int(settings['scale_perc'])             # image resizing value (over 100), for screen plotting
            square_ratio = float(settings['square_ratio'])       # acceptance threshold for square sides difference
            rhombus_ratio = float(settings['rhombus_ratio'])     # acceptance threshold for rhombus axes difference
            delta_area_limit = float(settings['delta_area_limit'])    # acceptance threshold for facelet area dev from median
            sv_max_moves = int(settings['sv_max_moves'])         # max moves requested to the Kociemba solver
            sv_max_time = float(settings['sv_max_time'])         # timeout requested to the Kociemba solver
            collage_w = int(settings['collage_w'])               # image width for unfolded cube collage
            marg_coef = float(settings['marg_coef'])             # cropping margin arounf faces for immage collage
            cam_led_bright = int(settings['cam_led_bright'])     # PWM level for the 3W led at Top_cover
            detect_timeout = int(settings['detect_timeout'])     # timeout for cube status detection
            show_time = int(settings['show_time'])               # showing time of the unfolded cube image collage
            gap_w = int(settings['gap_w'])                       # horizontal gap between cv2 windows when cv_wow
            gap_h = int(settings['gap_h'])                       # vertical gap between cv2 windows when cv_wow
              
        
        except:   # exception will be raised if json keys differs, or parameters cannot be converted to int/float
            print('error on converting the imported parameters to int or float')   # feedback is printed to the terminal                                  
            return False, ''                                     # return robot_init_status variable, that is False
    
    else:                                                        # case the settings file does not exists, or name differs
        print('could not find Cubotone_settings.txt')            # feedback is printed to the terminal                                  
        return False, ''                                         # return robot_init_status variable, that is False
    
    
    # importing Kociemba solver
    # this import takes some time to be uploaded
    # there are thre import attempts, that considers different solver installation methods
    try:                                                  # attempt
        print()
        import solver as sv                               # import Kociemba solver copied in /home/pi/cube
        import cubie as cubie                             # import cubie Kociemba solver part
        solver_found = True                               # boolean to track no exception on import the copied solver
        if debug:                                         # case debug variable is set true on __main__            
            print('\nfound Kociemba solver copied in active folder')   # feedback is printed to the terminal
    except:                                               # exception is raised if no library in folder or other issues
        solver_found = False                              # boolean to track exception on importing the copied solver
       
    if not solver_found:                                  # case the solver library is not in active folder
        import os.path, pathlib                           # library to check file presence            
        folder = pathlib.Path().resolve()                 # active folder (should be home/pi/cube)  
        fname = os.path.join(folder,'twophase','solver.py')   # active folder + twophase' + solver name
        if os.path.exists(fname):                         # case the solver exists in 'twophase' subfolder
            try:                                          # attempt
                print()
                import twophase.solver as sv              # import Kociemba solver copied in twophase sub-folder
                import twophase.cubie as cubie            # import cubie Kociemba solver part
                solver_found = True                       # boolean to track the  import the copied solver in sub-folder
                if debug:                                 # case debug variable is set true on __main__            
                    print('\nfound Kociemba solver copied in subfolder')  # feedback is printed to the terminal
            except:                                       # exception is raised if no library in folder or other issues
                pass                                      # pass, to keep solver_found = False as it was before
        
    if not solver_found:                                  # case the library is not in folder or twophase' sub-folder
        try:                                              # attempt
            print() 
            import twophase.solver as sv                  # import Kociemba solver installed in venv
            import twophase.cubie as cubie                # import cubie Kociemba solver part
            twophase_solver_found = True                  # boolean to track no exception on import the installed solver
            if debug:                                     # case debug variable is set true on __main__
                print('\nfound Kociemba solver installed')  # feedback is printed to the terminal
        except:                                           # exception is raised if no library in venv or other issues
            twophase_solver_found = False                 # boolean to track exception on importing the installed solver
    

    if not solver_found and not twophase_solver_found:    # case no one solver has been imported
        print('\nnot found Kociemba solver')              # feedback is printed to the terminal
        quit_func()                                       # script is quitted

    return True, settings 





def check_screen_presence(debug, printout=False):
    """ Checks if a display is connected, eventually via VNC."""
    import os
    if 'DISPLAY' in os.environ:                          # case the environment variable DISPLAY is set to 1 (there is a display)
        if debug and printout:                           # case debug variable is set true on __main__
            print('screen function is available')        # feedback is printed to the terminal
        return True                                      # function returns true, meaning there is a screen connected
    else:                                                # case the environment variable DISPLAY is set to 0 (there is NOT a display)
        if debug and printout:                           # case debug variable is set true on __main__
            print('display function is not availbale')   # feedback is printed to the terminal
        return False                                     # function returns false, meaning there is NOT a screen connected





def webcam():
    """Sets the camera and its resolution."""
    
    global camera
    
    if device == 'laptop':                                        # case the script is running on a PC/laptop (not the robot)
        camera = cv2.VideoCapture(0, cv2.CAP_DSHOW)               # laptop camera, when no USB webcam connected
        #     camera = cv2.VideoCapture(1, cv2.CAP_DSHOW)         # laptop, when USB webcam is connected
        camera_width_resolution = 640                             # laptop camera, width resolution setting
        camera_hight_resolution = 360                             # laptop camera, height resolution setting
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width_resolution)    # camera width resolution is set  
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_hight_resolution)   # camera height resolution is set
        width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))         # return the camera reading width 
        height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))       # return the camera reading higth 
        if side == 0:
            print(f'Camera resolution: {width} x {height}\n')
        time.sleep(0.5)
#         clear_terminal(device)
        return camera, width, height
    
    elif device == 'Rpi':                      # case the script is running at the robot
        s_mode = 7                             # sensor mode 7 means 4x4 binning
        camera = PiCamera(sensor_mode=s_mode)  # PiCamera is set as camera object
        rawCapture = PiRGBArray(camera)        # returns (uncoded) RGB array from the camera
        
        # necessary to use a large FOW (Field of view) as the camera is rather close to the cube
        width = camera_width_res               # PiCamera width resolution setting
        height =  camera_hight_res             # PiCamera height resolution setting
        camera.resolution = (width, height)    # camera's resolution is set
        
        binning = camera.sensor_mode           # PiCamera sensor_mode is checked
        
        # sensor_mode answer from the camera is interpreted
        if binning <=3:
            binning='none'
        elif binning == 4 or binning == 5:
            binning='2x2'
        elif binning > 5:
            binning='4x4'
            
        if side == 0:                     # only at cube reading start the camera's resolution in print at terminal
            print(f'\nPiCamera resolution: {width} x {height},   binning: {binning}\n')
            time.sleep(0.2)
#       clear_terminal(device)
        return camera, rawCapture, width, height






def robot_camera_warmup():
    """According to PiCamera documentation it is required a couple of seconds to warmup the camera.
    If the robot start-button is pressed right after placing the cube, the camera has to update the gains set before,
    that means from when tha camera was pointing the black cube support; this can take up to 20 secs to get all
    the gains stable (awb gains are rather slow to update).
    Much different is when the cube is on the cube support for few secs when the button is pressed.
    To properly cover all the possible situations, this fuction releases the camera warm-up phase only after
    aLL the gains are stable, meaning an absolute variation < 2% from the average of last 2 seconds."""
    
    if device=='laptop':              # case the script is running on a PC/laptop (not the robot)
        return ''                     # an empty string is returned, instead of the PiCamera parameters
    
    elif device=='Rpi' and robot_stop==False: # case the script is running at the robot, and no stopping requests
        print('\nPiCamera: waiting for AWB and Exposure gains to get stable')  # feedback is printed to the terminal
        
        if led_usage:                         # case the leds at Top_cover are activated
            time.sleep(0.3)                   # little delay to ensure servos have reached their target position and the innertia has fully dropped
            servo.led_freq(debug)             # set high frequency to PCA 9685 board, to prevent visible flickering from the led
            servo.led_cover(cam_led_bright)   # switch the cover led ON at value
        
        PiCamera_param = ''           # set the PiCamera_parame variable as an empty string
        camera.exposure_mode = 'auto' # set to auto exposure at the start, to adjust according to light conditions
        time.sleep(0.15)              # not found documentation if a delay is needed after this PiCamera setting 
        camera.awb_mode = 'auto'      # set to auto white balance at the start, to adjust according to light conditions
        time.sleep(0.15)              # not found documentation if a delay is needed after this PiCamera setting
        camera.shutter_speed = 0      # set to shutter speed to auto at the start, to adjust according to light conditions
        time.sleep(0.15)              # not found documentation if a delay is needed after this PiCamera setting
        
        
#         camera.zoom = (0, 0.02, 0.75, 0.82)              # (x,y,width, height)
        camera.zoom = (zoom_x, zoom_y, zoom_w, zoom_h)     # (x,y,width, height)
 
###############################################################################
#################### something to try out #####################################
#         print("camera zoom enquired: ", camera.zoom)
#         print('camera brightness: ', camera.brightness)
#         camera.brightness = 52
#         print('camera brightness set 55... the true is: ', camera.brightness)
#         print('camera contrast: ', camera.contrast)
#         camera.contrast = 10
#         print('camera contrast set 20... the true is: ', camera.contrast)
###############################################################################
###############################################################################
        
        
        a_gain_list=[]                # list to store the Picamera analog gain, during warmup period
        d_gain_list=[]                # list to store the Picamera digital gain, during warmup period
        awb_blue_list=[]              # list to store the Picamera AWB gain, for blue, during warmup period
        awb_red_list=[]               # list to store the Picamera AWB gain, for red, during warmup period
        exp_list=[]                   # list to store the Picamera exposure time, during warmup period
        t_list=[]                     # list to store the warmup progressive time of checkings
        
        #kl=0.98                       # lower koefficient to define acceptance bandwidth (98%)
        ku=2-kl                       # Upper koefficient to define acceptance bandwidth (102%)
        if screen:                    # case a screen is connected
            pts=12                    # amount of consecutive datapoints to analyse if within acceptable range
                                      # few points less than no screen as the system is slower
        else:
            pts=14                    # amount of consecutive datapoints to analyse if within acceptable range
        
        t_start=time.time()           # time reference is allocated as reference for the next task
        
        while time.time()-t_start <20:                                  # timeout for camera warm-up phase is 20 seconds
            if robot_stop:
#                 break
                quit_func()
            
            frame, w, h, scale = pre_read_camera()                      # camera start reading the cube, and adjusts the awb/exposure
            a_gain=camera.analog_gain                                   # analog gain is inquired to the PiCamera
            d_gain=camera.digital_gain                                  # digital gain is inquired to the PiCamera
            awb_gains=camera.awb_gains                                  # awb blue and red gains are inquired to the PiCamera
            exposure=camera.exposure_speed                              # exposure is inquired to the PiCamera
            
            a_gain_list.append(round(float(a_gain),2))                  # analog gain is appended to a list
            d_gain_list.append(round(float(d_gain),2))                  # digital gain is appended to a list
            awb_blue_list.append(round(float(awb_gains[0]),2))          # awb blue part gain is appended to a list
            awb_red_list.append(round(float(awb_gains[1]),2))           # awb blue part gain is appended to a list
            exp_list.append(exposure)                                   # exposure time (micro secs) is appended to a list
            t_list.append(round(time.time()- t_start,2))                # time (from AWB and Exposition start adjustement) is appended to a list
            
            a_check=False                                               # a flag is negatively set for analog gain (being stable...)
            d_check=False                                               # a flag is negatively set for digital gain (being stable...)
            awb_blue_check=False                                        # a flag is negatively set for awb blue gain (being stable...)
            awb_red_check=False                                         # a flag is negatively set for awb red gain (being stable...)
            exp_check=False                                             # a flag is negatively set for the exposure time (being stable...)
            
            if len(a_gain_list) > pts:                                  # requested a minimum amount of datapoints
                check = a_gain_list[-1]/(sum(a_gain_list[-pts:])/pts)   # last analog gain value is compared to the average of last pts points
                if check > kl and check < ku:                           # if comparison within acceptance boundaries
                    a_check=True                                        # flag is positively set for this gain

                check = d_gain_list[-1]/(sum(d_gain_list[-pts:])/pts)   # last digital gain value is compared to the average of last pts points
                if check > kl and check < ku:                           # if comparison within acceptance boundaries
                    d_check=True                                        # flag is positively set for this gain               
            
                check = awb_blue_list[-1]/(sum(awb_blue_list[-pts:])/pts) # last awb_blue gain is compared to the average of last pts points
                if check > kl and check < ku:                           # if comparison within acceptance boundaries
                    awb_blue_check=True                                 # flag is positively set for this gain    
                    
                check = awb_red_list[-1]/(sum(awb_red_list[-pts:])/pts) # last awb_red gain is compared to the average of last pts points
                if check > kl and check < ku:                           # if comparison within acceptance boundarie
                    awb_red_check=True                                  # flag is positively set for this gain
                
                check = exp_list[-1]/(sum(exp_list[-pts:])/pts)         # last exposure time is compared to the average of last pts points
                if check > kl and check < ku:                           # if comparison within acceptance boundarie
                    exp_check=True                                      # flag is positively set for the exposure time                
         
                if a_check and d_check and awb_blue_check and awb_red_check and exp_check: # if all gains are stable
                    PiCamera_param=(a_gain, d_gain, awb_gains, exposure)  # latest parameters returned by the PiCamera are assigned to a tuple
                    break                                                 # camera warmup while loop cab be break

            if screen:                           # case a screen is connected
                text_bg(frame, w, h)             # generates a rectangle as backgroung for text in Frame
                cv2.putText(frame, 'Camera Gains & AWB setting', (10, 30), font, fontScale*1.2,fontColor,lineType)  
                cv2.namedWindow("cube")          # create the cube window
                cv2.moveWindow("cube", 0,0)      # move the window to (0,0)
                cv2.imshow("cube", frame)        # shows the frame 
                key=cv2.waitKey(1)               # refresh time is minimized to 1ms
                    
        
        if screen and debug:                          # case a screen is connected and debug variable is set ON at __main__
            print('\nPiCamera warmup function:')      # feedback is printed to the terminal
            print('analog_gain_list',a_gain_list)     # feedback is printed to the terminal
            print('digital_gain_list',d_gain_list)    # feedback is printed to the terminal
            print('awb_blue_list',awb_blue_list)      # feedback is printed to the terminal
            print('awb_red_list',awb_red_list)        # feedback is printed to the terminal
            print('camera exp_list', exp_list)        # feedback is printed to the terminal
            print('time:', t_list)                    # feedback is printed to the terminal
            print('datapoints:', len(t_list))         # feedback is printed to the terminal
            print(f'\nPiCamera: AWB and Exposure stable in {t_list[-1]} seconds\n')
            
#############################################################################################
################### plotting the PiCamera parameters adjustment trend over time #############
#
#             import matplotlib.pyplot as plt
#             plt.scatter(t_list,a_gain_list, c='k')
#             plt.scatter(t_list,d_gain_list, c='g')
#             plt.scatter(t_list,awb_blue_list, c='blue')
#             plt.scatter(t_list,awb_red_list,c='red')
#             plt.legend(['a_gain', 'd_gain', 'awb_blue', 'awb_red'])
#             plt.show()
#
#############################################################################################

        return PiCamera_param





def robot_consistent_camera_images(PiCamera_param):
    """Picamera is left in Auto mode for Exposure and AWB, untill the first 9 facelets of first side are detected.
    For consistent color detection, on the following sides, these two parameters are retrieved from PiCamera right after
    the first side detection, and se back to PiCamera as 'manual' parametes.
    This prevents the PiCamera to keep adjusting the AWB and Exposition while reading the following 5 cube faces (sides)."""
    
    if device=='laptop':      # case the script is running on a PC/laptop (not the robot)
        return                # function is terminated
    
    elif device=='Rpi':       # case the script is running at the robot
        global robot_stop     # variable used as global
        if robot_stop:        # case the robot has been requested to stop
            return            # function is terminated
        
        a_gain = PiCamera_param[0]                # analog gain picamera setting when warmup got stable
        d_gain = PiCamera_param[1]                # digital gainpicamera setting when warmup got stable
        awb_gains = PiCamera_param[2]             # AutoWhiteBalance gain picamera setting when warmup got stable
        
        if not robot_stop:                        # case of no request to stop the robot
            align = 'read'  # in case a motor alinment is needed the cover will be positioned to (picamera) read position
            print('PiCamera: Reading the exposure time on four cube sides\n')  # feedback is printed to the terminal
            if debug:                             # case debug variable is set true on __main__
                print('camera awb set off')       # feedback is printed to the terminal
            
            camera.awb_mode = 'off'                             # sets white balance off
            time.sleep(0.1)                                     # small (arbitrary) delay after setting a new parameter to PiCamera
            camera.awb_gains = awb_gains                        # sets AWB gain to PiCamera, for consinsent images 
            time.sleep(0.1)                                     # small (arbitrary) delay after setting a new parameter to PiCamera
            camera_set_gains.set_analog_gain(camera, a_gain)    # sets analog gain to PiCamera, for consinsent images
            time.sleep(0.1)                                     # small (arbitrary) delay after setting a new parameter to PiCamera
            camera_set_gains.set_digital_gain(camera, d_gain)   # sets digital gain to PiCamera, for consinsent images
            time.sleep(0.1)                                     # small (arbitrary) delay after setting a new parameter to PiCamera
            camera.shutter_speed = 0      # set the shutter speed to auto at the start, to adjust according to light conditions
            time.sleep(0.1)                                     # small (arbitrary) delay after setting a new parameter to PiCamera


            # PiCamera exposition is inquired to PiCamera on 4 cube sides reachable via a simple cube flip
            # later the average exposition time is set as a fix manual parameter to PiCamera
            exp_list=[]                                # list to store the Picamera exposure time, for the first 4 faces
            for i in range(4):                         # iterate over the 4 faces reachable via a single cube flip
                frame, w, h, scale = read_camera()     # camera start reading the cube, and adjusts the awb/exposure           
                if screen and not robot_stop:          # case a screen is connected and no requests to stop the robot
                    text_bg(frame, w, h)               # generates a rectangle as backgroung for text in Frame
                    cv2.putText(frame, 'Exposure measurement', (10, 30), font, fontScale*1.2,fontColor,lineType)  
                    cv2.namedWindow("cube")            # create the cube window
                    cv2.moveWindow("cube", 0,0)        # move the window to (0,0)
                    cv2.imshow("cube", frame)          # shows the frame 
                    key=cv2.waitKey(1)                 # refresh time is minimized to 1ms
                time.sleep(0.3)                        # small (arbitrary) delay before reading the exposition time at PiCamera
                exp_list.append(camera.exposure_speed) # read picamera exposure time (microsec) on the face i
                if led_usage:                          # case the leds at Top_cover are activated
                    servo.led_off()                    # switch the cover led OFF
                    servo.servo_freq(debug)            # set 60Hz frequency for servo control
                robot_next_side(i)                     # flipping the cube, to reach the next side
                if robot_stop:
#                     break
                    quit_func()
            
            shutter_time = int(sum(exp_list)/len(exp_list))     # set the shutter time to the average exposure time
            camera.shutter_speed = shutter_time        # sets the shutter time to the PiCamera, for consinstent images
   
            if debug:                                                   # case debug variable is set true on __main__
                print('\nexposition time on UBDF faces : ', exp_list)   # feedback is printed to the terminal
                print('\naverage exposition time: ', int(sum(exp_list)/len(exp_list)), 'micro secs') # feedback is printed to the terminal
                print('shutter_speed time set by PiCamera: ',camera.shutter_speed, ' micro secs')    # feedback is printed to the terminal
                print('\nPiCamera parameters, for consistent images, are set to:')                   # feedback is printed to the terminal
                print('analog_gain:',round(float(a_gain),2))            # feedback is printed to the terminal
                print('digital_gain:',round(float(d_gain),2))           # feedback is printed to the terminal
                print('awb_mode:', camera.awb_mode)                     # feedback is printed to the terminal
                print('awb_blue:',round(float(awb_gains[0]),2))         # feedback is printed to the terminal
                print('awb_red:',round(float(awb_gains[1]),2))          # feedback is printed to the terminal
                print(f'camera shutter_speed (0=auto): {shutter_time} micro secs, as average measured on UBDF sides')




def pre_read_camera():
    """Returns the camera reading, and dimensions
    This function is used the first few seconds after setting up the camera, to visualize the camera
    capture, while letting the AWB and Exposition to adjust.
    Function not called when device is laptop."""
    
    camera.capture(rawCapture, format="bgr")                     # bgr is the picamera format directly compatible with CV2
    frame = rawCapture.array                                     # picamera array allows usgae of numpy array
    if len(frame)==0:
        print("Webcam frame not available: 'ret' variable == False")
    elif len(frame)>0:
        oneframe=False                                           # boolean flag to capture only one frame 
        if oneframe == False:
            frame, w, h = frame_cropping(frame, width, height)    # frame is cropped in order to limit the image area to analyze
            frame, w, h, scale = frame_resize (frame, w, h)       # frame is resized
            oneframe = True                                       # flag for a single frame analysis at the time
            rawCapture.truncate(0)                                # empties the array in between each camera's capture   
            return frame, w, h, scale







def read_camera():
    """ Returns the camera reading, and dimensions """
    
    if device == 'laptop':                                              # case the script is running on a PC/laptop (not the robot)
        ret, frame = camera.read()                                      # ret is the boolean if the image array is available, and the image
        if ret==False:
            print("Webcam frame not available: 'ret' variable == False")
        elif ret==True:
            frame, w, h = frame_cropping(frame, width, height)          # frame is cropped in order to limit the image area to analyze
            frame, w, h, scale = frame_resize (frame, w, h)             # frame is resized
            return frame, w, h, scale
    
    elif device == 'Rpi':                                               # case the script is running at the robot
        camera.capture(rawCapture, format="bgr")                        # bgr is the picamera format directly compatible with CV2
        frame = rawCapture.array                                        # picamera array allows usgae of numpy array
        if len(frame)==0:
            print("Webcam frame not available: 'ret' variable == False")
        elif len(frame)>0:
            oneframe=False
            if oneframe == False:
                frame, w, h = frame_cropping(frame, width, height)      # frame is cropped in order to limit the image area to analyze
                frame, w, h, scale = frame_resize (frame, w, h)         # frame is resized, that gives speed
                frame, w, h = left_bg(frame, offset, w, h)              # covers the left frame bandwidth (offset) in gray
                
                oneframe = True                                         # flag for a single frame analysis at the time
                rawCapture.truncate(0)                                  # empties the array in between each camera's capture      
                #print(f'fps: {1/(time.time()-previous_time)}')
                return frame, w, h, scale






def frame_cropping(frame, width, height):
    """Frame cropping, to increase overal speed, when Rpi is used.
    When laptop is used, cropping is not done (camera/webcam resolution should be used instead).
    From 20220903 changed approach, from cropping the image to reduce the PiCamera sensor area, called zoom"""
    
#     global first_cycle, k_kernel, d_iterations, e_iterations, facelets_in_width, min_area, max_area
    
    if device == 'laptop':           # case the script is running on a PC/laptop (not the robot)
        return frame, width, height
    
    elif device == 'Rpi':             # case the script is running at the robot
        h, w = frame.shape[:2]        # frame height and width
        return frame, w, h            # 20220817 looking to zoom property of PiCamera moodule






def left_bg(frame, offset, w, h):
    """Generates a gray vertical bandwith at frame left, as backgroung for the cube sketches"""
    frame = cv2.rectangle(frame, (0, 0), (offset, h), (230,230,230), -1)   # gray background bandwidth (=offeset) at frame left
    h, w = frame.shape[:2]        # frame height and width
    return frame, w, h





def frame_resize(frame, w, h):        
    """Re-sizes the image taken from the camera.
    This is useful on Rpi, as high resolution is needed due to short camera distance from the cube,
    Image is resized to a given value, or percentage of original image.
    When laptop is used the min amd max area to accept facelet countours, are calculated in relation to some 
    parameters, like the frame width after resizing, amount fo facelet in width, and other parameters for edge detection.
    
    At Rpi (robot), this is rather easy, as the cube and camera have fix location/distance"""
    
    if device == 'laptop':                           # case the script is running on a PC/laptop (not the robot)
        global first_cycle, edge, offset, k_kernel, d_iterations, e_iterations, facelets_in_width, min_area, max_area
        
        scale_percent = 80                           # percent of original size (resizing factor)
        if scale_percent*w/100<400:
            scale_percent = int(40000/w) 
        if not cv_wow:                               # case cv_wow variable is set false on __main__
            ww = w                                   # global variable w is assigned to local variable ww
            hh = h                                   # global variable h is assigned to local variable hh
        elif cv_wow:                                 # case cv_wow variable is set true on __main__
            ww = int(frame.shape[1] * scale_percent / 100)                       # new calculated frame width
            hh = int(frame.shape[0] * scale_percent / 100)                       # new calculated frame width
            frame = cv2.resize(frame, (ww, hh), interpolation = cv2.INTER_AREA)  # resized frame
        if first_cycle==True:                        # below part is done once
            if cv_wow:                               # case cv_wow variable is set true on __main__
                edge = int(edge*scale_percent/100)   # edge dimension is resized
                offset = int(14.2*edge)              # offset resized (left part of the frame not usable for cube facelet detection)
            min_area = int(ww/facelets_in_width-k_kernel*(1+d_iterations-e_iterations))**2
            max_area = int(ww/5.5-k_kernel*(1+d_iterations-e_iterations))**2
#             print(f'Min area:{min_area},   Max area:{max_area}')
            first_cycle=False
        return frame, ww, hh, scale_percent

    elif device == 'Rpi':                                                    # case the script is running at the robot
#         scale_perc = 60 # percent of original size                           # resizing factor   
        ww = int(frame.shape[1] * scale_perc / 100)                          # new calculated frame width
        hh = int(frame.shape[0] * scale_perc / 100)                          # new calculated frame width
        frame = cv2.resize(frame, (ww, hh), interpolation = cv2.INTER_AREA)  # resized frame
        return frame, ww, hh, scale_perc







def edge_analysis(frame):
    """Image analysis that returns a balck & white image, based on the colors borders.""" 
        
    global k_kernel, d_iterations, e_iterations
    
    if cv_wow and screen:                                    # case screen and cv_wow variables are set true on __main__
        global gray, blurred, canny, dilated, eroded         # images are set as global variable
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)           # from BGR color space to gray scale
    blurred = cv2.GaussianBlur(gray, (9, 9), 0)              # low pass filter is applied, with a 9x9 gaussian filter
    canny = cv2.Canny(blurred, 10, 30)                       # single pixel edges, having intensity gradient between  10 and 30                      
     
    if device=='laptop':                                     # case the script is running on a PC/laptop (not the robot)
        kernel = np.ones((k_kernel, k_kernel), np.uint8)     # on laptop the kernel is set at setup() function
        dilated = cv2.dilate(canny, kernel, iterations = d_iterations)  # higher "iterations" is overall faster
        eroded = cv2.erode(dilated, kernel, iterations = e_iterations)  # smaller "iterations" keeps the contour apart from the edges  
    
    elif device=='Rpi':                                      # case the script is running at the robot
        kernel = np.ones((5,5), np.uint8)                    # at robot the kernel is fixed
        dilated = cv2.dilate(canny, kernel, iterations = 4)  # higher "iterations" is overall faster
        kernel = np.ones((3,3), np.uint8)                    # smaller kernel is used for the erosion
        eroded = cv2.erode(dilated, kernel, iterations = 2)  # smaller "iterations" keeps the contour apart from the edges
    
    return eroded







def show_cv_wow(cube, show=2000):
    """ shows how the image from the camera is altered to detect the facelets.
    Also possible to set a boolean to save those images."""  
    
    h, w = frame.shape[:2]   # frame height and width
    w-=offset
    
    # note: for precise window position, after windows name at cv2.namedWindow, the parameter cv2.WINDOW_NORMAL  
    # should be used instead of the cv2.WINDOW_RESIZE (default), but image quality on screen changes too much....
    cv2.namedWindow('Gray')                                                # create the Gray window
    cv2.moveWindow('Gray', w+gap_w+offset, gap_h+background_h)             # move the Gray window to coordinate
    cv2.namedWindow('blurred')                                             # create the Blurred window
    cv2.moveWindow('blurred', 2*(w+gap_w)+offset, gap_h+background_h)      # move the Blurred window to coordinate
    cv2.namedWindow('Canny')                                               # create the Canny window
    cv2.moveWindow('Canny', 3*(w+gap_w)+offset, gap_h+background_h)        # move the Canny window to coordinate
    cv2.namedWindow('Eroded')                                              # create the Eroded window
    cv2.moveWindow('Eroded', 2*(w+gap_w)+offset, h+2*gap_h+background_h)   # move the Eroded window to coordinate
    cv2.namedWindow('Dilated')                                             # create the Dilated window
    cv2.moveWindow('Dilated', 3*(w+gap_w)+offset, h+2*gap_h+background_h)  # move the Dilated window to coordinate
    cv2.namedWindow('Cube')                                                # create the Cube window
    cv2.moveWindow('Cube', 0, h+2*gap_h)                                   # move the Cube window to coordinate
    
    cv2.imshow("Gray", gray)                                    # gray is shown, on a window called Gray
    cv2.imshow("blurred", blurred)                              # blurred is shown, on a window called Blurred
    cv2.imshow("Canny", canny)                                  # canny is shown, on a window called Canny
    cv2.imshow("Dilated", dilated)                              # dilated is shown, on a window called Dilated
    cv2.imshow("Eroded", eroded)                                # eroded is shown, on a window called Eroded
    cv2.imshow("Cube", cube)                                    # cube is shown, on a window called Cube
    
    cv2.waitKey(show)                                           # windows show time (ms), for longer time increase timeout variable at start_up()
            
    save_images = False                                         # boolean to enable/disable saving cv_wow images
    import os.path, pathlib                                     # needed library
    folder = pathlib.Path().resolve()                           # active folder (should be home/pi/cube)  
    if save_images:                                             # case the save_image is True    
        folder = os.path.join(folder,'cv_wow_pictures')         # folder to store the cv_wow pictures
        if not os.path.exists(folder):                          # if case the folder does not exist
            os.makedirs(folder)                                 # folder is made if it doesn't exist
        datetime = dt.datetime.now().strftime('%Y%m%d_%H%M%S')  # date_time variable is assigned, for file name
        for i, image in enumerate(('Gray', 'Blurred', 'Canny', 'Dilated', 'Eroded', 'Cube')):
            if side == 1 and i == 0:                                            # case for first image to save
                print('\n\n###############################################')    # feedback is printed to the terminal
                print('##                                          ###')        # feedback is printed to the terminal
                print('##    save_images = True at show_cv_wow     ###')        # feedback is printed to the terminal
                print('##                                          ###')        # feedback is printed to the terminal
                print('##    saving 48 cv_wow pictures  !!!        ###')        # feedback is printed to the terminal
                print('##                                          ###')        # feedback is printed to the terminal
                print('###############################################\n\n')    # feedback is printed to the terminal
            fname = datetime + '_Side'+ str(side) + '_' + str(i) + '_'+ image + '.png'  # filename constructor
            fname = os.path.join(folder, fname)                 # folder+filename for the cube data
            if image == 'Gray':                                 # case the image equals to Gray
                cv2.imwrite(fname, gray)                        # gray image is saved
            elif image == 'Blurred':                            # case the image equals to Blurred
                cv2.imwrite(fname, blurred)                     # blurred image is saved
            elif image == 'Canny':                              # case the image equals to Canny
                cv2.imwrite(fname, canny)                       # canny image is saved
            elif image == 'Dilated':                            # case the image equals to Dilated
                cv2.imwrite(fname, dilated)                     # dilated image is saved
            elif image == 'Eroded':                             # case the image equals to Eroded
                cv2.imwrite(fname, eroded)                      # eroded image is saved
            elif image == 'Cube':                               # case the image equals to Cube
                cv2.imwrite(fname, cube)                        # cube image is saved
            else:                                               # case the does not match proposed strings
                print('error in image list')                    # feedback is printed to the terminal







def square_check(data):  
    """Sanity check if reading a contour with square like shape; Argument is a contour
    Calculates quadrilateral's edge delta lenght: [(max edge - min edge)/average of sides length]
    Calculates the ratio between the 2 diagonals (rhonbus axes): max diagonal / min diagonal
    These parameter are later used to verify if the contour can be considered like a square."""
    
    edges=[]   # list of the 4 edges of the quadrilateral
    axes=[]    # List of axes of symmetry length of the rhombus
    for i in range(len(data)):
        j = i + 1
        if i==3:
            j=0
        edges.append(math.sqrt((data[j][0]-data[i][0])**2 + (data[j][1]-data[i][1])**2))  # list of the 4 edge's length
        edge_delta = (max(edges)-min(edges))*4/sum(edges)  # max side delta over the mean
    
    for i in range(2):
        j = i + 2
        axes.append(math.sqrt((data[j][0]-data[i][0])**2 + (data[j][1]-data[i][1])**2))  # list of the 2 rhobus axes
        axes_delta = min(axes)/max(axes)
    
    return edge_delta, axes_delta







def inclination_check(data):
    """Calculates the facelets inclination from the horizon, usefull while presenting the cube manually to the webcam.
    Argument is a contour.
    The inclination is calculates on the first square edge (first 2 square points after being ordered).
    This info is later used to only accept facelets having a contour within an inclination limit from horizon."""
    
    if device == 'Rpi':              # case the script is running at the robot
        inclination = 0              # inclination is set to 0 as on the robot the cube is always pretty much aligned
    
    elif device == 'laptop':         # case the script is running on a PC/laptop (not the robot)
        inclination = 0              # inclination is initiallt set to 0
        if data[1][1] != data[0][1]: # edge not horizontal (by chance, and giving error on math calculation)
            inclination = - math.atan(((data[1][1]-data[0][1]))/((data[1][0]-data[0][0])))*180/math.pi #inclination in degrees
        else : inclination == 0
    
    return inclination






def read_facelets(frame, scale, det_face_time, proceed):
    """Function that uses cv2 to retrieve contours, from an image (called frame in this case)
    Contours are searched on the 'eroded edges' frame copy
    ROI (Region Of Interest) restricts the image to where the cube images really is

    Notes on 'cv2 find contours'
    Contour's tree is used (cv2.RETR_TREE), to identify children contours (contours within other contrours)
    Approximation (v2.CHAIN_APPROX_SIMPLE) reduces the amount of pixel down to only vertes
    Offset allows to use same coordinate related to the frame, on left not used to overlay info 
    background_h prevents searching for contours on the top text bandwidth."""
    
    
    font_size = fontScale*scale/100               # font size is adjusted according to the frame resizing factor

    if device=='laptop':                          # case the script is running on a PC/laptop (not the robot)
        wait_time = int(delay_facelets_dec - (time.time()- det_face_time))+1
        if wait_time > 0:
            # informative text is added on frame top, as guidance and for decoration purpose
            cv2.putText(frame, str(f'Prepare side {sides[side]}, reading in {wait_time} s'), (10, 30), font, font_size*1.2, fontColor, lineType)
            
            # informative text is added on frame bottom, as guidance
            cv2.putText(frame, str('ESC to escape, spacebar to proceed'), (10, int(h-12)), font, fontScale*1.2,fontColor,lineType)
        else:
            # informative text is added on frame top, as guidance and for decoration purpose
            cv2.putText(frame, str(f'Reading side {sides[side]}'), (10, 30), font, font_size*1.2, fontColor, lineType)
            
            # informative text is added on frame bottom, as guidance
            cv2.putText(frame, str('ESC to escape'), (10, int(h-12)), font, fontScale*1.2,fontColor,lineType)
        
        roi = frame.copy()[background_h:h-background_h, offset:w]       # roi is made on a slice from the copy of the frame image


    elif device == 'Rpi':                         # case the script is running at the robot
        # informative text is added on frame top, as guidance and for decoration purpose
        cv2.putText(frame, str(f'Reading side {sides[side]}'), (10, 30), font, font_size*1.2, fontColor, lineType)
        roi = frame.copy()[background_h:h, offset:w]       # roi is made on a slice from the copy of the frame image
        
    roi_height, roi_width, _ = roi.shape          # returns roi's dimensions
    frame_height, frame_width, _ = frame.shape    # returns roi's dimensions
    
    cube_centers_color_ref(frame)                 # returns the colored centers cube (1 facelet) within the cube's frame

    plot_colors(URFDLB_facelets_BGR_mean, edge, frame, background_h, font, fontScale, lineType) # plot a cube decoration with detected colors
    edges = edge_analysis(roi)                    # edges analysis restriceted to RegionOfInterest (roi)      
    (contours, hierarchy) = cv2.findContours(edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE, offset=(offset,background_h))

    return (contours, hierarchy)







def get_approx_contours(component):
    """Function that simplifies contours (from: https://docs.opencv.org/4.5.3/dd/d49/tutorial_py_contour_features.html)
    Argument is a contour, having at least 4 vertex (contours with less than 4 vertex were previously filtered out)
    Returns approximated contours, having 4 vertex.""" 
    
    contour = component[0]
    hierarchy = component[1][2]
    peri = cv2.arcLength(contour, True)
    contour_convex = cv2.convexHull(contour, False)
    contour = cv2.approxPolyDP(contour_convex, 0.1*peri, True)
    
    return contour, hierarchy, len(contour)







def get_facelets(facelets, contour, hierarchy):
    """Contours are analyzed in order to detect the cube's facelets
    Argument are simplified contours
    Returns contours having square characteristics
    
    [parameter to be square like: Area within limits, limited egde lenght deviation, limited diagonals lenght deviation
    (to prevent rhonbus), limited inclination, limited area variation between the 9 facelets,
    limited distante between the 9 facelets centers].""" 
    
    global square_ratio, rhombus_ratio
    
    if device=='laptop':               # case the script is running on a PC/laptop (not the robot)
        global min_area, max_area      # variable used as global
        square_ratio=0.3               # considered like a square when [(max_side-min_side)/avg edge  < square_ratio], where 0 = perfect square
        rhombus_ratio=0.8              # considered like a square when [max_diagonal/min_diagonal > rhombus_ratio], where 1 = perfect square
        max_inclination=20             # max inclination limit for the 1st facelet vs horizon
        
    elif device == 'Rpi':              # case the script is running at the robot
        image_area=camera_width_res*zoom_w*camera_hight_res*zoom_h
        min_area_k = 134               # coefficient to calculate the min area limit to accept a single facelet's contour
        max_area_k = 33                # coefficient to calculate the max area limit to accept a single facelet's contour
        min_area = int(image_area/min_area_k)   # min area limit to accept a single facelet's contour
        max_area = int(image_area/max_area_k)   # max area limit to accept a single facelet's contour     
        
#         square_ratio=2   # (AF 2)      # considered like a square when min side/max side < square_ratio: Extremely permissive at robot
#         rhombus_ratio=0  # (AF 0)      # considered like a square when diagonal1/diagonal2 > rhombus_ratio: Extremely permissive at robot
        max_inclination=20             # max inclination limit for the 1st facelet vs horizon
         
    area = cv2.contourArea(contour)            # area of each passed contour is retrieved
    
    if device=='laptop' and hierarchy>=0:      # case laptop camera and contour is an outermost (it has inner contour/contours) 
        area =0                                # contour area is forced to zero to skip that contour
    
    if min_area < area < max_area:             # filter out too small and too large contours (areas)                                              # that contour isn't  
        contour_squeeze = np.squeeze(contour)                     # flattens out the list of list used by contours
        edges_delta, axes_ratio = square_check(contour_squeeze)   # sanity check on square and ronbhus shapes
        if edges_delta < square_ratio and axes_ratio > rhombus_ratio:      # check if the contour looks like a square
            out_cont_ord, in_cont_ord = order_4points(contour_squeeze)      # vertex of each contour are ordered CW from top left
            inclination = inclination_check(out_cont_ord)         # facelet inclination is measured
            if abs(inclination)<max_inclination:                  # filter out when the (1st) facelets (cube faces) are too inclined
                contour_tmp = [out_cont_ord]                      # list is made with the ordered contour
                cv2.drawContours(frame, contour_tmp, -1, (0, 0, 0), 1)  # a balck polyline is drawn on the contour (1 px thickness)
                contour_tmp = [in_cont_ord]                      # list is made with the ordered contour
                cv2.drawContours(frame, contour_tmp, -1, (254, 254, 254), 1)  # a white polyline is drawn on the contour (1 px thickness)
                
                # cv2.circle(frame, (contour_ordered[0][0],contour_ordered[0][1]), 5, (0, 0, 0), -1)    # a circle is drawn on the 1st vertex (top left)
                M = cv2.moments(contour)                    # the shape moment (center) of the contour is retrieved
                if M["m00"]:                                # compute the center of the contour   
                    cX = int(M["m10"] / M["m00"])           # X value for the contour center
                    cY = int(M["m01"] / M["m00"])           # Y value for the contour center
                
                tmp = {'area': area, 'cx': cX, 'cy': cY, 'contour': contour, 'cont_ordered':in_cont_ord}  # dict with relevant contour info
                
                facelets.append(tmp)                        # list with the dictionary of the potential facelets contrours 
                
                if len(facelets)>=5:                        # when potential contours are at least 5
                    a_to_exclude = area_deviation(facelets) # function that analyzes facelets area, and list those eventually with high dev from median
                    if len(a_to_exclude)>=1:                # case when there are facelets to be excluded, due to too different area from median one
                        a_to_exclude.sort(reverse=True)     # list order is reversed, making easy easy to remove
                        for i in a_to_exclude:              # contour deviating too much on area are removed from list of potential facelets
                            facelets.pop(i)                 # contour deviating too much on area are removed from list of potential facelets
                    else:                                   # case there are not facelets to be excluded, due to large area deviation
                        if device == 'Rpi':                 # case the script is running at the robot           
                            facelets = estimate_facelets(facelets,frame, w, h)  # calls the function to estimate the remaining facelets
                
                if screen and cv_wow:                       # case a screen is connectedand and cv_wow variables is set true on __main__                            
                    show_cv_wow(frame, show=1)              # calls the cv_wow function
    
    return facelets   # list of potential facelet's contour is returned


   





def facelet_grid_pos(x, y):
    """Returns the face facelet number, based on the coordinates of the contour center, and other parameters.
    This is used to map which in facelets a contour has been detected, and which not."""
    
    # Below dict has the face facelets number as value; keys are facelet coordinate expressed as string combination of column and row
    facelet_number = {'11':0, '21':1, '31':2,
                      '12':3, '22':4, '32':5,
                      '13':6, '23':7, '33':8}
    
    facelet = str(x) + str(y)                 # facelet string combining column (x) and row (y) of the facelet
    if facelet in facelet_number:             # case the facelecet is in the dict
        return facelet_number[facelet]        # facelet number is returned







def estimate_facelets(facelets, frame, w, h):
    """Estimates the remaing facelets location, when there are at least 5 detected facelets.
        This function is interrupted if one row or column is fully empty; In this way the cube width and height
        is well known, enabling a good estimation for the missed facelets position.
        This function is called when the device is Rpi.
    
    
                x_1     x_2     x_3              X
           --|----------------------------------->
             |       |       |       |
        y_1  |   1   |   2   |   3   |   row 1
             |       |       |       |
             |-----------------------
             |       |       |       |
        y_2  |   4   |   5   |   6   |   row 2
             |       |       |       |
             |----------------------- 
             |       |       |       |
        y_3  |   7   |   8   |   9   |   row 3
             |       |       |       |
             |-----------------------
             |
             | clmn1   clmn2   clmn3
             |
             |
           Y v
         
    """
    
    
    cont_x = []                                # empty list to fill with contours centers x coordinates
    cont_y = []                                # empty list to fill with contours centers y coordinates
    cont_area = []                             # empty list to fill with contours areas
    facelets_detected = len(facelets)          # number of facelets alerady detected
    
    for i in range(facelets_detected):         # iteration over the quantity of facelets already detected
        cont_x.append(facelets[i]['cx'])       # all the contours centers x coordinates are listed
        cont_y.append(facelets[i]['cy'])       # all the contours centers y coordinates are listed
        cont_area.append(facelets[i]['area'])  # all the contours areas are listed
    
    med_a = int(statistics.median(cont_area))             # median area for the facelets in function argument
    
    cx = cont_x.copy()         # list copy of contours centers x coordinates
    cy = cont_y.copy()         # list copy of contours centers y coordinates
    cont_x.sort()              # sorted list with contours centers x coordinates
    cont_y.sort()              # sorted list with contours centers y coordinates
    
    x_1 = []                   # empty list to fill with x coordinates of first column (smaller x)
    x_2 = []                   # empty list to fill with x coordinates of second column (medium x)
    x_3 = []                   # empty list to fill with x coordinates of third column (larger x)
    y_1 = []                   # empty list to fill with y coordinates of first row (smaller y)
    y_2 = []                   # empty list to fill with y coordinates of second row (medium y)
    y_3 = []                   # empty list to fill with y coordinates of third row (larger y)

    x_low = cont_x[0]          # smaller x coordinate of facelets countours is assigned to the variable x_low
    x_high = cont_x[-1]        # bigger x coordinate of facelets countours is assigned to the variable x_high
    y_low = cont_y[0]          # smaller y coordinate of facelets countours is assigned to the variable y_low
    y_high = cont_y[-1]        # bigger y coordinate of facelets countours is assigned to the variable y_high 
    
    dist = int(max(x_high-x_low, y_high-y_low)/4) # facelets separation distance from min/max detected contours centers
    
    x_1.append(x_low)          # smaller x coordinate of facelets countours is appended to the first column list
    y_1.append(y_low)          # smaller y coordinate of facelets countours is appended to the first row list
    
    for i in range(1, facelets_detected):                              # iteration on detected facelets contours
        if x_low <= cont_x[i] and cont_x[i] < x_low + dist:            # case the contour center x coordinate is "small"
            x_1.append(cont_x[i])                                      # contour x coordinate is appended to the of first column list (small x)
        elif x_low + dist <= cont_x[i] and cont_x[i] < x_high - dist:  # case the contour center x coordinate is "medium"
            x_2.append(cont_x[i])                                      # contour x coordinate is appended to the of second column list (medium x)
        else:                                                          # case the contour center x coordinate is "large"
            x_3.append(cont_x[i])                                      # contour x coordinate is appended to the of third column list (large x)
        
        if y_low <= cont_y[i] and cont_y[i] < y_low + dist:            # case the contour center y coordinate is "small"
            y_1.append(cont_y[i])                                      # contour y coordinate is appended to the of first row list (small y)
        elif y_low + dist <= cont_y[i] and cont_y[i] < y_high - dist:  # case the contour center y coordinate is "medium"
            y_2.append(cont_y[i])                                      # contour y coordinate is appended to the of second row list (medium y)
        else:                                                          # case the contour center y coordinate is "large"
            y_3.append(cont_y[i])                                      # contour y coordinate is appended to the of third row list (large y)
    
    if len(x_1)==0 or len(x_2)==0 or len(x_3)==0 or len(y_1)==0 or len(y_2)==0 or len(y_3)==0: # case one or more of the six lists are empty
        return facelets                                                # function returns the already detected facelets
    
    else:                                      # case no one of the six lists is empty
        x1_avg = int(sum(x_1)/len(x_1))        # average x coordinate for the contours on first column (small x)
        x2_avg = int(sum(x_2)/len(x_2))        # average x coordinate for the contours on second column (medium x)
        x3_avg = int(sum(x_3)/len(x_3))        # average x coordinate for the contours on third column (large x)
        y1_avg = int(sum(y_1)/len(y_1))        # average y coordinate for the contours on first row (small y)
        y2_avg = int(sum(y_2)/len(y_2))        # average y coordinate for the contours on second row (medium y)
        y3_avg = int(sum(y_3)/len(y_3))        # average y coordinate for the contours on third row (large y)
    
    dist = int((x_high - x_low + y_high - y_low)/8)   # facelets separation distance from min/max detected contours centers
    detected = []                                     # list for the column row of the face grid
    for i in range(facelets_detected):                # iteration over the detected facelets
        if cx[i]<x_low+dist:                          # case the facelet contour center x coordinate is on first grid column
            x=1                                       # 1 (as column 1) is assigned
        elif cx[i]>x_low+dist and cx[i]<x_high-dist:  # case the facelet contour center x coordinate is on second grid column
            x=2                                       # 2 (as column 2) is assigned
        elif cx[i]>x_high-dist:                       # case the facelet contour center x coordinate is on third grid column
            x=3                                       # 3 (as column 3) is assigned
        if cy[i]<y_low+dist:                          # case the facelet contour center x coordinate is on first grid row                          
            y=1                                       # 1 (as row 1) is assigned
        elif cy[i]>y_low+dist and cy[i]<y_high-dist:  # case the facelet contour center x coordinate is on second grid row
            y=2                                       # 2 (as row 2) is assigned
        elif cy[i]>y_high-dist:                       # case the facelet contour center x coordinate is on third grid row
            y=3                                       # 3 (as row 3) is assigned
        detected.append(facelet_grid_pos(x, y))       # list with facelet number is populated
    
    s = set(detected)                                        # list with detected facelets numbers is transformed to set
    missed = [x for x in (0,1,2,3,4,5,6,7,8) if x not in s]  # list with missed facelets numbers
    
    est = []                                  # list for xy coordinates for the estimated facelet center locations
    for facelet in missed:                    # iteration over the missed facelets numbers
        if facelet == 0:                      # case the missed facelet is 0
            est.append((x1_avg, y1_avg))      # average xy coordinates for column 1 and row 1 are appended
        elif facelet == 1:                    # case the missed facelet is 1
            est.append((x2_avg, y1_avg))      # average xy coordinatees for column 2 and row 1 are appended
        elif facelet == 2:                    # case the missed facelet is 2
            est.append((x3_avg, y1_avg))      # average xy coordinats for column 3 and row 1 are appended
        elif facelet == 3:                    # case the missed facelet is 3
            est.append((x1_avg, y2_avg))      # average xy coordinates for column 1 and row 2 are appended
        elif facelet == 4:                    # case the missed facelet is 4
            est.append((x2_avg, y2_avg))      # average xy coordinates for column 2 and row 2 are appended
        elif facelet == 5:                    # case the missed facelet is 5
            est.append((x3_avg, y2_avg))      # average xy coordinates for column 3 and row 2 are appended
        elif facelet == 6:                    # case the missed facelet is 6
            est.append((x1_avg, y3_avg))      # average xy coordinates for column 1 and row 3 are appended
        elif facelet == 7:                    # case the missed facelet is 7
            est.append((x2_avg, y3_avg))      # average xy coordinates for column 2 and row 3 are appended
        elif facelet == 8:                    # case the missed facelet is 8
            est.append((x3_avg, y3_avg))      # average xy coordinates for column 3 and row 3 are appended
        else:                                 # case that shouldn't exist
            print("error on estimating the missed facelets")  # feedback is printed to the terminal

    semi_side = int(0.85*dist/2)                              # half side dimension for the estimated contour square
    for i in range(len(missed)):                              # iteration over the missed facelets
        tl = [est[i][0] - semi_side, est[i][1] - semi_side]   # top left contour coordinate, calculated from the estimated contour center point
        tr = [est[i][0] + semi_side, est[i][1] - semi_side]   # top right contour coordinate, calculated from the estimated contour center point
        br = [est[i][0] + semi_side, est[i][1] + semi_side]   # bottom right contour coordinate, calculated from the estimated contour center point
        bl = [est[i][0] - semi_side, est[i][1] + semi_side]   # bottom left contour coordinate, calculated from the estimated contour center point
        pts=np.array([tl, tr, br, bl], dtype="int32")         # estimated contour coordinates      
        
        gap=5                                  # pixels gap
        tl[0]=max(tl[0]-gap,0)                 # top left x coordinate, shifted toward the contour outer side
        tl[1]=max(tl[1]-gap,0)                 # top left y coordinate, shifted toward the contour outer side
        tr[0]=min(tr[0]+gap,w)                 # top right x coordinate, shifted toward the contour outer side
        tr[1]=max(tr[1]-gap,0)                 # top right y coordinate, shifted toward the contour outer side
        br[0]=min(br[0]+gap,w)                 # bottom right x coordinate, shifted toward the contour outer side
        br[1]=min(br[1]+gap,h)                 # bottom right y coordinate, shifted toward the contour outer side
        bl[0]=max(bl[0]-gap,0)                 # bottom left x coordinate, shifted toward the contour outer side
        bl[1]=min(bl[1]+gap,h)                 # bottom left y coordinate, shifted toward the contour outer side
        outer_pts=np.array([tl, tr, br, bl], dtype="int32")     # estimated contour coordinates, sligtly shifted toward the contour outer side
        
        contour_tmp = [pts]                                           # list is made with the ordered outer contour
        cv2.drawContours(frame, contour_tmp, -1, (0, 0, 0), 1)        # a black polyline is drawn on the contour (1 px thickness)
        contour_tmp = [outer_pts]                                     # list is made with the ordered outer contour
        cv2.drawContours(frame, contour_tmp, -1, (255, 255, 255), 1)  # a white polyline is drawn on the outer contour (1 px thickness)

        tmp = {'area': med_a, 'cx': est[i][0], 'cy': est[i][1], 'contour': pts, 'cont_ordered':pts} # dict with relevant contour info
        facelets.append(tmp)                   # estimated facelets relevant info are appended to the detected facelets list
    
    if screen and debug and device == 'Rpi':  # case there is a connected screen, debug is set ON at __main__ and device is Rpi
        cv2.line(frame, (x_low+dist,0), (x_low+dist,h), (255, 255, 255), 2)    # vertical line is drawn between first and second row of facelets
        cv2.line(frame, (x_high-dist,0), (x_high-dist,h), (255, 255, 255), 2)  # vertical line is drawn between second and third row of facelets
        cv2.line(frame, (0,y_low+dist), (w,y_low+dist), (255, 255, 255), 2)    # horizontal line is drawn between first and second column of facelets
        cv2.line(frame, (0,y_high-dist), (w,y_high-dist), (255, 255, 255), 2)  # horizontal line is drawn between second and third column of facelets
        
    return facelets     # detected facelets combined with estimated facelets







def area_deviation(data):
    """Checks whether the areas of 9 facelets are within a certain deviation from the median one
    Aim of this function is to force the user to present the cube with face somehow parallel to the camera
    This function is called when there are a certain amount of potential facelet contours
    Argument is a list of dictionary, wherein the area is one of the dict values
    Returns a list of facelets (index) to be removed from the potential facelets, having area deviating
    too much from the median one.
    At the robotthiss analysis isn't necessary, as camera a nd cube are always rather parallel.
    At robot the min_area and max_are more than are sufficient to filter out wrong facelet contours."""
    
    to_exclude = []               # list of the contours index to be removed, due to excess of their area deviation
    
    if device=='Rpi':             # case the script is running at the robot
        delta = delta_area_limit  # (AF 0.7) 70% of area deviation from the median is set as threshold
    
    elif device=='laptop':        # case the script is running on a PC/laptop (not the robot)
        delta = 0.3               # 30% of area deviation from the median is set as threshold
    
    area_list=[]                  # list to store the contour areas
    for i in range(len(data)):
        area_list.append(data[i]['area'])                  # all the contour areas are listed
    area_median = statistics.median(area_list)             # median area values
    for i in range(len(area_list)):          
        delta_area=(area_list[i]-area_median)/area_median  # area deviation from the median
        if delta_area > delta:                             # filter on area deviating more than threshold
            to_exclude.append(i)                           # list of the contours to exclude is populated
    
    return to_exclude            # returns list of contours to be removed








def distance_deviation(data, delta=0.25):
    """Checks whether the distances between the 9 contours (centers) are within a certain deviation from the median
    In other words, a sanity check if all the 9 facelet are compatible with a 3x3 square array shape
    Aim of this funtion is to exclude contours generated outside the cube, due to square like shapes the webcam
    detects at the user background, face, cloths.
    The approach checks indipendently the 6 horizontal distances from the contours center, from the 6 vertical
    Considering the cube can have a certain tiltin angle (inclination), Pitagora theorem is used.
    Function return a list with the index of the countours to be removed from the list of potential facelets."""
    
    d_to_exclude = []           # list of the contour index to be removed, due to excess of distance deviation
    
    if device=='Rpi':           # case the script is running at the robot
        return d_to_exclude     # Rpi (robot) do not apply this filter
    
    elif device =='laptop':     # case the script is running on a PC/laptop (not the robot)
        distance_list_h = []    # list of the horizontal distance, of each contour from the median one
        distance_list_v = []    # list of the vertical distance, of each contour from the median one

        points_h=[1,2,4,5,7,8]  # points to consider for the distance along the "horizontal" array 
        for i in points_h:
            j=i-1
            # horizontal distance between the contours centers
            dist=math.sqrt((data[i]['cx']-data[j]['cx'])**2 + (data[i]['cy']-data[j]['cy'])**2)
            distance_list_h.append(dist)  # list with horizontal distance between the contours centers

        points_v=[3,4,5,6,7,8]  # points to consider for the distance along the "vertical" array 
        for i in points_v:
            k=i-3
            # vertical distance between the contours centers
            dist=math.sqrt((data[i]['cx']-data[k]['cx'])**2 + (data[i]['cy']-data[k]['cy'])**2)
            distance_list_v.append(dist)

        dist_median_h = statistics.median(distance_list_h)                # median value for horiz distances
        for i in range(len(distance_list_h)):
            delta_dist_h=(distance_list_h[i]-dist_median_h)/dist_median_h # deviation in horiz distance
            if delta_dist_h > delta:                                      # filter if horiz deviation > threshold
                d_to_exclude.append(i) # list with contours indexto exlude, due excess on horiz deviation from median

        dist_median_v = statistics.median(distance_list_v)                # median value for vert distances
        for i in range(len(distance_list_v)):
            delta_dist_v=(distance_list_v[i]-dist_median_v)/dist_median_v # deviation in vert distance
            if delta_dist_v > delta and i not in d_to_exclude: # filter if horiz deviation > threshold
                d_to_exclude.append(i)  # list with contours indexto exlude, due excess on vert deviation from median
        
        return d_to_exclude






def order_4points(pts):
    """From: https://www.pyimagesearch.com/2016/03/21/ordering-coordinates-clockwise-with-python-and-opencv/
    Orders the 4 vertex of (simplified) contours, so that the first one is top left (CW order)
    Argument is a contour
    Returns a contour with ordered coordinates
    
    0  1
    2  3    """
    
    xSorted = pts[np.argsort(pts[:, 0]), :]        # sort the points based on their x-coordinates
    leftMost = xSorted[:2, :]                      # grab the left-most point from the sorted x-coodinate points
    rightMost = xSorted[2:, :]                     # grab the right-most point from the sorted x-coodinate points
    # sort the left-most according to their y-coordinates, to grab the top-left and bottom-left points, respectively
    leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
    (tl, bl) = leftMost
    
    #########   starting from 20220828, based on Numpy library (no need for Scipy library)   #####################
    # Euclidean distance from top-left and right-most points: the point with largest distance will be bottom-right
    tl = tl.reshape(1, 2)                                    # reshaping top-left coordinate array
    D = tl[:, np.newaxis, :] - rightMost[np.newaxis, :, :]   # matrix difference, by broadcasting
    D = np.linalg.norm(D, axis=-1)[0]                        # matrix distance by using numpy (instead of scipy)
    tl = tl.reshape(2,)                                      # reshaping top-left coordinate array back
    ##############################################################################################################
    
    # sort the right-most coordinates according to their distance from top-left coordinate
    (br, tr) = rightMost[np.argsort(D)[::-1], :]
    
    if device=='laptop':                      # case the script is running on a PC/laptop (not the robot)
        inner_pts=np.array([tl, tr, br, bl])
        
        #few pixel shift toward the outside, to draw the contour on the outer_pts without affecting the inner_pts color detection
        gap=3
        tl[0]=tl[0]-gap
        tl[1]=tl[1]-gap
        tr[0]=tr[0]+gap
        tr[1]=tr[1]-gap
        br[0]=br[0]+gap
        br[1]=br[1]+gap
        bl[0]=bl[0]-gap
        bl[1]=bl[1]+gap
        outer_pts=np.array([tl, tr, br, bl])    # these are the ordered coordinates of the detected contour, sligtly enlarged
        
    
    elif device=='Rpi':                         # case the script is running at the robot
        outer_pts=np.array([tl, tr, br, bl])    # these are the ordered coordinates of the detected contour
        
        #few pixel shift toward the inside, to reduce mount of pixels for color detection
        gap=3           
        tl[0]=tl[0]+gap
        tl[1]=tl[1]+gap
        tr[0]=tr[0]-gap
        tr[1]=tr[1]+gap
        br[0]=br[0]-gap
        br[1]=br[1]-gap
        bl[0]=bl[0]+gap
        bl[1]=bl[1]-gap
        inner_pts=np.array([tl, tr, br, bl])# these are the ordered coordinates of the detected contour, sligtly reduced
     
    return outer_pts, inner_pts   







def order_9points(data, new_center):
    """based on https://www.pyimagesearch.com/2016/03/21/ordering-coordinates-clockwise-with-python-and-opencv/
    Orders the 9 countorus centers, in clockwise order, so that the first one is top left:
    
    0  1  2
    3  4  5
    6  7  8     """
    
    pts=np.zeros([9,2], dtype=int)
    for i in range(len(data)):
        pts[i]=[data[i]['cx'], data[i]['cy']]
        
    xSorted = pts[np.argsort(pts[:, 0]), :]        # sort all the points based on their x-coordinates
    leftMost = xSorted[:3, :]                      # grab the left-most 3 points from the sorted x-coodinate points
    rightMost = xSorted[6:, :]                     # grab the right-most 3 points from the sorted x-coodinate points
    mid = xSorted[3:6, :]                          # remaining 3 points in the x middle
    ySortedMid = mid[np.argsort(mid[:, 1]), :]     # sorts the 3 points in the x middle by the y coordinate
    (tm, mm, bm) = ySortedMid                      # top-middle, middle-middle, bottom-midle points
    
    # sort the 3 left-most points according to their y-coordinates, to grab the top/mid/bottom one respectively
    leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
    (tl, ml, bl) = leftMost   
    
    #########   starting from 20220828, based on Numpy library (no need for Scipy library)   #####################
    # Euclidean distance from top-left and right-most points: the point with largest distance will be bottom-right
    tl = tl.reshape(1, 2)                                    # reshaping top-left coordinate array
    D = tl[:, np.newaxis, :] - rightMost[np.newaxis, :, :]   # matrix difference, by broadcasting
    D = np.linalg.norm(D, axis=-1)[0]                        # matrix distance by using numpy (instead of scipy)
    tl = tl.reshape(2,)                                      # reshaping top-left coordinate array back
    ##############################################################################################################
    
    # sort the right-most according to their distance from top-left coordinate
    (br, mr, tr) = rightMost[np.argsort(D)[::-1], :]

    ordered_points = np.array([tl, tm, tr, ml, mm, mr, bl, bm, br])   # ordered coordinates (centers of 9 facelets)   
    for coordinates in ordered_points:
        for i in range(len(data)):
            if data[i]['cx'] == coordinates[0] and data[i]['cy'] == coordinates[1]:        
                new_center.append(data[i])        # new_center is a new list with data ordered by xy coordinates
                data.pop(i)                       # used data-element is removed to speed up next iterations
                break                             # inner for loop can be break once the if if found and data appended
    return new_center







def cube_sketch_coordinates(x_start, y_start, edge):
    """Generates a list and a dict with the top-left coordinates of each facelet, as per the URFDLB order.
    These coordinates are later used to draw a cube sketch
    The cube sketch (overall a single rectangle with all the facelets in it) starts at x_start, y_start
    Each facelet on the sketch has a square side dimention = edge, defined at start_up() function."""
    
    d = edge             # edge length
    square_start_pt=[]   # lits of all the top-left vertex coordinate for the 54 facelets
    
    starts={0:(x_start+3*d, y_start), 1:(x_start+6*d, y_start+3*d), 2:(x_start+3*d, y_start+3*d), 3:(x_start+3*d, y_start+6*d),
            4:(x_start, y_start+3*d), 5:(x_start+9*d, y_start+3*d)}
    
    for value in starts.values():
        x_start=value[0]
        y_start=value[1]
        y = y_start
        for i in range(3):
            x = x_start
            for j in range(3):
                square_start_pt.append([x, y])
                x = x+d
                if j == 2: y = y+d
    square_dict = {k:tuple(square_start_pt[k]) for k in range(len(square_start_pt))}
    
    return square_start_pt, square_dict     #square_start_center, dictiony cube<->1st vertex, cube edge







def inner_square_points(square_dict,i,edge):
    """Generates the 4 square vertex coordinates, to later color the cube sketch
    These square vertex coordinates are shifted by 1 pixel to the inner side, based on the top-left square vertex (dict of top left
    vertex of the 54 facelets); The dict index defines the facelet number, and the edge is the square side lenght
    The returned array defines the 4 points coordinate, of the area within each of the 54 facelets."""
    
    x=square_dict[i][0]+1
    y=square_dict[i][1]+1
    d=edge-2
    return np.array([(x,y),(x+d,y),(x+d,y+d),(x,y+d)])







def cube_centers_color_ref(frame):
    """On the cube sketch, when the laptop is used, it suggests the faces (center) color as guidance.
    This function fills the center facelets with refence color."""
    
    if device=='Rpi':         # case the script is running at the robot
        return                # this function is not peformed
    
    elif device=='laptop':    # case the script is running on a PC/laptop (not the robot)
        x_start = edge
        y_start = background_h+2*edge+1
        square_start_pt, square_dict = cube_sketch_coordinates(x_start,y_start, edge)
        d = edge         # edge lebght for each facelet reppresentation
        m = edge         # m=margin around the cube reppresentation
        
        cv2.rectangle(frame, (x_start-d, y_start-int(2*d)), (x_start+13*d, int(y_start+9.5*d)), (230,230,230), -1) #gray background
        cv2.putText(frame, 'Reference:', (x_start, y_start-int(0.5*d)), font, fontScale*0.85,(0,0,0),lineType)
        
        for point in square_start_pt:
            cv2.rectangle(frame, tuple(point), (point[0]+edge, point[1]+edge), (0, 0, 0), 1)
        
        # color definition in BGR per each of the 6 centers
        center_facelets={4:(255,255,255), 13:(0,0,204), 22:(0,132,0), 31:(0,245,245), 40:(0,128,255), 49:(204,0,0)}
        center_facelet_colors= {4:'white', 13:'red', 22:'green', 31:'yellow', 40:'orange',  49:'blu'}   
        
        for key, bgr_color in center_facelets.items():
            points=inner_square_points(square_dict,key,edge)  # coordinates for the center facelet on the iterator
            color= bgr_color                                  # BGR color is returned
            cv2.fillPoly(frame, pts = [points], color=color)  # facelet is fille with the BGR color for tha center






def plot_colors(BGR_mean, edge, frame, background_h, font, fontScale, lineType):
    """This function plots the detected color of each facelet on the cube sketch.
    This function is called each time a cube face is detected
    The color used on each facelet is the mean BGR calculated on that facelet
    In essence this as decoration to compare interpreted vs detected colors."""
    
    x_start = edge
    y_start = background_h+14*edge
    square_start_pt, square_dict = cube_sketch_coordinates(x_start,y_start, edge)
    d = edge         # edge lebght for each facelet reppresentation
    m = edge         # m=margin around the cube reppresentation
    
    cv2.rectangle(frame, (x_start-d, y_start-2*d), (x_start+13*d, int(y_start+9.5*d)), (230,230,230), -1) #gray background
    cv2.putText(frame, 'Detected:', (x_start, y_start-int(0.5*d)), font, fontScale*0.85,(0,0,0),lineType)
    
    for point in square_start_pt:
        cv2.rectangle(frame, tuple(point), (point[0]+edge, point[1]+edge), (0, 0, 0), 1)

    # iteration on the global dict with stored the avg BGR value of each facelet detected so far
    for i in range(len(BGR_mean)):   
        B=BGR_mean[i][0]                                    # blue component of the mean BGR, for the "í" facelet
        G=BGR_mean[i][1]                                    # green component of the mean BGR, for the "í" facelet
        R=BGR_mean[i][2]                                    # red component of the mean BGR, for the "í" facelet
#         start_point=square_dict[i]              
        points=inner_square_points(square_dict,i,edge)      # coordinates for "i" facelet at the sketch
        cv2.fillPoly(frame, pts = [points], color=(B,G,R))  # "i" facelet is colored with the detected average BGR color






def cube_colors_interpreted(BGR_detected):
    """This function is used to decide wich color belongs to which facelet (cube's side) and related facelet position
    From the mean BGR color, detected per each facelet, the euclidean distance is calculated toward the 6 reference colors (centers).
    The basic principle is to measure the color distance from the cube's center facelets.
    
    1st addition:
    Due to camera vignetting and light conditions, some color interpretation issues were sometimes happening (i.e. swapped red and orange)
    To solve this problem the reference colors are averaged with the just interpreted facelets (dinamic color references)
    In this approach the facelets need to be ordered by increasing distance from the reference; This allows to initially choose the more
    certain facelets, and use them to adapt the references.
    After completing this process, for all the facelets, the interpreted colors have to be ordered back to URFDLB order.
    This improved version has proved to solve the color interpetation errors from the simpler original function; The verification
    has been done on some BGR_detected cube status logged with errors on interpreted colors
    
    2nd addition:
    1st addition brougth to a success rate of 97%, rather good yet not fully sattisfactory.
    With very bright light conditions the Red/Orange were sometimes swapped on color assigment.
    On this latest improvement the HVS color space is added, and used for facelet's color interpretation, 
    in case the BGR color distance doesn't provide coherent result."""

    # Step1: dict with BGR_detected and facelet's position as key
    #        dict with HSV (detected color) and facelet's position as key
    BGR_detected_dict={} 
    HSV_detected={}
    for i in range(len(BGR_detected)):
        BGR=BGR_detected[i]
        BGR_detected_dict[i]=BGR
        B,G,R=BGR
        BGR_mean = np.array([[[B,G,R]]], dtype=np.uint8)
        hsv = cv2.cvtColor( BGR_mean, cv2.COLOR_BGR2HSV)
        H=hsv[0][0][0]
        S=hsv[0][0][1]
        V=hsv[0][0][2]
        HSV_detected[i]=(H,S,V)

    if debug:
        print(f'\nBGR_detected: {BGR_detected}')
        print(f'\nHSV: {HSV_detected}')

    # Step2: center's facelets are used as initial reference
    cube_ref_colors = {'white':BGR_detected[4], 'red':BGR_detected[13], 'green':BGR_detected[22],
                       'yellow':BGR_detected[31], 'orange':BGR_detected[40], 'blue':BGR_detected[49]}
    
    # Step3: dictionary with the color distances from the (initial) references
    color_distance={}                                             # empty dict to store all the color distances for all the facelets
    cube_ref_colors_lab={}                                        # empty dictionary to store color refences in Lab color space
    for color, BGR in cube_ref_colors.items():
        B,G,R = BGR                                               # BGR values are unpact from the dict
        cube_ref_colors_lab[color]=tuple(rgb2lab([R,G,B]))        # BGR conversion to lab color space and dict feeding
            
    for facelet, color_measured in BGR_detected_dict.items():         
        B,G,R = color_measured
        lab_meas = rgb2lab([R,G,B])                               # conversion to lab color space (due CIEDE2000 function)
        distance=[]                                               # list with the distance from the 6 references, for each facelet
        for color, lab_ref in cube_ref_colors_lab.items():        # iteration over the 6 reference colors
            distance.append(CIEDE2000(tuple(lab_meas), lab_ref))  # Euclidean distance toward the 6 reference colors
        color_distance[facelet]=distance                          
    
    
    # Step4: Ordering the color distance (the min value per each facelet) by increasing values
    color_distance_copy=color_distance.copy()                     # a dict copy is made, to drop items while the analysis progresses
    color_distance_ordered={}                                     # empty dictiony to store the (min) color distance by increasing values
    for i in range(len(color_distance_copy)):
        index = color_distance_copy.keys()
        key_min_dist = min(color_distance_copy,key=lambda key:min(color_distance_copy[key]))
        color_distance_ordered[key_min_dist] = color_distance_copy[key_min_dist]
        color_distance_copy.pop(key_min_dist)
    
    
    # Step5: List with facelets position ordered according to the color distance increasing order
    # this is needed to come back later to original kociemba facelets order
    key_ordered_by_color_distance = [x for x in color_distance_ordered.keys()]


    # Step6: Ordering the facelets BGR color values according to color distance from the reference colors
    BGR_ordered={}
    for key in color_distance_ordered.keys():
        BGR_ordered[key]=BGR_detected[key]


    # Step7: Color interpretation
    cube_status_by_color_distance={}          # dict to store the cube status reppresentation wih the interpreted colors
    distance={}                               # dict to store the color distance during each facelet check
#     distance_value=[]                       # list to store the color distance for the selectec color/facelet association
    
    i=0
    for value in BGR_ordered.values():            # iteration on the facelet's BGR values ordered by increasing color distance from ref
        B,G,R =value
        lab_meas = rgb2lab([R,G,B])                                         # conversion to lab color space (due CIEDE2000 function)
        for color, lab_ref in cube_ref_colors_lab.items():                  # iteration over the 6 reference colors
            distance[color]=CIEDE2000(tuple(lab_meas), lab_ref)             # Euclidean distance toward the 6 reference colors
        color = min(distance, key=distance.get)                             # chosem color is the one with min distance from reference
  
        cube_status_by_color_distance[i]=color                              # dict of cube status wih the interpreted colors  
#         distance_value.append(distance[min(distance, key=distance.get)])  # list with the color distance of the chosen facelet's color
        distance.clear()                                                    # distance dict is cleared for the next facelet
        
        B_avg = math.sqrt((B**2+ (cube_ref_colors[color][0])**2)/2)   # average Red color id made from the chosen color and previous reference
        G_avg = math.sqrt((G**2+ (cube_ref_colors[color][1])**2)/2)   # average Green color id made from the chosen color and previous reference
        R_avg = math.sqrt((R**2+ (cube_ref_colors[color][2])**2)/2)   # average Blue color id made from the chosen color and previous reference

        cube_ref_colors[color]=(B_avg, G_avg, R_avg)                    # Color reference dict is updated with the new BGR averaged color
        cube_ref_colors_lab[color]=tuple(rgb2lab([R_avg,G_avg,B_avg]))  # Lab color space reference dict is updated with the new color reference 
        i+=1
    
    
    # Step8: Cube detection status is generated
    cube_status={}
    for i in range(54):
        index = key_ordered_by_color_distance.index(i)
        cube_status[i]=cube_status_by_color_distance[index]
    
    
    # Step9: Cube color sequence for a nicer decoration on interpreted colors (using the HSV color space)
    VS_value={}                            # dict to store the V-S (Value-Saturation) value of all facelets
    Hue={}                                 # dict to store the Hue value of all facelets
    
    i=0
    for H,S,V in HSV_detected.values():           # Hue, Saturation and Value are retrieved from the HSV dict
        VS_value[i]=int(V)-int(S)                 # V-S (value-Saturation) value for all the facelet, populates the related dict
        Hue[i]=int(H)                             # Hue, for all the facelets, populates the related dict
        i+=1
    
    #step 10: function to get the color (sides) order, list of colored center's facelets and white center facelet  
    cube_color_sequence, HSV_analysis = retrieve_cube_color_order(VS_value, Hue)
    if debug:
        print(f'\nCube_color_sequence: {cube_color_sequence}')
    
    return cube_status, HSV_detected, cube_color_sequence, HSV_analysis







def retrieve_cube_color_order(VS_value,Hue):
    """Determines the cube colors order, meaning the sides order while manually presenting the cube in front of the laptop
    .... or differently the cube's orientation as it has been dropped on the robot.
    The function returns a list with the color order, as per kociemba sequence.
    The function also returns a boolean if the HSV analysis is so far coherent."""
    
    HSV_analysis=True                                # flag used to validate the analysis
    
    centers=[4, 13, 22, 31, 40, 49]                       # facelets of the cube's sides centers 
    cube_color_sequence=[4, 13, 22, 31, 40, 49]           # list of center facelets, to later store the cube's side COLORS
    
    if debug:
        print(f'\nHue centers: {Hue[centers[0]]}, {Hue[centers[1]]} , {Hue[centers[2]]}, {Hue[centers[3]]}, {Hue[centers[4]]}, {Hue[centers[5]]}')
    
    VS_centers=[VS_value[facelet] for facelet in centers]  # V-S value measured on the cube side center under analysis
    Hcenters=[Hue[facelet] for facelet in centers]        # Hue value measured on the cube side center under analysis
    
    
    # white and yellow facelet face (center side facelet number)
    white_center=centers[VS_centers.index(max(VS_centers))]   # white center facelet (max V-S distinguishes the white)
    if white_center<27: 
        yellow_center=white_center+27                # yellow center facelet (yellow is at opposite side of white)
    else:
        yellow_center=white_center-27                # yellow is at opposite side of white
    
    try:
        centers.remove(white_center)                     # white facelet is removed from the list of center's facelets
    except:
        HSV_analysis=False
        if debug:
            print(f'\nIssue with the white_center')
    
    try:
        centers.remove(yellow_center)                    # yellow facelet is removed from the list of center's facelets
    except:
        HSV_analysis=False
        if debug:
            print(f'\nIssue with the yellow_center')
       
    
    # searching for the red and orange cube's side
    for facelet in centers:                   # iteration on the 4 cube's remaining sides centers
        if facelet<=27:            
            opp_facelet=facelet+27            # opposite side center facelet
        elif facelet>27:
            opp_facelet=facelet-27            # opposite side center facelet
        Hc=Hue[facelet]                       # Hue value measured on the cube side center under analysis
        H_opp=Hue[opp_facelet]                # Hue value measured on the cube opposite side center

        if (Hc>150 and H_opp<30) or (Hc<30 and H_opp<30 and Hc<H_opp) or (Hc>160 and H_opp>170 and Hc<H_opp):   # Hue filter for red vs orange
            red_center=facelet                # red center facelet
        elif (Hc<30 and H_opp>150) or (Hc<30 and Hc>H_opp) or (Hc>170 and Hc>H_opp):              # Hue filter for orange vs red
            orange_center=facelet             # orange center facelet
    
    try:
        centers.remove(red_center)            # red facelet is removed from the list of center's facelets
    except:
        HSV_analysis=False
        if debug:
            print(f'\nIssue with the red_center')
    
    try:
        centers.remove(orange_center)         # orange facelet is removed from the list of center's facelets
    except:
        HSV_analysis=False
        if debug:
            print(f'\nIssue with the orange_center')

################ DEBUG  #########
#     HSV_analysis=False    # uncoment this row to force an error on the 6 center facelets color detection
#################################

    if HSV_analysis==True:
        # last two colors are blue and green, wherein blue has the highest Hue
        Hc=[Hue[facelet] for facelet in centers]  # Hue value for the last two center's facelets
        blue_center=centers[Hc.index(max(Hc))]    # blue center facelet (higher Hue than green)
        green_center=centers[Hc.index(min(Hc))]   # green center facelet (lower Hue than blue)
        
        for index, value in enumerate(cube_color_sequence):
            if value == white_center:
                cube_color_sequence[index] = 'white'
            elif value == yellow_center:
                cube_color_sequence[index] = 'yellow'
            elif value == red_center:
                cube_color_sequence[index] = 'red'
            elif value == orange_center:
                cube_color_sequence[index] = 'orange'
            elif value == blue_center:
                cube_color_sequence[index] = 'blue'
            elif value == green_center:
                cube_color_sequence[index] = 'green'

    else:                                                                            # case the HSV_analysis is false
        print('\nNot found 6 different colors at center facelets (HSV analysis)')    # feedback is printed to the terminal

    return cube_color_sequence, HSV_analysis







def cube_colors_interpreted_HSV(BGR_detected, HSV_detected):
    """In case the color interpretation, based on BGR color distance, doesn't give coherent cube status, this function is called.
    Under bright/sunny light condition the colors, chosen via color distance analysys, are sometimes wrongly interpreted
    (=not coherent cube status);
    This function make use on the HSV color space, converted from the avg BGR read on facelets.
    This funcion re-determines the colors assigned to all facelets, based on HSV value measured.
    Baseline:  White has higest V (Value) and lowest S (Saturation) than the other colors; Used "(V-S)" parameter
               Not-white facelets are evaluated only by their H (Hue) value
    Hue (range 0 to 180) average values:  Orange 6, Yellow 32, Green 85, Blue 110, Red 175 (also < 10)
    Note: Depending on the light conditions could be 0< Red_Hue <10 ; In these cases Orange_Hue > Red_Hue
    Note: In some extreme cases orange_hue >175; In these cases Orange_Hue > Red_Hue
    
    This function returns the interpreted facelet colors, on the URFDLB order.
    The function also returns the cube color sequence, to generate a nice decoration on screen."""
    
    print('cube_colors_interpr_HSV function has been called')   # feedback is printed to the terminal
    
    VS_value={}                             # dict to store the V-S (Value-Saturation) value of all facelets
    Hue={}                                  # dict to store the Hue value of all facelets
    
    i=0
    for H,S,V in HSV_detected.values():           # Hue, Saturation and Value are retrieved from the HSV dict
        VS_value[i]=int(V)-int(S)                 # V-S (value-Saturation) delta value for all the facelet, populates the related dict
        Hue[i]=int(H)                             # Hue, for all the facelets, populates the related dict
        i+=1
    if debug:
        print(f'\nV_values: {VS_value}')
        print(f'\nHue: {Hue}')
    
    
    # function to get the color (sides) order, list of colored center's facelets and white center facelet  
    cube_color_sequence, HSV_analysis = retrieve_cube_color_order(VS_value,Hue)    
    
    while HSV_analysis == True:                   # flag on positive analysis is placed on true, and later evenbtually changed
        
        # getting the white center's facelet and the colored facelets
        centers=[4, 13, 22, 31, 40, 49]                              # center facelet numbers
        white_center = centers[cube_color_sequence.index('white')]   # facelet number for the white center
        centers.remove(white_center)                                 # white facelet center is removed from the list of facelets centers
        colored_centers=centers                                      # colored centers (without the white)
        
        if debug:
            print(f'\nWhite facelet: {white_center}')
            print(f'\nCube_color_sequence: {cube_color_sequence}')
        
        # searching all the 9 white facelets
        Hw,Sw,Vw=HSV_detected[white_center]                              # HSV of the white center
        if debug:
            print(f'White Hw,Sw,Vw: {Hw}, {Sw}, {Vw}')                   # HSV of the white center
            print(f'Colored center facelets: {colored_centers}')         # colored center facelets

        VSdelta={}                                                       # dict to store the V-S (value-Satuartion) value of all facelets
        i=0
        for H,S,V in HSV_detected.values():                              # Hue, Saturation and Value are retrieved from the HSV dict
            VSdelta[i]=int(V)+int(V)-int(S)-abs(3*(int(H)-int(Hw)))      # V+(V-S)+abs(3*deltaH) value for all the facelets
            i+=1
        
        # ordering the VSdelta by increasing values (9 highest values are the 9 white facelets)
        VSdelta_copy=VSdelta.copy()       # a dict copy is made, to drop items while the analysis progresses
    
        # a new dict with V-S delta value ordered is generated, in order to have the white facelets close to each other
        VSdelta_ordered={k: v for k, v in sorted(VSdelta_copy.items(), key=lambda item: item[1])}
        key_ordered_by_VSdelta = [x for x in VSdelta_ordered.keys()]    # list with the key of the (ordered) dict is generated
        white_facelets_list=key_ordered_by_VSdelta[-9:]                 # white facelets have the biggest H-S value, therefore are the last 9

        if debug:
            print(f'White facelets: {white_facelets_list}')
            print(f'\nHue dict all facelets {Hue}')
        
        
        white_facelets={}  # empty dict to store the association of facelet position for the white facelets
        
        # white facelets are removed from Hue dictionary, as meant for colored facelets
        for facelet in white_facelets_list:                # iteration on all the white facelets listed
            try:
                del Hue[facelet]                           # all white facelets are removed from the Hue dictionary
                white_facelets[facelet]='white'            # dictionary with white facelets is populated
            except:
                HSV_analysis = False
                break
        
        if debug:
            print(f'\nHue dict colored facelets {Hue}')
        
        # facelet's color selection by Hue distance from each center's Hue
        centers=[4,13,22,31,40,49]                                # facelets of centers
        centers.remove(white_center)                              # facelets for colored centers
        cube_color_sequence_no_white=cube_color_sequence.copy()   # cube color sequence copy for list without white
        cube_color_sequence_no_white.remove('white')              # cube color sequence, for colored centers
        
        try:
            Hcenters=[Hue[x] for x in colored_centers]            # list of Hue values for the colored centers
            # an error can happen if the same cube side has been showed twice or more
        except:
            if debug:
                print('\nNot found 6 different colors at center facelets')
            HSV_analysis=False
            break
        
        if debug:
            print(f'\nHcenters (no white): {Hcenters}')
        
        Hc_red=Hue[colored_centers[cube_color_sequence_no_white.index('red')]]     # Hue value for the red center facelet
        Hc_blue=Hue[colored_centers[cube_color_sequence_no_white.index('blue')]]   # Hue value for the blue center facelet
        red_blue_avg=(Hc_red+Hc_blue)//2                                           # mid Hue value between red and blue
        
        # red is supposed to have Hue >160, yet sometime it is below 10
        # distance from the Hue of center facelets is used to decide the facelet's color
        color_facelets={}                                           # dict to store all the colored facelets (NO WHITE)
        for facelet, H in Hue.items():                              # iteration on all the color facelets
            Hdistance=[]                                            # list to store the Hue distance from the 5 colored centers                   
            if sum(value>red_blue_avg for value in Hue.values())<5: # condition suggesting some of the red facelets have Hue < 10
                for Hc in Hcenters:                                 # iteration over the 5 reference Hue colors
                    if H>red_blue_avg:                              # when the facelet's Hue is above mid distance between Red center and Blue center
                        H=0                                         # Hue is force to 0
                    if Hc>red_blue_avg:                             # Red center Hue (when the center facelet is above mid distance between Red center and Blue )
                        Hc=0                                        # Red center Hue is forced to 0
                    dist=min(abs((H-Hc)),180-Hc+H)                  # red can either be 170<Hue<180 as well as <10
                    Hdistance.append(dist)                          # absolute Hue distance toward the 5 colored centers Hue
            else:                                                   # else case is when all the 6 red's facelts Hue are above mid Hue distance blue / red
                for Hc in Hcenters:                                 # iteration over the 5 reference Hue colors
                    dist=min(abs((H-Hc)),180-Hc+H)                  # red can either be 170<Hue<180 as well as <10
                    Hdistance.append(dist)                          # absolute Hue distance toward the 5 colored centers Hue

            color=cube_color_sequence_no_white[Hdistance.index(min(Hdistance))] # right color has min distance from center facelet of same color
            color_facelets[facelet]=color                                       # color is assigned to dictionary


        # Merging the white and color facelets dictionaries, and sorting it by key
        cube_status_detected={**white_facelets, **color_facelets}                                      # dict with white and colored facelets
        cube_status_detected={k: v for k, v in sorted(cube_status_detected.items(), key=lambda item: item[0])}   # dict with white & color ordered by facelets

        # cube can be positioned on the robot with an orientationon different from conventional URFDLB order
        # cube can be presented to the laptop camera on a different color order than used by Kociemba solver
        std_cube_color=['white', 'red', 'green', 'yellow', 'orange', 'blue']             # conventional color sequence 
        if cube_color_sequence != std_cube_color:
            cube_status_URFDLB={}                                                        # dict with cube status using conventional colors
            i=0
            for color in cube_status_detected.values():                                  # iteration on dict with detected colors
                cube_status_URFDLB[i]=std_cube_color[cube_color_sequence.index(color)]   # colors are exchanged, and assigned to the dict for solver
                i+=1
            if debug:
                print(f'\nCube_status_detected_colors: {cube_status_detected}')
                print(f'\nCube_status_conventional_colors: {cube_status_URFDLB}\n')
            break   # wrong identation until 27 April 2022, additionally to wrong word (breake)
            
        elif cube_color_sequence == std_cube_color:
            cube_status_URFDLB=cube_status_detected
            if debug:
                print('\nCube colors and orientation according to URFDLB order')
                print(f'\nCube_status_detected_colors: {cube_status_detected}\n')
            break   # wrong identation until 27 April 2022
    
    if HSV_analysis==False:
        cube_status_URFDLB={}
        cube_status_detected={}
    
    # cube_status_URFDLB uses the conventional colors for kociemba solver
    # cube_status_detected has the detected colors, via the HSV approach. This dict is used for decoration purpose
    return cube_status_URFDLB, cube_status_detected, cube_color_sequence, HSV_analysis







def plot_interpreted_colors(device, cube_status, cube_color_sequence, cube_status_string, detect_winner, HSV_analysis,edge, frame, font, fontScale, lineType):
    """Based on the detected cube status, a sketch of the cube is plot with bright colors."""
    
    x_start=edge            # top lef corner of the rectangle where all the cube's faces are plot
    y_start=5*edge          # top lef corner of the rectangle where all the cube's faces are plot
    
    _, square_dict = cube_sketch_coordinates(x_start, y_start, edge)
    d = edge                # edge lenght for each facelet reppresentation
    m = edge                # m=margin around the cube reppresentation 
    cv2.rectangle(frame, (x_start-m, y_start-2*edge), (x_start+13*d, int(y_start+9.2*d)), (230,230,230), -1) #gray background
    cv2.putText(frame, 'Interpreted:', (x_start, y_start-int(0.5*d)), font, fontScale*0.85,(0,0,0),lineType)
    
    if detect_winner == 'Error':     # case the cube status detection ended with an error
        # on the pictures collage a text is added indicating the Error
        cv2.putText(frame, str(detect_winner), (x_start+7*d, y_start+int(8*d)), font, fontScale*0.85,(0,0,0),lineType)
    else:  # case the cube status detection succeeded via BGR color distance or by HSV color analysis
        # on the pictures collage a text indicating which method was successfull (BGR color distance is done first, HSV only if BGR color distance fails)
        cv2.putText(frame, str(detect_winner), (x_start+7*d, y_start+int(8*d)), font, fontScale*0.85,(0,0,0),lineType)
    
    
    
    cube_bright_colors = {'white':(255,255,255), 'red':(0,0,204), 'green':(0,132,0), 'yellow':(0,245,245),
                          'orange':(0,128,255), 'blue':(204,0,0)}
    std_color_sequence = list(cube_bright_colors.keys())        
    
    for i, color in enumerate(cube_status.values()):
        start_point=square_dict[i]                                       # top-left poiint coordinate
        if HSV_analysis:                                                 # case the detected 6 center facelets have 6 different colors
            cv2.rectangle(frame, tuple(start_point), (start_point[0]+d, start_point[1]+d), (0, 0, 0), 1) # squre black frame
            col=cube_color_sequence[std_color_sequence.index(color)]     # from standard color sequence to the one detected, due to cube orientation at start
            B,G,R = cube_bright_colors[col]                              # BGR values of the bright colors for the corresponding detected color
            inner_points=inner_square_points(square_dict,i,d)            # array with the 4 square inner vertex coordinates
            cv2.fillPoly(frame, pts = [inner_points], color=(B,G,R))     # inner square is colored with bright color of the interpreted one
        else:                                                            # case the detected 6 center facelets do not have 6 different colors
            
            cv2.putText(frame, cube_status_string[i], (start_point[0]+int(0.2*d), int(start_point[1]+int(0.8*d))),\
                        font, fontScale*0.5,(0,0,0),lineType)            # facelets side LETTER is printed on the sketch

#         col=cube_color_sequence[std_color_sequence.index(color)]  # from "URFDLB cube status" color to detected color
#         B,G,R = cube_bright_colors[col]                           # decorating a cube reppresentation with bright colors
#         for i, color in enumerate(cube_status.values()):
#             col=cube_color_sequence[std_color_sequence.index(color)]  # from "URFDLB cube status" color to detected color
#             B,G,R = cube_bright_colors[col]                           # decorating a cube reppresentation with bright colors
#             start_point=square_dict[i]                                # top-left poiint coordinate
#             cv2.rectangle(frame, tuple(start_point), (start_point[0]+d, start_point[1]+d), (0, 0, 0), 1) # squre black frame
#             inner_points=inner_square_points(square_dict,i,edge)            # array with the 4 square inner vertex coordinates
#             cv2.fillPoly(frame, pts = [inner_points], color=(B,G,R))        # inner square is colored with interpreted color
        




def rgb2lab(inputColor):
    """Convert RGB (not BGR !!!) in L*a*b colors space
    from: https://gist.github.com/manojpandey/f5ece715132c572c80421febebaf66ae (RGB to CIELab color space conversion)
        Step 1: RGB to XYZ
                http://www.easyrgb.com/index.php?X=MATH&H=02#text2
        Step 2: XYZ to Lab
                http://www.easyrgb.com/index.php?X=MATH&H=07#text7
    
    L*a*b color space is a device-independent, "standard observer" model.
    It is useful in industry for detecting small differences in color."""
    
    num = 0
    RGB = [0, 0, 0]
    for value in inputColor:
        value = float(value) / 255
        if value > 0.04045:
            value = ((value + 0.055) / 1.055) ** 2.4
        else:
            value = value / 12.92
        RGB[num] = value * 100
        num = num + 1
    XYZ = [0, 0, 0, ]
    X = RGB[0] * 0.4124 + RGB[1] * 0.3576 + RGB[2] * 0.1805
    Y = RGB[0] * 0.2126 + RGB[1] * 0.7152 + RGB[2] * 0.0722
    Z = RGB[0] * 0.0193 + RGB[1] * 0.1192 + RGB[2] * 0.9505
    XYZ[0] = round(X, 4)
    XYZ[1] = round(Y, 4)
    XYZ[2] = round(Z, 4)

    # Observer= 2°, Illuminant= D65
    XYZ[0] = float(XYZ[0]) / 95.047         # ref_X =  95.047
    XYZ[1] = float(XYZ[1]) / 100.0          # ref_Y = 100.000
    XYZ[2] = float(XYZ[2]) / 108.883        # ref_Z = 108.883

    num = 0
    for value in XYZ:
        if value > 0.008856:
            value = value ** (0.3333333333333333)
        else:
            value = (7.787 * value) + (16 / 116)
        XYZ[num] = value
        num = num + 1
    Lab = [0, 0, 0]
    L = (116 * XYZ[1]) - 16
    a = 500 * (XYZ[0] - XYZ[1])
    b = 200 * (XYZ[1] - XYZ[2])

    Lab[0] = round(L, 4)
    Lab[1] = round(a, 4)
    Lab[2] = round(b, 4)
    return Lab







def CIEDE2000(Lab_1, Lab_2):
    """Calculates CIEDE2000 color distance between two CIE L*a*b* colors
    from: https://github.com/lovro-i/CIEDE2000
    It returns the Euclidean distance between two colors, and it is used to compare each facelet toward the 6 centers."""
    
    C_25_7 = 6103515625 # 25**7

    L1, a1, b1 = Lab_1[0], Lab_1[1], Lab_1[2]
    L2, a2, b2 = Lab_2[0], Lab_2[1], Lab_2[2]
    C1 = math.sqrt(a1**2 + b1**2)
    C2 = math.sqrt(a2**2 + b2**2)
    C_ave = (C1 + C2) / 2
    G = 0.5 * (1 - math.sqrt(C_ave**7 / (C_ave**7 + C_25_7)))
    
    L1_, L2_ = L1, L2
    a1_, a2_ = (1 + G) * a1, (1 + G) * a2
    b1_, b2_ = b1, b2
    
    C1_ = math.sqrt(a1_**2 + b1_**2)
    C2_ = math.sqrt(a2_**2 + b2_**2)
    
    if b1_ == 0 and a1_ == 0: h1_ = 0
    elif a1_ >= 0: h1_ = math.atan2(b1_, a1_)
    else: h1_ = math.atan2(b1_, a1_) + 2 * math.pi
    
    if b2_ == 0 and a2_ == 0: h2_ = 0
    elif a2_ >= 0: h2_ = math.atan2(b2_, a2_)
    else: h2_ = math.atan2(b2_, a2_) + 2 * math.pi

    dL_ = L2_ - L1_
    dC_ = C2_ - C1_    
    dh_ = h2_ - h1_
    if C1_ * C2_ == 0: dh_ = 0
    elif dh_ > math.pi: dh_ -= 2 * math.pi
    elif dh_ < -math.pi: dh_ += 2 * math.pi        
    dH_ = 2 * math.sqrt(C1_ * C2_) * math.sin(dh_ / 2)
    
    L_ave = (L1_ + L2_) / 2
    C_ave = (C1_ + C2_) / 2
    
    _dh = abs(h1_ - h2_)
    _sh = h1_ + h2_
    C1C2 = C1_ * C2_
    
    if _dh <= math.pi and C1C2 != 0: h_ave = (h1_ + h2_) / 2
    elif _dh  > math.pi and _sh < 2 * math.pi and C1C2 != 0: h_ave = (h1_ + h2_) / 2 + math.pi
    elif _dh  > math.pi and _sh >= 2 * math.pi and C1C2 != 0: h_ave = (h1_ + h2_) / 2 - math.pi 
    else: h_ave = h1_ + h2_
    
    T = 1 - 0.17 * math.cos(h_ave - math.pi / 6) + 0.24 * math.cos(2 * h_ave) + 0.32 * math.cos(3 * h_ave + math.pi / 30) - 0.2 * math.cos(4 * h_ave - 63 * math.pi / 180)
    
    h_ave_deg = h_ave * 180 / math.pi
    if h_ave_deg < 0: h_ave_deg += 360
    elif h_ave_deg > 360: h_ave_deg -= 360
    dTheta = 30 * math.exp(-(((h_ave_deg - 275) / 25)**2))
    
    R_C = 2 * math.sqrt(C_ave**7 / (C_ave**7 + C_25_7))  
    S_C = 1 + 0.045 * C_ave
    S_H = 1 + 0.015 * C_ave * T
    
    Lm50s = (L_ave - 50)**2
    S_L = 1 + 0.015 * Lm50s / math.sqrt(20 + Lm50s)
    R_T = -math.sin(dTheta * math.pi / 90) * R_C

    k_L, k_C, k_H = 1, 1, 1
    
    f_L = dL_ / k_L / S_L
    f_C = dC_ / k_C / S_C
    f_H = dH_ / k_H / S_H
    
    dE_00 = math.sqrt(f_L**2 + f_C**2 + f_H**2 + R_T * f_C * f_H)
    
    return dE_00







def URFDLB_facelets_order(BGR_mean):
    """Order the facelet's colors (BGR values) according to the URFDLB order.
    When a laptop is used, the user is asked to present the cube in a given order, that follows the URFDLB order.
    When the robot is used, faces are detected according to a convenient (robot) order.
    Argument of this function is a list with the BGR mean values of the 54 facelets, following
    the detection order at the robot.
    Return of this function is a dict with the BGR mean values of the 54 facelets, ordered as per URFDLB order."""
    
    if device == 'laptop':                      # case the script is running on a PC/laptop (not the robot)
        URFDLB_facelets_BGR_mean = BGR_mean     # when laptop is used it is assumed the user to follow the suggested (URFDLB) order
        pass
    
    elif device == 'Rpi':                                                # case the script is running at the robot
        robot_facelets_BGR_mean=[]                                       # empty list is generated 
        robot_facelets_BGR_mean = BGR_mean.copy()                        # empty list is populated with a copy of detected BGR mean values
        
        robot_sides_order = ['U', 'B', 'D', 'F', 'R', 'L']               # robot sides order when detecting facelets color
        kociemba_sides_order = ['U', 'R', 'F', 'D', 'L', 'B' ]           # kociemba conventional sides order, for solver
        
        robot_facelets_list = [x for x in range(54)]                     # list of the facelets
        robot_facelets_order = {}                                        # dictiorary of facelets values per side (key)
        for side in robot_sides_order:                                     
            robot_facelets_order[side] = robot_facelets_list[:9]         # dictiorary of facelets values per side (key)
            robot_facelets_list = robot_facelets_list[9:]                # remaining facelets to be assigned to following sides
        for i in range(54-len(BGR_mean)):
            robot_facelets_BGR_mean.append((230,230,230))                # gray facelets are added on faces not yet detected
        
        kociemba_facelets_list=[]                                        # list of facelets collected to be properly filles
        kociemba_facelets_order={}                                       # dictiorary of facelets values per side (key)
        for side in kociemba_sides_order:                                
            kociemba_facelets_order[side] = robot_facelets_order[side]   # dictiorary of facelets values per side (key)
            for facelet in robot_facelets_order[side]:
                kociemba_facelets_list.append(facelet)                   # facelets order as per URFDLB order
        URFDLB_facelets_BGR_mean=[]
        for facelet in kociemba_facelets_list:
            URFDLB_facelets_BGR_mean.append(robot_facelets_BGR_mean[facelet])  #BGR tuples ordered as kociemba facelets order
    
    return URFDLB_facelets_BGR_mean    







def decoration(deco_info):
    """Plot the cube's status made by a collage of images taken along the facelets color detection
    On the collage is also proposed the cube's sketches made with detected and interpreted colors
    This picture collage is saved as image, by adding date and time on the file name as reference."""
    
    fixWindPos = deco_info[0]
    screen = deco_info[1]
    device = deco_info[2]
    frame = deco_info[3]
    background_h = deco_info[4]
    faces = deco_info[5]
    edge = deco_info[6]
    cube_status_string = deco_info[7]
    cube_status = deco_info[8]
    cube_color_sequence = deco_info[9]
    HSV_analysis = deco_info[10]
    URFDLB_facelets_BGR_mean = deco_info[11]
    font = deco_info[12]
    fontScale = deco_info[13]
    lineType = deco_info[14]
    
    detect_winner = deco_info[15]
    show_time = deco_info[16]
    timestamp = deco_info[17]
    robot_stop = deco_info[18]
    
    resume_width = int(14*edge)
    resume_height = 29*edge
    cv2.rectangle(frame, (0, 0), (resume_width, resume_height), (230, 230, 230), -1)    # gray background for the cube sketch
    plot_colors(URFDLB_facelets_BGR_mean, edge, frame, background_h, font, fontScale, lineType)     # plot a cube decoration with detected colors
    plot_interpreted_colors(device, cube_status, cube_color_sequence, cube_status_string, detect_winner,\
                            HSV_analysis, edge, frame, font, fontScale, lineType)       # cube sketch with with (bright) colors interpreted

    # frame slice, with the sketches of detected and interpreted colours
    faces[7] = frame[3*edge:resume_height, :resume_width] 
    
    collage=faces_collage(faces, device)                 # collage function is called

    if device=='Rpi':                                    # case the script is running at the robot
        import os                                        # os is imported to ensure the folder check/make
        folder = os.path.join('.','CubesStatusPictures') # folder to store the collage pictures
        if not os.path.exists(folder):                   # if case the folder does not exist
            os.makedirs(folder)                          # folder is made if it doesn't exist
        fname = folder+'/cube_collage'+timestamp+'.png'  # folder+filename with timestamp for the resume picture
        status=cv2.imwrite(fname, collage)               # cube sketch with detected and interpred colors is saved as image
    
    if screen:                                           # case a screen is connected
        cv2.namedWindow("cube_collage")                  # create the collage window
        cv2.moveWindow("cube_collage", 0,0)              # move the collage window to (0,0)
        cv2.imshow("cube_collage", collage)              # while the robot solves the cube the starting status is shown
        key=cv2.waitKey(int(show_time*1000))             # showtime is trasformed from milliseconds to seconds
        if key == 27 & 0xFF or robot_stop==True:         # ESC button can be used to escape each window
            try: cv2.destroyWindow("cube_collage")       # cube window is closed via esc button          
            except: pass
        
        try: cv2.destroyWindow("cube_collage")           # cube window is closed at function end
        except: pass







def faces_collage(faces, device):
    """This function merges multipe pictures, and it returns a single image.
    The 6 cube faces images, taken while detecting the facelets colors, are used for this collage.
    The Sketch with detected and interpreted cune is also used on this collage.
    The 6 cube faces images are resized to a predefined dimension.
    Gray rectangles are generated and used to complete the picture.
    Once the collage is made, the original dict of images is cleared, to save some memory."""
    
    face_h = faces[1].shape[0]                                   # height of the cube's face1 image, still on memory
    face_h = min(face_h,250)                                     # if the cube's face1 image height if bigger than 250 then 250 is chosen
    for i in range(1,7):                                         # image of all cube faces (1 to 6 )
        faces[i]=cv2.resize(faces[i], (face_h, face_h), interpolation = cv2.INTER_AREA)  # are resized to square of 300 pixels, or face1 height
    
    empty_face = np.zeros([face_h, face_h, 3],dtype=np.uint8)    # empty array having same dimensions of cube's faces images
    empty_face.fill(230)                                         # array is filled with light gray
    
    # faces[7] is a resume image, still on memory, with detected and interpreted facelets colors
    resume_h = faces[7].shape[0]            # width of faces[7]
    resume_w = faces[7].shape[1]            # height of faces[7]
    resume_ratio=resume_w/resume_h          # image ratio (w/h) is calculated, as this image differs from the cube's faces images
    resume_h=3*face_h                       # resume image height, to occupy 3 cube's faces
    resume_w=int(3*face_h*resume_ratio)     # resume image width is calculated from the imposed height, by keeping aspect ratio
    resume_resized = cv2.resize(faces[7], (resume_w, resume_h), interpolation = cv2.INTER_AREA) # resume is resized to occupy a full "column"   
    
    if device=='Rpi':                                          # cube's faces orientation, under Picamera, are different than the conventional URFDLB order
        for face in [1, 3, 4]:                                 # these faces are 180 rotate
            faces[face]=cv2.rotate(faces[face],cv2.ROTATE_180) # these images are then rotated by 180
        for face in [5,6]:                                     # these faces are rotate by 90 CW
            faces[face]=cv2.rotate(faces[face],cv2.ROTATE_90_COUNTERCLOCKWISE) # these images are then rotated by 90 CCW

    
    if device == 'laptop':                  # case the script is running on a PC/laptop (not the robot)
        seq=[1,2,3,4,5,6]                   # faces order suggested to laptop camera follow URFDLB order
    elif device == 'Rpi':                   # case the script is running at the robot
        seq=[1,5,4,3,6,2]                   # faces order at robot is hard coded for movements convenience

    col1=np.vstack([empty_face, faces[seq[4]], empty_face])        # vertical stack of images, to generate 1st column for the pictures collage
    col2=np.vstack([faces[seq[0]], faces[seq[2]], faces[seq[3]]])  # vertical stack of images, to generate 2nd column for the pictures collage
    col3=np.vstack([empty_face, faces[seq[1]], empty_face])        # vertical stack of images, to generate 3rd column for the pictures collage
    col4=np.vstack([empty_face, faces[seq[5]], empty_face])        # vertical stack of images, to generate 4th column for the pictures collage
    
    faces.clear()                                                  # dictionary of images is cleared

    collage = np.hstack([col1,col2,col3,col4,resume_resized])      # horizzontal stack of 5 columns of images, to generate the pictures collage
    collage_ratio = collage.shape[1] / collage.shape[0]            # collage ratio (width/height) is calculated for resizing 
    collage_w=1024                                                 # colleage width is fixed for consistent pictures archiving at Rpi
    collage_h=int(collage_w/collage_ratio)                         # colleage heigth is calculated to maintain proportions
    collage = cv2.resize(collage, (collage_w, collage_h), interpolation = cv2.INTER_AREA) # resized collage  
    
    return collage







def cube_string(cube_status):
    """Generates the cube detected status string, compatible with the solver:
    All uppercase letters indicating the color's initial
    Argument is the cube status generated, whein the values are the facelet colors (full color name)."""
    
    cube_in_letters = {'white':'U', 'red':'R', 'green': 'F', 'yellow':'D', 'orange':'L', 'blue':'B'}  
    string=''
    for color in cube_status.values():         
        string=string+str(cube_in_letters[color])
    return string







def cube_solution(cube_string):
    """Calls the Hegbert Kociemba solver, and returns the solution's moves
    from: https://github.com/hkociemba/RubiksCube-TwophaseSolver 
    (Solve Rubik's Cube in less than 20 moves on average with Python)
    The returned string is slightly manipulated to have the moves amount at the start."""
    
    s = sv.solve(cube_string, 20, 2)  # solves with a maximum of 20 moves and a timeout of 2 seconds for example
    solution = s[:s.find('(')]        # solution capture the sequence of manouvre
    
    # solution_text places the amount of moves first, and the solution (sequence of manouvere) afterward
    solution_Text = s[s.find('(')+1:s.find(')')-1]+' moves  '+ s[:s.find('(')] 
    if solution[:5] =='Error':        # solution could start with "Error" in case of incoherent cusbe string sent to the solver
        solution_Text = 'Error'       # in that case a short error string is returned
    
    return solution, solution_Text







def text_bg(frame, w, h):
    """Generates a black horizontal bandwith at frame top (also bottom when device=='laptop'), as backgroung for the text
    This is usefull when the program runs on a lapton, to provide guidance/feedback text to user
    On the robot, when a screen is connected, it provides feedback info."""
    
    global background_h                                                 # this variable is re-used to reduce the frame to a ROI
    background_h=42                                                     # height of a black bandwidth used as back ground for text
    cv2.rectangle(frame, (0, 0), (w, background_h), (0,0,0), -1)        # black background bandwidth at frame top
    
    if picamera_test or device=='laptop':   # case the script is running on a PC/laptop (not the robot)
        cv2.rectangle(frame, (0, h-background_h), (w, h), (0,0,0), -1)  # black background bandwidth at frame bottom







def text_font():
    """Sets the (main) text parameters, used on CV2."""
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.8
    fontColor = (255,255,255)
    lineType = 2
    return font, fontScale, fontColor, lineType







def robot_facelets_rotation(facelets):
    """Rotates the facelets order, from robot's camera/cube orientation to the kociemba point of view
    This has to do with the way the PiCamera is mounted on the robot, as well as how the faces are presented
    during the cube status reading
    Argument is the dictionary of facelets (key:interpreted color) are retrieved by the robot
    Return is a dictionary of facelets (key:interpreted color) that follows the URFDLB order
    The return allows to have an interpreted cube status as per user point of view."""
    
    if device == 'laptop':    # case the script is running on a PC/laptop (not the robot)
        return                # this function is not executed
    
    elif device == "Rpi":

        # On sides 1, 3, 4, PiCamera reads facelets at 180deg with reference to user point of view
        if side in [1, 3, 4]:                              
            facelets.reverse()   # reversiong the order on the original facelet list solves the problem
        
        
        # On sides 5, 6, PiCamera reads facelets at 90deg ccw with reference to user point of view
        # to solve this, the facelets are re-ordered
        #
        #  from facelets     7  4  1     to   0  1  2
        #                    8  5  2          3  4  5
        #                    9  6  3          6  7  8
        elif side in [5, 6]:
            rot_90_cw = [2, 5, 8, 1, 4, 7, 0, 3, 6]
            facelets[:] = [facelets[i] for i in rot_90_cw]

        # in case the face was rotated 90deg ccw: rot_90_ccw = [6, 3, 0, 7, 4, 1, 8, 5, 2]
        






def average_color(image, x, y):
    """From: https://sighack.com/post/averaging-rgb-colors-the-right-way
     Averages the pixels within a square defined area on an image
     The average is calculated as the square root of the sum of the squares for the BGR colors
     region centered at (x, y) meant as the square center, with 2*side as quare side lenght in pixels.
     The function return a tuple with the averaged BGR colors
     Pixel by pixel iteration doesn't sound efficient, yet for the small aarea I couldn't notice a time increment."""
    
    global edge
    # square edge, used for sketching the cube, is used as (half) side of the square to calculate the averaged color
    
    blue=float(0)
    green=float(0)
    red=float(0)
    
    #Iterate through pixels of a bounding box having 2*edge as square side length in pixels
    for i in range(2*edge):         # foor loop used to iterate the colums on the image square 
        j=i-edge                    # iterator j is "shifted" by half of the square of pixels to analyse
        for i in range(2*edge):     # for loops to iterate trhough the rows of th eimage square
            bgr=image[y+j,x-edge+i]      # gbr of a singele pixel
            b,g,r = bgr                  # bgr components
            b=int(b)                     # from uint8 to integer
            g=int(g)                     # from uint8 to integer
            r=int(r)                     # from uint8 to integer
            
            #Sum the squares of components
            blue=blue+b*b                # progressive sum of the square values for the blue component
            green=green+g*g              # progressive sum of the square values for the green component
            red=red+r*r                  # progressive sum of the square values for the red component
    num=4*edge*edge                      # amount of pixels in the image square under analysis    
    
    # for debug purpose it is drawn the contour of the used area where the facelet's color is averaged 
    if debug and screen:
        tl=(x-edge, y-edge)                  # top left coordinate 
        tr=(x+edge, y-edge)                  # top right coordinate 
        br=(x+edge, y+edge)                  # bottom left coordinate 
        bl=(x-edge, y+edge)                  # bottom left coordinate 
        pts=np.array([tl, tr, br, bl])       # array of coordinates
        contour = [pts]                      # list is made with the array of coordinates
        cv2.drawContours(frame, contour, -1, (230, 230, 230), 2)  # a white polyline is drawn on the contour (2 px thickness)
    
    #Return the sqrt of the mean of squared B, G, and R sums 
    return (int(math.sqrt(blue/num)), int(math.sqrt(green/num)), int(math.sqrt(red/num)))







def read_color(facelets, candidates, BGR_mean, H_mean, scale, wait=20, index=0):
    """Reads the average BGR color on the each facelet of the cube face just detected.
    Draw the contour used on each facelect (eventually the facelet number), to feedback on correct facelet reading/ordering
    Wait is the time (in ms) to keep each facelet visible while the remaining frame is forced black
    The fuction returns (or updates) global variables, like BGR_mean, hsv, hue, s, v, H_mean."""
    
    # this part requires to uncomment some imports at the startup()
#     if device=='laptop' and debug:  # case the script is running on a PC/laptop (not the robot), and debug is set true on _main_
#         img=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)            # frame's image is converted to RGB and plotted via matplotlib 
#         imgplot = plt.imshow(img)                             # frame's image is converted to RGB and plotted via matplotlib 
#         plt.show()                                            # frame's image is converted to RGB and plotted via matplotlib 
    
    for facelet in facelets:                                  # iteration over the 9 facelets just detedcted
        contour = facelet.get('contour')                      # contour of the facelet under analysis
        candidates.append(contour)                            # new contour is added to the candidates list
        mask = np.zeros(frame.shape[:2], dtype="uint8")       # mask of zeros is made for the frame shape dimension
        cv2.drawContours(mask, [contour], -1, 255, -1)        # mask is applied to vsualize one facelet at the time
        
        if device=='laptop':                                  # case the script is running on a PC/laptop (not the robot)
            roi = cv2.bitwise_and(frame, frame, mask=mask)    # ROI is used to shortly display one facelet at the time
        
        cm_point=facelet['cx'],facelet['cy']                  # contour center coordinate
        bgr_mean_sq = average_color(frame, facelet['cx'], facelet['cy'])  # color is averaged with sqr sum os squares
        BGR_mean.append(bgr_mean_sq)                          # Initially used a simpler mean to average the facelet color
        b,g,r = bgr_mean_sq                                   # BGR (avg) components are retrieved
        BGR_mean_sq = np.array([[[b,g,r]]], dtype=np.uint8)   # BGR are positioned in cv2 array form
        hsv = cv2.cvtColor( BGR_mean_sq, cv2.COLOR_BGR2HSV)   # HSV color space equilavent values, for the average facelet color
        hue, s, v = cv2.split(hsv)                            # HSV components are retrieved
        H_mean.append((hue[0][0]))                            # the (avg) Hue value is stored on a list
        
        if device=='laptop':                     # case the script is running on a PC/laptop (not the robot)
            if cv_wow:                           # case cv_wow variable is set true on __main__
                cv2.imshow("Cube", roi)          # ROI is shortly display one facelet at the time
                cv2.waitKey(200)                 # this waiting time is meant as decoration to see each facelet being detected
            
            elif not cv_wow:                     # case cv_wow variable is set false on __main__
                if fixWindPos:                   # case the fixWindPos variable is set true on __main__ 
                    cv2.namedWindow("cube")      # create the cube window
                    cv2.moveWindow("cube", 0,0)  # move the window to (0,0)
                cv2.imshow("cube", roi)          # ROI is shortly display one facelet at the time
                cv2.waitKey(wait)                # this waiting time is meant as decoration to see each facelet being detected
            
            facelet_num = str(index+1)                   # string of the facelet number within the face (1 to 9)
            x = int(facelet.get('cx'))-int(12*scale/100) # x coordinate to add the facelet number text on the facelet
            y = int(facelet.get('cy'))+int(6*scale/100)  # y coordinate to add the facelet number text on the facelet
            font_size = fontScale*scale/100              # font size is adjusted according to the frame resizing factor
            
            # a progressive facelet numer, 1 to 9, is placed over the facelets
            cv2.putText(frame, facelet_num, (x, y), font, font_size, (0,0,0), lineType)
        index+=1
    







def rotate_image(image, image_center, angle):
    """This function rotates an image of a given angle with reference to the given center.
    This function is used to align the cropped cube face image, before cropping it.
    This function is only called when the laptop is used (cube is manually presented to the laptop camera).
    Overall scope of this funtion is decorative, to generate an unfoalded cube collage of images."""
    
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    rotated_img = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return rotated_img





def face_image(frame, facelets, side, faces):
    """Slice a frame rectangular portion to temporary store the cube image.
    On both laptop and robot the cube is initialy cropped from the frame.
    1st vertex of Facelets 0, and 3rd vertedx of facelet 8, are used as reference for the cubre cropping from the frame
    On laptop version the cube is first rotated to be aligned with the horizon
    When robot is used, the facelets are first re-ordered from top left
    The function returns a dictionary with the (cropped) images of the 6 cube faces
    This function enables the generation of a cube images collage to be plotted (and saved on Rpi), for decoration purpose
        
    A         B 
      0  1  2
      3  4  5 
      5  7  8  
    D         C     """
    
    
    if device == 'laptop':                                   # case the script is running on a PC/laptop (not the robot)
        global frame_width
        
        ##################################################
        # cube is rotated for a better (visual) cropping #
        ##################################################
        avgAx=0; avgAy=0; avgBx=0; avgBy=0                   # variables for facelets contour's average centers 
#         print(len(facelets))
#         for i in range (len(facelets)):
#             print(int(facelets[i].get('cx')), int(facelets[i].get('cy')))
                      
        for i in [0,3,6]:                                    # iteration on the left columns facelets
            avgAx=avgAx+int(facelets[i].get('cx'))           # sum cx coordinate for "i" facelet (left column)
            avgAy=avgAy+int(facelets[i].get('cy'))           # sum cy coordinate for "i" facelet (left column)
            i=i+2                                            # index increment to the right columns facelets
            avgBx=avgBx+int(facelets[i].get('cx'))           # sum cx coordinate for "i" facelet (right column)
            avgBy=avgBy+int(facelets[i].get('cy'))           # sum cy coordinate for "i" facelet (right column)

        p=(avgBx//3-avgAx//3, avgBy//3-avgAy//3)             # average (delta x, delta y) for facelets on right column  
        ang = np.arctan2(*p[::-1])                           # angle (in radians) of the cube
        angle = np.rad2deg(ang%(2*np.pi))                    # angle (in degrees) of the cube
        Ax = int(facelets[0].get('cont_ordered')[0][0])      # x coordinate for the top-left vertex 1st facelet
        Ay = int(facelets[0].get('cont_ordered')[0][1])      # y coordinate for the top-left vertex 1st facelet
        center=(Ax,Ay)  
        
        frame=rotate_image(frame, center, angle)             # frame is rotated, for a better cropping & view
        
        
        #################
        # cube cropping #
        #################
        avgAx=0; avgAy=0; avgBx=0; avgBy=0
        cube_width=0
        for i in [0,3,6]:
            avgAx = int(facelets[i].get('cont_ordered')[0][0])    # avg x coordinate for the top-left vertex "i" facelet
            avgAy = int(facelets[i].get('cont_ordered')[1][0])    # avg y coordinate for the top-left vertex "i" facelet
            i=i+2
            avgBx = int(facelets[i].get('cont_ordered')[0][0])    # avg x coordinate for the top-left vertex "i" facelet
            avgBy = int(facelets[i].get('cont_ordered')[1][0])    # avg y coordinate for the top-left vertex "i" facelet
            cube_width = cube_width+int(math.sqrt((avgBy-avgAy)**2+(avgBx-avgAx)**2))
        
        cube_width = cube_width//3
        margin = int(0.1*cube_width)                        # 10% of cube width is used as crop's margin     
        
        if Ax >= margin:
            Ax = Ax-margin    # shifted coordinate
        if Ay >= margin:
            Ay = Ay-margin    # shifted coordinate
        Cx=Ax+cube_width+margin
        Cy=Ay+cube_width+margin

        
        # text over the cube face's images, mostly for debug purpose
#         text_x = Ax + int(0.2*(Cx-Ax))       # X coordinate for the text starting location
#         text_y = int((Ay+Cy)/2)              # Y coordinate for the text starting location
#         fontscale_coef = (Cx-Ax)/150         # coefficient to adapt the text size to almost fit the cube
#         cv2.putText(frame, str(f'Side {sides[side]}'), (text_x, text_y), font, fontScale*fontscale_coef, fontColor,lineType)
        
        faces[side] = frame[Ay:Cy, Ax:Cx]    # sliced image of the cube
        
        if side == 1:
            frame_width = cube_width+2*margin
        elif side>1:
            faces[side] = cv2.resize(faces[side], (frame_width, frame_width), interpolation=cv2.INTER_LINEAR)
        if not cv_wow:
            cv2.imshow('cube', faces[side])
            
        return faces


    
    ###############################################################################################
    # quite different approach for the robot, as the cube is always well oriented toward the camera
    ###############################################################################################
    elif device == "Rpi": 
        if side in [1, 3, 4]:
            facelets.reverse()                               # facelets are ordered to simply determine the "vertex" including the cube
        elif side in [5, 6]:
            rot_90_ccw = [6, 3, 0, 7, 4, 1, 8, 5, 2]         # facelets are ordered to simply determine the "vertex" including the cube
            facelets[:] = [facelets[i] for i in rot_90_ccw]
    
        Ax = int(facelets[0].get('cont_ordered')[0][0])      # x coordinate for the top-left vertex 1st facelet
        Ay = int(facelets[0].get('cont_ordered')[0][1])      # y coordinate for the top-left vertex 1st facelet
        Cx = int(facelets[8].get('cont_ordered')[2][0])      # x coordinate for the bottom-right vertex 9th facelet
        Cy = int(facelets[8].get('cont_ordered')[2][1])      # y coordinate for the bottom-right vertex 9th facelet
        diagonal = int(math.sqrt((Cy-Ay)**2+(Cx-Ax)**2))     # cube diagonal length
        
        robot_facelets_rotation(facelets)                    # facelets are rotated to Kociemba reltaed order
 
        margin = int(0.1*diagonal)                           # 10% of cube diagonal is used as crop margin
        if Ax >= margin:   Ax = Ax-margin    # shifted coordinate
        elif Ax >= margin: Ax = 0
        if Ay >= margin:   Ay = Ay-margin    # shifted coordinate
        elif Ay >= margin: Ay = 0
        if Cx >= margin:   Cx = Cx+margin    # shifted coordinate
        elif Cx >= margin: Cx = 0
        if Cy >= margin:   Cy = Cy+margin    # shifted coordinate
        elif Cy >= margin: Cy = 0

        
        # text over the cube face's images, mostly for debug purpose
#         text_x = Ax + int(0.2*(Cx-Ax))       # X coordinate for the text starting location
#         text_y = int((Ay+Cy)/2)              # Y coordinate for the text starting location
#         fontscale_coef = (Cx-Ax)/150         # coefficient to adapt the text size to almost fit the cube
#         cv2.putText(frame, str(f'Side {sides[side]}'), (text_x, text_y), font, fontScale*fontscale_coef, fontColor,lineType)
        faces[side] = frame[Ay:Cy, Ax:Cx]    # sliced image of just the cube
#         if screen:                           # case a screen is connected
#             cv2.imshow('cube', faces[side])

        return faces




def robot_next_side(side):
    """Cube movements at robot, during the cube reading phase.
    Cube is flipped 4 times to read the first 4 faces, then some spins and flippings are required to read the
    remaining 2 faces.
    Top cover remains open, so the picamera is at sufficient distance from the cube's side to be read
    After the 6th face is read, the cube is positioned back to its initial position, called home.
    Home position is the baseline condition to apply the Kociemba solution."""

    global robot_stop
    
    if device == 'laptop':                           # case the script is running on a PC/laptop (not the robot)
        return                                       # this function is not executed
    
    elif device == 'Rpi':                            # case the script is running at the robot
        aligned = True                               # aligned variable is set initially True
#         align = 'read'                               # in case a motor alinment is needed the cover will be positioned to (picamera) read position
        if side in range(4):                         # first 4 sides (3 sides apart the one already in front of the camera)
            if led_usage:                            # case the leds at Top_cover are activated
                servo.led_off()                      # led on top_cover is switched off, for power management
                servo.servo_freq(debug)              # set 60Hz frequency for servo control
            aligned = servo.flip(1, debug, 'read', not robot_stop, fast=False)      # side is reached by simply flipping the cube
#             print('moving to side', side, 'and alignment is', aligned)
            if led_usage:                            # case the leds at Top_cover are activated
                time.sleep(0.1)                      # little delay to ensure servos have reached their target position and the innertia has fully dropped        
                servo.led_freq(debug)                # set high frequency to PCA 9685 board, to prevent visible flickering from the led
                servo.led_cover(cam_led_bright)      # led on top_cover is switched on
                time.sleep(0.2)                      # small (arbitrary) delay to let the led and PiCamera) to stabilize
        
        if side == 4 :                               # at side 4 is needed to change approach to show side 5 to the camera
            if led_usage:                            # case the leds at Top_cover are activated
                servo.led_off()                      # led on top_cover is switched off, for power managements
            if robot_stop == False:                  # case stop_button has not being pressed
                if led_usage:                        # case the leds at Top_cover are activated
                    servo.servo_freq(debug)          # set 60Hz frequency for servo control
                servo.spin(1, debug)                 # one spin (90 deg) is applied to the cube
                aligned = servo.flip(1, debug, 'read', not robot_stop, fast=False)  # one flip is applied to the cube to show the side 5 to the camera
#                 print('moving to side', side, 'and alignment is', aligned)
                if led_usage:                        # case the leds at Top_cover are activated
                    time.sleep(0.1)                  # little delay to ensure servos have reached their target position and the innertia has fully dropped
                    servo.led_freq(debug)            # set high frequency to PCA 9685 board, to prevent visible flickering from the led
                    servo.led_cover(cam_led_bright)  # led on top_cover is switched on
                    time.sleep(0.2)                  # small (arbitrary) delay to let the led and PiCamera) to stabilize
                
        if side == 5:                                # when side5 is in front of the camera 
            if led_usage:                            # case the leds at Top_cover are activated
                servo.led_off()                      # led on top_cover is switched off, for power management             
                servo.servo_freq(debug)              # set 60Hz frequency for servo control
            aligned = servo.flip(2, debug, 'read', not robot_stop, fast=False)      # at side 5 are needed two flips to show side 6 to the camera
            if led_usage:                            # case the leds at Top_cover are activated
                time.sleep(0.1)                      # little delay to ensure servos have reached their target position and the innertia has fully dropped        
                servo.led_freq(debug)                # set high frequency to PCA 9685 board, to prevent visible flickering from the led
                servo.led_cover(cam_led_bright)      # led on top_cover is switched on
                time.sleep(0.2)                      # small (arbitrary) delay to let the led and PiCamera) to stabilize
#             print('moving to side', side, 'and alignment is', aligned)


        elif side == 6:                              # when side6 is in front of the camera 
            if led_usage:                            # case the leds at Top_cover are activated
                servo.led_off()                      # led on top_cover is switched off, for power management             
                servo.servo_freq(debug)              # set 60Hz frequency for servo control
            if robot_stop == False:                  # case stop_button has not being pressed 
                servo.open_cover()                   # top cover is moved from reading to open position
        
        if not aligned:
            print("\nCannot align the motor (cube_holder), script is terminated")
            quit_func()                              # script is closed, in case of irresponsive camera
        
        if robot_stop == True:                       # case stop_button has being pressed
            return







def robot_move_cube(robot_moves, total_robot_moves, solution_Text):
    """This fuction provide the robot the sequence of movements: It drives the robot movable parts according
        to the robot movements sequence (dictionary); This includes
            Spin (= cube rotation over the laying face, to change its orientation).
            Flip (= cube changing the face where it lays on).
            Rotate (= rotating the first layer on bottom with referent to the other two layers above).
    
    Argument of this function are:
        robot_moves, a dictionary with the calculated movements for the robot based on the kociemba solution
        total_robot_moves value, used to visualize on display a robot moves count-down
        solution_Text, used to detect error cases on the Kociemba solution."""
    
    if device == 'laptop':                   # case the script is running on a PC/laptop (not the robot)
        return                               # this function is not executed

    
    elif device == 'Rpi':                    # case the script is running at the robot
        
        tot_time_secs = 0
        elapsed_time_robot = 0
        
        start_robot_time = time.time()       # this time is used as reference to measure (and visualize) how long the robot takes to solve the cube
        remaining_moves = total_robot_moves  # remaining movements are visualized, to manage expectations while in front of the robot
        aligned = True                       # aligned variable is set initially true
        
        i = 0
        print()
        for move in robot_moves.values():                               # iterates over the dictionary values (the keys is the amounto of Kociemba moves)
            if robot_stop==False:                                       # case stop_button has not being pressed
                print(f'Cube move: {i+1}\tRobot moves: {move}') 

                spins = int(move[move.find('S')+1:move.find('F')])      # amount (and direction) of spins is retrieved
                flips = int(move[move.find('F')+1:move.find('R')])      # amount of flips is retrieved (flips are only positive)
                rotations = int(move[move.find('R')+1:])                # amount (and direction) of rotations is retrieved

                if i < len(robot_moves)-1:                              # checks the spins at next robot move
                    next_move = robot_moves[i+1]                        # checks the spins at next robot move
                    next_spin = int(next_move[next_move.find('S')+1:next_move.find('F')]) # amount (and direction) of spins at next robot move
                elif i==len(robot_moves)-1:                             # last robot move
                    next_spin = 0                                       # last robot move isn't followed by an immediate spin
                    # print(f'Robot move {i}, Spin(s) at next move: {next_spin}')

                if spins!=0:
                    if robot_stop==False:                                    # case stop_button has not being pressed
                        servo.spin(spins, debug)                             # servo package is called for the required spins
                        remaining_moves = remaining_moves - 1                # remaining robot moves is updated (multiple rotations are considered as 1)
                robot_show_remaining_moves(remaining_moves)                  # remaining robot moves is sent to the function to display it
                
                if flips!=0 :
                    if rotations !=0:
                        if robot_stop==False:                                # case stop_button has not being pressed
                            for flip in range(flips):
                                aligned = servo.flip(1, debug, 'open', not robot_stop) # servo package is called for the required flips
                                remaining_moves = remaining_moves - 1        # remaining robot moves is updated (each flip is counted)
                    elif rotations ==0:
                        if robot_stop==False:                                # case stop_button has not being pressed
                            for flip in range(flips):
                                aligned = servo.flip(1, debug, 'open', not robot_stop) # servo package is called for the required flips
                                remaining_moves = remaining_moves - 1        # remaining robot moves is updated (each flip is counted)
                robot_show_remaining_moves(remaining_moves)                  # remaining robot moves is sent to the function to display it
            
            
                if rotations !=0:
                    if next_spin==0:                             # case when the next robot move has no spin
                        if robot_stop==False:                    # case stop_button has not being pressed
                            servo.rotate(rotations, debug)       # the rotation ends with less sleep, allowing flipping while the vore is still opening
                    elif next_spin!=0:    
                        if robot_stop==False:                    # case stop_button has not being pressed
                            servo.rotate(rotations, debug)       # servo package is called for the required rotations (lower cube's layer)
                    remaining_moves = remaining_moves - 1        # remaining robot moves is updated (multiple rotations are considered as 1)
                robot_show_remaining_moves(remaining_moves)      # remaining robot moves is sent to the function to display it
                
                if not aligned:
                    print("\nCannot align the motor (cube_holder), script is terminated")
                    quit_func()                    # script is closed, in case of irresponsive camera
                
            else:
                break
            i+=1
            
            
        if solution_Text == 'Error':      # if there is an error (tipicallya bad color reading, leading to wrong amount of facelets per color)                                      
            print('An error occured')     # error feedback is print at terminal
            tot_time_secs = 0             # total time is set to zero to underpin the error
            elapsed_time_robot = 0        # elapsed_time_robot is set to zero to underpin the error
        
        elif solution_Text != 'Error' and robot_stop==False and not cube_scrambling:   # if there are not error on the cube solution
            solved, tot_time_secs, elapsed_time_robot = robot_time_to_solution(start_time, start_robot_time, total_robot_moves)  # cube solved function is called
        
        else:
            tot_time_secs = 0             # total time is set to zero to underpin the error
            
        return tot_time_secs, elapsed_time_robot







def robot_show_remaining_moves(moves):
    """Amount of robot remaining movements, to solve the cube.
    The result is visualized on display2, just to manage user expectations in front of the robot."""
    robot_display2.Clear()
 
    h, d = divmod(moves, 100)  # hundred and decades
    d, u = divmod(d, 10)       # decades and units
    to_show = [36, h, d, u]    # first display's digit is left off
    robot_display2.Show(to_show)     # data is sent to display, twice
    robot_display2.Show(to_show)     # data is sent to display, twice







def window_for_cube_rotation(w, h, side, frame):
    """Window on monitor to show the cube rotation (from a side to the following one).
    During these phases the Vision part doesnt search for contours.
    This function is also used to increment the cube side variable to the next face (side), while preventing to re-capture
    aleady detected facelets one more time, by simply re-freshing the frame with a new camera read."""
    
    side+=1                                                # cube side is increased
    
    if device == 'laptop':                                 # case the script is running on a PC/laptop (not the robot)
        print(f'Rotate the cube to side {sides[side]}')
        time.sleep(0.6)                                    # freeze the program while the user realizes the cube has been fully read
        if cv_wow and side>=1:
            time.sleep(2)
       
    elif device=='Rpi':                                    # case the script is running at the robot
        if robot_stop==False:                              # case stop_button has not being pressed
            print(f'Cube reading: side {sides[side]}')     # cube side is increased
    
    # a camera reading now prevents from re-using the previous frame, therefore from re-capturing the same facelets twice
    frame, w, h, scale  = read_camera()
    
#     if screen:                                             # case a screen is connected
#         if fixWindPos:                                     # case the fixWindPos variable is set true on __main__ 
#             cv2.namedWindow("cube")                        # create the cube window
#             cv2.moveWindow("cube", 0,0)                    # move the window to (0,0)
#         cv2.imshow("cube", frame)                          # frame is showed to viewer
    
    return side                                            # return the new cube side to be anayzed






def window_for_cube_solving(solution_Text, w, h, side, frame, scale):
    """When the laptop is used, this function keeps the webcam active and a window on monitor.
    The window allows the user to see the Kociemba solution on screen, and to keep life the cube resolution.
    When this function is active, facelets contours aren't searched ! """
    
    if device == 'Rpi':             # case the script is running at the robot
        # I've to still check how to visualize use the picamera on separated thread
        return                      # this funcion is not executed
    
    elif device == 'laptop':        # case the script is running on a PC/laptop (not the robot)
        new_cycle=False             # False is assigned to local variable new_cycle
        
        while quitting == False:                 # case the global quitting variable is false
            frame, w, h, scale=read_camera()     # video stream and frame dimensions
            if cv_wow:                           # case cv_wow variable is set true on __main__
                frame_copy = frame.copy()        # copy of the frame is assigned, to be used without editing
            text_bg(frame, w, h)                 # generates a rectangle as backgroung for text in Frame

            if solution_Text != 'Error':         # case when no error retrieved from the Kociemba solver
                if solution_Text == '0 moves  ': # case when no cube movements are needed
                    font_size = fontScale*scale/100 # font size is set according to the window resizing factor
                    cv2.putText(frame, str(f'THE CUBE IS SOLVED'), (10,30), font, font_size, fontColor,lineType)
                else:                            # case when at cube movements are needed               
                    font_size = fontScale*scale/100  # font size is set according to the window resizing factor
                    chars = len(solution_Text)   # characters on the solution text string
                    if chars > 36:               # case there are more than 36 characters
                        font_size = font_size*(1.48-0.0132*chars) # smaller font is set, to accomodate up to 21 movements
                    cv2.putText(frame, str(f'{solution_Text}'), (10,30), font, font_size, fontColor,lineType)
            
            elif solution_Text == 'Error':       # case when error is retrieved from the Kociemba solver                  
                error_msg = 'Error: Incoherent cube detection' # error message
                font_size = 1.2*fontScale*scale/100            # font size is set according to the window resizing factor
                cv2.putText(frame, str(f'{error_msg}'), (10,30), font, font_size, fontColor, lineType)
                
            font_size = 1.2*fontScale*scale/100                # font size is set according to the window resizing factor
            cv2.putText(frame, str('ESC to escape, spacebar to proceed'), (10, int(h-12)), font, font_size, fontColor, lineType)
            
            if cv_wow:                               # case cv_wow variable is set true on __main__ 
                cv2.namedWindow("camera")            # create the camera window
                cv2.moveWindow("camera", 0, gap_h)   # move the camera window to (0, gap_h)
                cv2.imshow("camera", frame_copy)     # shows the frame copy, meaning the frame without contours and other additions 
                key=cv2.waitKey(1)                   # refresh time is minimized (1ms), yet real time is much higher
                if key == 32 & 0xFF:                 # case spacebar is pressed
                    new_cycle=True                   # true is assigned to local variable new_cycle 
                if cv2.getWindowProperty("camera", cv2.WND_PROP_VISIBLE) <1 or (key == 27 & 0xFF): # X on top bar or ESC button
                    quit_func()                      # quitting function
                
                cv2.namedWindow('Cube')              # create the Cube window
                cv2.moveWindow('Cube', 0, h+2*gap_h) # move the Cube window to (0, 2xgap_h)
                cv2.imshow('Cube', frame)            # shows the frame
                key=cv2.waitKey(10)                  # refresh time is 10ms, yet real time is much higher
                if key == 32 & 0xFF:                 # case spacebar is pressed
                    new_cycle=True                   # true is assigned to local variable new_cycle
                if cv2.getWindowProperty("Cube", cv2.WND_PROP_VISIBLE) <1 or (key == 27 & 0xFF): # X on top bar or ESC button
                    quit_func()                      # quitting function
            
            elif not cv_wow:                         # case cv_wow variable is set false on __main__
                if fixWindPos:                       # case the fixWindPos variable is set true on __main__ 
                    cv2.namedWindow('cube')          # create the cube window
                    cv2.moveWindow('cube', 0,0)      # move the window to (0,0)
                cv2.imshow('cube', frame)            # shows the frame 
                key=cv2.waitKey(10)                  # refresh time is 10ms, yet real time is much higher
                if key == 32 & 0xFF:                 # case spacebar is pressed
                    new_cycle=True                   # true is assigned to local variable new_cycle             
                if cv2.getWindowProperty("cube", cv2.WND_PROP_VISIBLE) <1 or (key == 27 & 0xFF): # X on top bar or ESC button
                    quit_func()                      # quitting function

                
            if new_cycle:
                print("\n=========================================================================================\n")
                clear_terminal(device)  # cleares the terminal
                side=0                  # cube side is set to zero to start a new cycle
                BGR_mean.clear()        # empties the dict previoously filled with 54 facelets colors
                H_mean.clear()          # empties the dict previoously filled with 54 facelets Hue
                return side             # return the new cube side (zero) to be anayzed







def robot_time_to_solution(start_time, start_robot_time, total_robot_moves):
    """Calculates the time the robot takes to read and solve the cube.
    Converts the time from milliseconds to mm:ss
    Prints the total time, and the time used to manouvre the cube, to the solution
    Returns also a (global variable) boolean that the cube is solved, differently this function isn't called."""
    
    elapsed_time = round(time.time() - start_time,1)           # total elapsed time
    elapsed_time_robot = round(time.time() - start_robot_time,1)  # time elapsed for the robot solving the cube, from the solver answers
    
    elapsed_time_int = int(round(elapsed_time,0))              # total elapsed time as integer
    elapsed_time_robot_int = int(round(elapsed_time_robot,0))  # time elapsed to integer for visualization at display
    m_tot, s_tot = divmod(elapsed_time_int, 60)                # minutes and seconds for total cube solving time
    m_rob, s_rob = divmod(elapsed_time_robot_int, 60)          # minutes and seconds for total cube solving time
    time_values = [m_tot, s_tot, m_rob, s_rob]                 # list is generated for easy c
    
    # preparing the time in strings, according to amount of digits, for an easier to read print
    time_str = []
    for i in time_values:
        if i < 10:
            i = '0'+ str(i)
            time_str.append(i)
        else:
            time_str.append(str(i))
            
    if total_robot_moves >0 : 
        tot_time= str(time_str[0]+":"+time_str[1])
        move_time = str(time_str[2]+":"+time_str[3])
        print(f'\nCube solved in: {elapsed_time} (wherein {elapsed_time_robot} for robot moves)')
    
    elif total_robot_moves == 0:
        print(f'\nCube was already solved, status read in: {time_str[0]}:{time_str[1]}')
    
    # after printing the time to solve the cube, the robnot is considered as solved
    solved = True                                 
    return solved, elapsed_time, elapsed_time_robot







def scrambling_cube():
    """function to scramble the cube via the robot.
        The function first generate a random cube status, via a function available form the Kociemba solver package.
        After, the robot generates the moves to solve that specific cube status.
        In case of --timer argument, a timer is visualized after the scrambling function."""
    
    import Cubotone_moves as rm     # RobotMoves: convert Kociemba solver solution in robot movements sequence

    cc = cubie.CubieCube()          # cube in cubie reppresentation
    cc.randomize()                  # randomized cube in cubie reppresentation 
    random_cube_string = str(cc.to_facelet_cube())   # randomized cube in facelets string reppresentation
    print("\n\n\n\nRandom cube status:", random_cube_string) # feedback is printed to the terminal
    solution, solution_Text = cube_solution(random_cube_string) # Kociemba solver is called to have the "scrambling" solution string
    print(solution_Text)        # feedback is printed to the terminal
    
    # dict and string with robot movements, and total movements
    robot_moves, total_robot_moves = rm.robot_moves(solution, solution_Text)  # dict with robot movements, and total movements
    print(f'\nTotal robot movements: {total_robot_moves}')  # nice information to print at terminal, sometime useful to copy
    
    if robot_stop==False:
        robot_running = True                  # robot flag when servos and motor are activated
        tot_time_secs, _ =robot_move_cube(robot_moves, total_robot_moves, solution_Text)   # movements to the robot are applied
        robot_time_display.cancel()           # after the robot has made the last (solving) move the countdown time is cancelled            
    else:
        tot_time_sec = 0                      # robot solution time is forced to zero when the solving i interrupted by the stop button
        
    cube_scrambled = True
    return cube_scrambled







def clear_terminal(device):
    """Removes all the text from the terminal and positions the cursors on top left."""
    
    if device == 'laptop':                      # case the script is running on a PC/laptop (not the robot)
        import subprocess, platform
        if platform.system()=="Windows":
            try:
                subprocess.Popen("cls", shell=True).communicate() 
            except:
                pass
            try:
                print("\x1b[H\x1b[2J")  # escape sequence
#                 clear_output()
            except:
                pass
        
        else: #Linux and Mac
            print("\033c", end="")
    
    elif device == 'Rpi':                       # case the script is running at the robot
            if check_screen_presence(debug, printout=False):
                print("\x1b[H\x1b[2J")







def quit_func():
    """Quitting function, that properly closes stuff:
            Camera is closed
            Open cv windows are closed
            At robot prints a sentence on screen indicating the quit funcion has been called
            At robot the servos and motors are positioned to their start position, and shortly after de-energized
            At the robot all the displays segments are shortly powered ON, before powering them OFF
            Eventual opened threads are closed."""

    
    if device == 'laptop':                       # case the script is running on a PC/laptop (not the robot)
        try: cv2.destroyAllWindows()             # all cv2 windows are removed
        except: pass
        close_camera(device)                     # webcam is closed
        pass
    

    elif device == 'Rpi': # case the script is running at the robot, and quitting is not already happening
        global quitting, robot_stop, robot_running, deco
        
        if not quitting:
            print('\nRequested to quit: Cubotone.py\n')
            quitting = True                      # flag when quitting process (i.e. to prevent further camera reading while quitting)
        
        robot_stop=True                          # flag to stops robot movements is set 
        robot_running=False                      # flag "robot not moving" is set

        
        try:
            robot_load_status.cancel()           # reset the "loading" status at robot_display1
        except:
            if debug:
                print(f'\nIssue on closing robot_load_status thread')
            pass
            
        try:
            robot_time_display.cancel()          # de-activates visualization of elapsed time on robot_display1
        except:
            if debug:
                print(f'\nIssue on closing robot_time_display thread')
            pass
        
        try:
            robot_reading_status_display.cancel()   # de-activates visualization of cube reading status at robot_display2
        except:
            if debug:
                print(f'\nIssue on closing robot_reading_status_display thread')
            pass
        
        try:
            robot_done_status_display.cancel()   # de-activates visualization of cube done on robot_display2
        except:
            if debug:
                print(f'Issue on closing robot_done_status_display thread')
            pass
            
        try:
            robot_show_press.cancel()            # de-activates the press feedback on robot_display2
        except:
            if debug:
                print(f'\nIssue on closing robot_press_feedback thread')
            pass
        
        
        if not motors_hw:                        # case motors_hw is set False
            try:
                servo.servo_freq(debug)          # set 60Hz frequency for servo control
            except:
                pass
            
        if screen:                               # case a screen is connected
            try:
                deco.terminate()                 # de-activates the decoration (picture collage of cube side)
            except:
                pass
        

            try:
                cv2.destroyAllWindows()
            except:
                pass
        

        exiting = [8, 8, 8, 8]                   # all segments at the robot's displays
        robot_display1.Show(exiting)             # light all the segments at the robot's robot_display1              
        robot_display2.Show(exiting)             # light all the segments at the robot's robot_display2

        
        for i in range(2):
            if i == 0:                           # first iteration
                print_out=debug                  # prints are set as per debug
            else:                                # from the second iteration onward
                print_out=False                  # prints are set false
            
            try:
                servo.led_off()                  # switches the top-cover led OFF
                servo.read_cover()               # top cover is opened
                time.sleep(0.5)                  # little time to ensure the top cover being out from the cube
                servo.servo_start_positions(print_out)  # top cover and cube lifter set to start position
                time.sleep(1)                    # time to ensure servos are at their start positions
                servo.servo_off(print_out)       # de-energizes the servos
                servo.motor_off(print_out)       # de-energizes the motor
                time.sleep(0.5)                  # time to ensure servos are at their start positions
                servo.servo_off(print_out)       # de-energizes the servos
                
            except:
                if debug:
                    print(f'\nIssue on rotating servos to initial position')
        
        try:
            close_camera('device')               # closes the camnera object (should be the latest command, as per close camera)
        except:
            if debug:
                print('\nIssues at camera closing while quitting')
            pass
        
        if debug:
            print('\n\n\n')
        
        try:
            robot_clear_displays()               # displays at robot are cleared (before "cleaning" GPIO!!! )
            robot_clear_displays()               # displays at robot are cleared (before "cleaning" GPIO!!! )
        except:
            pass
        
        exit()







def camera_opened_check(device):
    """Verifies if the camera is opened (if it provides a feedback). Funtion returns a boolean."""
    
    if device == 'laptop':                      # case the script is running on a PC/laptop (not the robot)
        return camera.isOpened()                # checks if webcam is responsive or not
    
    elif device == 'Rpi':                       # case the script is running at the robot
        try:
            binning = camera.sensor_mode        # PiCamera sensor_mode is checked
            if binning >=0:
                return True
        except:
            return False







def close_camera(device):
    """Closes the camera object.
        It's important to close the camera, if the script runs again
    
        On PiCamera it's importan to close it, at the end of a cube solving cycle to drop the AWB
        and Exposition setting used before."""
    
    if device == 'laptop':                  # case the script is running on a PC/laptop (not the robot)
        try:
            camera.release()                # if the program gets stuk it's because the camera remained open from previour run
        except:
            pass
    
    elif device == 'Rpi':                   # case the script is running at the robot
        try:
            camera.close()                  # necessary to close the camera to release the fix gains (analog/digital) settings
            if debug:
                print(f'\nClosed {device} camera')
        except:
            pass







def cpu_temp():
    """Funtion to read/print CPU temperature at Raspberry pi.
    This gives an idea about the temperature into the robot case."""
    
    if device == 'laptop':        # case the script is running on a PC/laptop (not the robot)
        return                    # this fuction is not executed 
    
    elif device == 'Rpi':         # case the script is running at the robot
        try:
            tFile = open('/sys/class/thermal/thermal_zone0/temp')
            cpu_temp = round(float(tFile.read()) /1000, 1)

        except:
            tFile.close()
        print(f'\nRpi CPU temp: {cpu_temp} degrees C\n')







def hd_check():
    """Extremely trivial check if the code is running on Rpi, if not then it is on laptop.
    Initial aim of this function was to make it easier the debug on the robot, while the script was already working on the laptop;
    By using this function I could proceed with the robot by working out on the same spript, instead of re-typing or copying.
    Afterward I realized that was nice to keep improving on both, therefore with the aim to have a single script, working on both."""
    
    try:
        with open('/sys/firmware/devicetree/base/model') as model:
            RPi_model = model.read()
            if RPi_model[:12] == 'Raspberry Pi':
                print(f'\nUsing Raspberry Pi device\n')              # print to terminal, that confirms the used device
                print(f'Python version: {sys.version}\n')            # print to termina the python version
                print(f'CV2 version: {cv2.__version__}\n')           # print to terminal the cv2 version
                return 'Rpi'                                         # 'Rpi'string is returned
    
    except Exception:
        import platform
        if platform.system() == "Windows":
            print(f'\nUsing laptop device\n')                        # print to terminal, that confirms the used device
            print(f'Python version: {sys.version}\n')                # print to termina the python version
            print(f'CV2 version: {cv2.__version__}\n')               # print to terminal the cv2 version
        return 'laptop'                                              # 'laptop' string is returned







def robot_pressed_button(new_cycle_button):
    """Function that initially was measuring how long the NEW CYCLE is pressed, by returning two pressing conditions (short, long).
    After implementing a 'stop_function' (as interrupt) this function has much less sense, yet was easier to keep it."""
    
    new_cycle_button = GPIO.input(13)                   # GPIO reading value is assigned
    pressed_start_time = time.time()                    # the initial time is assigned
    elapsed = 0                                         # variable to assign the elapsed time is initially set on zero
    while new_cycle_button == 0:                        # while loop is started, and the condition to maintain is later checked
        elapsed = time.time() - pressed_start_time      # elapse time is calculated
        new_cycle_button = GPIO.input(13)               # updated the variable that sustains the while loop

    if elapsed >0:                                      # in case the elapsed time is >0 (obviously it is !)
        if debug:
            print(f'Button pressed for {round(elapsed,4)} secs')
        return 'short'







def stop_cycle(channel):
    """Function called as an interrupt in case the "start/stop button" is pressed.
    Time delay is used to prevent unwanted interruptions: Button has to be pressed at least for 0.5 seconds.
    The function change (global) variables used on roboto movements, to prevent further movements to happen.
    The function calls the quitting function, that closes the script."""
    
    global robot_running, robot_stop
    
    time.sleep(0.5)                        # delay between function being called, and new GPIO check
    stop_cycle_button = GPIO.input(13)     # GPIO is checked, and it values assigned
    if stop_cycle_button == 0:             # in case, after the initial delay, the button is (still) pressed
        if robot_running:                  # in case the robot was working
            robot_stop = True              # flag to immediatly interrup the robot movements is set
            robot_running=False            # flaf of robot running is set to false
            quit_func()                    # quitting function is called
    else:
        pass







class InfiniteTimer():
    """from: https://stackoverflow.com/questions/12435211/python-threading-timer-repeat-function-every-n-seconds
    This timer runs threads with defined timers.
    These timers are used to alternate text on displays at the robot.
    """

    def __init__(self, seconds, target):
        self._should_continue = False
        self.is_running = False
        self.seconds = seconds
        self.target = target
        self.thread = None

    def _handle_target(self):
        self.is_running = True
        self.target()
        self.is_running = False
        self._start_timer()

    def _start_timer(self):
        if self._should_continue: # Code could have been running when cancel was called.
            self.thread = Timer(self.seconds, self._handle_target)
            self.thread.start()

    def start(self):
        if not self._should_continue and not self.is_running:
            self._should_continue = True
            self._start_timer()
        else:
            print("Timer already started or running, please wait if you're restarting.")

    def cancel(self):
        if self.thread is not None:
            self._should_continue = False # Just in case thread is running and cancel fails.
            self.thread.cancel()
        else:
            #print("Timer never started or failed to initialize.")
            pass







def robot_set_displays(debug, printout=False):
    """Sets the displays at the robot."""
    
    global robot_display1, robot_display2
    
    if debug and printout:             # case debug variable is set true on __main_ and 
        print('robot displays are set')                                     # feedback is printed to the terminal
    
    robot_display2 = tm1637.TM1637(CLK=9, DIO=10, brightness=0.0)           # display2 is set
    robot_display2.Clear()                                                  # display2 is cleared
    
    robot_display1 = tm1637.TM1637(CLK=26, DIO=19, brightness=0.0)          # display1 is set
    robot_display1.Clear()                                                  # display1 is cleared
    
    robot_display2.SetBrightness(0)                                         # display2 brightness is set low
    robot_display1.SetBrightness(0)                                         # display1 brightness is set low






def robot_clear_displays():
    """Clears the displays at the robot
    The used displays and/or the library, aren't very reliable and many times the same command
    requires to be sent multiple times."""
    
    robot_set_displays(debug, printout=False)
    robot_display1.Clear()
    robot_display1.Clear()
    robot_display2.Clear()
    robot_display2.Clear()
    robot_display1.Clear()
    robot_display2.Clear()







def robot_loading_feedback():
    """On robot_display1, a loading feedback pattern is visualized at the script start up.
    In essence the character 8 is shoved from digit to digit (left to right)."""
    
    if device == 'laptop':                      # case the script is running on a PC/laptop (not the robot)
        return                                  # this function is not executed
    
    elif device == 'Rpi':                       # case the script is running at the robot
        global robot_stop
        if robot_stop==False:                   # case stop_button has not being pressed
            robot_display1.Clear()              # complete display is cleared
            for i in range(4):                  # iterates on the 4 digits of the display
                robot_display1.Show1(i, 8)      # character 8 is showed on the iterator digit
                robot_display1.Show1(i, 8)      # repeated command: character 8 is showed on the iterator digit
                time.sleep(0.1)                 # little delay
                robot_display1.Show1(i, 36)     # empty character is showed on the iterator digit
                robot_display1.Show1(i, 36)     # repeated command: empty character is showed at the display
            robot_display1.Clear()              # complete display is cleared







def robot_press_feedback():
    """On robot_display2 of the robot, is suggested to PRESS when waiting for a user feedback to start a reading cycle.""" 
    
    if device == 'laptop':                    # case the script is running on a PC/laptop (not the robot)
        return                                # this function is not executed
    
    elif device == 'Rpi':                     # case the script is running at the robot
        global robot_stop
        if robot_stop==False:                 # case stop_button has not being pressed
            robot_display2.Press()            # "Pres" is showed 
            robot_display2.Press()            # "Pres" is showed 







def robot_time_elapsed():
    """On robot_display1 of the robot, is visualized thetime elapsed since the cube reading status has started.
    Note: If the robot is running, and it gets connected to internet (ie., via SSH) the it will adjust the date and time,
    to a more precise one than the initial estimated. This means the difference between initial estimated time, and adjusted one,
    will be reflected on the time displayed on the screen, and on the robot solving time text log file.
    This could be solved by adding RTC extension borad, with related battery."""
    
    if device == 'laptop':                              # case the script is running on a PC/laptop (not the robot)
        return                                          # this function is not executed
    
    elif device == 'Rpi':                               # case the script is running at the robot
        global robot_stop
        
        if not GPIO.input(12) and side <6:              # case the GPIO12 is at level 0 (switch is closed, keep the show) and cube scan not completed
            return                                      # robot_display1 is not updated
        
        else:
            if robot_stop==False:                           # case stop_button has not being pressed
                robot_display1.Clear()                      # complete display is cleared
                robot_display1.ShowDoublepoint(True)        # double point separation between minutes and seconds is activated
                seconds = int(time.time() - start_time)     # time in seconds is calculated
                m, s = divmod(seconds, 60)                  # minutes and seconds are generated
                d0,d1 = divmod(m, 10)                       # at digit d0 the decades of minutes are assigned, at d1 the minutes
                d2, d3 = divmod(seconds-m*60, 10)           # at digit d2 the decades of seconds are assigned, at d3 the seconds
                
                robot_time_elapsed = [d0, d1, d2, d3]       # list with the fouyr digits is made
#                 print(robot_time_elapsed)
                
                robot_display1.Show (robot_time_elapsed)    # time is showed at the display
#                 robot_display1.Show (robot_time_elapsed)    # command is repeated: time is showed at the display
#                 robot_display1.Show (robot_time_elapsed)    # command is repeated: time is showed at the display







def robot_solve_cube(fixWindPos, screen, device, frame, faces, edge, cube_status, cube_color_sequence, HSV_analysis,
                     URFDLB_facelets_BGR_mean, font, fontScale, lineType, show_time, timestamp, solution, solution_Text,
                     detect_winner, cube_status_string, BGR_mean, HSV_detected,
                     start_time, camera_ready_time, cube_detect_time, cube_solution_time):
    
    """Sequence of commands involving the robot, after the cube status detection.
    Within this function the robot is called to solve the cube.
    This function calls many other functions."""
    
    global robot_stop, robot_running, deco
    
    robot_reading_status_display.cancel()             # de-activates visualization of elapsed time on robot_display2
    robot_clear_displays()
#     robot_display2.Clear()                            # clears dysplay2
#     robot_display2.Clear()                            # clears dysplay2
    
    robot_moves, total_robot_moves = rm.robot_moves(solution, solution_Text)  # dict with robot movements, and total movements
#     print(f'\nRobot movements sequence: {robot_moves}')   # nice information to print at terminal, sometime useful to copy 
    
    if solution_Text != 'Error':
        print(f'\nTotal robot movements: {total_robot_moves}')  # nice information to print at terminal, sometime useful to copy

    if detect_winner == 'BGR': facelets_data=BGR_mean                    # data to be later logged in a text file         
    elif detect_winner == 'HSV': facelets_data=HSV_detected              # data to be later logged in a text file
    else: facelets_data=BGR_mean, HSV_detected                                    # data to be later logged in a text file    
    
    if screen:                                # case a screen is connected
        try:
            cv2.destroyWindow("cube")
        except:
            pass

    show_time=show_time+int(0.7*total_robot_moves)   # show times is calculated to use all the robot solving time 
    deco_info = fixWindPos, screen, device, frame, background_h, faces, edge, cube_status_string, cube_status, \
                cube_color_sequence, HSV_analysis, URFDLB_facelets_BGR_mean, font, fontScale, lineType, detect_winner, \
                show_time, timestamp, robot_stop
        
    if screen:                                # case a screen is connected a multiprocess is used to show the cube status while solving it
        deco = mp.Process(target=decoration, args=(deco_info,)) # decoration showing (or just saving is screen=False) cube's faces pictures
        deco.start()                          # decoration thread is started
    
    if robot_stop==False:
        deco_time = time.time()
        tot_time_secs, elapsed_robot_time = robot_move_cube(robot_moves, total_robot_moves, solution_Text)   # movements to the robot are applied
        robot_time_display.cancel()           # after the robot has made the last (solving) move the countdown time is cancelled
    else:
        tot_time_sec = 0                      # robot solution time is forced to zero when the solving i interrupted by the stop button
    
    if robot_stop==False:                     # repeated conditional, to re-check the condition as the previous task takes some time
        servo.fun(debug)                      # cube is rotated wing increasing and decreasing speed, just for FUN
            
    # relevant info to be logged  into a text file
    log_data_info = device, timestamp, facelets_data, cube_status_string, solution, detect_winner, start_time, \
                    camera_ready_time, cube_detect_time, cube_solution_time, elapsed_robot_time, deco_time, tot_time_secs
    
    if solution_Text != 'Error':              # if there are not errors from the Kocimeba solver
        robot_done_status_display.start()     # activates visualization of cube done on robot_display2
        
    elif solution_Text == 'Error':            # if error (tipically bad color reading, so wrong amount of facelets per color) 
        robot_show_error_status()             # feedback at robot_display2
    
    if robot_stop==False:                     # case stop_button has not being pressed   
        time.sleep(1)                         # delay
        for i in range(2):
            if i == 0:                        # first iteration
                print_out=debug               # prints are set as per debug
            else:                             # from the second iteration onward
                print_out=False               # prints are set false
            servo.servo_start_positions(print_out)  # top cover and cube lifter to start position
            time.sleep(1)                     # delay
#             servo.servo_off(print_out)        # servo is de-energized
            servo.motor_off(print_out)        # motor is de-energized
            robot_running = False             # robot working flag is set to False, meaning no motor/servo actions

    return deco_info, log_data_info





def robot_timeout_func():
    """Robot reaction mode in case of timeout.
    Cube state reading, in case of high reflection or pollutted facelets, could get stuck therefore the need for a timeout;
    This function takes care of quitting the reading status."""
    
    clear_terminal(device)
    print(f'\nTimeout for cube status detection: Check if too much reflections, or polluted facelets\n')
    
    robot_time_display.cancel()           # robot_display1 at robot is cleared
    
    for i in range(2):
        if i == 0:                        # first iteration
            print_out=debug               # prints are set as per debug
        else:                             # from the second iteration onward
            print_out=False               # prints are set false
        servo.servo_start_positions(print_out)  # top cover and cube lifter to start position
        time.sleep(1)                     # delay
#         servo.servo_off(print_out)        # servo is de-energized
        servo.motor_off(print_out)        # motor is de-energized
        robot_running = False             # robot working flag is set to False, meaning no motor/servo actions

    
#     close_camera(device)  # this is necessary to get rid of analog/digital gains previously blocked
    
    try: robot_load_status.cancel()                 # reset the "loading" status at robot_display1
    except: pass
        
    try: robot_time_display.cancel()                # de-activates visualization of elapsed time on robot_display1
    except: pass
    
    try: robot_reading_status_display.cancel()      # de-activates visualization of cube reading status at robot_display2
    except: pass
    
    try: robot_done_status_display.cancel()         # de-activates visualization of cube done on robot_display2
    except: pass
        
    try: robot_press_feedback.cancel()              # de-activates the press feedback on robot_display2
    except: pass
    
    if screen:                  # case a screen is connected
        try: cv2.destroyAllWindows()
        except: pass
    
    robot_clear_displays()      # clears displays on the robot
    robot_clear_displays()      # repeat of clears displays on the robot
    side=6                      # side is forced to 6, to simulate the cube solving was done so that the script restart for a next cycle 
    timeout=True                # boolean variable (also global) used to manage the timeout case
    
    return timeout







def robot_show_camera_cal():
    """On display 2 of the robot, a feedback is provided when the PiCamera is under calibration."""
    
    global robot_stop
    
    if robot_stop==False:                        # case stop_button has not being pressed
        robot_display2.Clear()                   # display2 is cleared
        seconds = int(time.time() - start_time)  # time in seconds, express in integers 

        if (seconds % 2) == 0:                   # every even second 
            robot_display2.Cam()                 # the word 'CAM" is showed
            robot_display2.Cam()                 # repeated command: the word 'CAM" is showed

        else:                                    # every odd second 
            robot_display2.Cal()                 # the word 'CAL" is showed
            robot_display2.Cal()                 # repeated command: he word 'CAL" is showed





def robot_show_read_status():
    """On display 2 of the robot, a feedback is provided when the cube is in cube reading status."""
    
    global robot_stop
    
    if robot_stop==False:                        # case stop_button has not being pressed
        robot_display2.Clear()                   # display2 is cleared
        seconds = int(time.time() - start_time)  # time in seconds, express in integers 

        if (seconds % 2) == 0:                   # every even second 
            robot_display2.Cube()                # the word 'CubE" is showed
            robot_display2.Cube()                # repeated command: the word 'CubE" is showed

        else:                                    # every odd  second 
            robot_display2.Read()                # the word 'REAd" is showed
            robot_display2.Read()                # repeated command: he word 'REAd" is showed







def robot_show_error_status():
    """On robot_display2 of the robot, a feedback is provided when the cube solver returns an error."""
    
    if device == 'laptop':            # case the script is running on a PC/laptop (not the robot)
        return                        # this function is not executed
    
    elif device=='Rpi':               # case the script is running at the robot
        robot_display2.Clear()        # display2 is cleared
        robot_display2.Error()        # the word "Err" is showed
        robot_display2.Error()        # repeated command: the word "Err" is showed







def robot_show_cube_done():
    """On robot_display2 of the robot, a feedback is provided when the cube is is solved."""
    
    if device == 'laptop':                           # case the script is running on a PC/laptop (not the robot)
        return                                       # this function is not executed
    
    elif device=='Rpi':                              # case the script is running at the robot
        global robot_stop
        
        if robot_stop==False:                        # case stop_button has not being pressed
            robot_display2.Clear()                   # display2 is cleared
            seconds = int(time.time() - start_time)  # time in seconds, express in integers 
            if (seconds % 2) == 0:                   # every even second 
                robot_display2.Cube()                # the word 'CubE" is showed
                robot_display2.Cube()                # repeated command: the word 'CubE" is showed
            else:                                    # every odd second 
                robot_display2.Done()                # the word 'DonE" is showed
                robot_display2.Done()                # repeated command: the word 'DonE" is showed







def log_data(log_data_info):
    
    """Main cube info are logged in a text file
    This function is called obly on the robot (Rpi), to generate a database of info usefull for debug and fun
    BGR color distance is the first approach used to detect cube status, therefore the winner if it succedes.
    HSV color approach is used when the BGR approach fails; If the HSV succedes on cube status detection it become the winner.
    If the cube solver returns an error it means both the approaches have failed on detecting a coherent cube status."""
    
    import os                                            # os is imported to ensure the file presence check/make
    
    device = log_data_info[0]
    timestamp = log_data_info[1]
    facelets_data = log_data_info[2]
    cube_status_string = log_data_info[3]
    solution = log_data_info[4]
    detect_winner = log_data_info[5]
    start_time = log_data_info[6]
    camera_ready_time = log_data_info[7]
    cube_detect_time = log_data_info[8]
    cube_solution_time = log_data_info[9]
    elapsed_robot_time = log_data_info[10]
    deco_time = log_data_info[11]
    tot_time_secs = log_data_info[12]
    
    folder = pathlib.Path().resolve()                    # active folder (should be home/pi/cube)  
    folder = os.path.join(folder,'CubesDataLog')         # folder to store the relevant cube data
    if not os.path.exists(folder):                       # if case the folder does not exist
        os.makedirs(folder)                              # folder is made if it doesn't exist
    
    fname = folder+'/Cubotone_solver_log_Rpi.txt'        # folder+filename for the cube data
    if not os.path.exists(fname):                        # if case the file does not exist, file with headers is generated
        if debug:
            print(f'\ngenerated Cubotone_solver_log_Rpi.txt file with headers')
    
        a = 'Date'
        b = 'Screen'
        c = 'CV_wow'
        d = 'Debug'
        e = 'Led_usage'
        f = 'ColorAnalysisWinner'
        g = 'TotalTime(s)'
        h = 'CamWarmUpTime(s)'
        i = 'FaceletsDetectionTime(s)'
        k = 'CubeSolutionTime(s)'
        l = 'CollageImageTime (s)'
        m = 'RobotSolvingTime(s)'
        n = 'CubeStatus(BGR or HSV or BGR,HSV)'
        o = 'CubeStatus'
        p = 'CubeSolution'
        
        # tab separated string of the the headers
        s = a+'\t'+b+'\t'+c+'\t'+d+'\t'+e+'\t'+f+'\t'+g+'\t'+h+'\t'+i+'\t'+k+'\t'+l+'\t'+m+'\t'+n+'\t'+o+'\t'+p+'\n'
        
        os.umask(0) # The default umask is 0o22 which turns off write permission of group and others
        # 'a'means: file will be generated if it does not exist, and data will be appended at the end
        with open(os.open(fname, os.O_CREAT | os.O_WRONLY, 0o777), 'a') as f:    # text file is temporary opened
            f.write(s)                                   # data is appended

    
    # info to log
    a=str(timestamp)                                     # date and time
    b='screen'if screen else 'no screen'                 # screen presence or absence (it influences the cube detection time)
    c='cv_wow'if cv_wow else 'no cv_wow'                 # cv_wow variable status (it influences the cube detection time)
    d='debug'if debug else 'no debug'                    # debug variable status (it influences the cube detection time)
    e='Led used'if led_usage else 'no led'               # led_usage variable status (it influences the cube detection time)
    f=str(detect_winner)                                 # wich method delivered the coherent cube status
    g=str(tot_time_secs)                                 # total time from camera warm-up to solved cube
    h=str(round(camera_ready_time-start_time,1))         # time to get the camera gains stable and exposure time defined/set
    i=str(round(cube_detect_time-camera_ready_time,1))   # time to read the 6 cube faces
    k=str(round(cube_solution_time-cube_detect_time,1))  # time to get the cube solution from the solver
    l=str(round(deco_time-cube_solution_time,1))         # time to make (and present when screen) the cube collage image
    m=str(elapsed_robot_time)                            # time to apply the solving movements
    n=str(facelets_data)                                 # according to which methos delivered the solution (BGR, HSV, both)
    o=str(cube_status_string)                            # string with the detected cbe status
    p=str(solution)                                      # solution returned by Kociemba solver
    
    # tab tab separated string with all the info to log
    s = a+'\t'+b+'\t'+c+'\t'+d+'\t'+e+'\t'+f+'\t'+g+'\t'+h+'\t'+i+'\t'+k+'\t'+l+'\t'+m+'\t'+n+'\t'+o+'\t'+p+'\n'
    
    os.umask(0) # The default umask is 0o22 which turns off write permission of group and others
    
    # 'a'means: file will be generated if it does not exist, and data will be appended at the end
    with open(os.open(fname, os.O_CREAT | os.O_WRONLY, 0o777), 'a') as f:    # text file is temporary opened
        f.write(s)                                       # data is appended
  







def robot_set_GPIO():
    """Raspberry Pi requires some settings at the GPIO (General Purpose imput Output)
    This function sets the GPIO way of working
    This function also sets an interrupt for the start/stop button."""
    
    global GPIO, new_cycle_button
    
    if device=='Rpi':                                            # case the script is running at the robot
        import RPi.GPIO as GPIO                                  # GPIO module
        GPIO.setwarnings(False)                                  # GPIO warning set to False to reduce effort on handling them
        GPIO.setmode(GPIO.BCM)                                   # GPIO modulesetting
        GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_UP)        # start/stop button setting
        new_cycle_button = GPIO.input(13)                        # "normal" usage of the button, for the cycle(s) start
        GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)      # switch s1 for setting purpose
        GPIO.setup(20, GPIO.IN, pull_up_down=GPIO.PUD_UP)        # switch s2 for setting purpose
        GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP)        # switch s3 for setting purpose
        GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_UP)        # switch to force cv_wow plots to remain on screen while the switch is closed
        

        try:
            GPIO.add_event_detect(13, GPIO.FALLING, callback=stop_cycle, bouncetime=100)  # interrupt usage of the same input pin, to interrupt the cy
        except:
            pass







def robot_set_servo(first_cycle, motors_hw):
    """The robot uses a couple of servos and one motor
    This function import the library I made to control these components
    This functions positions the servos and motor on the start position
    After the initial settings and positioning, servos and motor are de-energize to prevent heating until usage."""
    global servo
    
    try:
        import Cubotone_servos as servo        # setup and functions to control the servos
    except:
        print("\nIssue on importing Cubotone_servos")
        return False
    
    if first_cycle:
        servo.init_servo(False, debug, motors_hw)   # servos and motor settings are uploaded, servos and motor are initialized
    
#     servo.servo_off(debug)                     # servo is de-energized
    servo.motor_off(debug)                     # motor is de-energized
    
    return True
            







def check_window_close_req(cv_wow, key):
    """Function that verifies is the ESC button is pressed when a CV2 window is opened.
    It also checks if the window closing 'X' of 'cube'or 'Cube' windows are pressed.
    The function returns True u=in case of window closig request."""
    
    close_win = False            # close_win variable is set false
    if cv_wow:                   # case cv_wow variable is set true on __main__
        try:                     # method to close CV2 windows, via the mouse click on the windows bar X
            if cv2.getWindowProperty("Cube", cv2.WND_PROP_VISIBLE) <1:  # X on top bar 
                quit_func()      # quit function is called
                close_win = True # close_win variable is set true
                return close_win # close_win variable is returned
        except:                  # except case     
            pass                 # exception not really necessary, apart for having the try option
    
    if not cv_wow:               # case cv_wow variable is set false on __main__
        try:                     # method to close CV2 windows, via the mouse click on the windows bar X
            if cv2.getWindowProperty("cube", cv2.WND_PROP_VISIBLE) <1:  # X on top bar 
                quit_func()      # quit function is called
                close_win = True # close_win variable is set true
                return close_win # close_win variable is returned
        except:                  # except case     
            pass                 # exception not really necessary, apart for having the try option
    
    if key == 27 & 0xFF:         # ESC button method to close CV2 windows
        quit_func()              # quit function is called
        close_win = True         # close_win variable is set true
        return close_win         # close_win variable is returned







def time_system_synchr():
    """ Checks the time system status; In case of internet connection, waits for synchronization before proceeding.
        In case of internet connection, a max 20 synchronization attempts (10 seconds) are done.
        This choice to prevent time mismatch when the synchronization happens during a cube solving process.
        Raspberry pi doesn't have an RTC, and time module updates once an internet connection is made.
        When the time module is initially synchronized, there will also be later adjustments (I believe every 5 minutes);
        these later adjustments aren't of a problem for the time calculation in this script."""

    if device == 'laptop':                                  # case the script is running on a PC/laptop (not the robot)
        return                                              # this fuction is not executed
    
    elif device=='Rpi':                                     # case the script is running at the robot
        import socket
        
        try:
            res = socket.getaddrinfo('google.com',80)       # trivial check if internet is available
            print('Internet is connected')                  # feedback is printed to the terminal
            internet = True                                 # internet variable is set true
        except:                                             # exception is used as no internet availability
            print('No internet connection')                 # feedback is printed to the terminal
            internet = False                                # internet variable is set false
        
        if internet:                                        # case internet is available
            from subprocess import Popen, PIPE              # classes importing        
            once = True                                     # variable once is set true, to print a feedback only once
            i = 0                                           # iterator
            while True:                                     # infinite loop              
                try:
                    if i == 20:                             # case the iteration has been done 20 times (10 seconds)
                        break                               # while loop is interrupted
                    
                    # inquiry to timedatectl status 
                    ps = Popen("timedatectl status | grep 'System clock synchronized'  | grep -Eo '(yes|no)'", shell=True, stdout=PIPE)
                    output = ps.stdout.read()                        # process output
                    ps.stdout.close()                                # closing the pipe
                    ps.wait()                                        # waits until the ps child completes
                    
                    if b'yes' in output:                             # case the timedatectl status returns true
                        date_time = dt.datetime.now().strftime('%d/%m/%Y %H:%M:%S')   # updated date and time assigned to date_time variable
                        print('Time system is synchronized: ', str(date_time))        # feedback is printed to the terminal
                        time.sleep(1.5)
                        disp.show_on_display(str(date_time[11:]), '', fs1=26)         # feedback is printed to the display
                        break                                        # while loop is interrupted
                    else:                                            # case the timedatectl status returns false
                        if once:                                     # case the variable once is true
                            print('Waiting for time system update')  # feedback is printed to the terminal
                            once = False                             # variable once is set false, to print a feedback only once
                        time.sleep(0.5)                              # sleep time before inquiry to timedatectl status again
                        i+=1                                         # iterator is increased
                except:                                              # case there is an exception
                    break                                            # while loop is interrupted
                    
        else:                                                        # case the is not an internet connection
            date_time = dt.datetime.now().strftime('%d/%m/%Y %H:%M:%S')   # updated date and time assigned to date_time variable
            print('Time system not synchronized yet')                # feedback is printed to the terminal
            print('Estimated datetime:',date_time)                   # feedback is printed to the terminal





def keep_the_show(debug):
    """function meant for the Maker Fair, to keep presenting the cv_wow in case switch at GIPO12 is closed."""
    global detection_timeout
    
    if debug:                               # case debug variable is set true
        print('close switch at GPIO12 to keep cv_wow windows on screen') # feedback is printed to the terminal
    delay = 0.2                             # time assigned to the delay variable
    time.sleep(delay)                       # little delay to prevent switch contact bouncing from creating bad reading
    initial_time = time.time()              # time in which this function has been called
    time_ref = time.time()                  # time reference to monitor how long this function lasts
    while not GPIO.input(12):               # case the GPIO12 is at level 0 (switch is closed)
        time.sleep(delay)                   # little delay to prevent useless iterations quantity
        if time.time() - time_ref >= 1:     # case 1 second is elapsed
            time_ref = time.time()          # time reference is updated
            detection_timeout += 1          # the detection_timeout variable is increased by the time spent on this while loop
    
    return time.time()-initial_time         # returns the time spent in this function





def test_picamera():
    """funtion to allow a quick fedback of PiCamera working fine, when the robot is not fully assembled yet."""
    
    frame, w, h, scale = read_camera()           # video stream and frame dimensions
    font, fontScale, fontColor, lineType = text_font()  # font characteristics
    font_size = fontScale*scale/100              # font size is adjusted according to the frame resizing factor            

    start = time.time()                          # current time is assigned to variable start
    show_time = 20
    while time.time()-start < show_time:         # iteration until the elapsed time is smaller than show_time
        countdown = int(show_time-(time.time()-start))  # remaining time 
        frame, w, h, scale = read_camera()       # video stream and frame dimensions
        text_bg(frame, w, h)                     # black background added to the frame
        cv2.putText(frame, str('PiCamera TEST'), (10, 30), font, font_size*1.2, fontColor, lineType)
        text = f'Closing in {countdown} seconds,   ESC to quit earlier'
        cv2.putText(frame, text, (10, int(h-12)), font, font_size*1.2, fontColor, lineType)
        cv2.imshow('PiCamera test', frame)       # shows the frame 
        key = cv2.waitKey(1)                     # refresh time is minimized (1ms), yet real time is much higher
        if key == 27 & 0xFF:                     # ESC button method to close CV2 windows
            break                                # while loop is interrupted
    
    quit_func()                                  # quitting funtion is used to close all the threads and the script








def start_up():
    """Start up function, that aims to run (once) all the initial settings needed."""
    
    # global variables used in both devices, laptop and Rpi (at robot)
    global font, fontScale, fontColor, lineType, camera, width, height, sv, quitting
    global sides, side, BGR_mean, H_mean, URFDLB_facelets_BGR_mean, edge, offset, faces, w, h, background_h, gap_w, gap_h

    
    side=0   # set the initial cube side (cube sides are 1 to 6, while zero is used as starting for other setting)
    
    if device == 'laptop':                              # case the script is running on a PC/laptop (not the robot)
        global first_cycle, plt                         # function and variables used as global
        global k_kernel, d_iterations, e_iterations, facelets_in_width   # function and variables used as global
    
        
        solver_imported = False                         # solver_imported is set False
        try:                                            # tentative
            import solver as sv                         # Kociemba solver is imported
            solver_imported = True                      # solver_imported is set True
        except:                                         # case an exception is raise
            pass                                        # do nothing
        
        if not solver_imported:                         # case solver_imported is still set False
            try:                                        # tentative
                import twophase.solver as sv            # Kociemba solver is imported from a different folder
                solver_imported = True                  # solver_imported is set True
            except:                                     # case an exception is raise
                pass                                    # do nothing
        
        first_cycle=True                                # variable to be used only at first program loop
        camera, width, height = webcam()                # webcam relevant info are returned after cropping, resizing, etc
        gap_w = 30                                      # horizontal gap between windows when cv_wow is set True on _main_
        gap_h = 30                                      # vertical gap between windows when cv_wow is set True on _main_
        edge = 16                                       # edge dimension of each facelet used on cube sketches
        offset=int(14.2*edge)                           # left part of the frame not usable for cube facelet detection, as used to depict the cube sketches
        sides={0:'Empty', 1:'U', 2:'R', 3:'F', 4:'D', 5:'L', 6:'B'}  # kociemba side order to follow, while detecting facelets colors
        font, fontScale, fontColor, lineType = text_font()  # setting text font paramenters
        
        # general parameters for facelet's edges detection
        k_kernel=5                      # parameter for the edge detection
        d_iterations=10                 # d=Dilated (the higher the faster), for edge detection
        e_iterations=4                  # e=eroded (the larger the bigger the distance from real edge), for edge detection              
        facelets_in_width = 10          # Amount of facelets in frame width, to determin the min/max acceptable contour's area
            

        
    elif device == 'Rpi':                                # case the script is running at the robot
        if first_cycle:                                  # case this is the first robot cycle
            global rm, tm1637, PiCamera, PiRGBArray, rawCapture, camera_set_gains, servo, GPIO, new_cycle_button, mp
            global robot_reading_status_display, robot_done_status_display, robot_running, robot_stop
            global Timer, InfiniteTimer, robot_calib_status_display, robot_load_status, robot_time_display, robot_show_press
            global timeout, detection_timeout, path, pathlib
            
            from picamera.array import PiRGBArray        # Raspberry pi specific package for the camera, using numpy array
            from picamera import PiCamera                # Raspberry pi specific package for the camera
            import Cubotone_set_picamera_gain as camera_set_gains  # script that allows to fix some parameters at picamera        
            import Cubotone_tm1637 as tm1637             # tm1637 modified library, for 4 x 7 segments display
            import Cubotone_moves as rm                  # RobotMoves: convert Kociemba solver solution in robot movements sequence
            from threading import Timer                  # Timer function from threading is used to manage displays
            import multiprocessing as mp                 # multiprocessing is used to display the cube status while solving it
            import os.path, pathlib                      # libraries for folder path management
            
            robot_set_displays(debug, printout=True)     # initializes the two displays
            if not debug:                                # case the debug is set False
                clear_terminal(device)                   # cleares the terminal
            time_system_synchr()  # checks the time system status (if internet connected, it waits till synchronization)
            param_imported, settings = import_parameters()   # imports the parameter from a json file
            if not param_imported:                       # case the function import_parameters returns False
                quit_func()                              # quitting function is called
            camera, rawCapture, width, height = webcam() # camera relevant info are returned after cropping, resizing, etc
            mp.set_start_method('spawn')                 # multiprocess method used 
            cpu_temp()                                   # cpu temp is checked at start-up
            robot_running = False                        # flag of the robot working or waiting for a new cycle start
            robot_stop = False                           # flag to stop the robot movements
            timeout = False                              # timeout flag is initialli set on False
            if not picamera_test:
                robot_load_status = InfiniteTimer(0.5, robot_loading_feedback)  # set timer thread to display a loading feedback on robot_display1
                robot_load_status.start()                # activates display's segments as feedbacks the program has started
                time.sleep(0.05)                         # little delay for the next timer start
                robot_time_display = InfiniteTimer(0.5, robot_time_elapsed)     # set the timer thread for visualizing the elapsed time on robot_display1
                time.sleep(0.05)                         # little delay for the next timer start
                robot_show_press = InfiniteTimer(0.5, robot_press_feedback)     # set the timer thread for visualizing press when the button is enabled
                time.sleep(0.05)                         # little delay for the next timer start
                robot_calib_status_display = InfiniteTimer(0.5, robot_show_camera_cal) # activates visualization of robot camera calibration robot_display2
                time.sleep(0.05)                         # little delay for the next timer start
                robot_reading_status_display = InfiniteTimer(0.5, robot_show_read_status) # activates visualization of robot reading status on robot_display2
                time.sleep(0.05)                         # little delay for the next timer start
                robot_done_status_display = InfiniteTimer(0.5, robot_show_cube_done)   # activates visualization cube done on robot_display2
            edge = 12                                    # edge dimension of each facelet used on cube sketches
            offset=int(14.2*edge)                        # left part of the frame not usable for cube facelet detection, as used to depict the cube sketches
            font, fontScale, fontColor, lineType = text_font()   # setting text font paramenters
            sides={0:'Empty', 1:'U', 2:'B', 3:'D', 4:'F', 5:'R', 6:'L'}  # robot side order to follow, while detecting facelets colors
            detection_timeout = 40                       # timeout for the detection (in secs), to de-energize motor/servos if it fails
            if cv_wow:                                   # case the cv+wow variable is set true
                detection_timeout += 20                  # timeout for the detection (in secs), to de-energize motor/servos if it fails
#             gap_w = 60    # (AF 60)                    # horizontal gap between windows when cv_wow is set True on _main_
#             gap_h = 100   # (AF 100)                   # vertical gap between windows when cv_wow is set True on _main_

  
    # common settings
    quitting = False                 # quitting variable is set False
    BGR_mean=[]                      # empty list to be filled with with 54 facelets colors while reading cube status
    H_mean=[]                        # empty list to be filled with with 54 facelets HUE value, while reading cube status
    URFDLB_facelets_BGR_mean=[]      # empty list to be filled with with 54 facelets colors, ordered according URFDLB order
    faces={}                         # dictionary that store the image of each face
    
    







def cubeAF():
    """This function is substantially the main function.
    It covers all the different phases after the initial settings:
        Camera setting for 1st side and remaining
        Keeps interrogating the camera
        Cube status detection
        Cube solver call
        Cube solving at robot."""
    
    global font, fontScale, fontColor, lineType, cap, width, height, h, w, device, sides, side, robot_load_status
    global BGR_mean, H_mean, URFDLB_facelets_BGR_mean, offset, gap_w, gap_h, frame, cube
    global facelets, faces, start_time, servo, camera, robot_stop, fixWindPos, screen, detection_timeout, timeout
    
    
    if device == 'Rpi':                             # case the script is running at the robot
        aligned = servo.align_motor(debug, 'read')  # servos are set to start position, prior to activate the camera   
        if not aligned:                             # case the align_motor function returns an error
            print("\nCannot align the motor (cube_holder), script is terminated")   # feedback is printed to the terminal
            quit_func()                             # script is closed, in case of irresponsive camera
    
    if not camera_opened_check(device):             # checks if camera is irresponsive
        print("\nCannot open the camera")           # feedback is printed to the terminal
        quit_func()                                 # script is closed, in case of irresponsive camera
    
    if side == 0:                                   # case the cube side is zero (staring a new cycle)
        faces.clear()                               # empties the dict of images (6 sides) recorded during previous solving cycle 
        facelets = []                               # empties the list of contours having cube's square characteristics
        if device=='Rpi':                           # case the script is running at the robot
            start_time = time.time()                # initial time is stored as timer for alternating words at displays
            robot_load_status.cancel()              # reset the "loading" status feedback at robot_display1
            robot_clear_displays()                  # displays at robot are cleared
            robot_time_display.start()              # activates visualization of elapsed time on robot_display1
            robot_calib_status_display.start()      # activates visualization of PiCamera calibration status on robot_display2
            PiCamera_param = robot_camera_warmup()  # calls the warmup function for PiCamera
            if not robot_stop:                      # case there are no requests to stop the robot
                robot_consistent_camera_images(PiCamera_param)  # sets PiCamera to capture consistent images
            if screen:                              # case a screen is connected
                try: cv2.destroyWindow("cube")      # windows "cube" is closed
                except: pass                        # in case an exception is raise, nothing is done
            camera_ready_time=time.time()           # time stored after picamera warmup and settings for consistent pictures
            robot_calib_status_display.cancel()     # reset the "calibration" status feedback at robot_display1
            robot_clear_displays()                  # displays at robot are cleared
            robot_reading_status_display.start()    # activates visualization of robot reading status on robot_display2
            show_time = 7                           # min showing time of the unfolded cube images (its initial status)
            proceed = True                          # proceed variable is set True (variable to jump into facelet reading mode)
            
        elif device == 'laptop':                    # case the script is running on a PC/laptop (not the robot)
            show_time = 12                          # showing time of the unfolded cube images (its initial status)
            proceed = False                         # proceed variable is set False (variable to jump into facelet reading mode)
        
        timestamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S')   # date_time variable is assigned, for file name and log purpose


    while not quitting:                             # substantially the main loop, it can be interrupted by quit_func() 
        
        frame, w, h, scale = read_camera()          # video stream and frame dimensions
        frame_copy = frame.copy()                   # frame image is duplicated, to be able to show it before and after image processing
        text_bg(frame, w, h)                        # generates a rectangle as backgroung for text in Frame
        
        if side == 0:
            side = window_for_cube_rotation(w, h, side, frame) # keeps video stream while suggesting which cube's face to show
            det_face_time = time.time()             # used at PC to enable a delay before cube face reading
        
        (contours, hierarchy)=read_facelets(frame, scale, det_face_time, proceed) # reads cube's facelets and returns the contours
        candidates = []                                   # empties the list of potential contours
        
        if hierarchy is not None:                         # analyze the contours in case these are previously retrieved
            hierarchy = hierarchy[0]                      # only top level contours (no childs)
            facelets = []                                 # empties the list of contours having cube's square characteristics
            
            if device=='Rpi':                             # case the script is running at the robot
                if timeout==True or robot_stop == True:   # in case of reached timeout or stop_button pressed
                    break                                 # script is closed
            
            for component in zip(contours, hierarchy):    # each contour is analyzed   
                contour, hierarchy, corners = get_approx_contours(component)   # contours are approximated
                
                if device=='Rpi':                         # case the script is running at the robot
                    if  time.time()-camera_ready_time > detection_timeout:  # timeout is calculated for the robot during cube status reading
                        timeout=robot_timeout_func()      # in case the timeout is reached
                        break                             # script is closed
                    if robot_stop==True:                  # in case stop_button pressed
                        break                             # script is closed
                
                if screen:                                # case a screen is connected
                    if cv_wow:                            # case cv_wow variable is set true on __main__
                        cv2.namedWindow("camera")         # create the camera window
                        cv2.moveWindow("camera", 0, gap_h)# move the camera window to (0, gap_h)
                        cv2.imshow("camera", frame_copy)  # shows the frame copy, meaning the frame without contours and other additions 
                    elif not cv_wow:                      # case cv_wow variable is set false on __main__
                        if fixWindPos:                    # case the fixWindPos variable is set true on __main__ 
                            cv2.namedWindow("cube")       # create the cube window
                            cv2.moveWindow("cube", 0,0)   # move the cube window to (0,0)
                        cv2.imshow("cube", frame)         # shows the frame 
                    key=cv2.waitKey(1)                    # refresh time is minimized to 1ms, meaning the refresh time mostly depends from all other functions
                    
                if device == 'laptop':
                    if time.time() < det_face_time + delay_facelets_dec: # case the delay time is not elapsed yet
                        if key == 32:                     # case spacebar is pressed
                            proceed = True                # proceed is set true (from preparing the cube to read facelets)
                        if not proceed:                   # cese proceed variable is False
                            break                         # for loop is interrupted 
                    
                if corners==4:                            # contours with 4 corners are of interest
                    facelets = get_facelets(facelets, contour, hierarchy)  # returns a dict with cube compatible contours

                if len(facelets)==9:                                   # 9 contours have cube compatible characteristics
                    facelets = order_9points(facelets, new_center=[])  # contours are ordered from top left
                    d_to_exclude = distance_deviation(facelets)        # facelets to remove due inter-distance not as regular 3x3 array
                    
                    if len(d_to_exclude)>=1:                           # check if any contour is too far to be part of the cube
                        d_to_exclude.sort(reverse=True)                # reverse the contours order
                        for i in d_to_exclude:                         # remove the contours too faar far to be part of the cube
                            facelets.pop(i)                            # facelet is removed

                if len(facelets)==9:                                   # 9 contours have cube compatible characteristics    
                    if device == 'Rpi':                                # case the script is running at the robot
                        robot_facelets_rotation(facelets)              # order facelets as per viewer POW (due to cube/camera rotations on robot)
                    elif device == 'laptop':                           # case the script is running on a PC/laptop (not the robot)
                        proceed = False                                # proceed is set false, to force a delay on facelets detection at cube face changing
                    read_color(facelets, candidates, BGR_mean, H_mean, scale) # each facelet is read for color, decoration for the viewer is made
                    URFDLB_facelets_BGR_mean = URFDLB_facelets_order(BGR_mean) # facelets are ordered as per URFDLB order
                    plot_colors(URFDLB_facelets_BGR_mean, edge, frame, background_h, font, fontScale, lineType) # plot a cube decoration with detected colors                
                    faces = face_image(frame, facelets, side, faces)   # image of the cube side is taken for later reference
                    if screen:                                         # case a screen is connected
                        if cv_wow:                                     # case cv_wow variable is set true on __main__
                            cv2.namedWindow('Cube')                    # create the Cube window
                            cv2.moveWindow('Cube', 0, h+2*gap_h)       # move the Cube window to coordinates
                            cv2.imshow('Cube', frame)                  # shows the frame 
                            if not GPIO.input(12):                     # case the GPIO12 is at level 0 (switch is closed)
                                showed_time = keep_the_show(debug)     # function keeping the cv_wow windows on screen if GPIO12 in at 1 (Fair show)
                                start_time += showed_time              # time spent at the keep_the_show is added from the cube start for compensation
                                camera_ready_time += showed_time       # time spent at the keep_the_show is added from the cube start for compensation
                                
                        elif not cv_wow:                               # case cv_wow variable is set false on __main__
                            if fixWindPos:                             # case the fixWindPos variable is set true on __main__ 
                                cv2.namedWindow('cube')                # create the cube window
                                cv2.moveWindow('cube', 0,0)            # move the window to (0,0)
                            cv2.imshow('cube', frame)                  # shows the frame 
                        cv2.waitKey(1)                                 # refresh time is minimized (1ms), yet real time is much higher
                    
                    robot_next_side(side)                              # cube is rotated/flipped to the next face

                    if side < 6:                               # actions when a face has been completely detected, and there still are other to come
                        side = window_for_cube_rotation(w, h, side, frame) # image stream while viewer has time to positione the cube for next face
                        det_face_time = time.time()            # current time is assigned to det_face_time for timing analysis
                        break                                  # with this break the process re-start from contour detection at the next cube face



                    if side == 6:   # last cube's face is acquired   
                        cube_detect_time = time.time()         # time stored after detecting all the cube facelets
                        
                        if screen:                             # case a screen is connected
                            try: cv2.destroyAllWindows()       # all cv2 windows are removed
                            except: pass                       # no actions in case of errors
                        cube_status, HSV_detected, cube_color_sequence, HSV_analysis = cube_colors_interpreted(URFDLB_facelets_BGR_mean)  # cube string status with colors detected 
                        cube_status_string = cube_string(cube_status)    # cube string for the solver
                        solution, solution_Text = cube_solution(cube_status_string)   # Kociemba solver is called to have the solution string
                        detect_winner='BGR'                    # variable used to log which method gave the solution
                        cube_solution_time=time.time()         # time stored after getting the cube solution
                        print(f'\nCube status (via BGR color distance): {cube_status_string}\n')
                   
         
                        if solution_Text == 'Error':           # if colors interpretation on BGR color distance fail an attempt is made on HSV
                            print(f'Solver return: {solution}\n')
                            cube_status, cube_status_HSV, cube_color_sequence, HSV_analysis = cube_colors_interpreted_HSV(URFDLB_facelets_BGR_mean,\
                                                                                HSV_detected)  # cube string status with colors detected 
                            cube_status_string = cube_string(cube_status)  # cube string for the solver
                            solution, solution_Text = cube_solution(cube_status_string)   # Kociemba solver is called to have the solution string
                            detect_winner='HSV'                            # variable used to log which method give the solution
                            cube_solution_time=time.time()                 # time stored after getting the cube solution
                            if solution_Text == 'Error':                   # in case color color detection fail also with HSV approach
                                detect_winner='Error'                      # the winner approach goes to error, for log purpose
                                print(f'Solver return: {solution}\n')
                            else: 
                                print(f'\nCube status (via HSV color distance): {cube_status_string}') # nice information to print at terminal, sometime useful to copy
                                print(f'\nCube solution: {solution_Text}') # nice information to print at terminal, sometime useful to copy 

      
                        elif solution_Text != '0 moves  ':                 # case of interest, the cube isn't already solved
                            print(f'\nCube solution: {solution_Text}')     # nice information to print at terminal, sometime useful to copy 
                        
                        if device == 'laptop': # case the script is running on a PC/laptop (not the robot)
                            robot_stop=False   # variable needed for compatibility with the decoration() function used also by the robot
                            
                            # tuple of variables needed for the decoration function
                            deco_info = fixWindPos, screen, device, frame, background_h, faces, edge, cube_status_string, cube_status,\
                                        cube_color_sequence, HSV_analysis, URFDLB_facelets_BGR_mean,\
                                        font, fontScale, lineType, detect_winner, show_time, timestamp, robot_stop

                            decoration(deco_info) # Cube images as seen + sketch with recognized and interpreted colors          
                            side = window_for_cube_solving(solution_Text, w, h, side, frame, scale)  # image stream while user manually solves the cube (laptop mode)
                            
                            break        # the while (not quitting) loop is interrupted
                        
                        
                        
                        elif device == 'Rpi':   # case the script is running at the robot
                            # function related to cube solving part via the robot, when the last face has been completely detected
                            print("\ncube_status_string",cube_status_string)
                            if debug:
                                print("\ncube status", cube_status)
                            
                            # tuple of variables needed for the decoration function
                            deco_info, log_data_info = robot_solve_cube(fixWindPos, screen, device, frame, faces, edge, cube_status, cube_color_sequence,
                                                         HSV_analysis, URFDLB_facelets_BGR_mean, font, fontScale, lineType, show_time,
                                                         timestamp, solution,solution_Text, detect_winner, cube_status_string, BGR_mean,
                                                         HSV_detected,start_time, camera_ready_time, cube_detect_time, cube_solution_time)
                            
                            if not screen:               # case there is not  screen connected
                                decoration(deco_info)    # the cube collage is made after the cube solving phase    
                            
                            log_data(log_data_info)      # some relevant info are logged into a text file
                            time.sleep(3)                # little delay
                            if device == 'Rpi' and robot_stop==False:   # case the script is running at the robot and no stopping request
                                try:                     # try case, as isn't certain the robot has done the cube
                                    robot_done_status_display.cancel()  # de-activates visualization of cube done on robot_display2
                                except:                  # except case
                                    pass                 # exception not really necessary, apart for having the try option
                                robot_clear_displays()   # clears displays at robot

                            return                       # this return closes a successfull cycle

                
                
                
                if device=='laptop':                     # case the script is running on a PC/laptop (not the robot)
                    if cv_wow:                           # case cv_wow variable is set true on __main__
                        cv2.namedWindow('Cube')          # create the Cube window
                        cv2.moveWindow('Cube', 0, h+2*gap_h)  # move the Cube window to coordinate
                        cv2.imshow('Cube', frame)        # shows the frame 
                        cv2.waitKey(1)                   # refresh time is minimized (1ms), yet real time is much higher
                    elif not cv_wow:                     # case cv_wow variable is set false on __main__
                        if fixWindPos:                   # case the fixWindPos variable is set true on __main__ 
                            cv2.namedWindow('cube')      # create the cube window
                            cv2.moveWindow('cube', 0,0)  # move the window to (0,0)
                        cv2.imshow('cube', frame)        # shows the frame 
                        cv2.waitKey(1)                   # refresh time is minimized (1ms), yet real time is much higher



                    ######################################################################
                    #### below part relates to the way to interrupt the CV2 windows,  ####
                    #### at different moment in time of the script execution          ####
                    ######################################################################
                    
                    try:                  # tentative
                        close_win = check_window_close_req(cv_wow, key)  # function that verifies if requests to close the window
                        if close_win:     # case the function returns a booleas true
                            break         # the for loop is interrupted
                    except:               # case an exception is raised
                        pass              # nothing is done


            if device=='laptop':      # case the script is running on a PC/laptop (not the robot)
                try:                  # tentative
                    close_win = check_window_close_req(cv_wow, key)  # function that verifies if requests to close the window
                    if close_win:     # case the function returns a booleas true
                        break         # the while loop is interrupted
                except:               # case an exception is raised
                    pass              # nothing is done
        
        
        if device=='laptop':      # case the script is running on a PC/laptop (not the robot)
            try:                  # tentative
                close_win = check_window_close_req(cv_wow, key)  # function that verifies if requests to close the window
                if close_win:     # case the function returns a booleas true
                    break         # the while loop is interrupted
            except:               # case an exception is raised
                pass              # nothing is done


        elif device=='Rpi':       # case the script is running at the robot
            if timeout==True or robot_stop==True:   # timeout or motor_motor check for Rpi
                return            # the function is terminated



    


if __name__ == "__main__":
    """This function is extremely simple in case of laptop.
    In case of robot the function takes care of few more things:
    1) initial settings.
    2) checking if it is the first cycle from the start.
    3) waits for user to press the button, and it starts the cube reading phase."""
    
    global device, side, screen, fixWindPos, first_cycle, robot_running
    
    
    ################    general settings on how the PC/robot is operated ############################
    device = hd_check()               # verify if the script is running at Rpi (robot), if not then it is the laptop
    fixWindPos = True                 # flag to fix the CV2 windows position, starting from coordinate 0,0
    first_cycle = True                # boolean variable to execute some settings only once

    debug = False                     # flag to enable/disable the debug related prints
    if args.debug != None:            # case 'debug' argument exists
        if args.debug:                # case the Cubotone.py has been launched with 'debug' argument
            debug = True              # flag to enable/disable the debug related prints is set True
    
    cv_wow = False                    # flag to enable/disable the visualization of the image analysis used to find the facelets
    if args.cv_wow != None:           # case 'cv_wow' argument exists
        if args.cv_wow:               # case the Cubotone.py has been launched with 'cv_wow' argument
            cv_wow = True             # flag to enable/disable the visualization of the image analysis used to find the facelets is set True
    
    led_usage = False                 # flag to enable/disable the usage of the led lights at top_cover
    if args.led != None:              # case 'led' argument exists
        if args.led:                  # case the Cubotone.py has been launched with 'led' argument
            led_usage = True          # flag to enable/disable the usage of the led lights at top_cover is set True
    
    delay_facelets_dec = 3            # delay to prevent the facelets detection in between cube faces change 
    if args.delay != None:            # case 'delay' argument exists
        delay_facelets_dec = abs(int(args.delay))   # delay to prevent the facelets detection in between cube faces change
        if delay_facelets_dec < 2:    # case the provided time is smaller than 2 seconds
            delay_facelets_dec = 2    # 2 seconds are assigned
        elif delay_facelets_dec > 30: # case the provided time is bigger than 30 seconds
            delay_facelets_dec = 30   # 30 seconds are assigned

    cube_scrambling = False           # flag to enable the cube scrambling before solving it
    if args.scramble != None:         # case 'scramble' argument exists
        if args.scramble:             # case the Cubotone.py has been launched with 'scramble' argument
            cube_scrambling = True    # flag to enable the cube scrambling test is set True
            cube_scrambled = False    # flag to track once the cube is scrambled is set False
    
    picamera_test = False             # flag to enable/disable the PiCamera test
    if args.picamera_test != None:    # case 'picamera_test' argument exists
        if args.picamera_test:        # case the Cubotone.py has been launched with 'picamera_test' argument
            picamera_test = True      # flag to enable/disable the PiCamera test is set True

    motors_hw = True                  # flag to enable/disable the servos and motors
    if args.no_motors != None:        # case 'picamera_test' argument exists
        if args.no_motors:            # case the Cubotone.py has been launched with 'picamera_test' argument
            motors_hw = False         # flag to enable/disable the servos and step motor is set False
    
    cube_size_adj = 0                 # cube_size_adj is initially set to zero (no adjustment)  
    if args.size != None:             # case 'size' argument exists
        cube_size_adj = int(args.size)   # cube_size_adj (wrt default set at settings) is assigned

# ###############################################################################################
    
    
    
    
    #####################    PC running specific   ##################################################
    if device == 'laptop':            # case the script is running on a PC/laptop (not the robot)
        screen = True                 # laptop or PC have a screen
        
        if debug:                     # case debug variable is set true
            print(f'\nDebug prints activated')   # feedback is printed to the terminal 
        if cv_wow:                    # case the cv image analysis plot is set true and screen related functions are activated
            fixWindPos = False        # fix position for final image is set false
        if fixWindPos:                # case the fixWindPos variable is set true     
            print(f'\nCV2 windows forced to top-left screen corner')
        if cv_wow:                    # case the cv image analysis plot is set true
            print(f'\nCv image analysis is plot on screen')   # feedback is printed to the terminal
        if args.delay != None:        # case the argument delay has been provided
            # feedback is printed to the terminal
            print(f'\nDelay of {delay_facelets_dec} secs on detecting facelets after a cube face change (PC use)')
        print()
        
        start_up()                    # starts the Kociemba server, Webcam and other settings
        cubeAF()                      # cube reading/solving function is called
        quit_func()                   # quitting function is called
    

    
    
    
    #####################    roobot running specific   ##############################################
    elif device == 'Rpi':             # case the script is running at the robot
        global  timeout, new_cycle_button   # global variables        
        
        
        #################    Faire specific settings    #############################################
        # some GPIO imputs are used to overwrite some of the settings
        # this approach to quickly change setting when demonstarting the robot at a Faire
        robot_set_GPIO()           # GPIO settings used on the Raspberry pi
        if GPIO.input(16):         # case the GPIO16 is at level 1 (s1 switch is closed)
            debug = True           # debug related prints are activated
        if GPIO.input(20):         # case the GPIO20 is at level 1 (s2 switch is closed)
            cv_wow = True          # visualization of the image analysis used to find the facelets
        if GPIO.input(21):         # case the GPIO21 is at level 1 (s3 switch is closed)
            led_usage = True       # usage of the led lights at top_cover
        # ###########################################################################################
        
        
        start_up()                 # function to start the Kociemba server, the webcam and other settings
        
        screen_presence = check_screen_presence(debug, printout=True)  # checks if a graphical desktop screen is connected (i.e. VNC)
        
        if not screen_presence:                              # case there is not a screen connected 
            print(f'Screen related function are not activated')  # feedback is printed to the terminal 
            screen = False                                   # screen variable is set false
        
        elif screen_presence:                                # case there is a screen connected 
            print(f'\nScreen related function are activated')   # feedback is printed to the terminal 
            screen = True                                    # screen flag is set true
            if led_usage:                                    # case led_usage variable is set true
                print(f'\nLeds light activated')             # feedback is printed to the terminal 
            if debug:                                        # case debug variable is set true
                print(f'\nDebug prints activated')           # feedback is printed to the terminal 
            if cv_wow:                                       # case the cv image analysis plot is set true and screen related functions are activated
                fixWindPos = False                           # fix position for final image is set false
            if fixWindPos:                                   # case the fixWindPos variable is set true     
                print(f'\nCV2 windows forced to top-left screen corner')
            if cv_wow:                                       # case the cv image analysis plot is set true
                print(f'\nCv image analysis is plot on screen')   # feedback is printed to the terminal
            if picamera_test:                                # case the picamera_test is set true (servos and motor disabling)
                print(f'\nPiCamera test enabled')            # feedback is printed to the terminal
                test_picamera()                              # test_picamera func is called
            if not motors_hw:                                # case the motor_hd is set False
                print(f'\nPiCamera servos and step motor disabled')   # feedback is printed to the terminal
        

        res = robot_set_servo(first_cycle, motors_hw)        # settings for the servo and motor, connected to the raspberry pi
        if not res:                                          # case the servos setting returns error
            quit_func()                                      # quit function is called
        
        if cube_size_adj != 0:                               # case the cube_size_adj has been set (default is zero)
            print(f'\nCube size adjustment parameter set to {cube_size_adj} (tents of mm)')   # feedback is printed to the terminal
            servo.extra_rotation_adj(debug, cube_size_adj)   # call to the function to adjust the Cube_holder extra rotations
        
        first_cycle = False                                  # boolean variable to execute some settings only once
        if robot_stop==False and not cube_scrambling:        # case stop_button has not being pressed
            print('\n\n\nPress NEW CYCLE to start a cycle')  # print on terminal to suggest to press the robot NEW CYCLE button, as settings are done
            robot_show_press.start()                         # suggestion on the display to press the NEW CYCLE button, as settings are done
        
        while True:                                          # infinite loop, waiting for the start button to be pressed
            new_cycle_button = GPIO.input(13)                # GPIO input is assigned to a new_cycle_button variable
            if cube_scrambling:                              # case the cube_scrambling is set True
                crambling_cube()                             # scrambling_cube function is called
                time.sleep(1)                                # little sleep time
                robot_show_press.start()                     # suggestion on the display to press the NEW CYCLE button, as settings are done
            cube_scrambling = False                          # cube_scrambling variable is set False
                    
            if side == 6:                                    # this case gives the possibility to start a new cycle
                close_camera(device)                         # this is necessary to get rid of analog/digital gains previously blocked
                time.sleep(0.2)                              # little delay between the camera closing, and a new camera opening
                camera, rawCapture, width, height = webcam() # camera has to be re-initialized, to get rid of analog/digital gains previously blocked
                side = 0                                     # side is set back to 0, that allows few other start up setting at each new cycle
                
                print('\n\n\nPress NEW CYCLE to start a cycle')  # print on terminal to suggest to press the robot NEW CYCLE button, as settings are done
                robot_show_press.start()                     # suggestion on the display to press the NEW CYCLE button, as settings are done
                robot_load_status.start()                    # start the "loading" status feedback at robot_display1

            while new_cycle_button == 0:                     # after completion of a cycle (side=6) the options are to do a new cycle or quit
                press = robot_pressed_button(new_cycle_button)  # function that verifies the button pressing time  
                
                if press == 'short':                         # button short pressing time starts a new cycle
                    robot_running = True                     # robot flag when servos and motor are activated
                    clear_terminal(device)                   # cleares the terminal
                    robot_show_press.cancel()                # at robot_display2 is removed the suggestion to press the button
                    robot_display2.Clear()                   # robot_display2 is cleared
#                     BGR_mean.clear()                         # needed???  # dictionary with read colors is cleraed
#                     URFDLB_facelets_BGR_mean.clear()         # needed???  # disctionary with the read colors on URFDLB order is cleared
#                     H_mean=[]                                # needed???  # list with HUE values is cleared
                    
                    robot_stop = False                       # flag used to stop or allow robot movements: Now the robot can move
                    timeout=False                            # flag to limit the cube facelet detection time, to de-energize servo/motors in case
                    cubeAF()                                 # cube reading/solving function is called
                    
                    if quitting:                             # case the global variable quitting is set True
                        robot_stop=True                      # global variable robot_stop is set True
                        quit_func()                          # quitting function is called
                        
                    else:                                    # case the global variable quitting is set False
                        side = 6                             # side == 6 is used as precondition to start a new cycle
                        robot_running = False                # robot flag when servos and motor are activated
                        cpu_temp()                           # cpu temperature is verified, and printed at terminal
                    
                    break                                    # the cycle is closed by breaking the button related while loop
                        
                        

    #####################    PC running specific   ##################################################
    if device=='laptop':           # case the script is running on a PC/laptop (not the robot)
        print("\nClosed script")   # feedback is printed to the terminal

