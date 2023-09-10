#!/usr/bin/python
# coding: utf-8

"""
#############################################################################################################
# Andrea Favero   rev 10 September 2023
#
# This script relates to CUBOTone, my first simple Rubik's cube solver robot
# This specific script controls two servos, one stepper motor, and led module
# 
# Possible moves with this robot
# 1) Spins the complete cube ("S") laying on the bottom face: 1 means CW 90deg turns, while 3 means 90CCW turn
# 2) Flips the complete cube ("F") by "moving" the Front face to Bottom face: Only positive values are possible
# 3) Rotates the bottom layer ("R") while costraining the 2nd and 3rd layer.
# 4) The order of S, F has to be strictly followed
# 5) Example 'F1R1S3' means: 1x cube Flip, 1x (90deg) CW rotation of the 1st (Down) layer, 1x (90deg) CCW cube Spin
#
# For Rotations, the stpper motor makes a little extra rotation than target, before coming back to target; This
# is needed to recover the gaps between cube_hoder - cube - top_cover, and still getting a decent cube layers alignment.
#
#
#############################################################################################################
"""

import Adafruit_PCA9685              # Import the PCA9685 module, used to control two servos via the I2C
import RPi.GPIO as GPIO              # import RPi GPIO
import time
from time import sleep

# GPIO pins assigments
stepper_enable = 22                  # GPIO pin used to ebale/disable the stepper motor driver
stepper_dir = 17                     # GPIO pin used to control the stepper motor direction 
stepper_step = 27                    # GPIO pin used to generate the stepper motor steps
stepper_ms2 = 5                      # GPIO pin used to change the stepper motor micro-step
light_gate = 4                       # GPIO pin used to for the light_gate input signal
srv_cover = 15                       # GPIO pin used to control the top cover servo 
srv_flipper = 14                     # GPIO pin used to control the flipper servo
led1=13                              # GPIO pin used to control the led1 on top cover 
led2=12                              # GPIO pin used to control the led2 on top cover 

# overall GPIO settings
GPIO.setmode(GPIO.BCM)               # setting GPIO pins as "Broadcom SOC channel" number, these are the numbers after "GPIO"
GPIO.setwarnings(False)              # setting GPIO to don't return allarms

# stepper motor GPIO pins settings
GPIO.setup(stepper_enable,GPIO.OUT)  # GPIO pin used to ebale/disable the stepper motor driver is set as output
GPIO.setup(stepper_dir,GPIO.OUT)     # GPIO pin used to control the stepper motor direction is set as output
GPIO.setup(stepper_step,GPIO.OUT)    # GPIO pin used to generate the stepper motor steps is set as output
GPIO.setup(stepper_ms2,GPIO.OUT)     # GPIO pin used to change the stepper motor micro-step (MS2) is set as output

# light_gate GPIO pin setting
GPIO.setup(4,GPIO.IN)                # motor position sensor


def init_PCA9685():
    """ Function to initialize the PCA9685 board.
        Adafruit 16-Channel 12-bit PWM/Servo Driver - I2C interface - PCA9685."""
    
    global pwm

    try:
        pwm = Adafruit_PCA9685.PCA9685()     # Initialise the PCA9685 using the default address (0x40).
        pwm.set_pwm_freq(60)                 # sets the frequency to 60hz, good for servos.
        pwm.set_pwm(srv_cover, 0, 0)         # servo is forced off
        pwm.set_pwm(srv_flipper, 0, 0)       # servo is forced off
        pwm.set_pwm(led1, 0, 0)              # led1 is forced off, 60Hz is ok to set the output off for the led
        pwm.set_pwm(led2, 0, 0)              # led2 is forced off, 60Hz is ok to set the output off for the led
    except:
        print(f"\n\n#############################################################################")
        print("WARNING: failed to load Adafruit_PCA9685 module")
        print("This happens if the module is not found at the I2C, or library not installed.")
        print(f"#############################################################################\n")
        raise




def init_servo(servos_init, debug, motor_hw = True):
    """ Function to initialize the robot (servos and motor position) and some global variables, do be called once, at the start.
    Parameters are imported from a json file, to make easier to list/document/change the variables
    that are expected to vary on each robot.
    These servo_settings are under a function, instead of root, to don't be executed when this script is used from CLI
    with arguments; In this other case the aim is to help setting the servo to their mid position."""
    
    global cover_close, cover_open, cover_read, flipper_low, flipper_high
    global time_cover_closing, time_cover_opening, time_cover_reading
    global time_flipper_low, time_flipper_high, time_consec_flip
    global ramp, spin_fast_ramp, spin_slow_ramp, time_on_spin_fast, time_on_spin_slow
    global align_repeats, time_on_align_fast, time_on_align_precise, timeout
    global stp_rev, extra_steps_s, extra_steps_m, extra_steps_align, align_blind_steps
    global led_brightness, motor_reversed, motors    
    
    if not servos_init:                                               # case the inititialization status of the servos is false

        if motor_hw:                                                  # case the motor_hd variable is True
            motors = True                                             # global variable motors is set True
            init_PCA9685()                                            # PCA9685 initialization function is called
        else:                                                         # case the motor_hd variable is False
            motors = False                                            # global variable motors set False
        
        # convenient choice for Andrea Favero, to upload the settings fitting my robot, via mac address check                
        from getmac import get_mac_address                            # library to get the device MAC ddress
        import os.path, pathlib, json                                 # libraries needed for the json, and parameter import
        
        folder = pathlib.Path().resolve()                             # active folder (should be home/pi/cube)  
        eth_mac = get_mac_address()                                   # mac address is retrieved
        if eth_mac == 'e4:5f:01:0a:31:ce':                            # case the script is running on AF (Andrea Favero) robot
            fname = os.path.join(folder,'Cubotone_servos_settings_AF.txt')  # AF robot settings (do not use these at the start) 
        else:                                                         # case the script is not running on AF (Andrea Favero) robot
            fname = os.path.join(folder,'Cubotone_servos_settings.txt')  # folder and file name for the settings, to be tuned               
        
        if os.path.exists(fname):                                           # case the servo_settings file exists
            with open(fname, "r") as f:                                     # servo_settings file is opened in reading mode
                servo_settings = json.load(f)                               # json file is parsed to a local dict variable
            
            if debug:                                                       # case the variable debug is set True
                print('\nimporting servos settings from the text file:', fname)    # feedback is printed to the terminal
                print('imported servos settings:')                                 # feedback is printed to the terminal
                for parameter, setting in servo_settings.items():           # iteration over the settings dict
                    print(parameter,': ', setting)                          # feedback is printed to the terminal
                print()
                
            backup_fname = os.path.join(folder,'Cubotone_servos_settings_backup.txt')  # folder and file name for the settings backup
            with open(backup_fname, 'w') as f:                                    # servo_settings_backup file is opened in writing mode
                f.write(json.dumps(servo_settings, indent=0))                     # content of the setting file is saved in another file, as backup
                if debug:                                                         # case the variable debug is set True
                    print('\nsaving a backup text file of the settings:', fname)  # feedback is printed to the terminal
                    print()
            
            try:
                cover_close = int(servo_settings['cover_close'])                  # Top_cover close position
                cover_open = int(servo_settings['cover_open'])                    # Top_cover open position
                cover_read = int(servo_settings['cover_read'])                    # Top_cover camera read position
                flipper_low = int(servo_settings['flipper_low'])                  # Top_cover flip low position
                flipper_high = int(servo_settings['flipper_high'])                # Top_cover flip high position
                time_cover_closing = float(servo_settings['time_cover_closing'])  # time for Top_cover to reach the close position
                time_cover_opening = float(servo_settings['time_cover_opening'])  # time for Top_cover from close to flip position
                time_cover_reading = float(servo_settings['time_cover_reading'])  # time for Top_cover to reach the read position
                time_flipper_low = float(servo_settings['time_flipper_low'])      # time for Flipper to reach the low position
                time_flipper_high = float(servo_settings['time_flipper_high'])    # time for Flipper to reach the high position
                time_consec_flip = float(servo_settings['time_consec_flip'])      # time for consecutive flipping, when a flip is followed by others
                
                ramp = float(servo_settings['ramp'])                              # time variation for the stepper motor ramp
                spin_fast_ramp = float(servo_settings['spin_fast_ramp'])          # time variation for the stepper motor ramp, when spinning fast
                spin_slow_ramp = float(servo_settings['spin_slow_ramp'])          # time variation for the stepper motor ramp, when spinning slow
                time_on_spin_fast = float(servo_settings['time_on_spin_fast'])    # time ON stepper for spinning fast
                time_on_spin_slow = float(servo_settings['time_on_spin_slow'])    # time ON stepper for spinning slow
                time_on_align_fast = float(servo_settings['time_on_align_fast'])  # time ON stepper for alignment fast part
                time_on_align_precise = float(servo_settings['time_on_align_precise'])  # time ON stepper for alignment precise part
                
                stp_rev = int(servo_settings['stp_rev'])                          # number of motor steps per revolution, considering the microstep setting
                extra_steps_s = int(servo_settings['extra_steps_s'])              # number of motor additional steps when rotating 90 deg (s=single rot)
                extra_steps_m = int(servo_settings['extra_steps_m'])              # number of motor additional steps when rotating more than 90 deg (m=multi rot)
                extra_steps_align = int(servo_settings['extra_steps_align'])      # number of motor additional steps when aligning the motor
                align_blind_steps = int(servo_settings['align_blind_steps'])      # number of initial motor steps, at precise alignment, to 're-enter' the sync. disk slots
                align_repeats = int(servo_settings['align_repeats'])              # max number of motor alignment iterations, when not ending successfully
                timeout = float(servo_settings['timeout'])                        # timeout in secs for motor alignment
                motor_reversed = int(servo_settings['motor_reversed'])            # reverse the motor direction behavior (0=default, 1=reversed)
                
                if motor_reversed <= 0:                                           # case the motor_reversed argument is <= zero
                    motor_reversed = 0                                            # motor_reversed variable is set to zero
                elif motor_reversed >= 1:                                         # case the motor_reversed argument is >= one
                    motor_reversed = 1                                            # motor_reversed variable is set to one

                servo_start_positions(debug)                                      # servos are positioned to the start position
                led_test(debug)                                                   # top_cover Leds are shortly energized as feedback
                led_off()                                                         # top_cover Led are set off
                servos_init = True                                                # servos_init_status variable is set True
                
                return servos_init                                                # return servos_init_status variable
            
            
            except:   # exception will be raised if json keys differs, or parameters cannot be converted to float
                print('\nerror on converting to float the imported parameters')   # feedback is printed to the terminal                                  
                servos_init = False                                               # servos_init_status variable is set False
                return servos_init                                                # return servos_init_status variable
        
        else:                                                                     # case the servo_settings file does not exists, or name differs
            print('\ncould not find Cubotino_T_servo_settings.txt')                 # feedback is printed to the terminal                                  
            servos_init = False                                                   # servos_init_status variable is set False
            return servos_init                                                    # return servos_init_status variable





def extra_rotation_adj(debug, cube_size_adj):
    """modifies the extra rotations (SPIN and ROTATE) based on the cube size difference from the default setting.
       Cube size difference is expressed in tent of millimeters"""
    
    global extra_steps_s, extra_steps_m
    
    if debug:                                  # case the variable debug is set True
        print("original extra_steps_s", extra_steps_s)  # feedback is printed to Terminal
        print("original extra_steps_m", extra_steps_m)  # feedback is printed to Terminal
        
    extra_steps_s = int(extra_steps_s - cube_size_adj*extra_steps_s/20)  # extra_steps_s is adjusted
    extra_steps_m = int(extra_steps_m - cube_size_adj*extra_steps_m/20)  # extra_steps_m is adjusted
    
    if debug:                                  # case the variable debug is set True
        print("adjusted extra_steps_s", extra_steps_s)  # feedback is printed to Terminal
        print("adjusted extra_steps_m", extra_steps_m)  # feedback is printed to Terminal




def servo_freq(debug):
    """sets the PCA9685 output frequency for the servo. Servo require tipically 60Hz."""
    
    if not motors:
        return
    
    pwm.set_pwm_freq(60)                        # sets the frequency to 60Hz
    time.sleep(0.05)                            # little time sleep to let the parameter change to get processed by the PCA 9685 board
#     if debug:                                   # case the variable debug is set True
#         print('frequency to 60Hz to PCA 9685 board, for servo control') # feedback is printed to the terminal





def led_freq(debug):
    """sets the PCA9685 output frequency for the leds. PWM Leds at 60Hz is bad for the camera, and at least 1KHz needed."""
    
    if not motors:
        return
    
    servo_off(debug=False)                      # set the servo to off, to prevent unexpected reaction at the frequency change
    pwm.set_pwm_freq(1000)                      # sets the frequency to 1000Hz
    time.sleep(0.05)                            # little time sleep to let the parameter change to get processed by the PCA 9685 board
#     if debug:                                   # case the variable debug is set True
#         print('frequency set to 1000Hz to PCA 9685 board, for led control') # feedback is printed to the terminal





def servo_start_positions(debug,wait_time=True):
    """initial position for the top cover and cube flipper; The parameter defines the sleep time before returning"""
    
    if not motors:
        return

    servo_freq(debug)
    read_cover()                                # initial position for the cover is fully high, to prevent interference with base rotation while zeroing it                             
    if wait_time:                               # case wait_time in ar is left as per default
        time.sleep(0.4)                         # extra time to ensure the parts are moved and the innertia has dropped
    cube_flipper_low(time_flipper_low)          # initial position for the flipper is fully high, to prevent interference with base rotation while zeroing it
    if wait_time:                               # case wait_time in ar is left as per default
        time.sleep(0.4)                         # extra time to ensure the parts are moved and the innertia has dropped
    if debug:                                   # case the variable debug is set True
        print('servos to the start position')   # feedback is printed to the terminal




def servo_off(debug):
    """disable the servo."""
    
    if not motors:
        return
    
    time.sleep(0.2)                        # some little wait time to ensure servos have completed the eventual movement, prior removing the PWM
    pwm.set_pwm(srv_cover, 0, 0)           # servo is forced off, in case the target wasn't reached in time (blocked movement)
    pwm.set_pwm(srv_flipper, 0, 0)         # servo is forced off, in case the target wasn't reached in time (blocked movement)
    if debug:                              # case the variable debug is set True
        print('servos are disabled')       # feedback is printed to the terminal




def led_cover(value=200):
    """ Sets the top_cover led ON, with brightness in arg. Value ranges from 0 to 4095, yet it will be limited to 2000"""

    if not motors:
        return
    
    if value >= 2000:                     # case arg is bigger than, or equal to 1200
        pwm.set_pwm(led1, 0, 2000)        # led1 is energized at 2000 (ca 50% of max intensity)
        pwm.set_pwm(led2, 0, 2000)        # led2 is energized at 2000 (ca 50% of max intensity)
    else:                                 # case arg is smaller than 1200
        pwm.set_pwm(led1, 0, value)       # led1 is energized at value
        pwm.set_pwm(led2, 0, value)       # led2 is energized at value




def led_off():
    """disable the led."""
    
    if not motors:
        return
    
    pwm.set_pwm(led1, 0, 0)               # led1 is forced off
    pwm.set_pwm(led2, 0, 0)               # led2 is forced off




def led_test(debug):
    """ Fade the led On and Off at the init, to show the led worwing."""
    
    if not motors:
        return
    
    led_freq(debug)
    for i in range(0,800,50):            # iterates from 0 to 700 in steps of 100
        pwm.set_pwm(led1, 0, i)           # led1 is energized at iterator value
        pwm.set_pwm(led2, 0, i)           # led2 is energized at iterator value
        time.sleep(0.01)                   # very short time sleep
    
    for i in range(700,-50,-50):        # iterates from 700 to 0 in steps of -100 
        pwm.set_pwm(led1, 0, i)           # led1 is energized at iterator value
        pwm.set_pwm(led2, 0, i)           # led2 is energized at iterator value
        time.sleep(0.01)                  # very short time sleep
    servo_freq(debug)




def read_cover():
    """Top cover position fully opened; The parameter defines the sleep time before returning""" 

    if not motors:
        return
    
    pwm.set_pwm(srv_cover, 0, cover_read)   # pwm to the servo to reach the target position
    sleep(time_cover_reading)  # (AF 0.18)  # time necessary to servo to reach the target




def open_cover(fast_speed=False):
    """Top cover position fully opened; The parameter defines the sleep time before returning""" 
    
    if not motors:
        return

    if fast_speed==True:
        pwm.set_pwm(srv_cover, 0, cover_open)    # pwm to the servo to reach the target position, without sleep time afterward
        sleep(time_cover_opening-0.05)           # (AF 0.1)   # time necessary to servo to get the top_cover out from the cube
    
    else:
        pwm.set_pwm(srv_cover, 0, cover_open)    # pwm to the servo to reach the target position, followed by a sleep time
        sleep(time_cover_opening)  # (AF 0.15)   # time necessary to servo to reach the target




def close_cover():
    """Top cover position fully closed; The parameter defines the sleep time before returning""" 
    
    if not motors:
        return
    
    pwm.set_pwm(srv_cover, 0, cover_close)       # pwm to the servo to reach the target position
    sleep(time_cover_closing)  # (AF 0.22)       # time necessary to servo to reach the target




def cube_flipper_high(time_flipper_high):
    """Flipper lever fully high, he parameter defines the sleep time before returning""" 
    
    if not motors:
        return
    
    pwm.set_pwm(srv_flipper, 0, flipper_high)     # pwm to the servo to reach the target position
    sleep(time_flipper_high)  # (AF 0.35)         # time necessary to servo to reach the target




def cube_flipper_low(time_flipper_low):
    """Flipper lever fully low, he parameter defines the sleep time before returning""" 
    
    if not motors:
        return
        
    pwm.set_pwm(srv_flipper, 0, flipper_low)      # pwm to the servo to reach the target position
    sleep(time_flipper_low)   # (AF 0.35)         # time necessary to servo to reach the target




def flip(flips, debug, align, go, fast=False):
    """Flipper sequence to flip the cube, without using the top cover as guide;
        The parameter defines the sleep time before returning""" 
    
    if not motors:
        return True
    
    if align != 'read':                            # case the aglin is not equal to 'read'
        align = 'open'                             # align is set to 'open'
    
    aligned = True                                 # boolean 'aligned' is set initially true
    if sensor_check() == False:                    # case the light_gate is False, meaning the cube holder is not aligned
        aligned = align_motor(debug, align)        # alignment function is called. The function returns false is case of alignment failure
    
    if sensor_check() == True:                         # case the light_gate is True, meaning the cube holder is aligned
        for i in range(flips):                         # itaration over the number of flips to be done
            if go == True:                             # case 'go' variable is set true 
                cube_flipper_high(time_flipper_high)   # Flipper is raised to its max position
            if fast==True and i==flips-1:              # case a flip isn't followed by anoter one, then is possible to minimize the sleep time while the cover is lowering
                cube_flipper_low(time_consec_flip)     # AF(0.18)  # Flipper is lowered to its min position, with smaller sleeping time afterward 
            else:                                      # case a flip is followed by another one
                cube_flipper_low(time_flipper_low)     # Flipper is lowered to its min position, with sleeping time that also allows other flippings
    
    return aligned



def motor_off(debug):
    """disable the motor."""

    if not motors:
        return
    
    GPIO.output(stepper_enable,GPIO.HIGH)          # driver disabled
    if debug:                                    # case the variable debug is set True
        print('stepper motor is disabled')         # feedback is printed to the terminal




def motor_steps(n, on_time, off_time, variation, debug):
    """generates the steps for the stepper motor."""
    starting = int (n*0.4)                     # first amount of steps, used to accelerate: 0.4 means 40%
    keeping = int (n*0.2)                      # second amount of steps, used to maintaine the rotation speed
    stopping = n-starting-keeping              # remaining amount of steps, used to decelerate
    
#     variation = 0.012   # (AF DRV A4988) variation coefficient, over steps, to accelerate / decelerate the stepper motor
#     variation = 0.015   # (AF DRV8825) variation coefficient, over steps, to accelerate / decelerate the stepper motor
    
    if debug:                                # case the variable debug is set True
        start=time.time()                      # current time is assigned to the start variable, to measure the spin or rotation time
    for i in range(starting):                  # iteration over the steps intended for the speed increment ramp
        GPIO.output(stepper_step,GPIO.HIGH)    # set GPIO high for one motor step
        time.sleep(on_time)                    # set the pulse ON time
        GPIO.output(stepper_step,GPIO.LOW)     # set motor's GPIO low for
        time.sleep(off_time)                   # set the pulse OFF time
        on_time=on_time*(1-variation)          # ON waiting period is reduced to increase speed
        off_time=on_time*2.5                   # OFF waiting period is proportional to the ON perion

    for i in range(keeping):                   # iteration over the steps intended for the constant speed rotation part
        GPIO.output(stepper_step,GPIO.HIGH)    # set GPIO high for one motor step
        time.sleep(on_time)                    # set the pulse ON time
        GPIO.output(stepper_step,GPIO.LOW)     # set motor's GPIO low for
        time.sleep(off_time)                   # set the pulse OFF time
    
    for i in range(stopping):                  # iteration over the steps intended for the speed decrement ramp
        GPIO.output(stepper_step,GPIO.HIGH)    # set GPIO high for one motor step
        time.sleep(on_time)                    # set the pulse ON time
        GPIO.output(stepper_step,GPIO.LOW)     # set motor's GPIO low for
        time.sleep(off_time)                   # set the pulse OFF time
        on_time=on_time*(1.002+variation)      # ON waiting period is increased to reduce speed
        off_time=on_time*2.5                   # OFF waiting period is proportional to the ON perion
    
    if debug:                                # case the variable debug is set True
        stop=time.time()                       # current time is assigned to the stop variable, to measure the spin or rotation time
        if n>stp_rev//2:                       # case the spin or rotation takes at least one quarter of revolution (by considering microsteps)
            print('rotation time:', round(stop-start,3))         # time feedback is printed to the terminal




def spin(rotations, debug, speed='high'):
    """cube_holder spinning function (meaning rotation of the cube_holder with opened top_cover)."""
    
    if not motors:
        return
    
    if speed == 'high':                     # two main rotation speed are set: high is meant when there is no cube layer to rotate
        on_time=time_on_spin_fast           # (AF 0.001 for DRV8825, 0.00065 for DRV A4988)
        variation=spin_fast_ramp            # (AF 0.024) time variation used to generate a stepper ramp on speed
    
    elif speed == 'low':                    # two main rotation speed are set: low is meant when there is cube layer rotation (higher torque needed)
        on_time=time_on_spin_slow           # (AF 0.0014 for DRV8825 and for DRV A4988)
        variation=spin_slow_ramp            # (AF 0.018) time variation used to generate a stepper ramp on speed
    
    off_time=on_time*2.5                    # the "off" period at step motor signal is related to the "on" one
    GPIO.output(stepper_enable,GPIO.LOW)    # driver enabled
    time.sleep(0.05)                        # delay after parameter change
    
    
    forward_steps = stp_rev//2 + extra_steps_s      # (AF 108) # forward means the steps on the intended direction, including a small over-rotation to recover gaps
    backward_steps = extra_steps_s                  # backward means the steps to opposite of direction, to recover the extra-rotation (and tensions release)
    if abs(rotations)>1:
        forward_steps = stp_rev//2 + extra_steps_m  # forward means the steps on the intended direction, including a small over-rotation to recover gaps
        backward_steps = extra_steps_m              # backward means the steps to opposite of direction, to recover the extra-rotation (and tensions release)

    # this part relates to forward rotation, meant for the main rotation
    if rotations >0 :                               # case the rotation is CW (from motor point of view)
        if motor_reversed:                          # case the motor_reversed is set True
            GPIO.output(stepper_dir,GPIO.HIGH)      # motor is set to CW direction
        else:                                       # case the motor_reversed is set False
            GPIO.output(stepper_dir,GPIO.LOW)       # motor is set to CW direction
    
    elif rotations <0 :                             # case the rotation is CCW (from motor point of view)
        if motor_reversed:                          # case the motor_reversed is set True
            GPIO.output(stepper_dir,GPIO.LOW)       # motor is set to CCW direction
        else:                                       # case the motor_reversed is set False
            GPIO.output(stepper_dir,GPIO.HIGH)      # motor is set to CCW direction

    time.sleep(0.05)                                # short delay after parameter change     
    rot=(abs(forward_steps*rotations))              # amount of forward steps
    motor_steps(rot, on_time, off_time, variation, debug)  # motor is activated, face rotation(s) is made
    
    # his part relates to the backward rotation, meant to release tension between the cube and the cover
    if rotations >0 :                               # case the main rotation is CW the backward rot is CCW (from motor point of view)
        if motor_reversed:                          # case the motor_reversed is set True
            GPIO.output(stepper_dir,GPIO.LOW)       # motor is set to CCW direction
        else:                                       # case the motor_reversed is set False
            GPIO.output(stepper_dir,GPIO.HIGH)      # motor is set to CCW direction
    elif rotations <0 :                             # case the main rotation is CCW the backward rot is CW (from motor point of view)
        if motor_reversed:                          # case the motor_reversed is set True
            GPIO.output(stepper_dir,GPIO.HIGH)      # motor is set to CW direction
        else:                                       # case the motor_reversed is set False
            GPIO.output(stepper_dir,GPIO.LOW)       # motor is set to CW direction
        
    time.sleep(0.05)                                # delay after parameter change and for innertia/direction change 
    rot=(abs(backward_steps*rotations))             # amount of backforward steps
    motor_steps(rot, on_time, off_time, 0, debug)   # motor is activated, over-rotation is recovered
#     time.sleep(0.05)                                # small delay at rotation end





def rotate(rotations, debug, fast_speed=False):
    """cube_holder rotation function (meaning spinning the cube_holder with closed top_cover)."""
    
    if not motors:
        return
        
    if sensor_check() == False:         # check motor alignment prior to start
        align_motor(debug, 'open')      # motor is aligned in case it wasn't
    close_cover()                       # top cover is closed
    spin(rotations, debug, 'low')       # spis are made, at low speed as higher torques is needed to rotate the cube's layes
    if sensor_check() == False:         # motor sensor is check after the rotation
        align_motor(debug,'close')      # if needed the motor is aligned with closed cover, to properly align the cube's layer to the others
    open_cover(fast_speed)              # top cover is opened, without sleep time afterward





def sensor_check():
    """return a boolean according to the light_gate signal status."""
    return GPIO.input(light_gate)==0


            

def align_motor(debug, cover='read'):
    """Motor alignment to the motor's sensor')"""
    
    if not motors:        # case motors is set false (testing robot without servos and step motor)
        return True       # function is returned without aligning the motor
    
    alignment_ok = False                                 # boolean alignment_ok is set initially false
    
    if sensor_check() == 1:                              # case the light_gate sensor is not sensing the sync disk slot
        if debug:                                        # case the variable debug is set True
            print('\nsynch disk slot already located at the ligh_gate') # feedback is printed to the terminal
        GPIO.output(stepper_enable,GPIO.LOW)             # enable motor, to maintain the cube holder forced in the aligned position
        if cover == 'read':                              # case the top_cover is in 'read' position
            servo_start_positions(debug, False)          # servos are set to the start position
#         elif cover == 'open':                            # case the top_cover is in 'open' position
#             open_cover()                                 # top_cover is opened
        alignment_ok = True                              # boolean alignment_ok is set true
        return alignment_ok                              # function is terminated
    
    else:                                                # case the light_gate sensor is sensing the sync disk slot
#         if debug:                                        # case the variable debug is set True
        print('\nsynch disk slot not located at the ligh_gate')     # feedback is printed to the terminal
        print("motor alignment to the motor's sensor")              # feedback is printed to the terminal
    if cover == 'read':                                  # case the top_cover is in 'read' position
        servo_start_positions(debug, False)              # servos are set to the start position
#     elif cover == 'open':                                # case the top_cover is in 'open' position
#         open_cover()                                     # top_cover is opened
    
    reverse_main_align_dir = False
    alignment_timeout = False                            # boolean alignment_ok is set initially false
    align_start_time = time.time()                       # current time iss assigned to the align_start_time variable

    while alignment_ok == False:                         # while function until alignment_ok is false
        if alignment_timeout == True:                    # case alignment_timeout variable is true
            break                                        # while loop is interrupted
        
        if time.time()-align_start_time > timeout:       # case the alignment takes more than 2 seconds
            if alignment_timeout == False:               # case alignment_timeout variable is false
                alignment_timeout = True                 # alignment_timeout variable is set true
                print('alignment timeout\n')             # feedback is printed to the terminal    
                alignment_ok = False                     # boolean alignment_ok is set false
                return alignment_ok                      # function is terminated
            
        GPIO.output(stepper_enable,GPIO.LOW)             # enable motor
        GPIO.output(stepper_ms2,GPIO.HIGH)               # enable microsteps (1/8 step, 800pls/revolution)
        time.sleep(0.01)                                 # short delay after motor setting

        on_time = time_on_align_fast                     # (AF 0.0004) set the pulse ON time for fast rotation
        off_time = on_time*2.5                           # set pulse OFF time, based on ON pulse time
        position = 0                                     # counter used to know the initial position, when the motor alignment is called
        extra_steps = extra_steps_align                  # (AF 3) number of extra steps           
        
        if sensor_check() == 0:                          # in case the sensor is OFF, fast CW rotation until the sensing area (sync. disk slot)
            if motor_reversed:                           # case the motor_reversed is set True
                GPIO.output(stepper_dir,GPIO.HIGH)       # motor is set to CW direction
            else:                                        # case the motor_reversed is set False
                GPIO.output(stepper_dir,GPIO.LOW)        # motor is set to CW direction
            time.sleep(0.01)                             # short delay after motor direction setting
            while GPIO.input(light_gate):                # motor rotates until fist sensor ON reaction
                if time.time()-align_start_time > timeout:     # case the alignment takes more than 2 seconds
                    if alignment_timeout == False:       # case alignment_timeout variable is false
                        alignment_timeout = True         # alignment_timeout variable is set true
                        print('alignment timeout\n')     # feedback is printed to the terminal
                        alignment_ok = False             # boolean alignment_ok is set false
                        return alignment_ok              # function is terminated
                
                GPIO.output(stepper_step,GPIO.HIGH)      # set GPIO high for one motor step
                time.sleep(on_time)                      # set the pulse ON time
                GPIO.output(stepper_step,GPIO.LOW)       # set motor's GPIO low for
                time.sleep(off_time)                     # set the pulse OFF time
                position += 1              # position counter is used to check if the out of position is greater than 45degrees (>200 microsteps, or > stp_rev)
                
                if position > stp_rev:                          # case the rotation was taking more than 45 degrees
                    if debug:                                   # case the variable debug is set True
                        print('reversed the main alignment direction') # feedback is printed to the terminal
                    if motor_reversed:                          # case the motor_reversed is set True
                        GPIO.output(stepper_dir,GPIO.LOW)       # motor is set to CCW direction
                    else:                                       # case the motor_reversed is set False
                        GPIO.output(stepper_dir,GPIO.HIGH)      # motor is set to CCW direction
                    time.sleep(0.01)                            # short delay after motor direction setting
                    reverse_main_align_dir=True
                    break
            
            
            if reverse_main_align_dir:
                while GPIO.input(light_gate):                # motor rotates until fist sensor ON reaction
                    if time.time()-align_start_time > timeout:     # case the alignment takes more than 2 seconds
                        if alignment_timeout == False:       # case alignment_timeout variable is false
                            alignment_timeout = True         # alignment_timeout variable is set true
                            print('alignment timeout\n')     # feedback is printed to the terminal
                            alignment_ok = False             # boolean alignment_ok is set false
                            return alignment_ok              # function is terminated
                    
                    GPIO.output(stepper_step,GPIO.HIGH)      # set GPIO high for one motor step
                    time.sleep(on_time)                      # set the pulse ON time
                    GPIO.output(stepper_step,GPIO.LOW)       # set motor's GPIO low for
                    time.sleep(off_time)                     # set the pulse OFF time
    
            
            
            if alignment_timeout == False:               # case alignment_timeout variable is false
                print()
                for i in range(extra_steps):             # extra rotation to be sure to stop with the sensor into the sync disk slot
                    GPIO.output(stepper_step,GPIO.HIGH)  # set GPIO high for one motor step
                    time.sleep(on_time)                  # set the pulse ON time
                    GPIO.output(stepper_step,GPIO.LOW)   # set motor's GPIO low for
                    time.sleep(off_time)                 # set the pulse OFF time
            time.sleep(0.02)

        
        if alignment_timeout == False:                       # case alignment_timeout variable is false
            # precise alignment starts here, proceeding with the same main alignment rotation direction 
            for i in range(align_repeats):                   # the alignment function does max 2 attempts
                if debug:                                    # case the variable debug is set True
                    print('alignment attempt number:', i+1)  # feedback is printed to the terminal    
                width = 0                                    # counter for the motor's step while the sensor is active through the opening slots
                on_time = time_on_align_precise              # (AF 0.0012) set the pulse ON time for precise alignment
                off_time=on_time*2.5                         # set pulse OFF time
                while not GPIO.input(light_gate):            # keeps rotating until the sensor goes off (slot beginning)
                    if time.time()-align_start_time > timeout:    # case the alignment takes more than 2 seconds
                        if alignment_timeout == False:       # case alignment_timeout variable is false
                            alignment_timeout = True         # alignment_timeout variable is set true
                            print('alignment timeout\n')     # feedback is printed to the terminal
                            alignment_ok = False             # boolean alignment_ok is set false
                            return alignment_ok              # function is terminated
                    GPIO.output(stepper_step,GPIO.HIGH)      # set GPIO high for one motor step
                    time.sleep(on_time)                      # set the pulse ON time
                    GPIO.output(stepper_step,GPIO.LOW)       # set motor's GPIO low for
                    time.sleep(off_time)                     # set the pulse OFF time
                for i in range(extra_steps):                 # extra rotation to be sure to stop after the "on" sensor slot
                    GPIO.output(stepper_step,GPIO.HIGH)      # set GPIO high for one motor step
                    time.sleep(on_time)                      # set the pulse ON time
                    GPIO.output(stepper_step,GPIO.LOW)       # set motor's GPIO low for
                    time.sleep(off_time)                     # set the pulse OFF time
                time.sleep(0.02)
                
                if reverse_main_align_dir:                   # case the main alignment direction has been reversed
                    if motor_reversed:                       # case the motor_reversed is set True
                        GPIO.output(stepper_dir,GPIO.HIGH)   # motor is set to CW direction
                    else:                                    # case the motor_reversed is set False
                        GPIO.output(stepper_dir,GPIO.LOW)    # motor is set to CW direction
                elif not reverse_main_align_dir:             # case the main alignment direction has not been reversed
                    if motor_reversed:                       # case the motor_reversed is set True
                        GPIO.output(stepper_dir,GPIO.LOW)    # motor is set to CCW direction
                    else:                                    # case the motor_reversed is set False
                        GPIO.output(stepper_dir,GPIO.HIGH)   # motor is set to CCW direction
                time.sleep(0.02)
                
                if alignment_timeout == False:               # case alignment_timeout variable is false
                    for i in range(align_blind_steps):       # initial rotation doesn't look at the sensor, to ensure the re-entering in the ON part
                        GPIO.output(stepper_step,GPIO.HIGH)  # set GPIO high for one motor step
                        time.sleep(on_time)                  # set the pulse ON time
                        GPIO.output(stepper_step,GPIO.LOW)   # set motor's GPIO low for
                        time.sleep(off_time)                 # set the pulse OFF time
                        width += 1                           # counter is incremented

                    while not GPIO.input(light_gate):        # keeps rotating (CCW) until the sensor goes off (slot end)
                        if time.time()-align_start_time > timeout:    # case the alignment takes more than 2 seconds
                            if alignment_timeout == False:   # case alignment_timeout variable is false
                                alignment_timeout = True     # alignment_timeout variable is set true
                                print('alignment timeout\n') # feedback is printed to the terminal
                                alignment_ok = False         # boolean alignment_ok is set false
                                return alignment_ok          # function is terminated
                        
                        GPIO.output(stepper_step,GPIO.HIGH)  # set GPIO high for one motor step
                        time.sleep(on_time)                  # set the pulse ON time
                        GPIO.output(stepper_step,GPIO.LOW)   # set motor's GPIO low for
                        time.sleep(off_time)                 # set the pulse OFF time
                        width += 1                           # counter is incremented

                if debug:                                    # case the variable debug is set True 
                    print('sync disk slot width, in stepper steps:', width)        # feedback is printed to the terminal 
                
                if reverse_main_align_dir:                   # case the main alignment direction has been reversed
                    if motor_reversed:                       # case the motor_reversed is set True
                        GPIO.output(stepper_dir,GPIO.LOW)    # motor is set to CCW direction
                    else:                                    # case the motor_reversed is set False
                        GPIO.output(stepper_dir,GPIO.HIGH)   # motor is set to CCW direction
                    time.sleep(0.02)                         # small delay after setting change
                
                elif not reverse_main_align_dir:             # case the main alignment direction has not been reversed
                    if motor_reversed:                       # case the motor_reversed is set True
                        GPIO.output(stepper_dir,GPIO.HIGH)   # motor is set to CW direction
                    else:                                    # case the motor_reversed is set False
                        GPIO.output(stepper_dir,GPIO.LOW)    # motor is set to CW direction
                    time.sleep(0.02)                         # small delay after setting change
                
                positioning = int(width/2)                   # half slot is considered
                if debug:                                    # case the variable debug is set True 
                    print('positioning to half width:', positioning)   # feedback is printed to the terminal 
                
                for i in range(positioning):
                    GPIO.output(stepper_step,GPIO.HIGH)      # set GPIO high for one motor step
                    time.sleep(int(on_time/2))               # set the pulse ON time
                    GPIO.output(stepper_step,GPIO.LOW)       # set motor's GPIO low for
                    time.sleep(off_time)                     # set the pulse OFF time
            
                if sensor_check():                           # sensor is verified once the motor stops
                    alignment_ok = True                      # boolean alignment_ok is finally set true
                    if debug:                                # case the variable debug is set True 
                        print('alignment is OK\n')           # feedback is printed to the terminal
                    break                                    # if the sensor is ON then the alignment attempt is closed
    #             time.sleep(0.05)
            
        if sensor_check() != 1:                         # if sensor not ON after previous alignment attempts
            alignment_ok = False                        # boolean alignment_ok is set false
            if debug:                                   # case the variable debug is set True
                print('alignment did not succeeded')    # feedback is printed to the terminal
        
        GPIO.output(stepper_ms2,GPIO.LOW)               # microstep disabled (from 1/8 step to 1/2 step)
        time.sleep(0.05)                                # small delay after setting change
        
#         if position > stp_rev:   # case motor alignment took more than 45 degress (stp_rev steps, once in microsteps) a cube face rotation is needed (CCW)
#             if not alignment_timeout:                   # case alignment_timeout variable is false
#                 if cover == 'read' or cover == 'open':  # case the top_cover is in 'read' or 'open' position
#                     spin(-1, debug, 'high')             # the cube_holder is rotated back by one quarter of revolution, at high speed
#                 else:                                   # case the top_cover is not in 'read' or 'open' position
#                     spin(-1, debug, 'low')              # the cube_holder is rotated back by one quarter of revolution, at low speed
#                 
#                 if debug:                               # case the variable debug is set True
#                     time.sleep(0.2)                     # little delay to allows the cube holder stopping from inertia
#                     if sensor_check():                  # sensor is verified (1= sync disk slot at light_gate)
#                         print('quarter of revolution back, alignment is OK')        # feedback is printed to the terminal
#                     else:                               # sensor is not found as ok once the motor stops
#                         print('quarter of revolution back, alignment not perfect')  # feedback is printed to the terminal
    
    if alignment_timeout or not alignment_ok:           # case alignment_timeout variable is true or alignmento_ok is false
        motor_off(debug)                                # motor is de-energized
        print('\n\n\n\nMOTOR ALIGNMENT HAS FAILED\n\n\n\n')   # feedback is printed to the terminal
        return False
    else:
        return True
    




def solve_cube(robot, debug):
    """function that actuates the servos and the stepper motor to solve the cube, according to the received dict of robot movements."""

    if not motors:        # case motors is set false (testing robot without servos and step motor)
        return 'test with motors disabled'     # function is returned without aligning the motor
    
    for moves in robot.values():                                # iteration over the received dict values with the robot movements
        if debug:                                               # case the variable debug is set True
            print(moves)                                        # feedback is printed to the terminal
        spins = int(moves[moves.find('S')+1:moves.find('F')])   # retrives the spin direction and quantity
        flips = int(moves[moves.find('F')+1:moves.find('R')])   # retrives the flips quantity
        rotations = int(moves[moves.find('R')+1:])              # retrives the rotation direction and quantity
        if spins != 0:                                          # case the spin quantity is not zero
            spin(spins, debug)                                  # spin is applied
        if flips != 0:                                          # case the flips quantity is not zero
            aligned = flip(flips, debug, align='close', go=True)          # flips are applied 
            if not aligned:
                break
        if rotations != 0:                                      # case the spin quantity is not zero
            rotate(rotations, debug)                            # rotation is applied
    
    if not aligned:
        print("\nCannot align the motor (cube_holder), script is terminated")
        return 'alignment issue'
    else:
        return 'done'
    
    



def fun(debug):
    """spins the cube_holder back and forward a few times to capture attention once the cube is solved."""
    
    if not motors:        # case motors is set false (testing robot without servos and step motor)
        return            # function is returned without aligning the motor
    
    if debug:                                     # case the variable debug is set True
        print('\nfun function')                   # feedback is printed to the terminal
    
    GPIO.output(stepper_ms2,GPIO.HIGH)            # enable microsteps (1/8 step, 800pls/revolution)
    time.sleep(0.05)                              # short delay after parameter change 

    GPIO.output(stepper_enable,GPIO.LOW)          # driver enabled
    time.sleep(0.05)                              # delay after parameter change
        
    if motor_reversed:                            # case the motor_reversed is set True
        GPIO.output(stepper_dir,GPIO.HIGH)        # motor is set to CW direction
    else:                                         # case the motor_reversed is set False
        GPIO.output(stepper_dir,GPIO.LOW)         # motor is set to CW direction
    time.sleep(0.05)                              # short delay after parameter change     
    
    levels=stp_rev//5                             # times the motor changes speed level
    steps = int(4*stp_rev/levels)                 # number of steps per each speed level
    variation=0.1                                 # speed change between the levels (0.1 means 10%)
    on_time=0.002                          
    for i in range(levels):                       # iteration on speed levels
        on_time=on_time*(1-variation)             # step's ON time is reduced
        off_time=on_time*2.5                      # the "off" period at step motor signal is related to the "on" one
        for i in range(steps):                    # iteration on steps
            GPIO.output(stepper_step,GPIO.HIGH)   # set GPIO high for one motor step
            time.sleep(on_time)                   # set the pulse ON time
            GPIO.output(stepper_step,GPIO.LOW)    # set motor's GPIO low for
            time.sleep(off_time)                  # set the pulse OFF time

    for i in range(levels):                       # iteration on speed levels
        on_time=on_time*(1+variation)             # step's ON time is increased
        off_time=2.5*on_time                      # the "off" period at step motor signal is related to the "on" one
        for i in range(steps):                    # iteration on steps
            GPIO.output(stepper_step,GPIO.HIGH)   # set GPIO high for one motor step
            time.sleep(on_time)                   # set the pulse ON time
            GPIO.output(stepper_step,GPIO.LOW)    # set motor's GPIO low for
            time.sleep(off_time)                  # set the pulse OFF time
    
    GPIO.output(stepper_ms2,GPIO.LOW)             # microstep disabled (from 1/8 step to 1/2 step)
    GPIO.output(stepper_enable,GPIO.HIGH)         # driver disabled
    time.sleep(0.05)                              # small delay at rotation end






##########################
##### examples ###########
##########################        


if __name__ == "__main__":
    """this is only meant to be used with an IDE: below test functions have to be recalled at REPL."""
    import time
    
    motors = True        # motors is set true (testing robot with servos and step motor)
    s_debug = True
    servos_init_status = False
    led_brightness = 1000
    servos_init_status = init_servo(servos_init_status, s_debug)
    motor_off(s_debug)
    servo_off(s_debug)
    

def test1():
    align_motor(s_debug)
    for k in range(2):
        for j in ['low', 'high']:
            if j == 'low':
                close_cover()
                time.sleep(0.1)
            elif j == 'high':
                open_cover()
            for i in [-1,1,-2,2]:
                spin(i, s_debug, j)
                time.sleep(0.1)
                print(i,j,'sensor OK at motor stop: ',sensor_check())
    
    print(sensor_check())
    time.sleep(1)
    GPIO.output(stepper_enable,GPIO.HIGH)            # driver disabled
    servo_start_positions(s_debug)
    servo_off(s_debug)
    motor_off(s_debug)




def test2():
    for i in range(3):
        now=time.time()
        close_cover()
        print('close top_cover', round(time.time()-now,3))
        time.sleep(0.5)
        now=time.time()
        open_cover()
        print('open top_cover', round(time.time()-now,3))
        time.sleep(0.5)
        print()
    GPIO.output(stepper_enable,GPIO.HIGH)            # driver disabled
    servo_start_positions(s_debug)
    servo_off(s_debug)
    motor_off(s_debug)




def test3():
    align_motor(s_debug)
    for i in range(3):
        now=time.time()
        cube_flipper_high(time_flipper_high)
        print('flipper high', round(time.time()-now,3))
        time.sleep(0.5)
        now=time.time()
        cube_flipper_low(time_flipper_low)
        print('flipper low', round(time.time()-now,3))
        time.sleep(0.5)
        print()
    GPIO.output(stepper_enable,GPIO.HIGH)            # driver disabled
    servo_start_positions(s_debug)
    servo_off(s_debug)
    motor_off(s_debug)



def test4():
    align_motor(s_debug)
    open_cover()
    robot1={0: 'S2F1R2', 1: 'S1F1R2', 2: 'S0F2R1', 3: 'S-1F1R2', 4: 'S0F1R-1', 5: 'S1F1R1', 6: 'S1F1R-1', 7: 'S1F1R2', 8: 'S0F2R-1', 9: 'S0F1R1', 10: 'S1F1R1', 11: 'S0F1R1', 12: 'S0F1R2', 13: 'S1F1R2', 14: 'S1F1R-1', 15: 'S2F1R2', 16: 'S0F1R1', 17: 'S-1F1R2', 18: 'S1F1R2', 19: 'S0F2R2'}
    solve_cube(robot1, s_debug)
    fun(s_debug)
    print()
    GPIO.output(stepper_enable,GPIO.HIGH)            # driver disabled
    servo_start_positions(s_debug)
    servo_off(s_debug)
    motor_off(s_debug)



def test5():
    align_motor(s_debug)
    open_cover()
    robot2={2: 'S0F2R1', 4: 'S0F1R-1', 8: 'S0F2R-1', 9: 'S0F1R1', 11: 'S0F1R1', 12: 'S0F1R2', 16: 'S0F1R1', 19: 'S0F2R2'}
    solve_cube(robot2,s_debug)
    fun(s_debug)
    print()
    GPIO.output(stepper_enable,GPIO.HIGH)            # driver disabled
    servo_start_positions(s_debug)
    servo_off(s_debug)
    motor_off(s_debug)




def test6():
    align_motor(s_debug)
    open_cover()
    for i in range(30):
        print('\n',i)
        led_test(debug=False)
        open_cover()
        
        

