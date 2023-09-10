#!/usr/bin/env bash

#######     Andrea Favero, 10 Sept 2023   ########################################################
#  This bash script activates the venv, and starts the Cubotone.py script IF the push button (GPIO13) isn't pressed.    
#  When quitting Cubotone.py, by pressing the same button, there are two possible situations:   
# --> The button is maintained pressed for at least other 5 secs: The infinite loop at bash script ends 
# --> The button is not maintained pressed that long, then the infinite loop at bash re-launches Cubotone.py 
# 
#################################################################################################

cd /home/pi/cubotone/src
source .virtualenvs/bin/activate

printf "\r\n\n###############################################\r\n"
printf "Loaded virtualenvs\n"
printf "Starting Cubotone.py, from the bash script\n"
printf "###############################################\r\n"

python Cubotone.py

# setting the GPIO pin function
set_input()
{
# pin 13 is used either to start the robot (short pressing time) and to stop it (long pressing time)
GPIO=13
if [ -d "/sys/class/gpio/gpio"${GPIO} ]; then           # gpio folder does exist
  # printf "GPIO: ${GPIO} already exported\r\n"         # print feedbact to terminal
  echo "in" > /sys/class/gpio/gpio${GPIO}/direction     # set gpio as input

else                                                    # gpio folder does not exist
  # printf "exporting GPIO: ${GPIO}\r\n"                # print feedbact to terminal
  echo "${GPIO}" > /sys/class/gpio/export               # export gpio
  echo "in" > /sys/class/gpio/gpio${GPIO}/direction     # set gpio as input

fi
}


while true; do
  set_input

  if [ 0 == "$(</sys/class/gpio/gpio${GPIO}/value)" ]; then
    printf "Quitting the bash script\n"
    break

  else
    printf "\r\n\n\n\n###############################################\r\n"
    printf "Re-starting Cubotone.py, from the bash script\n"
    printf "###############################################\r\n"
    cd /home/pi/cubotone/src
    python Cubotone.py
  fi
  sleep 5

done

if [ -d "/sys/class/gpio/gpio"${GPIO} ]; then
  echo "${GPIO}" > /sys/class/gpio/unexport             # unexport gpio
  # printf "unexporting GPIO: ${GPIO}\r\n"              # print feedbact to terminal
fi


deactivate
cd /home/pi
printf "closed the bash script\r\n"
printf "\r\n"
