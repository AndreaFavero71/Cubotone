#!/usr/bin/env bash

#######     Andrea Favero, Sept 2022   ###########################################################
#  This bash script activates the venv, and starts the Cubotone.py script IF the push button (GPIO13) isn't pressed.    
#  When quitting Cubotone.py, by pressing the same button, there are two possible situations:   
# --> The button is maintained pressed for at least other 5 secs: The infinite loop at bash script ends 
# --> The button is not maintained pressd that long, then the infinite loop at bash re-launches Cubotone.py 
# 
#################################################################################################

source /home/pi/.virtualenvs/cv/bin/activate
cd /home/pi/cube/kociemba

i=1
printf "\r\r\r\r\r###############################################\r\r"
printf "Starting Cubotone.py, from the bash script\r"
echo "cycle number: $i"
python Cubotone.py

# setting the GPIO pin function
set_input()
{
# pin 13 is used either to start the robot (short pressing time) and to stop it (long pressing time)
GPIO=13

if [ ! -d /sys/class/gpio/gpio${GPIO} ]; then
  echo "${GPIO}" > /sys/class/gpio/export
  echo "in" > /sys/class/gpio/gpio"${GPIO}"/direction
else
  echo "in" > /sys/class/gpio/gpio"${GPIO}"/direction
fi
}

while true; do
  set_input

  if [ 0 == "$(</sys/class/gpio/gpio"${GPIO}"/value)" ]; then
    printf "Quitting the bash script\n"
    break

  else
    printf "\r\r\r\r\r###############################################\r\r"
    printf "Re-starting Cubotone.py, from the bash script\r"
    ((i=i+1))
    echo "cycle number: $i"
    cd /home/pi/cube/kociemba
    python Cubotone.py
  fi
  sleep 5

done

deactivate
cd /home/pi
