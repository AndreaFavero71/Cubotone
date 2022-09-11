# CUBOTone

This repo contains the relevant files for ....

How to make CUBOTone, my first Rubik’s cube solver robot./n
Differently from the newer CUBOTino series, CUBOTone uses servos and stepper motor resulting in a faster solving time.

![title image](/images/title.jpg)

Further robot info at: https://www.instructables.com/Rubik-Cube-Solver-Robot-With-Raspberry-Pi-and-Pica/

An impression of the robot: https://www.youtube.com/watch?v=oYRXe4NyJqs

This git repo simplifies the download of the individual files and automates as much a possible the deployment to the Raspberry Pi

This installation has proved to work on different Raspberry Pi models; Please report in case you experience issues.

# How to use it:
1. Flash your SD card according to the procedure in the [document here](doc/SD_and_Rpi_settings_20211003.pdf) , Step 1
2. Put the sd card in the pi and power it. You can monitor the boot process if you connect an hdmi monitor to it but it is not essential. 
3. Try to connect to the Raspberry Pi via SSH. On Windows you can use Putty. On linux and mac you can type directly:
```
ssh pi@cubotone.local
```
4. If you can’t reach the pi like this, you will have to scan your network to find the IP to use
5. After you are connected via ssh, type the following commands in the shell:
```
git clone https://github.com/AndreaFavero71/cubotone.git
cd cubotone/src
sudo ./install/setup.sh
```
6. Make sure the script runs without error until the end. It should ask you to reboot. Type ‘y’ and hit enter. You should get the proper environment after reboot at that point
7. If there is any error during the script execution try to fix it and rerun the script again

# Executing manually
From a shell, you can run the main python script like this:
```
cd ~/cubotone/src
source .virtualenvs/bin/activate
python Cubotone.py
```
of course, you can replace `Cubotone.py` by any other python scripts as mentioned in the documentation.


# Enabling autostart
When everything is tuned and you want to autostart the software automatically on reboot, just type :
```
    sudo crontab -e
```
select the nano editor ‘1’ then go to the last line of the window and remove the first ‘#’ character of the line . This uncomment the startup script that launches cubotino on startup. You can reboot after to test it.

# VNC connection
You can always connect with ssh. If you prefere VNC you can download the RealVNC client (this is the one I use). You just have that start it like this:
```
vncviewer cubotone.local
```
It will ask for the credential. Use ‘pi’ and the same password you use for ssh. You should have a desktop version in this way

Check out the "How_to_make ...  .pdf" document for further info

