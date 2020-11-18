# hri_client
A ROS package to control and execute scripted Human Robot Interaction actions using the P3-DX robots with mounted VISCA-interface cameras and sound output, and everything interfaced on an onboard Raspberry-PI mini computing module. 

# [1] Raspberry PI device setup

## Add user to groups:
```
sudo usermod -a -G plugdev $USER
sudo usermod -a -G dialout $USER
```

## Enable NetworkManager:
### Create a configuration file:
```
sudo nano /etc/NetworkManager/NetworkManager.conf
```
### Restart the manager:
```
sudo service network-manager restart
```

## Install some necessary upgrades:
```
sudo dpkg-reconfigure -plow unattended-upgrades
```
### Follow prompts and select:
```
NO - OK - INSTALL PACKAGE MAINTAINER'S VERSION
```

## Start hostname services
```
sudo nano /etc/hostname 
sudo nano /etc/hosts
sudo service hostname start
sudo reboot
```

## Install some necessary packages: 
```
sudo apt-get update
sudo apt-get install synaptic openssh-server setserial
```

## Setup ntpdate to look up as a time server the laptop
### This is necessary beacause the router that each raspberry pi connects will not provide internet, so to synchronize them they will all take the Macbook laptop time
*Instructions from: http://www.ubuntugeek.com/network-time-protocol-ntp-server-and-clients-setup-in-ubuntu.html*
```
sudo apt-get remove ntpdate
sudo apt-get install ntp ntpdate
sudo nano /etc/ntp.conf
```
### Add the IP of the laptop that runs the roscore (my IP for example was 192.168.8.171):
```
# Macbook
pool 192.168.8.171
```

## Install some more necessary packages
```
sudo apt-get install \\
gstreamer1.0-alsa \\
gstreamer1.0-clutter-3.0 \\
gstreamer1.0-libav \\
gstreamer1.0-nice \\
gstreamer1.0-plugins-bad \\
gstreamer1.0-plugins-bad-faad \\
gstreamer1.0-plugins-bad-videoparsers \\
gstreamer1.0-plugins-base \\
gstreamer1.0-plugins-base-apps \\
gstreamer1.0-plugins-good \\
gstreamer1.0-plugins-ugly \\
gstreamer1.0-pulseaudio \\
gstreamer1.0-tools \\
gstreamer1.0-x \\
libgstreamer-plugins-bad1.0-dev \\
libgstreamer-plugins-base1.0-0 \\
libgstreamer-plugins-base1.0-dev \\
libgstreamer-plugins-good1.0-0 \\
libgstreamer-plugins-good1.0-dev \\
libgstreamer1.0-0 \\
libgstreamer1.0-dev \\
libgstreamermm-1.0-0v5 \\
libgstreamermm-1.0-dev 
```

# [2] Setup a ROS workspace, bring in dependencies, and build hri_client ROS package

## Need to compile libAria from source, prepare enough swap space (otherwise compilation will crash)
```
dd if=/dev/zero of=~/.swapfile bs=1024 count=1M
mkswap ~/.swapfile
sudo swapon ~/.swapfile
swapon -s
```

## Download the ARIA API sources, build, and install (will take time)
```
cd ~/Downloads/ARIA/ARIA-src-2.9.4
sudo make install
sudo cp -r lib /usr/local/Aria
```

## Create a catkin workspace and download hri_client
```
mkdir -p ~/hri_ws/src
cd ~/hri_ws/src
git clone https://github.com/akatsila/hri_client
git submodule update --init --recursive
```

## Install / Download other required ROS packages
```
git clone https://github.com/amor-ros-pkg/rosaria
sudo apt-get install ros-kinetic-sound-play
```

## Now build the package
```
cd ~/hri_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```
### The hri_client.launch is responsible for launching: ROSAria, hri_client, and sound_play under a namespace that is different for every robot. It has to be edited to match the corresponding robot name
```
nano ~/hri_ws/hri_client.launch 
```
### And change ROBOTNAME to BADGER, HOG, or whatever other cool robot name you use.

# [3] Run the package

## If teleop script is automatically loaded at startup, remove it
```
sudo nano /etc/rc.local
sudo update-rc.d -f teleop_start.sh remove
sudo rm /etc/init.d/teleop_start.sh
```

## Edit some bashrc lines to automatically have ROS_MASTER_URI and ROS_IP exported (substitute correct values)
### ROS_MASTER_URI is the IP of the computer that runs roscore (the experimenter's laptop)
### ROS_IP is the IP of this robot's Raspberry PI
```
nano ~/.bashrc
```
### Add lines:
```
source /opt/ros/kinetic/setup.bash

source /home/$USER/catkin_ws/devel/setup.bash
export ROS_MASTER_URI='http://192.168.8.171:11311'
export ROS_IP='192.168.8.248'
```

## To playback sounds with sound_play, .wav need to be recorded and placed under a folder on the Raspberry PI e.g.
```
~/Audio
```
### The GUI is the one that specifies the filename to be played back. Currently it specifies one of the following names:
```
/home/$USER/Audio/Agree0 -- 4.wav
/home/$USER/Audio/Disagree0 -- 4.wav
/home/$USER/Audio/Intro.wav
/home/$USER/Audio/Line_A -- C.wav
```

## To launch the from the experimenter Main Control computer (e.g. Macbook laptop):
#### First launch the ros core
```
roscore
```
#### Then SSH into each robot
```
ssh <username>@192.168.8.248
```
#### Go to the catkin_ws and source it
```
cd catkin_ws
source devel/setup.bash
```
#### Then run the launch script that launches under a namespace so that the ROS topics of multiple robots do not conflict
```
roslaunch hri_client.launch
```
#### Then we can you can verify that the experimenter computer "sees" the robot's ros nodes by opening a new terminal and giving:
```
rostopic list
```
#### We should now see topics like "/HOG/..." and "/BADGER/..." and whatever other cool robot names are in use and on the ROS network

## Extra notes on how to install startup scripts
```
sudo cp hri_client_start.sh /etc/init.d/ 
sudo chmod 755 /etc/init.d/hri_client_start.sh 
sudo update-rc.d hri_client_start.sh defaults
sudo update-rc.d hri_client_start.sh enable
sudo update-rc.d -f hri_client_start.sh remove
service --status-all
```
