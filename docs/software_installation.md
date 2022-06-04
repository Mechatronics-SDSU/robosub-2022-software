# SCION COMPUTER INSTALLATION

### Setup
1. Boot computer with jp461 image
2. Accept terms, set up Language as English (US), timezone to Pacific time
3. Set username to mechatronics, computer name is scion-1 for the nano, scion-0 for the xavier. See google drive for passwords
4. Set to MAXN, NOT 5V (MAXN should be default)

### Important programs
1. sudo apt-get update
2. sudo apt install tmux nano python3-pip git curl -y

### Git
1. git clone https://github.com/Mechatronics-SDSU/robosub-2022-software.git 
(authenticate using your git key)

### pip requirements
1. cd robosub-2022-software/src
2. pip3 install -r requirements.txt (you may need to git checkout beta if its not on master)
3. sudo pip3 install -r requirements.txt

### opencv
1. sudo apt install python3-opencv -y

### ROS

1. sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
2. curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
3. sudo apt update
4. sudo apt install ros-melodic-ros-base
5. sudo apt install python3-all-dev python3-rospkg
6. sudo apt install ros-melodic-desktop-full --fix-missing
7. echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
8. source ~/.bashrc

### Static IP

1. ip a (note which device is the physical ethernet connector)
2. sudo nano /etc/network/interfaces
3. Edit file:
auto <ethernet device>
iface <ethernet device> inet static
	address 192.168.3.1
	netmask 255.255.255.0
4. sudo /etc/init.d/networking restart
