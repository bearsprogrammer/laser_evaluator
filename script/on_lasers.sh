#!/bin/bash

# ydlidar ver.1

# Red is 1
# Green is 2
# Reset is sgr0

######################
# modules ip
c1=192.168.0.5
c2=192.168.0.6
c3=192.168.0.4
c4=192.168.0.2
# module's password
pw='-pcona'
######################

source ./allen_watermarking.sh

echo -e "\vPlease input host's ip(192.168.0.xx): \c "
read ip

echo "===== Check network each of mudules ====="
echo -n "host${ip} ..."
tput setaf 2
echo "[cona1]"
tput sgr0
ping -c 3 ${c1} 

echo -e "\v"
echo -n "host${ip} ..."
tput setaf 2
echo "[cona2]"
tput sgr0
ping -c 3 ${c2}

echo -e "\v"
echo -n "host${ip} ..."
tput setaf 2
echo "[cona3]"
tput sgr0
ping -c 3 ${c3}

echo -e "\v"
echo -n "host${ip} ..."
tput setaf 2
echo "[cona4]"
tput sgr0
ping -c 3 ${c4}

echo -e "=========================================\v"
echo -e "\vWhole network of modules are available? (y/n): \c "
read on
if [ ${on} = 'y' ]
then 
	#host	
	export ROS_IP=${ip}
	export ROS_MASTER_URI=http://localhost:11311
	export ROS_HOSTNAME=${ip}
	#each of cona
	sshpass ${pw} ssh -o StrictHostKeyChecking=no cona1@${c1} "export ROS_MASTER_URI=http://${ip}:11311"
	sshpass ${pw} ssh -o StrictHostKeyChecking=no cona2@${c2} "export ROS_MASTER_URI=http://${ip}:11311"
	sshpass ${pw} ssh -o StrictHostKeyChecking=no cona3@${c3} "export ROS_MASTER_URI=http://${ip}:11311"
	sshpass ${pw} ssh -o StrictHostKeyChecking=no cona4@${c4} "export ROS_MASTER_URI=http://${ip}:11311"

	sshpass ${pw} ssh -o StrictHostKeyChecking=no cona1@${c1} "export ROS_HOSTNAME=${c1}"
	sshpass ${pw} ssh -o StrictHostKeyChecking=no cona2@${c2} "export ROS_HOSTNAME=${c2}"
	sshpass ${pw} ssh -o StrictHostKeyChecking=no cona3@${c3} "export ROS_HOSTNAME=${c3}"
	sshpass ${pw} ssh -o StrictHostKeyChecking=no cona4@${c4} "export ROS_HOSTNAME=${c4}"

	sshpass ${pw} ssh -o StrictHostKeyChecking=no cona1@${c1} "roslaunch ydlidar_ros G4.launch"
	sshpass ${pw} ssh -o StrictHostKeyChecking=no cona2@${c2} "roslaunch ydlidar_ros G4.launch"
	sshpass ${pw} ssh -o StrictHostKeyChecking=no cona3@${c3} "roslaunch ydlidar_ros G4.launch"
	sshpass ${pw} ssh -o StrictHostKeyChecking=no cona4@${c4} "roslaunch ydlidar_ros G4.launch"
	echo "on!" 
else 
	echo "bye!"
	exit
fi

