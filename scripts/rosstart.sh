#! /bin/bash 

#exec 2> /tmp/rosstart.log      # send stderr from rosstart to a log file  
#exec 1>&2                      # send stdout to the same log file  
#set -x                         # tell sh to display commands before execution

source /opt/ros/kinetic/setup.bash
source /home/cidi/sweeper_ws/devel/setup.bash
find /home/cidi/sweeper_ws/src/sweeper_haide/data/log -type f -mtime +5 -exec rm -f {} \;

# canUP=`ifconfig -a |grep can0`
# while [ "" == "$canUP" ]
# do
# 	cat /tmp/emuc2socketcan.log | grep 'Invalid module format'
# 	if [ $? -eq 0 ]; then
# 		cd /home/EMUC-B202_SocketCAN_Driver_v2.5_utility_v2.7
# 		echo "123456" | sudo -S make clean
# 		sleep 1
# 		echo "123456" | sudo -S make
# 		sleep 1
# 		cd /home/EMUC-B202_SocketCAN_Driver_v2.5_utility_v2.7/bootexec
# 		echo "123456" | sudo -S bash add_2_boot.sh
# 		sleep 1
# 		cd /home/EMUC-B202_SocketCAN_Driver_v2.5_utility_v2.7
# 		echo "123456" | sudo -S bash start.sh
# 		sleep 1
# 		canUP=`ifconfig -a |grep can0`
# 	else
# 		canUP=`ifconfig -a |grep can0`
# 		echo "<<<< wait can0 link up<<<" 
# 	fi
		
#     sleep 1
# done

while [ 1 ]
do
    procID=`pgrep roscore`
    if [ "" == "$procID" ];
    then

        killall sweeper
        killall display_lidar_points_node
        killall gnss
        killall wj_718_lidar
        killall lpms_ig1_node
        killall sweeper_controll
        killall planning
        killall radar_fusion
        killall app_server
        killall sweeper_pose_ekf_based_lidar
        killall mapping_node
        killall matcher_node
        killall fusion_imu_sweeper
        killall fusion_gps_imu

        echo "<<<< ros restart <<<"
        cd /home/cidi/sweeper_ws/src/sweeper_haide/scripts
        ./rosrestart      
    fi
    sleep 1
done
