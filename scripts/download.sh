#! /bin/bash 

exec 2> /tmp/update.log      # send stderr from rosstart to a log file  
exec 1>&2                      # send stdout to the same log file  
set -x                         # tell sh to display commands before execution

function update_param() {
   if [ ! -d $SWEEP_HOME/src/sweeper_${array[1]} ];then
       exit 1
   fi
   rm -rf $SWEEP_HOME/src/sweeper_${array[1]}/calibration/data
   mv calibration/data $SWEEP_HOME/src/sweeper_${array[1]}/calibration/
   rm -rf $SWEEP_HOME/src/sweeper_${array[1]}/sweeper/launch
   mv sweeper/launch $SWEEP_HOME/src/sweeper_${array[1]}/sweeper/
   rm -rf calibration
   rm -rf sweeper
}

function update_program() {
   if [ ! -d $SWEEP_HOME/src/sweeper_${array[1]} ];then
       exit 1
   fi
   mv $SWEEP_HOME/src/sweeper_${array[1]}/calibration/data src/sweeper_${array[1]}/calibration/
   mv $SWEEP_HOME/src/sweeper_${array[1]}/sweeper/launch src/sweeper_${array[1]}/sweeper
   echo $SWEEP_PW | sudo -S rm -rf $SWEEP_HOME/devel
   echo $SWEEP_PW | sudo -S rm -rf $SWEEP_HOME/src
   echo $SWEEP_PW | sudo -S mv devel $SWEEP_HOME/
   echo $SWEEP_PW | sudo -S mv src   $SWEEP_HOME/
}

function update_all() {

    mv calibration/data src/sweeper_${array[1]}/calibration/
    mv sweeper/launch   src/sweeper_${array[1]}/sweeper/
    echo $SWEEP_PW | sudo -S rm -rf $SWEEP_HOME/devel
    echo $SWEEP_PW | sudo -S rm -rf $SWEEP_HOME/src
    rm -rf calibration
    rm -rf sweeper
    echo $SWEEP_PW | sudo -S mv devel $SWEEP_HOME/
    echo $SWEEP_PW | sudo -S mv src   $SWEEP_HOME/
}

function check_server_online() {
    if ! ping -c 1 -w 3 $SERVER_IP >/dev/null 2>&1 ; then
        echo "netstate:-1" >> /tmp/update.txt
        exit 1
    fi
    echo "netstate:0" >> /tmp/update.txt
}

function check_car_number() {
    string=$(find /home/cidi -maxdepth 1 -name "type-*" -type f -print)
    if [ ! $string ]; then  
        exit 1 
    fi
    array=(${string//-/ })
}

function check_path() {
    if [ ! -d "/home/cidi/down" ];then
        mkdir /home/cidi/down
        echo $SWEEP_PW | sudo -S chmod 777 /home/cidi/down
    fi

    if [ ! -d "/home/cidi/sweeper_ws" ];then
        mkdir /home/cidi/sweeper_ws
        echo $SWEEP_PW | sudo -S chmod 777 /home/cidi/sweeper_ws
    fi

    if [ ! -d $SWEEP_HOME/src ];then
        rm /home/cidi/down/*
    fi
}

function main() {
    SERVER_IP=111.23.140.244
    SERVER_PORT=18880
    SWEEP_HOME="/home/cidi/sweeper_ws"
    SWEEP_PW="123456"

    check_car_number
    check_server_online
    check_path

    cd /home/cidi/down       
    rm /tmp/param.log


    wget -S -N  -t 10 http://$SERVER_IP:$SERVER_PORT/${array[1]}/${array[2]}/param -o /tmp/param.log
    cat /tmp/param.log | grep 100% > /dev/null
    if [ $? -eq 0 ]; then
        sleep 1
        tar -xvf param > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            PARAM_DOWN=0
        else
            PARAM_DOWN=-1
        fi
    else
        cat /tmp/param.log | grep "Omitting download" > /dev/null
        if [ $? -eq 0 ]; then
           PARAM_DOWN=1
        else
           PARAM_DOWN=-1 
        fi
    fi

    rm /tmp/upgrade.log
    wget -S -N  -t 10 http://$SERVER_IP:$SERVER_PORT/upgrade -o /tmp/upgrade.log
    cat /tmp/upgrade.log | grep 100% > /dev/null
    if [ $? -eq 0 ]; then
        sleep 1
        tar -xvf upgrade > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            PROGRAM_DOWN=0
        else
            PROGRAM_DOWN=-1
        fi
    else
        cat /tmp/upgrade.log | grep "Omitting download" > /dev/null
        if [ $? -eq 0 ]; then
           PROGRAM_DOWN=1
        else
           PROGRAM_DOWN=-1 
        fi
    fi

    if [ $PARAM_DOWN != 0 ] && [ $PROGRAM_DOWN == 0 ]; then
        update_program
        echo "program:0" >> /tmp/update.txt
        if [ $PARAM_DOWN == -1 ]; then
            echo "param:-1" >> /tmp/update.txt
        else
            echo "param:1" >> /tmp/update.txt
        fi
    elif [ $PARAM_DOWN == 0 ] && [ $PROGRAM_DOWN != 0 ]; then
        update_param
        echo "param:0"  >> /tmp/update.txt 
        if [ $PROGRAM_DOWN == -1 ]; then
            echo "program:-1" >> /tmp/update.txt
        else
            echo "program:1" >> /tmp/update.txt
        fi     
    elif [ $PARAM_DOWN == 0 ] && [ $PROGRAM_DOWN == 0 ]; then
        update_all
        echo "program:0" >> /tmp/update.txt
        echo "param:0" >> /tmp/update.txt
    else
        if [ $PARAM_DOWN == -1 ]; then
            echo "param:-1" >> /tmp/update.txt
        fi
        if [ $PROGRAM_DOWN == -1 ]; then
            echo "program:-1" >> /tmp/update.txt
        fi  
        if [ $PARAM_DOWN == 1 ]; then
            echo "param:1" >> /tmp/update.txt
        fi
        if [ $PROGRAM_DOWN == 1 ]; then
            echo "program:1" >> /tmp/update.txt
        fi      
    fi
}

main $@

