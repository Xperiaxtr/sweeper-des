#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
using namespace std;

pid_t LaunchRosCore()
{
    pid_t iRosCorePid = fork();

    if(iRosCorePid == 0)
    {
        system("roscore");
        exit(0);
    }

    return 0;
}

pid_t LaunchSweeper()
{
    pid_t iSweeperPid = fork();

    if(iSweeperPid == 0)
    {
        system("roslaunch sweeper sweeper.launch");
        exit(0);
    }
    return iSweeperPid;
}

pid_t getProcessPidByName(const char *proc_name)
{
    FILE *fp;
    char buf[100];
    char cmd[200] = {'\0'};
    pid_t pid = -1;
    sprintf(cmd, "pgrep %s", proc_name);
    if((fp = popen(cmd, "r")) != NULL)
    {
        if((fgets(buf, 255, fp)) != NULL)
        {        
           pid = atoi(buf);            
        }
    }
    pclose(fp);
    return pid; 
}

int main(int argc, char** argv)
{   
    pid_t getpid;

    do
    {
        LaunchRosCore();
        sleep(3);
        getpid = getProcessPidByName("roscore");             
    }while(getpid <= 0);
    
    do
    {
        system("killall sweeper");        
        LaunchSweeper();
        sleep(3);
        getpid = getProcessPidByName("sweeper");             
    }while(getpid <= 0);

    return 0;
}
