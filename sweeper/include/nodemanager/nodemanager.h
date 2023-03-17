#ifndef NODEMANAGER_H
#define NODEMANAGER_H
#include <ros/ros.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>

#include "../../common/log.h"

namespace sweeper {

class NodeManager
{

public:

    ros::Time reset_time_;

    pid_t node_pid_;

    NodeManager(std::string node, std::string launch);

    void GetPidNode();

    void LaunchNode();

    void KillNode();

    void RestartNode();

private:

    pid_t child_pid = -1;
    std::string node_name;
    std::string launch_name;    

};

}  // namespace sweeper

#endif // NODEMANAGER_H
