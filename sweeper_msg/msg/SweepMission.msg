# sweep mission

Header      header          # standard ROS message header
int32       mode            # sweep mode
int32       start_cmd       # 2:pause; 1:start; 0:cancel
int32       side            # 0:right sweep; 1:left sweep
string      map             # sweep map
string      line            # sweep line
int32       trim_accuracy   # Welding accuracy
int32       point_attribute # 0:init point 1:non-driving point 2:welt driving point 3:trace driving point  
                            # 4:turn driving point other:invalid, default last point attribute

# the start pose of map
geometry_msgs/Pose       init_pose      