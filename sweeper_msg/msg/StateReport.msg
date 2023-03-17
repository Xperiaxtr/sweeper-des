# sweep state report

Header      header         # standard ROS message header
int32       mode           # sweep mode
int32       start_cmd      # 2:pause; 1:start; 0:cancel
int32       side           # 0:right sweep; 1:left sweep
string      map            # sweep map
string      line           # sweep line
int32       trim_accuracy  # Welding accuracy
int32       default_mode
int32       default_side
string      default_line
string      default_left_trim_accuracy
string      default_right_trim_accuracy
int32[]     result_code    
