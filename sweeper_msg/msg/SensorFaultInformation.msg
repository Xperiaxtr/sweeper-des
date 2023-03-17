# state code

Header      header          # standard ROS message header
int16       type            # falut type
int32[]     state_code      # reuslt_code
bool[]      fault           # true represent sensor is normal
                            # false represent sensor is fault 