# object msg

int32       id
Point       position    # position
float64     theta       # heading in the world coordinate system.
Point       velocity    # velocity

# size of obstacle bounding box
float64     length
float64     width
float64     height

Point[]     polygon_point   # obstacle corner points. 
float64     tracking_time 

# 0 unknown; 1 unknown movable; 2 unknown unmovable; 3 pedestrian; 
# 4 bicycle; 5 vehicle; 
int32       type    