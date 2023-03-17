# Radar information

#0x60A && 0x60B
int32     id               #object id
float64   lon_dist         #longitude distance to the radar
float64   lat_dist         #lateral distance to the radar
float64   lon_vel          #longitude velocity relative to the radar
float64   lat_vel          #lateral velocity relative to the radar
int32     ob_dp            #dynamic property of the object (etc:0-moving,1-stationary)

#0x60C
int32   lon_dist_rms     #longitude distance standard deviation to the radar; (+) = forward
int32   lat_dist_rms     #lateral distance standard deviation to the radar; (+) = left
int32   lon_vel_rms      #longitude velocity standard deviation to the radar
int32   lat_vel_rms      #lateral velocity standard deviation to the radar
int32   lon_accr_rms     #longitude accleartion standard deviation to the radar
int32   lat_accr_rms     #lateral accleartion standard deviation to the radar
int32   orientation_rms  
int32   prob_exist        #probabiltiy of existence
int32   meas_state       

#0x60D
int32     ob_class         #the object's class
float64   acc_lon          #relative accleartion in longitude
float64   acc_lat          #relative accleartion in lateral
float64   orien_anle       #orientation angle of the object
float64   ob_len
float64   ob_width    



