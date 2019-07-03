Header header

#FingerVision ID (== camera name)
string fv

#Array of slips (slip distribution), which is a serialized list of 3x3 matrix.
#Each cell in the 3x3 matrix is the sum of moving pixels in the cell.
float32[] mv_s

#Array of object existence, which is a serialized list of 3x3 matrix.
#Each cell in the 3x3 matrix is the sum of object pixels in the cell.
float32[] obj_s

#Object center position [px,py] (position on the sensor frame).
float64[] obj_center

#Object orientation.
float64 obj_orientation

#Object area.
float64 obj_area

#obj_center/dt (velocity of the object center)
float64[] d_obj_center

#obj_orientation/dt (angular velocity of the object orientation)
float64 d_obj_orientation

#obj_area/dt (speed of the object area change)
float64 d_obj_area

#Filtered values (moving average filter is applied)
float64 obj_area_filtered
float64[] d_obj_center_filtered
float64 d_obj_orientation_filtered
float64 d_obj_area_filtered
