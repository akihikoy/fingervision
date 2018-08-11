Header header

#FingerVision ID (== camera name)
string fv

#Array of [px,py,fx,fy,fz] for all markers
#where [px,py]: position on the sensor frame, [fx,fy,fz]: force.
float32[] posforce_array

#Array of wrench=[fx,fy,fz,tx,ty,tz] for all markers
#where [fx,fy,fz]: force, [tx,ty,tz]: torque.
float32[] force_array

#Array of discrete state for all markers.
#A discrete state is in {0,1,3,5} that denotes a rough strength of the force.
int32[] dstate_array

#Average of force_array.
float64[] force

#Sum of dstate_array.
int32 dstate
