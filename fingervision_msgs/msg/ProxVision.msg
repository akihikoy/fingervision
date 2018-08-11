Header header

int32 camera_index
string camera_name
int32 width
int32 height

#Moment of object
#cf. http://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html
#cf. https://en.wikipedia.org/wiki/Image_moment
#Moment of object - spatial moments [m00, m10, m01, m20, m11, m02, m30, m21, m12, m03]
float32[]  ObjM_m
#Moment of object - central moments [mu20, mu11, mu02, mu30, mu21, mu12, mu03]
float32[]  ObjM_mu
#Moment of object - central normalized moments [nu20, nu11, nu02, nu30, nu21, nu12, nu03]
float32[]  ObjM_nu
#Shrunken object matrix [o(x0,y0),o(x1,y0),...,o(xDX,y0); o(x0,y1),o(x1,y1),...,o(xDX,y1); o(x0,yDY),o(x1,yDY),...,o(xDX,yDY)]
float32[]  ObjS
#Shrunken movement matrix [m(x0,y0),m(x1,y0),...,m(xDX,y0); m(x0,y1),m(x1,y1),...,m(xDX,y1); m(x0,yDY),m(x1,yDY),...,m(xDX,yDY)]
float32[]  MvS
