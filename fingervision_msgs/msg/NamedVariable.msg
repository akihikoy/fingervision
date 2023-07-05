#Kinds of type:
uint8 NONE=0
uint8 INT=1
uint8 FLOAT=2
uint8 BOOL=3
uint8 STR=4

#Kinds of form:
uint8 SCALAR=10
uint8 LIST_1D=11
uint8 LIST_2D=12

#Name of this variable:
string  name
#Type of this variable:
int32   type
#Form of this variable:
int32   form
#Size(s) of this variable used only when the form is a list:
int32[] sizes

#Content of this variable (only one of data_x is used):
int32[]   data_i
float64[] data_f
bool[]    data_b
string[]  data_s
