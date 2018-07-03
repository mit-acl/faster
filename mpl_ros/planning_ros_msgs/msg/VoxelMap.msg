std_msgs/Header header

# Resolution 
float32 resolution

# Origin is the origin of map stored as float
geometry_msgs/Point origin
# Dim is the dimension of the map stored as int
geometry_msgs/Point dim

# Subscript (xi, yi, zi) indices probability data[xi + dim.x * yi + zi * dim.y * dim.z]
int8[] data
