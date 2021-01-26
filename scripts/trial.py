from Task_4_VD_2373_utils import *
import math

def get_side_point(drone_position,last_point):
    x2=lat_to_x(drone_position[0])
    x1=lat_to_x(last_point[0])
    y2=long_to_y(drone_position[1])
    y1=long_to_y(last_point[1])
    print('x2',x2)
    print('x1',x1)
    print('y2',y2)
    print('y1',y1)

    if x2==x1:
        x3=x2+1
        y3=y2
    else:
        m=(y2-y1)/(x2-x1)
        x3 = x2 + 5 * m * math.sqrt(1/(1+(m*m)))
        y3 = y2 - (1/m) * (x3-x2)
    print('x3',x3)
    print('y3',y3)
    set_lat=x_to_lat(x3)
    set_long=y_to_long(y3)
    return (set_lat,set_long)

if __name__ == "__main__":
    get_side_point([50.56293574147937,-0.9214412771011903],[50.52218555286118,-0.9432312808791153])