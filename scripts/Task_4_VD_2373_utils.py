def x_to_lat(input_x):
    return input_x/110692.0702932625 + 19


def y_to_long(input_y):
    return -input_y/(105292.0089353767 )+ 72


def lat_to_x(input_latitude):
    return 110692.0702932625 * (input_latitude - 19)


def long_to_y(input_longitude):
    return -105292.0089353767 * (input_longitude - 72)


def limit_value(current, min, max):
    if current < min:
        return min
    elif current > max:
        return max
    else:
        return current

