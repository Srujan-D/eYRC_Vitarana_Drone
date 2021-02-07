import csv

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

def read_manifest_data():
    deliveries=[]
    returns=[]
    with open('/home/atharva/catkin_ws/src/vitarana_drone/scripts/manifest.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            if row[0]=='DELIVERY':
                coords=row[2].split(';')                                                          
                deliveries.append({'from':row[1],'to':[float(coords[0]),float(coords[1]),float(coords[2])]})

            else:
                coords=row[1].split(';')
                returns.append({'to':row[2],'from':[float(coords[0]),float(coords[1]),float(coords[2])]})
    # print(returns)                
    return deliveries,returns

        #     delivery_control.parcels_delivery_coords.append([float(row[1]),float(row[2]),float(row[3])+1]) # 1m height buffer 

def get_set_point_sequence():               #for pickup attachment
    A1 = [18.9998102845,72.000142461,16.757981 - 0.2]
    X1 = [18.9999367615,72.000142461,16.757981]
    delta_lat = 0.000013552 
    delta_long = 0.000014245

    deliveries,returns=read_manifest_data()
    for delivery in deliveries:
        if delivery['from'][0]=='A':
            delivery['from']=[A1[0]+delta_lat*(int(delivery['from'][1])-1) , A1[1] , A1[2]]
        elif delivery['from'][0]=='B':
            delivery['from']=[A1[0]+delta_lat*(int(delivery['from'][1])-1) , A1[1]+(delta_long) , A1[2]]
        elif delivery['from'][0]=='C':
            delivery['from']=[A1[0]+delta_lat*(int(delivery['from'][1])-1) , A1[1]+2*(delta_long) , A1[2]]

    for return_coord in returns:
        if return_coord['to'][0]=='X':
            return_coord['to']=[X1[0]+delta_lat*(int(return_coord['to'][1])-1) , X1[1] , X1[2]]
        elif return_coord['to'][0]=='Y':
            return_coord['to']=[X1[0]+delta_lat*(int(return_coord['to'][1])-1) , X1[1]+(delta_long) , X1[2]]
        elif return_coord['to'][0]=='Z':
            return_coord['to']=[X1[0]+delta_lat*(int(return_coord['to'][1])-1) , X1[1]+2*(delta_long) , X1[2]]

    print("")
    print("RETURNS:")
    # print(returns)
    print(deliveries)
    print("")
    parcels_delivery_coords=[]
    parcels_coords=[]

    # remove this hard coded stuff
    instructions=['DELIVERY','RETURN','DELIVERY','RETURN','DELIVERY','RETURN','DELIVERY']
    
    for delivery in deliveries:
        parcels_coords.append(delivery['from'])
        parcels_delivery_coords.append(delivery['to'])
        parcels_coords.append(returns[deliveries.index(delivery)]['from'])
        parcels_delivery_coords.append(returns[deliveries.index(delivery)]['to'])
        

    return parcels_coords,parcels_delivery_coords,instructions
