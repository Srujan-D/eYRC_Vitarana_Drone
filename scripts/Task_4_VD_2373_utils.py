import csv
import math 
from pprint import pprint
import copy
from Task_5_VD_2373_scheduler_2 import schedule


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

def write_schedule_to_csv(scheduled_ret,scheduled_del,orig_returns,orig_deliveries):
    print(orig_returns)
    with open('/home/atharva/catkin_ws/src/vitarana_drone/scripts/sequenced_manifest.csv','w') as fhand:
        writer = csv.writer(fhand)
        for i in range(max(len(scheduled_ret),len(scheduled_del))):
            if i<len(scheduled_del):
                deli = (filter(lambda orig_del: scheduled_del[i]['id'] == orig_del['id'], orig_deliveries))
                writer.writerow([deli[0]['type'],deli[0]['from'],"{};{};{}".format(deli[0]['to'][0],deli[0]['to'][1],deli[0]['to'][2])])
            if i<len(scheduled_ret):
                ret = (filter(lambda orig_ret: scheduled_ret[i]['id'] == orig_ret['id'], orig_returns))
                writer.writerow([ret[0]['type'],"{};{};{}".format(ret[0]['from'][0],ret[0]['from'][1],ret[0]['from'][2]),ret[0]['to']])
            pprint(ret)
            pprint(deli)
        # exit()
            # writer.writerow(scheduled_ret)

    return

def read_manifest_data():
    deliveries=[]
    returns=[]
    idx=0
    with open('/home/atharva/catkin_ws/src/vitarana_drone/scripts/original.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            idx+=1
            if row[0]=='DELIVERY':
                coords=row[2].split(';')                                                          
                deliveries.append({'id':idx,'type':'DELIVERY','from':row[1],'to':[float(coords[0]),float(coords[1]),float(coords[2])]})

            else:
                coords=row[1].split(';')
                returns.append({'id':idx,'type':'RETURN','to':row[2],'from':[float(coords[0]),float(coords[1]),float(coords[2])]})
    # print(returns)                
    return deliveries,returns

        #     delivery_control.parcels_delivery_coords.append([float(row[1]),float(row[2]),float(row[3])+1]) # 1m height buffer 
def get_dist(coord1,coord2):

    x1=lat_to_x(coord1[0])
    x2=lat_to_x(coord2[0])

    y1=long_to_y(coord1[1])
    y2=long_to_y(coord2[1])

    z1=coord1[2]
    z2=coord2[2]

    return math.sqrt((x2-x1)**2+(y2-y1)**2+(z2-z1)**2)

def get_set_point_sequence():               #for pickup attachment
    
    # defining the constants
    A1 = [18.9998102845,72.000142461,16.757981 - 0.2]
    X1 = [18.9999367615,72.000142461,16.757981]
    delta_lat = 0.000013552 
    delta_long = 0.000014245

    deliveries,returns = read_manifest_data()
    orig_deliveries = copy.deepcopy(deliveries)
    orig_returns =  copy.deepcopy(returns)

    for delivery in deliveries:
        if delivery['from'][0] == 'A':
            delivery['from'] = [A1[0] , A1[1]+delta_long*(int(delivery['from'][1])-1) , A1[2]]
        elif delivery['from'][0] == 'B':
            delivery['from'] = [A1[0]+(delta_lat), A1[1]+delta_long*(int(delivery['from'][1])-1) , A1[2]]
        elif delivery['from'][0] == 'C':
            delivery['from'] = [A1[0]+2*(delta_lat), A1[1]+delta_long*(int(delivery['from'][1])-1), A1[2]]
        delivery['from_to_dist'] = get_dist(delivery['from'],delivery['to'])

    for return_coord in returns:
        if return_coord['to'][0]=='X':
            return_coord['to']=[X1[0], X1[1]+delta_long*(int(return_coord['to'][1])-1)  , X1[2]]
        elif return_coord['to'][0]=='Y':
            return_coord['to']=[X1[0]+(delta_lat), X1[1]+delta_long*(int(return_coord['to'][1])-1)  , X1[2]]
        elif return_coord['to'][0]=='Z':
            return_coord['to']=[X1[0] +2*(delta_lat), X1[1]+delta_long*(int(return_coord['to'][1])-1) , X1[2]]
        return_coord['from_to_dist']=get_dist(return_coord['from'],return_coord['to'])


    parcels_delivery_coords=[]
    parcels_coords=[]

    returns=sorted(returns, key = lambda i: i['from_to_dist'])
    deliveries=sorted(deliveries, key = lambda i: i['from_to_dist'])

    scheduled_ret,scheduled_del,instructions = schedule(returns,deliveries)
    # scheduled_ret,scheduled_del,instructions = returns,deliveries,['DELIVERY','RETURN','DELIVERY','RETURN','DELIVERY','RETURN','DELIVERY']
    for re in returns:
        scheduled_ret.append(re)
        print("")
        print("ADDED ret:",re)
        print("")
        del re
    for deli in deliveries:
        scheduled_del.append(deli)
        print("")
        print("ADDED del:",deli)
        print("")
        del deli


    write_schedule_to_csv(scheduled_ret,scheduled_del,orig_returns,orig_deliveries)

    # for ret in returns:
    #     print(get_dist())
    #     for i in range(2):
    #         ret['to'][i]-=start_coords[i]
    #         ret['from'][i]-=start_coords[i]

    # for deli in deliveries:
    #     for i in range(2):
    #         deli['to'][i]-=start_coords[i]
    #         deli['from'][i]-=start_coords[i]

    for i in range(max(len(scheduled_del),len(scheduled_ret))):
        if i<len(scheduled_del):
            parcels_coords.append(scheduled_del[i]['from'])
            parcels_delivery_coords.append(scheduled_del[i]['to'])
        if i<len(scheduled_ret):
            parcels_coords.append(scheduled_ret[i]['from'])
            parcels_delivery_coords.append(scheduled_ret[i]['to'])

    pprint(parcels_coords)
    pprint(parcels_delivery_coords)
    pprint(instructions)

    return parcels_coords,parcels_delivery_coords,instructions

if __name__=="__main__":
    print("IN")
    get_set_point_sequence()
