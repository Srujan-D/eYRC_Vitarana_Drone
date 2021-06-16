'''
Team ID        VD#2373
Theme          Vitarana Drone
Author List    Atharva Chandak, Srujan Deolasse, Naitik Khandelwal, Ayush Agrawal
Filename       Task_6_VD_2373_scheduler.py
Functions      get_dist,x_to_lat,y_to_long,lat_to_x,long_to_y,calc_ret_del_distances,get_schedule_from_pairs,schedule
Global Variables None

'''

# from Task_4_VD_2373_utils import *
import math
from pprint import pprint
import numpy as np

def x_to_lat(input_x):
    return input_x/110692.0702932625 + 19


def y_to_long(input_y):
    return -input_y/(105292.0089353767 )+ 72


def lat_to_x(input_latitude):
    return 110692.0702932625 * (input_latitude - 19)


def long_to_y(input_longitude):
    return -105292.0089353767 * (input_longitude - 72)

def get_dist(coord1,coord2):

    x1=lat_to_x(coord1[0])
    x2=lat_to_x(coord2[0])

    y1=long_to_y(coord1[1])
    y2=long_to_y(coord2[1])

    z1=coord1[2]
    z2=coord2[2]

    return math.sqrt((x2-x1)**2+(y2-y1)**2+(z2-z1)**2)


def calc_ret_del_distances(rets,dels):
    rets_dist_from_dels=[] 
    for ret in rets:
        rets_dist_from_dels.append([])
        for deli in dels:
            rets_dist_from_dels[rets.index(ret)].append(get_dist(ret['from'],deli['to']))
    
    return rets_dist_from_dels

def get_schedule_from_pairs(sorted_pairs,remainder_del_rets=None):

    scheduled_ret = [] 
    scheduled_del = []
    instructions = []

    for pair in sorted_pairs:
        scheduled_del.append(pair['del'])
        instructions.append('DELIVERY')
        scheduled_ret.append(pair['ret'])
        instructions.append('RETURN')
    # pprint(scheduled_ret)
    # pprint(scheduled_del)
    # pprint(instructions)

    return scheduled_ret,scheduled_del,instructions


def schedule(rets,dels):

    rets_dist_from_dels=calc_ret_del_distances(rets,dels)
    rets_dist_from_dels=np.array(rets_dist_from_dels)

    pairs = []
    pair_id = 1

    while rets_dist_from_dels.shape[0]!=1 and rets_dist_from_dels.shape[1]!=1:
        # Find global min distance minimum in rets_dist_from_dels

        min_dist_index=np.argwhere(rets_dist_from_dels == np.min(rets_dist_from_dels))
        # print(min_dist_index)
        min_dist=rets_dist_from_dels[min_dist_index[0][0]][min_dist_index[0][1]]
        # print(min_dist)
        # pair up the ri & dj corresponding to this global min distance
        pairs.append({'ret':rets[min_dist_index[0][0]],'del':dels[min_dist_index[0][1]],'pair_id':pair_id})
        # print("VERIFIED DIST: ", get_dist(pairs[-1]['ret']['from'],pairs[-1]['del']['to']))
        pair_id+=1
        
        # remove the ri row & di column.
        rets_dist_from_dels=np.delete(rets_dist_from_dels,min_dist_index[0][0],0)
        rets_dist_from_dels=np.delete(rets_dist_from_dels,min_dist_index[0][1],1)
        del rets[min_dist_index[0][0]]
        del dels[min_dist_index[0][1]]

        # Repeat untill all are paired


    if rets_dist_from_dels.shape[0]==1 or rets_dist_from_dels.shape[1]==1:
        pairs.append({'ret':rets[0],'del':dels[0]})
        # print("VERIFIED DIST: ", get_dist(pairs[-1]['ret']['from'],pairs[-1]['del']['to']))
        
        # remove the ri row & di column.
        # rets_dist_from_dels=np.delete(rets_dist_from_dels,0,0)
        rets_dist_from_dels=np.delete(rets_dist_from_dels,0,1)
        del rets[0]
        del dels[0]
        # pprint(rets_dist_from_dels)

    # now, finally, sort all distances : ri + dj + ridj ;

    # nearest ones first:
    # sorted_pairs=sorted(pairs,reverse=True,key=lambda i: (get_dist(i['ret']['from'],i['ret']['to'])+get_dist(i['del']['from'],i['del']['to'])+get_dist(i['ret']['from'],i['del']['to'])))
    
    #farthest ones first
    # sorted_pairs=sorted(pairs,reverse=True,key=lambda i: (get_dist(i['ret']['from'],i['ret']['to'])+get_dist(i['del']['from'],i['del']['to'])+get_dist(i['ret']['from'],i['del']['to'])))
    
    #farthest ones with higher cost first -- increases middle timings, thus not ideal
    # sorted_pairs=sorted(pairs,reverse=True,key=lambda i: (get_dist(i['ret']['from'],i['ret']['to'])+get_dist(i['del']['from'],i['del']['to'])))
    
    #Penalise those orders with high intermediate distance 
    sorted_pairs=sorted(pairs,key=lambda i: (get_dist(i['ret']['from'],i['del']['to'])/(get_dist(i['ret']['from'],i['ret']['to'])+get_dist(i['del']['from'],i['del']['to']))))
    
    # dist=[]    
    # for i in sorted_pairs:
    #     dist.append(get_dist(i['ret']['from'],i['ret']['to'])+get_dist(i['del']['from'],i['del']['to']))
    #     print('dist:',dist[-1])
    # print(sum(dist))

    scheduled_ret,scheduled_del,instructions=get_schedule_from_pairs(sorted_pairs)            
    return scheduled_ret,scheduled_del,instructions
