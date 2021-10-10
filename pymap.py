from glob import glob
from operator import index
from sys import path
import pymap3d as pm
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from math import sqrt, atan2, pi, cos, sin


def pathreader(directory):
    f = open(directory, 'r')

    # f = open("/home/changjun/gps_logging_1.txt", 'r')
    # f = open("/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/scripts/src/Database/gps_logging_HDMAP/DeliveryLine1.csv", 'r')
    # f = open("/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/txt/main_path.txt", 'r')
    # f = open("/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/gps_logging.txt", 'r')
    path = []
    for i in f.readlines():
        line = i.split(',')
        # path.append([float(line[0].rstrip()), float(line[1].rstrip()), int(line[2].rstrip())])              # For lat, lon
        path.append( [float(line[1].rstrip()), float(line[0].rstrip()), int( line[2].rstrip() ) ] )              # For lon, lat
    f.close()
    path2 = np.array(path).T


    # lat0 = 37.293607 # deg
    # lon0 = 126.977482  # deg
    lat0 = 37.23861340 # deg
    lon0 = 126.77248216  # deg
    h0 = 0     # meters

    # The point of interest
    lat = path2[0]
    lon = path2[1]
    h = 0      # meters

    gps2coordinate = pm.geodetic2enu(lat, lon, h, lat0, lon0, h0, ell=pm.utils.Ellipsoid('grs80'))
    track = [gps2coordinate[0], gps2coordinate[1], path2[2]]

    # track = path2
    total_track = [[track[0][0], track[1][0], track[2][0]]]
    total_track_index = []
    track_index = 0
    prev_track_angle = 0
    curr_track_angle = 0
    # while ((track[0][track_index] - track[0][-1]) ** 2 + (track[1][track_index] - track[1][-1]) ** 2) ** 0.5 >= 0.5:
    #     for i in range(track_index, 300):
    #         dis = ((track[0][i] - track[0][track_index]) ** 2 + (track[1][i] - track[1][track_index]) ** 2) ** 0.5
    #         curr_track_angle = atan2(track[0][i] - track[0][track_index], track[1][i] - track[1][track_index])
    #         if dis >= 0.5 :
    #             print(prev_track_angle * 180/pi, curr_track_angle * 180/pi, track_index)
    #             total_track.append([track[0][i], track[1][i]]) 
    #             prev_track_angle = curr_track_angle
    #             track_index = i
    #             break
    while ((track[0][track_index] - track[0][-1]) ** 2 + (track[1][track_index] - track[1][-1]) ** 2) ** 0.5 >= 0.5:
        for i in range(track_index, len(track[0])):
            dis = ((track[0][track_index] - track[0][i]) ** 2 + (track[1][track_index] - track[1][i]) ** 2) ** 0.5
            if dis >= 0.5:
                total_track.append([track[0][i], track[1][i], track[2][i]])
                track_index = i
                total_track_index.append(track_index)
                break
    # print(total_track_index)
    global_path = np.array(total_track).T
    # global_path = track

    # print(global_path)
    # reference_angle = atan2(global_path[0][5] - global_path[0][0], global_path[1][5] - global_path[1][0])
    # reference_angle = 28.3168 * pi/180 # 북위 기준 처음 시작 각도, radian
    # print(reference_angle * 180/pi)

    # global_path_rotation = [cos(reference_angle) * (global_path[0] - global_path[0][0]) - sin(reference_angle) * (global_path[1] - global_path[1][0]), sin(reference_angle) * (global_path[0] - global_path[0][0]) + cos(reference_angle) * (global_path[1] - global_path[1][0])]
    # global_path_rotation = [cos(reference_angle) * global_path[0] - sin(reference_angle) * global_path[1] - global_path[0][0], sin(reference_angle) * global_path[0] + cos(reference_angle) * global_path[1] - global_path[1][0]]

    # print(global_path_rotation)
    # return global_path_rotation
    
    
    # return global_path, total_track_index
    return global_path

def curve_velocity(global_path):
    car_max_speed = 12 #km/h
    car_lanechange_speed = 10
    car_stopline_speed = 8
    car_min_speed =6



    out_vel_plan = []
    road_friction = 0.7
    velocity_gain = 0.8
    interval = 20 #속도 극솟값 기준 앞뒤로 10m, 총 20m  8


    for i in range(len(global_path[0])):
        if global_path[2][i] == 0:
            out_vel_plan.append(car_max_speed) # 직진
        elif global_path[2][i] == 1 or global_path[2][i] == -1:
            out_vel_plan.append(car_min_speed) # 회전
        elif global_path[2][i] == 2:
            out_vel_plan.append(car_stopline_speed) # 정지선
        else:
            out_vel_plan.append(car_lanechange_speed) # 차선 변경

    



    # dx_dt = np.gradient(global_path[0])
    # dy_dt = np.gradient(global_path[1])
    # dr_dt = np.array([dx_dt, dy_dt]).T

    # x_dot = np.gradient(global_path[0])
    # y_dot = np.gradient(global_path[1])
    # x_doubledot = np.gradient(x_dot)
    # y_doubledot = np.gradient(y_dot)

    # ds_dt = (dx_dt ** 2 + dy_dt ** 2) ** 0.5

    # curvature = abs(x_dot * y_doubledot - y_dot * x_doubledot) / (x_dot ** 2 + y_dot ** 2) ** 1.5
    # radius = 1/curvature

    # out_vel_plan = velocity_gain * (radius * 9.8 * road_friction) ** 0.5
    # for i in range(len(out_vel_plan)):
    #     if out_vel_plan[i] > car_max_speed:
    #         out_vel_plan[i] = car_max_speed

    # local_min_index, _ = find_peaks(-np.array(out_vel_plan), distance = interval * 2)
    # local_min = []

    # for i in range(len(local_min_index)):
    #     local_min.append(out_vel_plan[local_min_index[i]])

    # for i in range(len(out_vel_plan)):
    #     if i not in local_min_index:
    #         out_vel_plan[i] = car_max_speed

    # for i in local_min_index:
    #     if i > interval and i < len(out_vel_plan) - interval:
    #         for j in range(-int(interval/4) , int(interval/4)):
    #             out_vel_plan[i - j] = out_vel_plan[i]

    #         for j in range(-interval, interval):
    #             if (j in range(-interval, -int(interval/4)) or j in range(int(interval/4), interval)) and out_vel_plan[i] + (abs(j) - int(interval/4))/2 < car_max_speed:
    #                 out_vel_plan[i - j] = out_vel_plan[i] + (abs(j) - int(interval/4))/2





    # for i in range(len(out_vel_plan)):
    #     if out_vel_plan[i] < car_min_speed:
    #         out_vel_plan[i] = car_min_speed
    
    # out_vel_plan = out_vel_plan.tolist()
    # del(out_vel_plan[0:20])
    # for i in range(20):
    #     out_vel_plan.append(car_max_speed)



    # out_vel_plan[-1] = car_min_speed
    # stop_velocity_index = 0
    # while out_vel_plan[len(out_vel_plan) - stop_velocity_index] < car_max_speed:
    #     out_vel_plan[len(out_vel_plan) - stop_velocity_index] = stop_velocity_index - 1
    #     stop_velocity_index += 1

    # print(len(out_vel_plan))
    # print(out_vel_plan)


    plt.figure(1)
    plt.scatter(global_path[0], global_path[1], s = 3)

    for i in range(len(global_path[0])):
        if global_path[2][i] == 1 or global_path[2][i] == -1:
            plt.scatter(global_path[0][i], global_path[1][i], c = 'y')
        elif global_path[2][i] == 2:
            plt.scatter(global_path[0][i], global_path[1][i], c = 'r')
        elif global_path[2][i] == 0:
            plt.scatter(global_path[0][i], global_path[1][i], c = 'g')
        else :
            plt.scatter(global_path[0][i], global_path[1][i], c = 'b')

    # plt.scatter(global_path[0][0], global_path[1][0])
    # for i in range(len(local_min_index)):
    #     plt.plot(global_path[0][local_min_index[i]], global_path[1][local_min_index[i]], 'x')
    plt.axis('scaled')

    plt.figure(2)
    plt.plot(out_vel_plan)
    # plt.plot(local_min_index, local_min, 'x')
    # plt.plot(len(out_vel_plan)-20, 0, 'x')
    plt.show()

    # return out_vel_plan, local_min_index

    return out_vel_plan, 0


# main_path = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/txt/main_path.txt')

# practice_path = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/gps_logging.txt')


# pre
# TrafficPre1 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/scripts/src/Database/gps_logging_HDMAP/TrafficPre1.csv')
# GpsPre1 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/scripts/src/Database/gps_logging_HDMAP/GpsPre1.csv')
# SchoolZone = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/scripts/src/Database/gps_logging_HDMAP/SchoolZone.csv')
# GpsPre2 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/scripts/src/Database/gps_logging_HDMAP/GpsPre2.csv')
# GpsPreReturn = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/scripts/src/Database/gps_logging_HDMAP/GpsPreReturn.csv')

# pre_path_1 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/최종예선로깅/1번구간.csv')
# pre_path_2 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/최종예선로깅/2번구간.csv')
# pre_path_3 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/최종예선로깅/3번구간.csv')
# pre_path_4 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/최종예선로깅/4번구간.csv')
# pre_path_5 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/최종예선로깅/5번구간.csv')



# main
Parking = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/Parking.csv')
TrafficMain1 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/TrafficMain1.csv')
TrafficMain2 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/TrafficMain2.csv')
Obstacle = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/Obstacle.csv')
TrafficToDelivery = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/TrafficToDelivery.csv')
Delivery1 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/Delivery1.csv')
Delivery2 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/Delivery2_Half.csv')
TrafficAfterDelivery = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/TrafficAfterDelivery.csv')
TrafficEndOfMap = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/TrafficEndOfMap.csv')
TrafficStrange = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/TrafficStrange.csv')
TrafficMainReturn1 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/TrafficMainReturn1.csv')
TrafficMainReturn2 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/TrafficMainReturn2.csv')
TrafficMainReturn3 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/TrafficMainReturn3.csv')
GpsMainReturn = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/GpsMainReturn.csv')




# GpsSample1 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/Delivery2.csv')
# GpsSample2 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/GpsMainReturn.csv')
# GpsSample3 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/Delivery2_First.csv')
# GpsSample4 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/TrafficMainReturn2.csv')
# GpsSample5 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/TrafficMainReturn3.csv')
# GpsSample6 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/GpsMainReturn.csv')


GpsSample7 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/gps_logging_kcity_return_all.csv')
GpsSample8 = pathreader('/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/kcity/gps_logging_delivery.csv')

# plt.figure(1)
# for i in range(len(GpsSample3[0])):
#     if GpsSample3[2][i] == 2:
#         plt.scatter(GpsSample3[0][i], GpsSample3[1][i], c = 'g')
#     else:
#         plt.scatter(GpsSample3[0][i], GpsSample3[1][i], c = 'k')

# for i in range(len(GpsSample6[0])):
#     if GpsSample6[2][i] == 2:
#         plt.scatter(GpsSample6[0][i], GpsSample6[1][i], c = 'g')
#     else:
#         plt.scatter(GpsSample6[0][i], GpsSample6[1][i], c = 'r')

# plt.plot(GpsSample1[0], GpsSample1[1], c = 'r')
# plt.plot(GpsSample2[0], GpsSample2[1], c = 'r')
# plt.plot(GpsSample3[0], GpsSample3[1], c = 'k')
# plt.plot(GpsSample4[0], GpsSample4[1], c = 'r')
# plt.plot(GpsSample5[0], GpsSample5[1], c = 'r')
# plt.plot(GpsSample6[0], GpsSample6[1], c = 'r')


# plt.plot(GpsSample[0], GpsSample[1], c = 'k')



plt.figure(2)

plt.plot(Parking[0], Parking[1], c = 'k')
plt.plot(TrafficMain1[0], TrafficMain1[1], c = 'k')
plt.plot(TrafficMain2[0], TrafficMain2[1], c = 'k')
plt.plot(Obstacle[0], Obstacle[1], c = 'k')
plt.plot(TrafficToDelivery[0], TrafficToDelivery[1], c = 'k')
plt.scatter(Delivery1[0], Delivery1[1], c = 'b')
plt.scatter(Delivery2[0], Delivery2[1], c = 'r')
plt.plot(TrafficAfterDelivery[0], TrafficAfterDelivery[1], c = 'k')
plt.plot(TrafficEndOfMap[0], TrafficEndOfMap[1], c = 'k')
plt.plot(TrafficStrange[0], TrafficStrange[1], c = 'k')
plt.plot(TrafficMainReturn1[0], TrafficMainReturn1[1], c = 'k')
plt.plot(TrafficMainReturn2[0], TrafficMainReturn2[1], c = 'k')
plt.plot(TrafficMainReturn3[0], TrafficMainReturn3[1], c = 'k')
plt.plot(GpsMainReturn[0], GpsMainReturn[1], c = 'k')

plt.plot(GpsSample7[0], GpsSample7[1], c = 'r')
plt.plot(GpsSample8[0], GpsSample8[1], c = 'r')


plt.axis('scaled')

plt.show()




# target_velocity, curve_index = curve_velocity(GpsSample6)

# print(curve_index)
# for i in range(len(curve_index)):
#     print(track_index[curve_index[i]])



# plt.plot(pre_path_1[0], pre_path_1[1], c = 'k')
# plt.plot(pre_path_2[0], pre_path_2[1], c = 'b')
# plt.plot(pre_path_3[0], pre_path_3[1], c = 'b')
# plt.plot(pre_path_4[0], pre_path_4[1], c = 'k')
# plt.plot(pre_path_5[0], pre_path_5[1], c = 'k')


# plt.plot(path_1[0], path_1[1], c = 'k')
# plt.plot(path_2[0], path_2[1], c = 'k')
# plt.plot(path_3[0], path_3[1], c = 'k')
# plt.plot(path_4_1[0], path_4_1[1], c = 'k')
# plt.plot(path_4_2[0], path_4_2[1], c = 'k')
# plt.plot(path_5[0], path_5[1], c = 'k')
# plt.plot(path_6[0], path_6[1], c = 'k')
# plt.plot(path_7[0], path_7[1], c = 'k')
# plt.plot(path_8[0], path_8[1], c = 'k')
# plt.plot(path_9[0], path_9[1], c = 'k')
# plt.plot(path_10[0], path_10[1], c = 'k')
# plt.plot(path_11[0], path_11[1], c = 'k')
# plt.plot(path_12[0], path_12[1], c = 'k')
# plt.plot(path_13[0], path_13[1], c = 'k')
# plt.plot(path_14[0], path_14[1], c = 'k')


